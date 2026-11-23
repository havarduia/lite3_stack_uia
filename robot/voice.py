#!/usr/bin/env python3
"""voice - his voice: text-to-speech out of the robot's own speaker.

say() is Kokoro on the Orin's GPU (orin/bin/tts_server.py, RemoteSpeaker), or a
Piper voice on this computer when one is named. The sound goes out through the
SDK's speaker (lite3.speaker: ssh + aplay on the motion computer), so clips,
files and speech share one volume, lite3.speaker.VOLUME. play_url is the only
thing here that goes to the internet. docs/voice.md.

    from robot.voice import Talker
    Talker().play('OKstandup')                 # one of the robot's built-in clips
    Talker().say('watch your step')            # text-to-speech

From the shell:

    python3 -m robot.voice say "hello there"   # speak exactly this text
    python3 -m robot.voice play OKstandup      # a built-in clip
    python3 -m robot.voice file ~/alarm.wav    # any local audio
    python3 -m robot.voice url <youtube url>   # a video's audio
    python3 -m robot.voice clips | voices      # list the clips / Piper voices
  options: --voice PIPER_VOICE  --alien
"""
import glob
import json
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import wave
from functools import cached_property

import lite3.speaker as S
from lite3.speaker import HOST, Speaker, SpeakerError, aplay_command, ssh_command, to_speaker

from .protocol import ORIN_IP

# For play_url's yt-dlp: an HTTP proxy if the robot has no internet of its own. Empty = direct.
PROXY = os.environ.get('TALK_PROXY', '')


class TalkError(SpeakerError):
    pass


# play_url(): standalone aarch64 binaries copied in from the laptop (no pip
# or github from here). yt-dlp goes out via TALK_PROXY when one is set.
YTDLP = os.path.expanduser('~/.local/bin/yt-dlp')
DENO = os.path.expanduser('~/.local/bin/deno')    # yt-dlp's JS runtime for YouTube

# say(): Piper, the standalone aarch64 release copied in from the laptop
# (2026-09-18). ~/piper is not on PATH for non-interactive ssh, so it is found
# here by absolute path. Voices are ~/piper/<name>.onnx (+ .onnx.json).
PIPER = os.path.expanduser('~/piper/piper')
PIPER_DIR = os.path.expanduser('~/piper')
PIPER_VOICE = 'en_US-ryan-medium'    # his voice
# Low noise = flat intonation and even timing: the "translation computer" sound.
PIPER_ARGS = ['--noise_scale', '0.1', '--noise_w', '0.1', '--length_scale', '1.1']
# say() is Kokoro on the Orin's GPU (orin/bin/tts_server.py, voice bm_george, ~0.4 s a
# sentence, the user's pick 2026-11-09). No fallback: Orin's voice down, no speech.
TTS_SERVER = (ORIN_IP, 8047)
TTS_RATE = 24000                    # tts_server.RATE
TTS_CONNECT_S = 0.5                 # the Orin is on the cable: an answer is instant, or it is down
# Loudness: the codec PCM volume goes to max (192 = 0 dB) and speech gets a
# software gain, a compressor and a limiter (SPEECH_LOUD, docs/voice.md).
HW_PCM = 192
SOFT_GAIN = 2.0
SPEECH_LOUD = 'acompressor=threshold=0.1:ratio=6:attack=3:release=80:makeup=4,alimiter=limit=0.95'
# alien=True: thin, slightly raised, faintly metallic small-speaker voice.
ALIEN_FILTER = ('rubberband=pitch=1.12,highpass=f=300,lowpass=f=5000,'
                'aecho=0.8:0.6:12:0.2')


def speech_filter():
    """ffmpeg filters for speech: gain, compressor and limiter, then the volume.
    The fader after the compressor, or turning it down would only compress less."""
    return 'volume=%.2f,%s,volume=%.2f' % (SOFT_GAIN, SPEECH_LOUD, S.VOLUME)


class Talker(Speaker):
    """The SDK's speaker (clips, files) plus speech and internet audio."""

    # --- speech ------------------------------------------------------------
    def say(self, text, wait=True, voice=None, alien=False):
        """Speak exactly `text` in his voice (Kokoro on the Orin, RemoteSpeaker).
        `voice` picks a Piper model here instead (see voices()); alien=True is Rocky's sound.
        """
        if not wait:
            threading.Thread(target=self.say, args=(text,),
                             kwargs={'voice': voice, 'alien': alien}).start()
            return None
        sp = PiperSpeaker(voice=voice, alien=alien, flat=alien, host=self.host) if voice else \
            speaker(alien=alien, host=self.host)
        with sp:
            for s in sentences([text + ' ']):
                sp.say(s)
            sp.finish()
        return True

    # --- internet audio ----------------------------------------------------
    def play_url(self, url, wait=True):
        """Play the audio of a YouTube (or any yt-dlp-supported) URL."""
        if not os.path.exists(YTDLP):
            raise TalkError('yt-dlp not found at %s' % YTDLP)
        ytdlp = [YTDLP] + (['--proxy', PROXY] if PROXY else []) + [
                 '--js-runtimes', 'deno:' + DENO,
                 '--no-playlist', '-f', 'bestaudio', '-q', '--no-warnings',
                 '-o', '-', url]
        return self.stream('-', wait, 'play_url %s' % url, pre=ytdlp)

    def voices(self):
        """Names of the installed Piper voices."""
        return sorted(os.path.basename(f)[:-5]
                      for f in glob.glob(os.path.join(PIPER_DIR, '*.onnx')))


class PiperSpeaker:
    """Streaming Piper: say() queues ONE sentence and returns at once, finish()
    blocks until all of it has played. Piper stays running between calls; the
    speaker is opened at the first say() and released in finish().

        with PiperSpeaker(alien=True) as sp:
            sp.say('Hello friend.'); sp.say('You are well, question?')
            sp.finish()
    """

    def __init__(self, voice=None, alien=False, flat=True, host=HOST):
        """flat=True uses PIPER_ARGS (monotone); False is Piper's own
        expressive delivery."""
        model = os.path.join(PIPER_DIR, (voice or PIPER_VOICE) + '.onnx')
        if not os.access(PIPER, os.X_OK) or not os.path.exists(model):
            raise TalkError('Speaker needs piper at %s and voice %s' % (PIPER, model))
        self.rate = json.load(open(model + '.json'))['audio']['sample_rate']
        self.af = ALIEN_FILTER if alien else None
        self._ssh = ssh_command(host)
        self._dir = tempfile.mkdtemp(prefix='speaker-')
        self._piper = subprocess.Popen(
            [PIPER, '-q', '--model', model] + (PIPER_ARGS if flat else [])
            + ['--output_dir', self._dir],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, universal_newlines=True, bufsize=1)
        self._cv = threading.Condition()
        self._pending = 0
        self._player = None
        self._error = None
        threading.Thread(target=self._pump, daemon=True).start()

    def say(self, sentence):
        text = ' '.join(sentence.split())      # piper reads one line per utterance
        if not text:
            return
        with self._cv:
            if self._error:
                raise TalkError(self._error)
            if self._player is None:
                self._player = self._open_player()
            self._pending += 1
        self._send(text)

    def finish(self, timeout=120):
        """Wait until everything said so far has played, then free the speaker."""
        with self._cv:
            if not self._cv.wait_for(lambda: self._pending == 0 or self._error, timeout):
                self._error = 'speaker timed out'
            player, self._player = self._player, None
            err = self._error
        if player:
            ff, ap = player
            ff.stdin.close()
            ff.wait()
            if ap.wait() != 0 and not err:
                err = 'aplay on the robot failed (exit %d)' % ap.returncode
        if err:
            raise TalkError(err)

    def close(self):
        try:
            self._piper.stdin.close()
        except OSError:
            pass
        self._piper.wait()
        shutil.rmtree(self._dir, ignore_errors=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _open_player(self):
        gain = speech_filter()
        # One stream serves a whole reply, so the amp lead-in costs LEAD_IN_MS
        # once per reply, not per line.
        ff = (['ffmpeg', '-loglevel', 'error', '-f', 's16le', '-ar', str(self.rate),
               '-ac', '1', '-i', '-'] + to_speaker(self.af, gain))
        p_ff = subprocess.Popen(ff, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        p_ap = subprocess.Popen(
            self._ssh + ['amixer', '-q', '-c', '0', 'sset', 'PCM', str(HW_PCM), ';']
            + aplay_command(), stdin=p_ff.stdout,
            # the gap between sentences makes aplay print "underrun!!!", which
            # is expected here; real failures show up as its exit code.
            stderr=subprocess.DEVNULL)
        p_ff.stdout.close()
        return p_ff, p_ap

    def _send(self, text):
        self._piper.stdin.write(text + '\n')
        self._piper.stdin.flush()

    def _made(self):
        """Each sentence's samples, in the order said, as each is ready."""
        for line in self._piper.stdout:
            path = line.strip()
            with wave.open(path) as w:
                frames = w.readframes(w.getnframes())
            os.remove(path)
            yield frames

    def _pump(self):
        """Feed each finished sentence into the open player, in order."""
        try:
            for frames in self._made():
                with self._cv:
                    player = self._player
                if player:
                    player[0].stdin.write(frames)
                    player[0].stdin.flush()
                with self._cv:
                    self._pending -= 1
                    self._cv.notify_all()
        except (OSError, EOFError, wave.Error) as e:
            with self._cv:
                self._error = 'speaker: %s' % e
                self._cv.notify_all()
            return
        # The source only ends when Piper or the Orin's voice has gone, also between two Says:
        # nothing reads any more, so the next Say must fail at once, not wait out finish().
        with self._cv:
            self._error = 'the voice stopped unexpectedly'
            self._cv.notify_all()


class RemoteSpeaker(PiperSpeaker):
    """PiperSpeaker with each sentence made on the Orin (orin/bin/tts_server.py: Kokoro on its
    GPU). The same say()/finish()/close(); raises OSError at once if the Orin's voice is down."""

    def __init__(self, alien=False, host=HOST, server=TTS_SERVER):
        self._sock = socket.create_connection(server, timeout=TTS_CONNECT_S)
        self._sock.settimeout(None)         # idle between Says; finish() has its own timeout
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.rate = TTS_RATE
        self.af = ALIEN_FILTER if alien else None
        self._ssh = ssh_command(host)
        self._cv = threading.Condition()
        self._pending = 0
        self._player = None
        self._error = None
        threading.Thread(target=self._pump, daemon=True).start()

    def _send(self, text):
        self._sock.sendall((text + '\n').encode())

    def _made(self):
        f = self._sock.makefile('rb')
        while True:
            head = f.read(4)
            if len(head) < 4:
                return                      # closed: _pump marks the speaker broken
            n, = struct.unpack('<I', head)
            data = f.read(n)
            if len(data) < n:
                raise EOFError('the Orin stopped mid-sentence')
            yield data

    def close(self):
        self._sock.close()


def speaker(alien=False, host=HOST):
    """His voice: Kokoro on the Orin. No fallback (the user's choice, 2026-11-09)."""
    try:
        return RemoteSpeaker(alien=alien, host=host)
    except OSError as e:
        raise TalkError("the Orin's voice (kokoro-tts, %s:%d) is down: %s" % (TTS_SERVER + (e,)))


_END = re.compile(r'[.!?]["\')]*\s')


def sentences(chunks):
    """Regroup a stream of text chunks into whole sentences."""
    buf = ''
    for c in chunks:
        buf += c
        while True:
            m = _END.search(buf)
            if not m:
                break
            yield buf[:m.end()].strip()
            buf = buf[m.end():]
    if buf.strip():
        yield buf.strip()


class Voice:
    """Mixed into StackRobot: bot.say()."""

    @cached_property
    def voice(self):
        return Talker()

    def say(self, text, wait=True, voice=None, alien=False):
        """Speak `text` out of the robot - see Talker.say."""
        return self.voice.say(text, wait=wait, voice=voice, alien=alien)



def _cli(argv):
    voice, alien, args, it = None, False, [], iter(argv)
    for a in it:
        if a in ('-h', '--help', 'help'):
            print(__doc__); return 0
        elif a == '--alien':
            alien = True
        elif a == '--voice':
            voice = next(it)
        else:
            args.append(a)
    if not args:
        print(__doc__); return 2
    cmd, rest = args[0], ' '.join(args[1:])
    try:
        v = Talker()
        if cmd == 'say':
            v.say(rest, voice=voice, alien=alien)
        elif cmd == 'play':
            v.play(rest)
        elif cmd == 'file':
            v.play_file(rest)
        elif cmd == 'url':
            v.play_url(rest)
        elif cmd == 'clips':
            print('\n'.join(v.clips()))
        elif cmd == 'voices':
            print('\n'.join(v.voices()))
        else:
            print('unknown command %r - see --help' % cmd, file=sys.stderr)
            return 2
    except TalkError as e:
        print('error: %s' % e, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(_cli(sys.argv[1:]))
