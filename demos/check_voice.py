#!/usr/bin/env python3
"""check_voice.py - his voice on the SDK's speaker, checked with no robot and no Orin.

    python3 ~/robot/demos/check_voice.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import lite3.speaker as S  # noqa: E402
from robot import voice as V  # noqa: E402
from robot.stack import StackRobot  # noqa: E402


def main():
    # one volume for everything: the panel's slider sets the SDK's, and speech reads it when it starts
    S.VOLUME = 0.3
    gain = V.speech_filter()
    assert gain.endswith('volume=0.30'), gain          # the fader after the compressor, or turning down only compresses less
    S.VOLUME = 1.0
    assert V.speech_filter().endswith('volume=1.00')
    # clips and files are the SDK's; speech is the stack's
    assert issubclass(V.Talker, S.Speaker) and V.Talker.play_file is S.Speaker.play_file
    assert callable(V.Talker.say) and V.PiperSpeaker is not S.Speaker
    assert issubclass(V.TalkError, S.SpeakerError)     # an `except Lite3Error` catches a voice that is down
    assert list(V.sentences(['Hello there. How', ' are you? fine'])) == ['Hello there.', 'How are you?', 'fine']
    # the robot's say() goes to its Talker
    said = []
    bot = StackRobot.__new__(StackRobot)
    bot.__dict__['voice'] = type('T', (), {'say': lambda self, text, **kw: said.append((text, kw))})()
    bot.say('watch your step', wait=False)
    assert said == [('watch your step', {'wait': False, 'voice': None, 'alien': False})], said
    assert isinstance(StackRobot.__new__(StackRobot).voice, V.Talker)
    print('check_voice ok')


if __name__ == '__main__':
    main()
