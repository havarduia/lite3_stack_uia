"""voice - say() for StackRobot. Until talk.py moves here, it forwards to talk.Voice."""
from functools import cached_property


class Voice:
    @cached_property
    def voice(self):
        from .talk import Voice as Talk
        return Talk()

    def say(self, text, wait=True, voice=None, alien=False):
        """Speak `text` out of the robot - see talk.Voice.say."""
        return self.voice.say(text, wait=wait, voice=voice, alien=alien)
