import re
import string

import rospy
from hr_msgs.msg import TTS

LANGUAGE = "en-US" # a BCP-47 language tag
# LANGUAGE = "en-IN"

CONTEXT_PHRASES = ["Asha", "Hey Asha", "Hey, Asha", "Hi Asha", "Hi, Asha", 
                    "Okay Asha", "Okay, Asha"]
ACTION_PHRASES = ["emergency stop"]

SPEECH_TOPIC = '/hr/control/speech/say'

class TextStreamPublisher(object):
    def __init__(self, lang=LANGUAGE, context_phrases=CONTEXT_PHRASES, action_phrases=ACTION_PHRASES):
        self._lang = lang
        self.context_phrases = context_phrases
        self.action_phrases = action_phrases

        self._context_regex = '|'.join(self.context_phrases)
        self._context_regex = "^({})[{}]?".format(self._context_regex, string.punctuation)
        
        self._action_text = ""
        self._context_phrase = ""
        self._processing_context = False

        speech_topic = rospy.get_param('speech_topic', SPEECH_TOPIC)
        self.pub = rospy.Publisher(speech_topic, TTS, queue_size=10)

    def reset(self):
        self._action_text = ""
        self._context_phrase = ""
        self._processing_context = False

    def publish(self, stream, reset=False):
        if stream:
            context_phrase = re.search(self._context_regex, stream[0])

            if context_phrase or self._processing_context:
                self._handle_context_phrase(context_phrase, stream, reset=reset)

            else:
                for text in stream:
                    print('Recognised "{}" in "{}"\n'.format(text, self._lang))
                    self.pub.publish(text=text, lang=self._lang)
                                    # , request_id=None, agent_id=None)

    def _handle_context_phrase(self, context_phrase, stream, reset=False):
        context_phrase = context_phrase.group(0) if context_phrase else ""

        if not self._processing_context:
            stream[0] = stream[0][len(context_phrase):].strip()
            self._processing_context = True
            self._context_phrase = context_phrase

        self._action_text += (" " + " ".join(stream))

        if reset:
            text = "Context phrase, {}, is recognized. Intended action for, {}, will be executed."\
                    .format(self._context_phrase, self._action_text)
            self.reset()
            print(text)