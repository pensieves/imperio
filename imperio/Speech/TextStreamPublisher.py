import re
import string

import rospy
import genpy
from hr_msgs.msg import TTS, SetExpression
from hr_msgs.srv import SetActuatorsControl, SetActuatorsControlRequest

from actuator_names import HEAD_ACTUATOR_NAMES

LANGUAGE = "en-US" # a BCP-47 language tag
# LANGUAGE = "en-IN"

CONTEXT_PHRASES = ["Asha", "Hey Asha", "Hey, Asha", "Hi Asha", "Hi, Asha", 
                    "Okay Asha", "Okay, Asha"]
ACTION_PHRASES = ["emergency stop"]

expression_args = {"magnitude": 1, "duration":genpy.Duration(2)}
expression_names = {"happy", "sad", "worry", "amused", "angry", "fear", 
                    "surprised", "confused", "engaged", "comprehending"}

ACTION_MAP = {name: SetExpression(name=name, **expression_args) for name in expression_names}
ACTION_PHRASES += list(expression_names)

TOPIC_MAP = {"SPEECH_TOPIC": "/hr/control/speech/say",
            "SET_EXPRESSION": "/hr/animation/set_expression",
            "CONTROL_ACTUATOR": "/hr/actuators/set_control"}

def set_actuator_control(actuator_control_service, actuator_names, control_type="CONTROL_ANIMATION"):
    """
    Set the actuators to desired control type.

    CONTROL_DISABLE: disable actuator
    CONTROL_MANUAL: control actuator by hand (code)
    CONTROL_ANIMATION: control actuator by animation
    """
    control_request = SetActuatorsControlRequest()
    control_request.control = getattr(SetActuatorsControlRequest, control_type)
    control_request.actuators = actuator_names
    actuator_control_service(control_request)

class TextStreamPublisher(object):
    def __init__(self, lang=LANGUAGE, context_phrases=CONTEXT_PHRASES, 
                action_phrases=ACTION_PHRASES, action_map=ACTION_MAP):
        self._lang = lang
        self.context_phrases = context_phrases
        self.action_phrases = action_phrases
        self._action_map = action_map

        self._context_regex = '|'.join(self.context_phrases)
        self._context_regex = "^\s+?({})[{}]?".format(self._context_regex, string.punctuation)

        self._action_regex = "({})".format("|".join(self.action_phrases))
        
        self._action_text = ""
        self._context_phrase = ""
        self._processing_context = False

        speech_topic = rospy.get_param('speech_topic', TOPIC_MAP["SPEECH_TOPIC"])
        self._speech_pub = rospy.Publisher(speech_topic, TTS, queue_size=10)

        self._set_control = rospy.ServiceProxy(TOPIC_MAP["CONTROL_ACTUATOR"], SetActuatorsControl)
        self._actuator_names = HEAD_ACTUATOR_NAMES

        self._express_pub = rospy.Publisher(TOPIC_MAP["SET_EXPRESSION"], 
                                            SetExpression, queue_size=10)

    def reset(self):
        self._action_text = ""
        self._context_phrase = ""
        self._processing_context = False

    def publish(self, stream, reset=False):
        
        set_actuator_control(self._set_control, self._actuator_names)

        if stream:
            context_phrase = re.search(self._context_regex, stream[0])

            if context_phrase or self._processing_context:
                self._handle_context_phrase(context_phrase, stream, reset=reset)

            else:
                for text in stream:
                    print('Recognised "{}" in "{}"\n'.format(text, self._lang))
                    self._speech_pub.publish(text=text, lang=self._lang)
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
            print(text)

            action_name = re.search(self._action_regex, self._action_text.lower())
            action_name = action_name.group(0) if action_name else ""
            action_msg = self._action_map.get(action_name)

            if action_msg:
                print("Performing action = {}".format(action_name))
                self._express_pub.publish(action_msg)
            else:
                print("No action can be performed. Action phrase consists of unknown action.")

            self.reset()
