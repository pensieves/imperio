import re
import string

import rospy
import genpy
from hr_msgs.msg import TTS, SetExpression, SetAnimation
from hr_msgs.srv import SetActuatorsControl, SetActuatorsControlRequest

from actuator_names import HEAD_ACTUATOR_NAMES

# LANGUAGE = "en-US" # a BCP-47 language tag
LANGUAGE = "en-IN"

CONTEXT_PHRASES = ["Asha", "Hey Asha", "Hey, Asha", "Hi Asha", "Hi, Asha", 
                    "Okay Asha", "Okay, Asha"]

ACTION_PHRASES = ["emergency stop"]
ACTION_MAP = dict() # empty dict for now. Action sequences to be coded.

expression_args = dict(magnitude=1, duration=genpy.Duration(2))
EXPRESSION_PHRASES = ["happy", "sad", "worry", "amused", "angry", "fear", 
                    "surprised", "confused", "engaged", "comprehending"]
expression_name_map = dict()
EXPRESSION_MAP = {name: SetExpression(name=expression_name_map.get(name, name), 
                                        **expression_args) for name in EXPRESSION_PHRASES}

animation_args = dict(magnitude=1, speed=1)
# animation_name_map = dict(nod="head_DownShort", blink="eyes_Blink")
animation_name_map = dict(laugh="happy_LaughterBack", wink="wink_1", 
                            nod="happy_Smiley01", blink="eyes_Blink")
ANIMATION_PHRASES = list(animation_name_map.keys())
ANIMATION_MAP = {animation_name: SetAnimation(name=animation, **animation_args)
                    for animation_name, animation in animation_name_map.items()}

TOPIC_MAP = {"SPEECH_TOPIC": "/hr/control/speech/say",
            "SET_EXPRESSION": "/hr/animation/set_expression",
            "SET_ANIMATION": "/hr/animation/set_animation",
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

class TextBatchPublisher(object):
    def __init__(self, lang=LANGUAGE, context_phrases=CONTEXT_PHRASES, 
                action_phrases=ACTION_PHRASES, action_map=ACTION_MAP, 
                expression_phrases=EXPRESSION_PHRASES, expression_map=EXPRESSION_MAP,
                animation_phrases=ANIMATION_PHRASES, animation_map=ANIMATION_MAP):

        self._lang = lang
        self.context_phrases = context_phrases

        self.action_phrases = action_phrases
        self._action_map = action_map

        self.expression_phrases = expression_phrases
        self._expression_map = expression_map

        self.animation_phrases = animation_phrases
        self._animation_map = animation_map

        self._context_regex = '|'.join(self.context_phrases)
        self._context_regex = "^(\s+)?({})[{}]?".format(self._context_regex, string.punctuation)

        self._action_regex = "({})".format("|".join(self.action_phrases + 
                                                    self.expression_phrases + 
                                                    self.animation_phrases))

        self._action_text = ""
        self._context_phrase = ""
        self._processing_context = False

        speech_topic = rospy.get_param('speech_topic', TOPIC_MAP["SPEECH_TOPIC"])
        self._speech_pub = rospy.Publisher(speech_topic, TTS, queue_size=10)

        self._set_control = rospy.ServiceProxy(TOPIC_MAP["CONTROL_ACTUATOR"], SetActuatorsControl)
        self._actuator_names = HEAD_ACTUATOR_NAMES.tolist()

        self._express_pub = rospy.Publisher(TOPIC_MAP["SET_EXPRESSION"], 
                                            SetExpression, queue_size=10)

        self._animation_pub = rospy.Publisher(TOPIC_MAP["SET_ANIMATION"], 
                                                SetAnimation, queue_size=10)

    def reset(self):
        self._action_text = ""
        self._context_phrase = ""
        self._processing_context = False

    def publish(self, batch, reset=False):
        set_actuator_control(self._set_control, self._actuator_names)

        if batch:
            context_phrase = re.search(self._context_regex, batch[0])

            if context_phrase or self._processing_context:
                self._handle_context_phrase(context_phrase, batch, reset=reset)

            else:
                for text in batch:
                    print('Recognised "{}" in "{}"\n'.format(text, self._lang))
                    self._speech_pub.publish(text=text, lang="en-US")#self._lang)
                                    # , request_id=None, agent_id=None)

    def _handle_context_phrase(self, context_phrase, batch, reset=False):
        context_phrase = context_phrase.group(0) if context_phrase else ""

        if not self._processing_context:
            batch[0] = batch[0][len(context_phrase):].strip()
            self._processing_context = True
            self._context_phrase = context_phrase

        self._action_text += (" " + " ".join(batch))

        if reset:
            text = "Context phrase, {}, is recognized. Intended action for, {}, will be executed."\
                    .format(self._context_phrase, self._action_text)
            print(text)

            action_names = re.findall(self._action_regex, self._action_text.lower())

            published_at_least_once = False
            type_msg_pub_dict = {"action": (self._action_map, None), "expression": (self._expression_map, self._express_pub), 
                                "animation": (self._animation_map, self._animation_pub)}
            
            if action_names:
                for name in action_names:
                    for action_type, (msg_map, pub) in type_msg_pub_dict.items():

                        msg = msg_map.get(name)

                        if msg:
                            print("Performing {} = {}".format(action_type, name))
                            pub.publish(msg)
                            published_at_least_once = True

            if not published_at_least_once:
                print("No action performed. Action phrase consists of unknown action.")

            self.reset()
