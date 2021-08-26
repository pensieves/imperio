import time
import rospy
import genpy
from hr_msgs.msg import Viseme, Visemes

from ..actuator.utils import set_actuator_control
from ..actuator.names import HEAD_ACTUATOR_NAMES

VISEME_TOPIC = "/hr/animation/queue_viseme"
VISEMES_TOPIC = "/hr/animation/queue_visemes"


def set_control(
    actuator_names=HEAD_ACTUATOR_NAMES.tolist(), control_type="CONTROL_ANIMATION"
):
    set_actuator_control(actuator_names, control_type=control_type)


def reset_control(
    actuator_names=HEAD_ACTUATOR_NAMES.tolist(), control_type="CONTROL_MANUAL"
):
    set_actuator_control(actuator_names, control_type=control_type)


class VisemesPublisher(object):
    def __init__(
        self,
        topic=VISEMES_TOPIC,
        pub_queue_size=10,
        set_control=set_control,
        reset_control=reset_control,
    ):

        self.viseme_pub = rospy.Publisher(topic, Visemes, queue_size=pub_queue_size)
        self.set_control = set_control
        self.reset_control = reset_control

    def publish(self, visemes):
        self.visemes_publish(visemes)

    def visemes_publish(self, visemes):

        if visemes:
            for v in visemes:
                if not isinstance(v["duration"], genpy.Duration):

                    if isinstance(v["duration"], dict):
                        v["duration"] = genpy.Duration(**v["duration"])
                    else:
                        v["duration"] = genpy.Duration(v["duration"])

            sleep_dur = visemes[-1]["duration"]
            sleep_dur = sleep_dur.secs + sleep_dur.nsecs * 1e-9
            sleep_dur += visemes[-1]["start"]

            visemes = Visemes([Viseme(**kwargs) for kwargs in visemes])

            self.set_control()
            self.viseme_pub.publish(visemes)
            time.sleep(sleep_dur)
            self.reset_control()
