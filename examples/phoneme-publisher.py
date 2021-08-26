import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

import argparse
import random
import numpy as np
import librosa

import rospy
from std_msgs.msg import UInt8MultiArray

from imperio.sonorus.audio.utils import audio_int2float
from imperio.robot.hr.lip_control import PhonemesPublisher
from imperio.robot.hr.lip_control.utils import drop_random, shift_time


class PhonemesPublisherOnVoice(object):
    def __init__(self, shift=0, duration=0.02, seed=None, drop_th=0.8):

        self.shift = shift
        self.duration = duration

        self.seed = seed
        self.drop_th = drop_th
        self.random = random.Random(seed)

        rospy.Subscriber("/hr/control/audio/stream", UInt8MultiArray, self.callback)

        self.pub = PhonemesPublisher(
            default_viseme_params=dict(magnitude=0.99, rampin=0.01, rampout=0.01,),
        )

    def callback(self, msg):

        phonemes = self.pub.random(self.duration, chunk=self.duration)
        phonemes = drop_random(phonemes, seed=self.seed, drop_th=self.drop_th)
        phonemes = shift_time(phonemes, shift=self.shift)

        # if phonemes:
        self.pub.publish(phonemes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-s",
        "--shift",
        type=float,
        default=0,
        help="Time shift in seconds after the audio message is received. Default is 0 secs.",
    )

    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=0.02,
        help="Duration in seconds for which phonemes will be played for each audio message received. Default is 0.02 secs.",
    )

    args = parser.parse_args()

    rospy.init_node("phoneme_publisher")
    PhonemesPublisherOnVoice(shift=args.shift, duration=args.duration)
    rospy.spin()
