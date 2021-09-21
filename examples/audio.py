import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

import argparse
import rospy

from imperio.audio import AudioOutputStreamer

parser = argparse.ArgumentParser()

parser.add_argument(
    "-r",
    "--sample_rate",
    type=int,
    help="Specify sample rate of the audio from the ROS topic",
    default=16000,
)

parser.add_argument(
    "-c",
    "--channels",
    type=int,
    default=1,
    help="Specify the number of channels in the audio from the ROS topic",
)

parser.add_argument(
    "-s",
    "--chunk_size",
    type=int,
    help="Specify chunk size for audio buffer",
    default=8000,
)

parser.add_argument(
    "-t",
    "--ros_topic",
    default="/hr/sensors/audio/speech_recognizer",
    help="Specify ROS topic of the audio",
)

args = parser.parse_args()

rospy.init_node("audio_output")
AudioOutputStreamer(
    sample_rate=args.sample_rate,
    channels=args.channels,
    chunk_size=args.chunk_size,
    topic=args.ros_topic,
)
rospy.spin()
