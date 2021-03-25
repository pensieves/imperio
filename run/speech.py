import argparse

import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

from imperio.speech import STT
from imperio.speech import TextBatcher
from imperio.speech import TextBatchPublisher

parser = argparse.ArgumentParser()

parser.add_argument(
    "-r",
    "--ros_init",
    help="Specify if ROS node is to be initialized",
    action="store_true",
)

parser.add_argument(
    "-l",
    "--lang",
    type=str,
    help="Specify the language as a BCP-47 language tag",
    default="en-US",
)

parser.add_argument(
    "-c",
    "--stream_count",
    type=int,
    help="Streams count to collect before starting transcription in STT",
    default=50,
)

parser.add_argument(
    "-o",
    "--stream_overlap",
    type=int,
    help="Streams common between consecutive stream processing batches for text correction",
    default=5,
)

parser.add_argument(
    "-w",
    "--word_overlap",
    type=int,
    help="Words common between consecutive stream processing batches for text correction.",
    default=5,
)

parser.add_argument(
    "-g",
    "--gpu_idx",
    type=int,
    help="index of GPU to be used for accelerated computation",
)

args = parser.parse_args()

text_batch_processor = None

if args.ros_init:
    import rospy

    rospy.init_node("STT")
    text_batch_processor = TextBatchPublisher(lang=args.lang)


STT(
    text_batch_processor=text_batch_processor,
    gpu_idx=args.gpu_idx,
    stream_overlap=args.stream_overlap,
    word_overlap=args.word_overlap,
).streaming_transcribe()

# rospy.spin()
