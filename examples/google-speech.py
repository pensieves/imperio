import argparse

import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

from imperio.speech import GoogleSTT
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
    default="en-IN",
)

parser.add_argument(
    "-b",
    "--text_batcher",
    help="Use batched text processing when interim transcriptions are available",
    action="store_true",
)

args = parser.parse_args()

text_batcher = None
text_batch_processor = None
audio_streamer = None

if args.ros_init:
    import rospy

    rospy.init_node("GoogleSTT")
    text_batch_processor = TextBatchPublisher(lang=args.lang)

if args.text_batcher:
    text_batcher = TextBatcher()


while True:
    try:
        GoogleSTT(
            lang=args.lang,
            text_batcher=text_batcher,
            text_batch_processor=text_batch_processor,
        ).streaming_transcribe()

    except Exception as exception:
        print(exception)

# rospy.spin()
