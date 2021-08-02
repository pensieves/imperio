import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

import argparse
import numpy as np
import parselmouth
import pyaudio

import rospy
from std_msgs.msg import UInt8MultiArray

from imperio.sonorus.audio import VADAudioInputStreamer
from imperio.sonorus.audio.utils import audio_float2int
from imperio.sonorus.audio.praat import reduce_noise, change_gender, change_pitch
from imperio.sonorus.speech.kaldi import PhonemeSegmenter

from imperio.robot.hr.lip_control import PhonemesPublisher

# from imperio.sonorus.utilities.multi_proc_thread import MultiProducerSingleConsumer


parser = argparse.ArgumentParser()

parser.add_argument(
    "-v",
    "--voice_conv_fn",
    choices=["change_gender", "change_pitch"],
    help="Specify function to be used for voice conversion",
)

parser.add_argument(
    "-t",
    "--topic",
    help="ROS topic to which the speech should be published",
    default="/hr/control/audio/stream",
)

parser.add_argument(
    "-m",
    "--multiplier",
    type=float,
    help="Factor/ratio multiplier to be used for formant change ratio in case "
    "of change_gender function or the factor in case of change_pitch "
    " fucntion. Multiplier > 1 for female target voice while < 1 for male"
    " target voice.",
    default=1.5,  # for female voice target using change_pitch function
)

args = parser.parse_args()

phoneme_segmenter = PhonemeSegmenter.from_url() # with default params

audio_streamer = VADAudioInputStreamer(pa_format=pyaudio.paInt16,)

phonemes_pub = PhonemesPublisher(
    default_viseme_params=dict(magnitude=0.99, rampin=0.01, rampout=0.01,)
)

audio_pub = rospy.Publisher(args.topic, UInt8MultiArray, queue_size=2500)


def producer_func(msg, segmenter=phoneme_segmenter.segment):
    data, dtype, sample_rate = msg
    audio = np.frombuffer(data, dtype=dtype)
    phonemes = (
        segmenter(audio, sample_rate=sample_rate).get("utt1", {}).get("phonemes", [])
    )
    return data, phonemes


def consumer_func(msg, chunk=320, audio_pub=audio_pub, phonemes_pub=phonemes_pub):
    audio_msg, phonemes = msg
    audio_stream = [
        UInt8MultiArray(data=audio_msg[i : i + chunk])
        for i in range(0, len(audio_msg), chunk)
    ]
    for audio_msg in audio_stream:
        audio_pub.publish(audio_msg)
    phonemes_pub.publish(phonemes)


# producer_consumer = MultiProducerSingleConsumer(
#     producer_target_func=producer_func,
#     consumer_target_func=consumer_func,
# )


def producer_consumer(msg):
    consumer_func(producer_func(msg))


rospy.init_node("streaming_speech")
# r = rospy.Rate(10) # 10hz


while not rospy.is_shutdown():
    with audio_streamer as streamer:
        for stream in streamer.stream():
            if stream is not None:

                dtype = streamer.FMT2TYPE[streamer.pa_format]

                if args.voice_conv_fn:

                    sound = parselmouth.Sound(
                        values=np.frombuffer(stream, dtype=dtype),
                        sampling_frequency=streamer.processing_rate,
                    )

                    if args.voice_conv_fn == "change_pitch":
                        changed_sound = change_pitch(sound, factor=args.multiplier)
                    else:
                        changed_sound = change_gender(
                            sound, formant_shift_ratio=args.multiplier,
                        )

                    stream = audio_float2int(changed_sound.values).tobytes()
                    dtype = np.int16

                producer_consumer((stream, dtype, streamer.processing_rate))
                # r.sleep()
