import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

import argparse
import numpy as np
import parselmouth
import pyaudio
import random
import librosa
from functools import partial

import rospy
from std_msgs.msg import UInt8MultiArray

from imperio.sonorus.audio import VADAudioInputStreamer
from imperio.sonorus.audio.utils import audio_float2int, audio_int2float
from imperio.sonorus.audio.praat import reduce_noise, change_gender, change_pitch
from imperio.sonorus.speech.kaldi import PhonemeSegmenter, VoskPhonemeSegmenter

from imperio.robot.hr.lip_control import PhonemesPublisher
from imperio.robot.hr.lip_control.utils import drop_random

from imperio.sonorus.utilities.multi_proc_thread import SingleProcessor

# from imperio.sonorus.utilities.multi_proc_thread import MultiProducerSingleConsumer

from datetime import datetime

CHUNK = 320

PHONEME_GROUPS = dict(
    large=(["AA", "AI", "AU", "AE", "AH", "AW", "AX", "AY", "EY"] + # A-I viseme
        ["IY", "EH"]), # E viseme
    mid=(["CH", "D", "DH", "G", "H", "HH", "JH", "K", "N", "NG", "S", "SH", "T", "TH", "Z", "ZH", "DX", "ER"] + # C-D-G-K-N-S-TH viseme
        ["IH", "L", "Y", "R"] + # L viseme
        ["AO", "OW", "OY"] + # O viseme
        ["W"] + # Q-W viseme
        ["UH", "UW"]), # U viseme
    small=(["F", "V"] + # F-V viseme
        ["B", "M", "P"]), # M viseme
)


def random_segmenter(audio, sample_rate, base_segmenter=None, chunk=None):
    phonemes = {"utt1": {"phonemes": []}}

    if base_segmenter:
        audio_dur = librosa.get_duration(audio_int2float(audio), sr=sample_rate)
        phonemes["utt1"]["phonemes"] = base_segmenter(audio_dur)

    elif chunk:
        phonemes["utt1"]["phonemes"] = [None for i in range(0, len(audio), chunk)]

    return phonemes


def publish_audio_stream(audio_stream, publisher):
    for audio in audio_stream:
        publisher.publish(audio)


def publish_proba_dropped_random_phonemes(
    phoneme_stream, publisher, duration=0.04, seed=None, drop_th=0.85,
):

    random_gen = random.Random(seed)
    for ph in phoneme_stream:
        phonemes = publisher.random(duration, chunk=duration)

        # replace with large movement phonemes
        phonemes[0][publisher.phoneme_key] = random_gen.choice(PHONEME_GROUPS["large"])
        
        phonemes = drop_random(phonemes, seed=seed, drop_th=drop_th)
        publisher.publish(phonemes)


def init_ros_node(name, anonymous=True, disable_signals=False):
    rospy.init_node(name, anonymous=anonymous, disable_signals=disable_signals)


def producer_func(msg, segmenter):
    data, dtype, sample_rate = msg
    audio = np.frombuffer(data, dtype=dtype)
    phonemes = (
        segmenter(audio, sample_rate=sample_rate).get("utt1", {}).get("phonemes", [])
    )
    return data, phonemes


def consumer_func(msg, audio_processor, phonemes_processor, chunk=CHUNK):
    # def consumer_func(msg, audio_pub, phonemes_pub, chunk=CHUNK):
    audio_msg, phonemes = msg
    chunk = chunk if chunk else len(audio_msg)
    audio_stream = [
        UInt8MultiArray(data=audio_msg[i : i + chunk])
        for i in range(0, len(audio_msg), chunk)
    ]
    # for audio_msg in audio_stream:
    #     audio_pub.publish(audio_msg)
    # phonemes_pub.publish(phonemes)

    audio_processor.in_queue.put(audio_stream)
    phonemes_processor.in_queue.put(phonemes)

    # print(f"phonemes = {phonemes}")
    # print(
    #     f"{datetime.now()}: Audio and Phonemes published to queue."
    # )


# producer_consumer = MultiProducerSingleConsumer(
#     producer_target_func=producer_func,
#     consumer_target_func=consumer_func,
# )


def producer_consumer(msg, producer_kwargs, consumer_kwargs):
    consumer_func(producer_func(msg, **producer_kwargs), **consumer_kwargs)


def change_audio(
    stream, dtype=np.int16, conv_fn="change_pitch", sample_rate=16000, **kwargs
):

    if conv_fn:

        sound = parselmouth.Sound(
            values=np.frombuffer(stream, dtype=dtype), sampling_frequency=sample_rate,
        )

        if conv_fn == "change_pitch":
            changed_sound = change_pitch(sound, **kwargs)
        else:
            changed_sound = change_gender(sound, **kwargs)

        # stream = audio_float2int(changed_sound.values).tobytes()
        # dtype = np.int16

        stream = changed_sound.values

        if np.issubdtype(dtype, np.integer):
            stream = audio_float2int(stream)
            dtype = np.int16

        stream = stream.tobytes()

    return stream, dtype


def stream_speech(
    audio_topic="/hr/control/audio/stream",
    visemes_topic="/hr/animation/queue_visemes",
    pub_queue_size=2500,
    phonemes_segmenter=None,
    phonemes_duration=0.04,
    phonemes_drop_th=0.85,
    vad_accumulate_count=20,
    voice_conv_fn=None,
    min_pitch=75,
    max_pitch=600,
    formant_shift_ratio=1.2,
    new_pitch_median=0,
    pitch_range_factor=1,
    duration_factor=1,
    pitch_factor=1.5,
    time_step=0.01,
    disable_ros_signals=False,
):

    voice_conv_kwargs = dict()
    if voice_conv_fn == "change_pitch":
        voice_conv_kwargs = dict(
            pitch_factor=pitch_factor,
            time_step=time_step,
            min_pitch=min_pitch,
            max_pitch=max_pitch,
        )

    elif voice_conv_fn == "change_gender":
        voice_conv_kwargs = dict(
            min_pitch=min_pitch,
            max_pitch=max_pitch,
            formant_shift_ratio=formant_shift_ratio,
            new_pitch_median=new_pitch_median,
            pitch_range_factor=pitch_range_factor,
            duration_factor=duration_factor,
        )

    phonemes_pub = PhonemesPublisher(
        default_viseme_params=dict(magnitude=0.99, rampin=0.01, rampout=0.01,),
        topic=visemes_topic,
        pub_queue_size=pub_queue_size,
    )

    audio_pub = rospy.Publisher(audio_topic, UInt8MultiArray, queue_size=pub_queue_size)

    audio_processor = SingleProcessor(
        target_func=partial(publish_audio_stream, publisher=audio_pub),
        starter_func=partial(init_ros_node, name="Speech"),
    )

    audio_streamer_kwargs = dict(pa_format=pyaudio.paInt16)

    if voice_conv_fn is None:

        audio_streamer_kwargs["accumulate"] = False

        phonemes_processor = SingleProcessor(
            target_func=partial(
                publish_proba_dropped_random_phonemes, 
                publisher=phonemes_pub,
                duration=phonemes_duration,
                drop_th=phonemes_drop_th,
            ),
            starter_func=partial(init_ros_node, name="Phonemes"),
        )

        segmenter = partial(random_segmenter, chunk=CHUNK)

    else:

        if phonemes_segmenter == "random":
            segmenter = partial(random_segmenter, base_segmenter=phonemes_pub.random)

        elif phonemes_segmenter == "kaldi":
            phonemes_segmenter = PhonemeSegmenter.from_url()  # with default params
            segmenter = phonemes_segmenter.segment

        elif phonemes_segmenter == "vosk":
            phonemes_segmenter = VoskPhonemeSegmenter.from_url()  # with default params
            segmenter = phonemes_segmenter.segment

        else:
            segmenter = random_segmenter

        phonemes_target_func = phonemes_pub.publish

        # -1 represents infinity
        if vad_accumulate_count != -1:

            audio_streamer_kwargs["accumulate_count"] = vad_accumulate_count

            phonemes_target_func=partial(
                publish_proba_dropped_random_phonemes, 
                publisher=phonemes_pub,
                duration=phonemes_duration,
                drop_th=phonemes_drop_th,
            )

            segmenter = partial(random_segmenter, chunk=CHUNK)

        phonemes_processor = SingleProcessor(
            target_func=phonemes_target_func,
            starter_func=partial(init_ros_node, name="Phonemes"),
        )        

    audio_streamer = VADAudioInputStreamer(**audio_streamer_kwargs)

    audio_phonemes_producer_kwargs = dict(segmenter=segmenter)
    audio_phonemes_consumer_kwargs = dict(
        audio_processor=audio_processor, phonemes_processor=phonemes_processor
    )

    init_ros_node(name="streaming_speech", disable_signals=disable_ros_signals)
    # r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        with audio_streamer as streamer:

            for stream in streamer.stream():

                if stream:
                    dtype = streamer.FMT2TYPE[streamer.pa_format]

                    try:
                        stream, dtype = change_audio(
                            stream,
                            dtype,
                            conv_fn=voice_conv_fn,
                            sample_rate=streamer.processing_rate,
                            **voice_conv_kwargs,
                        )
                    except Exception as e:
                        # print(e)
                        pass  # continue with the original stream and dtype

                    producer_consumer(
                        msg=(stream, dtype, streamer.processing_rate),
                        producer_kwargs=audio_phonemes_producer_kwargs,
                        consumer_kwargs=audio_phonemes_consumer_kwargs,
                    )
                    # r.sleep()

                # else:
                #     phonemes = phonemes_pub.random(phonemes_duration, chunk=phonemes_duration)
                #     phonemes[0][phonemes_pub.phoneme_key] = phonemes_pub.sil
                #     phonemes_pub.publish(phonemes)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "-a",
        "--audio_topic",
        default="/hr/control/audio/stream",
        help="ROS topic to which the speech audio should be published",
    )

    parser.add_argument(
        "-v",
        "--visemes_topic",
        default="/hr/animation/queue_visemes",
        help="ROS topic to which the visemes should be published",
    )

    parser.add_argument(
        "-q",
        "--pub_queue_size",
        type=int,
        default=2500,
        help="Queue size of the Speech and Phoneme Publishers",
    )

    parser.add_argument(
        "-p",
        "--phonemes_segmenter",
        choices=("random", "kaldi", "vosk"),
        help="Phoneme segmenter to be used. By default, no phonemes will be published.",
    )

    parser.add_argument(
        "-d",
        "--phonemes_duration",
        type=float,
        default=0.04,
        help="Phonemes duration for random phonemes. By default, 0.04 secs.",
    )

    parser.add_argument(
        "-t",
        "--phonemes_drop_th",
        type=float,
        default=0.85,
        help="Phonemes drop threshold for random phonemes. By default, 0.85 i.e. 85%.",
    )

    parser.add_argument(
        "-g",
        "--vad_accumulate_count",
        type=int,
        default=20,
        help="Frame accumulation count for VAD to facilitate further audio processing"
        " e.g. change_pitch or change_pitch. Default count is 10. Specify -1 for infinity.",
    )

    parser.add_argument(
        "-c",
        "--voice_conv_fn",
        choices=["change_gender", "change_pitch"],
        help="Specify function to be used for voice conversion",
    )

    parser.add_argument(
        "--min_pitch",
        type=float,
        default=75,
        help="Minimum pitch for change_pitch or change_gender function",
    )

    parser.add_argument(
        "--max_pitch",
        type=float,
        default=600,
        help="Maximum pitch for change_pitch or change_gender function",
    )

    parser.add_argument(
        "--formant_shift_ratio",
        type=float,
        default=1.2,
        help="Formant shift ratio for praat's change_gender function",
    )

    parser.add_argument(
        "--new_pitch_median",
        type=float,
        default=0,
        help="New pitch median for praat's change_gender function",
    )

    parser.add_argument(
        "--pitch_range_factor",
        type=float,
        default=1,
        help="Scaling factor for the new pitch values around the new pitch median"
        " for the praat's change_gender function",
    )

    parser.add_argument(
        "--duration_factor",
        type=float,
        default=1,
        help="Factor with which the sound duration will be lenghtened or shortened"
        " in the praat's change_gender function",
    )

    parser.add_argument(
        "--pitch_factor",
        type=float,
        default=1.5,
        help="Pitch factor to be used in case of change_pitch function."
        " Factor > 1 for female target voice while < 1 for male target voice.",
    )

    parser.add_argument(
        "--time_step",
        type=float,
        default=0.01,
        help="Time step to be used for the change_pitch function.",
    )

    args = parser.parse_args()

    stream_speech(
        audio_topic=args.audio_topic,
        visemes_topic=args.visemes_topic,
        pub_queue_size=args.pub_queue_size,
        phonemes_segmenter=args.phonemes_segmenter,
        phonemes_duration=args.phonemes_duration,
        phonemes_drop_th=args.phonemes_drop_th,
        vad_accumulate_count=args.vad_accumulate_count,
        voice_conv_fn=args.voice_conv_fn,
        min_pitch=args.min_pitch,
        max_pitch=args.max_pitch,
        formant_shift_ratio=args.formant_shift_ratio,
        new_pitch_median=args.new_pitch_median,
        pitch_range_factor=args.pitch_range_factor,
        duration_factor=args.duration_factor,
        pitch_factor=args.pitch_factor,
        time_step=args.time_step,
    )
