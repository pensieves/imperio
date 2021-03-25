import numpy as np
from scipy import signal
from collections import deque
import webrtcvad


def audio_resample(data, sample_rate, resample_rate, dtype=None):
    """
    Microphone/Audio source may not support native processing sample rate, so
    resample from given sample_rate for webrtcvad/subsequent processing.
    """
    if dtype is not None:
        data = np.frombuffer(data, dtype=dtype)

    size = (len(data) * self.resample_rate) // self.sample_rate
    data = signal.resample(data, size)

    if dtype is not None:
        data = np.array(data, dtype=dtype).tobytes()

    return data


def audio_float2int(data, float_type=None, int_type=np.int16):
    r"""Convert an audio array from float type to int type.
    float_type is REQUIRED when data is bytes object."""

    if float_type is not None:
        data = np.frombuffer(data, dtype=float_type)

    int_info = np.iinfo(int_type)
    abs_max = 2 ** (int_info.bits - 1)
    data = (data * abs_max).clip(int_info.min, int_info.max).astype(int_type)

    if float_type is not None:
        data = data.tobytes()

    return data


def audio_int2float(data, int_type=None, float_type=np.float32):
    r"""Convert an audio array from int type to float type.
    int_type is REQUIRED when data is bytes object."""

    if int_type is not None:
        data = np.frombuffer(data, dtype=int_type)
        int_info = np.iinfo(int_type)
    else:
        int_info = np.iinfo(data.dtype)

    abs_max = 2 ** (int_info.bits - 1)
    data = data.astype(float_type) / abs_max

    if int_type is not None:
        data = data.tobytes()

    return data


def vad_collector(
    streamer,
    vad=webrtcvad.Vad(3),
    sample_rate=16000,
    num_padding_frames=20,
    act_inact_ratio=0.9,
    frame_dtype_conv_fn=lambda frame, float_type, int_type: frame,
):
    r"""VAD based generator that yields voiced audio frames followed by a None 
    to mark end/break in speech. Collection of voiced frames is based on voice
    activity/inactivity ratio in num_padding_frames.
    """

    ring_buff = deque(maxlen=num_padding_frames)
    triggered = False
    voiced_frames = list()

    for frame in streamer:

        vad_frame = frame_dtype_conv_fn(frame, float_type=np.float32, int_type=np.int16)
        is_speech = vad.is_speech(vad_frame, sample_rate)

        if not triggered:
            ring_buff.append((frame, is_speech))
            num_voiced = len([f for f, speech in ring_buff if speech])

            if num_voiced > (act_inact_ratio * ring_buff.maxlen):
                triggered = True
                voiced_frames.extend((f for f, s in ring_buff))
                ring_buff.clear()

        else:
            voiced_frames.append(frame)
            ring_buff.append((frame, is_speech))
            num_unvoiced = len([f for f, speech in ring_buff if not speech])

            if num_unvoiced > (act_inact_ratio * ring_buff.maxlen):
                triggered = False

                # yield entire voiced frames
                yield b"".join(voiced_frames)

                # yield None to mark a break in consecutive but separate voiced
                # frames so that speech processor can start transcribing the
                # previously sent voice frames
                yield None

                ring_buff.clear()
                voiced_frames = list()
