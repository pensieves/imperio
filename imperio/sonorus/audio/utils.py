import numpy as np
from scipy import signal
from collections import deque
import webrtcvad
import subprocess
import librosa
from pathlib import Path


def get_duration(audio_path, use="sox"):
    r"""Returns duration in seconds. sox is faster hence set as default."""

    if use == "sox":
        dur, err = subprocess.Popen(
            ["soxi", "-D", audio_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        ).communicate()

        if err:
            raise RuntimeError(err)

        dur = float(dur)

    elif use == "librosa":
        dur = librosa.get_duration(filename=audio_path)

    else:
        raise NotImplementedError(
            "Unsupported program/library for use. Please provide either sox or librosa as a value for use."
        )

    return dur


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
    abs_max_int = 2 ** (int_info.bits - 1)

    abs_max_float = max(abs(data.min()), abs(data.max()))

    # normalize if required
    if abs_max_float != 1:
        data /= abs_max_float

    data = (data * abs_max_int).clip(int_info.min, int_info.max).astype(int_type)

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


def convert_to_wav(audio_in_path, out_sr=16000, audio_out_path=None):

    if audio_out_path is None:
        audio_out_path = Path(audio_in_path).with_suffix(".wav")

    subprocess.Popen(
        ["ffmpeg", "-i", audio_in_path, "-ar", str(out_sr), audio_out_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def pad_sil_frames(frames, dtype=np.float32, target_nframes=float("inf")):
    if np.isfinite(target_nframes) and frames:
        sil_bytes = np.array([0], dtype=dtype).tobytes()
        repeat_nbytes = len(frames[-1]) // len(sil_bytes)

        sil_frame = b"".join((sil_bytes for i in range(repeat_nbytes)))
        pad_frames = [sil_frame for i in range(target_nframes - len(frames))]

        frames.extend(pad_frames)
    return frames


def vad_collector(
    streamer,
    vad=webrtcvad.Vad(3),
    sample_rate=16000,
    num_padding_frames=20,
    act_inact_ratio=0.95,
    accumulate=True,
    accumulate_count=float("inf"),
    pad_dtype=np.float32,
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

                if accumulate:
                    voiced_frames.extend((f for f, s in ring_buff))

                    while len(voiced_frames) >= accumulate_count:
                        yield b"".join(voiced_frames[:accumulate_count])
                        voiced_frames = voiced_frames[accumulate_count:]

                else:
                    for f, s in ring_buff:
                        yield f

                ring_buff.clear()

        else:

            if accumulate:
                voiced_frames.append(frame)

                while len(voiced_frames) >= accumulate_count:
                    yield b"".join(voiced_frames[:accumulate_count])
                    voiced_frames = voiced_frames[accumulate_count:]

            else:
                yield frame

            ring_buff.append((frame, is_speech))
            num_unvoiced = len([f for f, speech in ring_buff if not speech])

            if num_unvoiced > (act_inact_ratio * ring_buff.maxlen):
                triggered = False

                if accumulate:
                    voiced_frames = pad_sil_frames(
                        voiced_frames, dtype=pad_dtype, target_nframes=accumulate_count
                    )

                    # yield entire voiced frames
                    yield b"".join(voiced_frames)

                # yield None to mark a break in consecutive but separate voiced
                # frames so that speech processor can start transcribing the
                # previously sent voice frames
                yield None

                ring_buff.clear()
                voiced_frames = list()
