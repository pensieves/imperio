import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parents[1].resolve()))
sys.path.append(str(Path(__file__).parents[2].resolve()))

from flask import (
    Flask,
    request,
    render_template,
    redirect,
    url_for,
)
import time

import pyaudio
import numpy as np
import librosa
import parselmouth
from pathlib import Path

from imperio.sonorus.utilities.utils import to_yaml, from_yaml
from imperio.sonorus.audio import AudioInputStreamer, AudioOutputStreamer
from imperio.sonorus.audio.utils import audio_int2float, audio_float2int
from imperio.sonorus.audio.praat import change_pitch, change_gender
from streaming_speech import stream_speech

app = Flask(__name__)

DATA_DIR = Path(__file__).parent / "data"

SAMPLE_RATE = 16000
PA_FORMAT = pyaudio.paInt16

TUNE_BUTTON_NAME = "Tune Voice Conversion"
RUN_BUTTON_NAME = "Stream Speech"

RECORD_BUTTON_VALUE = "Record Audio"
PLAY_BUTTON_VALUE = "Play Recorded"

SUBMIT_BUTTON_NAME = "submit_button"

PLAY_SOURCE_BUTTON_VALUE = "Play Source"
PLAY_CHANGED_BUTTON_VALUE = "Play Changed"
FINALIZE_PARAMS_BUTTON_VALUE = "Finalize Params"

SOURCE_AUDIO = b""
CHANGED_AUDIO = b""

CHANGE_PARAMS_YAML = "change_params.yaml"

VOICE_CONV_CHECKBOX = "Use Voice Conversion"

DECIBEL_SHIFT_NAME = "Decibel shift"
DECIBEL_SHIFT_DEFAULT = "12"

VAD_ACCUMULATE_COUNT_NAME = "Voice Frame accumulation count"
VAD_ACCUMULATE_COUNT_DEFAULT = "20"

PHONEME_SEGMENTER = "random"


def record_audio(audio_dur=10):

    audio = b""
    audio_streamer = AudioInputStreamer(sample_rate=SAMPLE_RATE, pa_format=PA_FORMAT)

    with audio_streamer as streamer:

        streamer_dtype = streamer.FMT2TYPE[streamer.pa_format]
        sample_rate = streamer.processing_rate

        for stream in streamer.stream():

            audio = b"".join((audio, stream))

            dur = librosa.get_duration(
                audio_int2float(np.frombuffer(audio, dtype=streamer_dtype)),
                sr=sample_rate,
            )

            if dur >= audio_dur:
                break

    return audio, dur


def play_audio(audio="source"):
    audio = SOURCE_AUDIO if audio == "source" else CHANGED_AUDIO
    print(f"Playing recorded audio of len = {len(audio)}...")
    with AudioOutputStreamer(sample_rate=SAMPLE_RATE) as streamer:
        streamer.stream(
            (
                audio[i : i + streamer.chunk]
                for i in range(0, len(audio), streamer.chunk)
            )
        )


@app.route("/")
def home():
    return render_template(
        "home.html", tune_button_name=TUNE_BUTTON_NAME, run_button_name=RUN_BUTTON_NAME,
    )


@app.route("/record_audio", methods=["GET", "POST"])
def record():

    template_html = "record_audio.html"
    recording_text = (
        '"Hi, I am Asha. I am an avatar robot. I will be assisting you today."'
    )

    recording_status = f"Cick on the '{RECORD_BUTTON_VALUE}' button to start recording."
    play_recorded = False

    if request.method == "POST":
        if request.form.get(SUBMIT_BUTTON_NAME) == RECORD_BUTTON_VALUE:
            global SOURCE_AUDIO
            SOURCE_AUDIO, audio_dur = record_audio()
            print(
                f"Recorded audio = {SOURCE_AUDIO[:5]} ... of len = {len(SOURCE_AUDIO)} and dur = {audio_dur}"
            )

        elif request.form.get(SUBMIT_BUTTON_NAME) == PLAY_BUTTON_VALUE:
            play_audio(audio="source")

        recording_status = (
            f"Audio recorded successfully."
            f" Click on the '{RECORD_BUTTON_VALUE}' button to record again"
            f" or use the '{PLAY_BUTTON_VALUE}' button to listen to the recorded audio."
        )

        play_recorded = True

    template = render_template(
        template_html,
        recording_status=recording_status,
        submit_button_name=SUBMIT_BUTTON_NAME,
        record_button_value=RECORD_BUTTON_VALUE,
        recording_text=recording_text,
        play_recorded=play_recorded,
        play_button_value=PLAY_BUTTON_VALUE,
    )

    return template


def update_params(params, request_form):
    for param in params:
        param["value"] = float(request_form[param["name"]])
    return params


def change_audio(change_func, params, request):

    template_html = "change_audio.html"

    if request.method == "POST":

        if request.form[SUBMIT_BUTTON_NAME] == PLAY_SOURCE_BUTTON_VALUE:
            play_audio(audio="source")

        if request.form[SUBMIT_BUTTON_NAME] == PLAY_CHANGED_BUTTON_VALUE:
            params = update_params(params, request.form)
            kwargs = {p["name"]: p["value"] for p in params}

            global CHANGED_AUDIO

            sound = parselmouth.Sound(
                values=np.frombuffer(
                    SOURCE_AUDIO, dtype=AudioInputStreamer.FMT2TYPE[PA_FORMAT],
                ),
                sampling_frequency=SAMPLE_RATE,
            )

            if change_func == "change_pitch":
                sound = change_pitch(sound, **kwargs)
            else:
                sound = change_gender(sound, **kwargs)

            CHANGED_AUDIO = audio_float2int(sound.values).tobytes()

            play_audio(audio="changed")

        if request.form[SUBMIT_BUTTON_NAME] == FINALIZE_PARAMS_BUTTON_VALUE:
            params = update_params(params, request.form)

            change_params = {p["name"]: p["value"] for p in params}
            change_params["voice_conv_fn"] = change_func

            to_yaml(data=change_params, file=Path(DATA_DIR) / CHANGE_PARAMS_YAML)

            return redirect(url_for("run_streaming_speech"))

    template = render_template(
        template_html,
        change_func=change_func,
        params=params,
        submit_button_name=SUBMIT_BUTTON_NAME,
        play_source_button_value=PLAY_SOURCE_BUTTON_VALUE,
        play_changed_button_value=PLAY_CHANGED_BUTTON_VALUE,
        finalize_params_button_value=FINALIZE_PARAMS_BUTTON_VALUE,
    )

    return template


@app.route("/change_pitch", methods=["GET", "POST"])
def change_audio_pitch():

    change_func = "change_pitch"
    params = [
        dict(
            name="pitch_factor", default_value=1.5, value=1.5, min=0, max=3, step=0.01,
        ),
        dict(
            name="time_step", default_value=0.01, value=0.01, min=0, max=0.1, step=0.005
        ),
        dict(name="min_pitch", default_value=75, value=75, min=50, max=100, step=1),
        dict(name="max_pitch", default_value=600, value=600, min=500, max=800, step=1),
    ]

    return change_audio(change_func, params, request)


@app.route("/change_gender", methods=["GET", "POST"])
def change_audio_gender():

    change_func = "change_gender"
    params = [
        dict(name="min_pitch", default_value=75, value=75, min=50, max=100, step=1),
        dict(name="max_pitch", default_value=600, value=600, min=500, max=800, step=1),
        dict(
            name="formant_shift_ratio",
            default_value=1.2,
            value=1.2,
            min=0.5,
            max=2,
            step=0.01,
        ),
        dict(name="new_pitch_median", default_value=0, value=0, min=0, max=100, step=1),
        dict(
            name="pitch_range_factor", default_value=1, value=1, min=0, max=2, step=0.1
        ),
        dict(name="duration_factor", default_value=1, value=1, min=0, max=3, step=0.1),
    ]

    return change_audio(change_func, params, request)


@app.route("/run_streaming_speech", methods=["GET", "POST"])
def run_streaming_speech():

    template_html = "run_streaming_speech.html"
    change_params = from_yaml(Path(DATA_DIR) / CHANGE_PARAMS_YAML)

    params = list()

    phonemes_params = [
        dict(
            name="phonemes_duration",
            default_value=0.04,
            value=0.04,
            min=0.02,
            max=0.2,
            step=0.01,
        ),
        dict(
            name="phonemes_drop_th",
            default_value=0.85,
            value=0.85,
            min=0,
            max=1,
            step=0.01,
        ),
    ]

    if change_params.get("voice_conv_fn"):
        params = [{"name": "voice_conv_fn", "value": change_params["voice_conv_fn"]}]
        params += [
            {"name": k, "value": v}
            for k, v in change_params.items()
            if k != "voice_conv_fn"
        ]

    if request.method == "POST":
        if request.form[SUBMIT_BUTTON_NAME] == RUN_BUTTON_NAME:

            run_kwargs = dict(phonemes_segmenter="random")

            if request.form.get(VOICE_CONV_CHECKBOX):
                run_kwargs.update(change_params)

            phonemes_params = update_params(phonemes_params, request.form)
            run_kwargs.update({p["name"]: p["value"] for p in phonemes_params})

            run_kwargs["decibel_shift"] = int(request.form.get(DECIBEL_SHIFT_NAME))

            run_kwargs["vad_accumulate_count"] = int(
                request.form.get(VAD_ACCUMULATE_COUNT_NAME)
            )
            print(run_kwargs)

            stream_speech(disable_ros_signals=True, **run_kwargs)

    template = render_template(
        template_html,
        params=params,
        tune_button_name=TUNE_BUTTON_NAME,
        voice_conv_checkbox=VOICE_CONV_CHECKBOX,
        phonemes_params=phonemes_params,
        decibel_shift_name=DECIBEL_SHIFT_NAME,
        decibel_shift_default=DECIBEL_SHIFT_DEFAULT,
        vad_accumulate_count_name=VAD_ACCUMULATE_COUNT_NAME,
        vad_accumulate_count_default=VAD_ACCUMULATE_COUNT_DEFAULT,
        submit_button_name=SUBMIT_BUTTON_NAME,
        run_button_name=RUN_BUTTON_NAME,
    )

    return template


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=7000)
