import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parents[1].resolve()))
sys.path.append(str(Path(__file__).parents[2].resolve()))

from flask import Flask, request, Response, render_template
import time

import pyaudio
import numpy as np
import librosa
import parselmouth

from imperio.sonorus.audio import AudioInputStreamer, AudioOutputStreamer
from imperio.sonorus.audio.utils import audio_int2float, audio_float2int
from imperio.sonorus.audio.praat import change_pitch, change_gender
from streaming_speech import stream_speech

app = Flask(__name__)

SAMPLE_RATE = 16000
PA_FORMAT = pyaudio.paInt16

RECORD_BUTTON_NAME = "record_button"
RECORD_BUTTON_VALUE = "Record Audio"

PLAY_BUTTON_NAME = "play_button"
PLAY_BUTTON_VALUE = "Play Recorded"

SUBMIT_BUTTON_NAME = "submit_button"

PLAY_SOURCE_BUTTON_VALUE = "Play Source"
PLAY_CHANGED_BUTTON_VALUE = "Play Changed"
FINALIZE_PARAMS_BUTTON_VALUE = "Finalize Params"

SOURCE_AUDIO = b""
CHANGED_AUDIO = b""

data = {"slider1": 0, "slider2": 0, "slider3": 0, "slider4": 0, "slider5": 0}


def record_audio(audio_dur=5):

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
    return render_template("home.html", record_button_value=RECORD_BUTTON_VALUE)


@app.route("/record_audio", methods=["GET", "POST"])
def record():

    template_html = "record_audio.html"
    recording_text = (
        '"Hi, I am Asha. I am an avatar robot. I will be assisting you today."'
    )

    recording_status = f"Cick on the '{RECORD_BUTTON_VALUE}' button to start recording."
    play_recorded = False

    if request.method == "POST":
        if request.form.get(RECORD_BUTTON_NAME) == RECORD_BUTTON_VALUE:
            global SOURCE_AUDIO
            SOURCE_AUDIO, audio_dur = record_audio()
            print(
                f"Recorded audio = {SOURCE_AUDIO[:5]} ... of len = {len(SOURCE_AUDIO)} and dur = {audio_dur}"
            )

        elif request.form.get(PLAY_BUTTON_NAME) == PLAY_BUTTON_VALUE:
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
        record_button_name=RECORD_BUTTON_NAME,
        record_button_value=RECORD_BUTTON_VALUE,
        recording_text=recording_text,
        play_recorded=play_recorded,
        play_button_name=PLAY_BUTTON_NAME,
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

            sound = change_func(sound, **kwargs)
            CHANGED_AUDIO = audio_float2int(sound.values).tobytes()

            play_audio(audio="changed")

        if request.form[SUBMIT_BUTTON_NAME] == FINALIZE_PARAMS_BUTTON_VALUE:
            params = update_params(params, request.form)
            # pass

    template = render_template(
        template_html,
        params=params,
        submit_button_name=SUBMIT_BUTTON_NAME,
        play_source_button_value=PLAY_SOURCE_BUTTON_VALUE,
        play_changed_button_value=PLAY_CHANGED_BUTTON_VALUE,
        finalize_params_button_value=FINALIZE_PARAMS_BUTTON_VALUE,
    )

    return template


@app.route("/change_pitch", methods=["GET", "POST"])
def change_audio_pitch():

    change_func = change_pitch
    params = [
        dict(
            name="pitch_factor", default_value=1.5, value=1.5, min=0, max=3, step=0.05
        ),
        dict(
            name="time_step", default_value=0.01, value=0.01, min=0, max=0.1, step=0.005
        ),
        dict(name="min_pitch", default_value=75, value=75, min=50, max=100, step=5),
        dict(name="max_pitch", default_value=600, value=600, min=500, max=800, step=5),
    ]

    return change_audio(change_func, params, request)


@app.route("/change_gender", methods=["GET", "POST"])
def change_audio_gender():

    change_func = change_gender
    params = [
        dict(name="min_pitch", default_value=75, value=75, min=50, max=100, step=5),
        dict(name="max_pitch", default_value=600, value=600, min=500, max=800, step=5),
        dict(
            name="formant_shift_ratio",
            default_value=1.2,
            value=1.2,
            min=0.5,
            max=2,
            step=0.05,
        ),
        dict(name="new_pitch_median", default_value=0, value=0, min=0, max=100, step=5),
        dict(
            name="pitch_range_factor", default_value=1, value=1, min=0, max=2, step=0.5
        ),
        dict(name="duration_factor", default_value=1, value=1, min=0, max=3, step=0.5),
    ]

    return change_audio(change_func, params, request)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=7000)
