import numpy as np
import parselmouth
from parselmouth.praat import call


def reduce_noise(
    sound: parselmouth.Sound,
    noise_time_min: float = 0,
    noise_time_max: float = -1,
    window_len: float = 0.025,
    filter_freq_min: float = 80,
    filter_freq_max: float = 10000,
    smoothing_bandwidth: float = 40,
    noise_reduction_db: float = -20,
    noise_reduction_method="spectral-subtraction",
) -> parselmouth.Sound:

    args = (
        noise_time_min,
        noise_time_max,
        window_len,
        filter_freq_min,
        filter_freq_max,
        smoothing_bandwidth,
        noise_reduction_method,
    )

    noise_reduced_sound = call(sound, "Remove noise", *args)

    return noise_reduced_sound


def change_gender(
    sound: parselmouth.Sound,
    min_pitch: float = 75,
    max_pitch: float = 600,
    formant_shift_ratio: float = 1.2,
    new_pitch_median: float = 0,
    pitch_range_factor: float = 1,
    duration_factor: float = 1,
) -> parselmouth.Sound:
    r"""Although directly using the change gender command of praat, 
    the results are often better when using change_pitch function, 
    so recommended to use the same."""

    args = (
        min_pitch,
        max_pitch,
        formant_shift_ratio,
        new_pitch_median,
        pitch_range_factor,
        duration_factor,
    )

    gender_changed_sound = call(sound, "Change gender", *args)

    return gender_changed_sound


def change_pitch(
    sound: parselmouth.Sound,
    pitch_factor: float = 1.5,
    time_step: float = 0.01,
    min_pitch: float = 75,
    max_pitch: float = 600,
) -> parselmouth.Sound:
    r"""Recommended to use instead of using change_gender function as
    the results are often better using this function. Composed using
    the pitch change example at:
    https://parselmouth.readthedocs.io/en/stable/examples/pitch_manipulation.html
    """
    args = (time_step, min_pitch, max_pitch)
    manipulation = call(sound, "To Manipulation", *args)

    pitch_tier = call(manipulation, "Extract pitch tier")

    args = (sound.xmin, sound.xmax, pitch_factor)
    call(pitch_tier, "Multiply frequencies", *args)

    call([pitch_tier, manipulation], "Replace pitch tier")

    pitch_changed_sound = call(manipulation, "Get resynthesis (overlap-add)")

    return pitch_changed_sound
