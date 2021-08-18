import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

import argparse
from pathlib import Path
import pandas as pd
import librosa
import parselmouth
from timeit import default_timer as timer
from imperio.sonorus.speech.kaldi import PhonemeSegmenter
from imperio.sonorus.audio.praat import change_pitch

parser = argparse.ArgumentParser()

parser.add_argument(
    "-d",
    "--dir",
    help="Directory containing the wav files",
)

args = parser.parse_args()
phoneme_segmenter = PhonemeSegmenter.from_url()

time_df = pd.DataFrame(columns=("File_Path", "Audio_Dur", "Seg_Dur"))
paths = sorted(Path(args.dir).glob("**/*.wav"))
for file in paths:
    audio, sr = librosa.load(file, sr=16000)
    audio_len = librosa.get_duration(audio, sr=sr)
    start = timer()
    audio = parselmouth.Sound(values=audio, sampling_frequency=sr)
    audio = change_pitch(audio, factor=1.5)
    seg = phoneme_segmenter.segment(audio.values, sample_rate=sr)
    end = timer()
    time_df = time_df.append(
        dict(
            File_Path=str(file), 
            Audio_Dur=audio_len, 
            Seg_Dur=end-start), 
        ignore_index=True
    )
    print(seg)
print(time_df)
time_df.to_csv("Seg_dur.csv", index=False)