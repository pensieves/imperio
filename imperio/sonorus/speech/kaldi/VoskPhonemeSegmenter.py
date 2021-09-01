import json
import numpy as np
import librosa
from pathlib import Path
from vosk import Model, KaldiRecognizer

from ...audio.utils import audio_int2float, audio_float2int
from .PhonemeSegmenter import PhonemeSegmenter, simplify_phoneme

from ... import CACHE_DIR
from .utils import VOSK_MODEL_GENERIC_EN_US, download_model


def load_lexicon(path):
    lexicon = dict()

    with open(path, "r") as f:
        for line in f:
            line = line.split()

            word = line[0]
            lex = " ".join(line[1:])

            lexicon[word] = lex

    return lexicon


def to_phonemes(text, lexicon_dict):
    phonemes = list()
    words = [w for w in text.split() if w]
    for w in words:
        phonemes.extend(lexicon_dict[w].split())
    return phonemes


class VoskPhonemeSegmenter(PhonemeSegmenter):
    def __init__(
        self,
        model_path,
        lexicon,
        chunk=320,
        sample_rate=16000,
        simplify_phoneme=simplify_phoneme,  # set to None if not desired
        as_dict=True,  # return phonemes info as dict if True else as tuple
        pos2key=(
            "name",
            "start",
            "duration",
        ),  # keys to be used for dict while repacking
    ):

        self.model = Model(model_path)

        if not isinstance(lexicon, dict):
            lexicon = load_lexicon(lexicon)
        self.lexicon = lexicon

        self.chunk = chunk
        self.sample_rate = sample_rate
        self.recognizer = KaldiRecognizer(self.model, sample_rate)

        self.simplify_phoneme = simplify_phoneme

        self.as_dict = as_dict
        self.pos2key = pos2key

    @classmethod
    def from_url(
        cls,
        url=VOSK_MODEL_GENERIC_EN_US,
        cache_dir=CACHE_DIR,
        force_download=False,
        chunk=320,
        sample_rate=16000,
    ):

        downloaded_dir = Path(
            download_model(url, cache_dir, force_download, prefix="vosk_model")
        )
        model_path = str(downloaded_dir / "model")
        lexicon = str(downloaded_dir / "lexicon/lexicon.txt")

        return cls(
            model_path=model_path,
            lexicon=lexicon,
            chunk=chunk,
            sample_rate=sample_rate,
        )

    def segment(
        self, audio, sample_rate=16000, time_level=True, update_recognizer=True
    ):

        # print(sample_rate, self.sample_rate)

        recognizer = self.recognizer
        if sample_rate != self.sample_rate:
            recognizer = KaldiRecognizer(self.model, sample_rate)

            if update_recognizer:
                self.sample_rate = sample_rate
                self.recognizer = recognizer

        if len(audio) == 1:  # single row but given as 2D matrix
            audio = audio[0]
        if np.issubdtype(audio.dtype, np.integer):
            audio = audio_int2float(audio)

        audio_dur = librosa.get_duration(audio, sample_rate)
        audio = audio_float2int(audio).tobytes()

        phonemes = {"utt1": {"phonemes": []}}

        for i in range(0, len(audio), self.chunk):
            recognized = recognizer.AcceptWaveform(audio[i : i + self.chunk])

            # decoded = json.loads(recognizer.Result())
            # if decoded["text"]:
            #     print(i, recognized, decoded)

            if recognized:

                decoded = json.loads(recognizer.Result())
                if decoded["text"]:
                    # import pdb; pdb.set_trace()
                    phones = to_phonemes(decoded["text"], self.lexicon)

                    phoneme_dur = round(audio_dur / len(phones), 3)
                    phones = [
                        (ph, i * phoneme_dur, phoneme_dur)
                        for i, ph in enumerate(phones)
                    ]

                    phones = self.repack(self.simplify(phones))
                    phonemes["utt1"] = dict(text=decoded["text"], phonemes=phones)

        return phonemes
