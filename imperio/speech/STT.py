import numpy as np
import Levenshtein

import torch
from transformers import Wav2Vec2Tokenizer, Wav2Vec2ForCTC

from .AudioStreamer import AudioStreamer
from .BaseSTT import BaseSTT, SAMPLE_RATE, CHUNK, PA_FORMAT
from .utils import lastargmax

MODEL_IDENTIFIER = "facebook/wav2vec2-base-960h"
STREAM_COUNT = 50
STREAM_OVERLAP = 5
STREAM_SILENCE = 5
WORD_OVERLAP = 5


class STT(BaseSTT):
    def __init__(
        self,
        rate=SAMPLE_RATE,
        chunk=CHUNK,
        pa_format=PA_FORMAT,
        lang="en-US",
        text_batcher=None,
        text_batch_processor=None,
        model="default",
        tokenizer="default",
        gpu_idx=None,
        stream_count=STREAM_COUNT,
        stream_overlap=STREAM_OVERLAP,
        stream_silence=STREAM_SILENCE,
        word_overlap=WORD_OVERLAP,
    ):

        super(STT, self).__init__(
            rate, chunk, pa_format, lang, text_batcher, text_batch_processor
        )

        self._model = model
        if model == "default":
            self._model = Wav2Vec2ForCTC.from_pretrained(MODEL_IDENTIFIER)
        self._to_device(self._model, gpu_idx)

        self._tokenizer = tokenizer
        if tokenizer == "default":
            self._tokenizer = Wav2Vec2Tokenizer.from_pretrained(MODEL_IDENTIFIER)

        self._stream_count = stream_count
        self._stream_overlap = stream_overlap
        self._stream_silence = stream_silence

        self._word_overlap = word_overlap
        if stream_overlap and not word_overlap:
            self._word_overlap = WORD_OVERLAP

    def _to_device(self, model, gpu_idx):
        if model is not None:
            if gpu_idx is not None and torch.cuda.is_available():
                device = torch.device(f"cuda:{gpu_idx}")
            else:
                device = torch.device("cpu")
            model.to(device).eval()

    def transcribe(self, audio_inp, model=None, tokenizer=None):
        model = self._model if model is None else model
        tokenizer = self._tokenizer if tokenizer is None else tokenizer
        device = model.device

        input_values = tokenizer(audio_inp, return_tensors="pt").input_values.to(device)
        with torch.no_grad():
            logits = model(input_values).logits
        predicted_ids = torch.argmax(logits, dim=-1)
        transcription = tokenizer.batch_decode(predicted_ids)[0]

        return transcription

    def streaming_transcribe(self, model=None, tokenizer=None, gpu_idx=None):

        self._to_device(model, gpu_idx)

        with AudioStreamer(self._rate, self._chunk, self._pa_format) as audio_streamer:

            audio_inp = np.array([], dtype=np.float32)
            text = ""

            for i, stream in enumerate(audio_streamer.stream()):

                cur_audio_inp = np.frombuffer(stream, np.float32)
                audio_inp = np.hstack((audio_inp, cur_audio_inp))

                if (i + 1) % self._stream_count == 0:

                    transcription = self.transcribe(audio_inp, model, tokenizer)
                    text = self._append_transcriptions(text, transcription)

                    silence_inp = audio_inp[
                        -self._stream_silence * len(cur_audio_inp) :
                    ]
                    silent = (
                        self.transcribe(silence_inp, model, tokenizer).strip() == ""
                    )

                    if not silent:
                        if self._stream_overlap:
                            audio_inp = audio_inp[
                                -self._stream_overlap * len(cur_audio_inp) :
                            ]
                        else:
                            audio_inp = np.array([], dtype=np.float32)
                    else:
                        audio_inp = np.array([], dtype=np.float32)
                        if text:
                            self._process_text(text, reset=True)
                        text = ""

    def _append_transcriptions(self, text, transcription):
        transcription = transcription.strip()
        print(transcription)
        if self._word_overlap:
            text = self._overlapped_append(text, transcription)
        elif transcription:
            text += " " + transcription
        return text

    def _overlapped_append(self, first, second):

        if (not first) or (not second):
            if not first:
                final = second
            else:
                final = first

        else:
            words_first = first.split()
            words_second = second.split()
            word_overlap = min(self._word_overlap, len(words_first), len(words_second))

            overlapped_first = first.split()[-word_overlap:]
            overlapped_second = second.split()[:word_overlap]

            first = first[
                : -(word_overlap - 1 + sum(len(i) for i in overlapped_first))
            ].strip()
            second = second[
                (word_overlap - 1 + sum(len(i) for i in overlapped_second)) :
            ].strip()

            similarity = np.array(
                [
                    [Levenshtein.ratio(w1, w2) for w2 in overlapped_second]
                    for w1 in overlapped_first
                ]
            )

            split_pos = lastargmax(similarity)

            first += " " + " ".join(overlapped_first[: split_pos[0]])
            second = (" ".join(overlapped_second[split_pos[1] :]) + " ") + second
            final = first + " " + second

        return final
