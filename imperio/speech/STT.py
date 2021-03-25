import numpy as np
import Levenshtein

import torch
from transformers import Wav2Vec2Tokenizer, Wav2Vec2ForCTC

from .BaseSTT import BaseSTT
from .utils import lastargmatch


class STT(BaseSTT):

    MODEL_IDENTIFIER = "facebook/wav2vec2-base-960h"
    STREAM_COUNT = 50
    STREAM_OVERLAP = 5
    WORD_OVERLAP = 5
    LEVENSHTEIN_SIM_TH = 0.5

    def __init__(
        self,
        lang="en-US",
        audio_streamer=None,
        text_batcher=None,
        text_batch_processor=None,
        model="default",
        tokenizer="default",
        gpu_idx=None,
        stream_count=STREAM_COUNT,
        stream_overlap=STREAM_OVERLAP,
        word_overlap=WORD_OVERLAP,
        levenshtein_sim_th=LEVENSHTEIN_SIM_TH,
    ):

        super(STT, self).__init__(
            lang, audio_streamer, text_batcher, text_batch_processor
        )

        self._model = model
        if model == "default":
            self._model = Wav2Vec2ForCTC.from_pretrained(self.MODEL_IDENTIFIER)
        self._to_device(self._model, gpu_idx)

        self._tokenizer = tokenizer
        if tokenizer == "default":
            self._tokenizer = Wav2Vec2Tokenizer.from_pretrained(self.MODEL_IDENTIFIER)

        self._stream_count = stream_count
        self._stream_overlap = stream_overlap

        self._word_overlap = word_overlap
        if stream_overlap and not word_overlap:
            self._word_overlap = WORD_OVERLAP

        self._levenshtein_sim_th = levenshtein_sim_th

    def _to_device(self, model, gpu_idx):
        if model is not None:
            if gpu_idx is not None and torch.cuda.is_available():
                device = torch.device(f"cuda:{gpu_idx}")
            else:
                device = torch.device("cpu")
            model.to(device).eval()

        return self

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

        with self._audio_streamer as audio_streamer:

            audio_inp = np.array([], dtype=np.float32)
            text = ""

            for i, stream in enumerate(audio_streamer.stream()):

                if stream is not None:
                    cur_audio_inp = np.frombuffer(stream, np.float32)
                    audio_inp = np.hstack((audio_inp, cur_audio_inp))

                if stream is None or ((i + 1) % self._stream_count == 0):

                    transcription = self.transcribe(audio_inp, model, tokenizer)
                    text = self._append_transcriptions(text, transcription)

                    if stream is not None:

                        audio_inp = (
                            audio_inp[-self._stream_overlap * len(cur_audio_inp) :]
                            if self._stream_overlap
                            else np.array([], dtype=np.float32)
                        )

                    else:
                        audio_inp = np.array([], dtype=np.float32)
                        if text:
                            self._process_text(text, reset=True)
                        text = ""

    def _append_transcriptions(self, text, transcription):
        transcription = transcription.strip()
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

            max_sim = similarity.max()
            if max_sim >= self._levenshtein_sim_th:
                split_pos = lastargmatch(similarity, match=max_sim)
            else:
                # effectively perform no split
                split_pos = (len(overlapped_first), 0)

            first += " " + " ".join(overlapped_first[: split_pos[0]])
            second = (" ".join(overlapped_second[split_pos[1] :]) + " ") + second
            final = first + " " + second

        return final
