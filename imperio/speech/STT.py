import numpy as np
import torch
from transformers import Wav2Vec2Tokenizer, Wav2Vec2ForCTC

from .BaseSTT import BaseSTT


class STT(BaseSTT):

    MODEL_IDENTIFIER = "facebook/wav2vec2-base-960h"

    def __init__(
        self,
        lang="en-US",
        audio_streamer=None,
        text_batch_processor=None,
        model="default",
        tokenizer="default",
        gpu_idx=None,
    ):

        super(STT, self).__init__(lang, audio_streamer, None, text_batch_processor)

        self._model = model
        if model == "default":
            self._model = Wav2Vec2ForCTC.from_pretrained(self.MODEL_IDENTIFIER)
        self._to_device(self._model, gpu_idx)

        self._tokenizer = tokenizer
        if tokenizer == "default":
            self._tokenizer = Wav2Vec2Tokenizer.from_pretrained(self.MODEL_IDENTIFIER)

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

            for i, stream in enumerate(audio_streamer.stream()):

                if stream is not None:
                    audio_inp = np.frombuffer(stream, np.float32)
                    text = self.transcribe(audio_inp, model, tokenizer)

                    if text:
                        self._process_text(text, reset=True)
