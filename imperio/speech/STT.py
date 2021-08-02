import numpy as np
import torch
from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC

from .BaseSTT import BaseSTT


class STT(BaseSTT):

    MODEL_IDENTIFIER = "facebook/wav2vec2-base-960h"

    def __init__(
        self,
        lang="en-US",
        audio_streamer=None,
        text_batch_processor=None,
        model="default",
        model_processor="default",
        gpu_idx=None,
    ):

        super(STT, self).__init__(lang, audio_streamer, None, text_batch_processor)

        self._model = model
        if model == "default":
            self._model = Wav2Vec2ForCTC.from_pretrained(self.MODEL_IDENTIFIER)
        self._to_device(self._model, gpu_idx)

        self._model_processor = model_processor
        if model_processor == "default":
            self._model_processor = Wav2Vec2Processor.from_pretrained(
                self.MODEL_IDENTIFIER
            )

    def _to_device(self, model, gpu_idx):
        if model is not None:
            if gpu_idx is not None and torch.cuda.is_available():
                device = torch.device(f"cuda:{gpu_idx}")
            else:
                device = torch.device("cpu")
            model.to(device).eval()

        return self

    def get_logits(self, audio_inp, model=None, model_processor=None):
        
        model = self._model if model is None else model
        
        model_processor = (
            self._model_processor if model_processor is None else model_processor
        )
        device = model.device

        input_values = model_processor(
            audio_inp, sampling_rate=16000, return_tensors="pt"
        ).input_values.to(device)

        with torch.no_grad():
            logits = model(input_values).logits

        return logits, model_processor

    def transcribe(self, audio_inp, model=None, model_processor=None):
        logits, model_processor = self.get_logits(audio_inp, model, model_processor)
        predicted_ids = torch.argmax(logits, dim=-1)
        transcription = model_processor.batch_decode(predicted_ids)[0]

        return transcription

    def streaming_transcribe(self, model=None, model_processor=None, gpu_idx=None):

        self._to_device(model, gpu_idx)

        with self._audio_streamer as audio_streamer:

            for i, stream in enumerate(audio_streamer.stream()):

                if stream is not None:
                    audio_inp = np.frombuffer(stream, np.float32)
                    text = self.transcribe(audio_inp, model, model_processor)

                    if text:
                        self._process_text(text, reset=True)
