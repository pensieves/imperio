import numpy as np
import torch
from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC
from ..audio import VADAudioInputStreamer
from .utils import to_device


class Wav2Vec2STT(object):
    def __init__(
        self,
        lang="en-US",
        audio_streamer=None,
        model="facebook/wav2vec2-base-960h",
        model_processor="facebook/wav2vec2-base-960h",
        decoder=None,
        gpu_idx=None,
    ):

        self._lang = lang
        self._audio_streamer = (
            VADAudioInputStreamer() if audio_streamer is None else audio_streamer
        )

        if isinstance(model, str):
            self.model = Wav2Vec2ForCTC.from_pretrained(model)
        else:
            self.model = model
        self.model = to_device(self.model, gpu_idx, for_eval=True)

        if isinstance(model_processor, str):
            self.model_processor = Wav2Vec2Processor.from_pretrained(model_processor)
        else:
            self.model_processor = model_processor

        self.set_decoder(decoder)

    def set_decoder(self, decoder):
        self.decoder = decoder
        return self

    def reset_decoder(self):
        return self.set_decoder(None)

    def get_logits(self, audio_inp, sampling_rate=16000):

        input_values = self.model_processor(
            audio_inp, sampling_rate=sampling_rate, return_tensors="pt"
        ).input_values.to(self.model.device)

        with torch.no_grad():
            logits = self.model(input_values).logits

        return logits

    def transcribe(self, audio_inp, sampling_rate=16000):
        logits = self.get_logits(audio_inp, sampling_rate)

        if self.decoder is not None:
            logits = logits.float().cpu().contiguous()
            decoded = self.decoder.decode(logits)
            # 1st sample, 1st best transcription
            transcription = self.decoder.post_process(decoded)[0][0]

        else:
            predicted_ids = torch.argmax(logits, dim=-1)
            transcription = self.model_processor.batch_decode(predicted_ids)[0]

        return transcription

    def streaming_transcribe(
        self, sampling_rate=16000, callback=print, **callback_kwargs
    ):

        with self._audio_streamer as audio_streamer:
            sampling_rate = getattr(audio_streamer, "processing_rate", sampling_rate)

            for i, stream in enumerate(audio_streamer.stream()):

                if stream is not None:
                    audio_inp = np.frombuffer(stream, np.float32)
                    text = self.transcribe(audio_inp, sampling_rate=sampling_rate)

                    if text:
                        callback(text, **callback_kwargs)
