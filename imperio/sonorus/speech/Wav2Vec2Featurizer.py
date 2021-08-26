import numpy as np
import torch
from transformers import Wav2Vec2Processor, Wav2Vec2Model
from ..audio import VADAudioInputStreamer
from .utils import to_device


class Wav2Vec2Featurizer(object):
    def __init__(
        self,
        lang="en-US",
        audio_streamer=None,
        model="facebook/wav2vec2-base-960h",
        model_processor="facebook/wav2vec2-base-960h",
        gpu_idx=None,
    ):

        self._lang = lang
        self._audio_streamer = (
            VADAudioInputStreamer() if audio_streamer is None else audio_streamer
        )

        if isinstance(model, str):
            self.model = Wav2Vec2Model.from_pretrained(model)
        else:
            self.model = model
        self.model = to_device(self.model, gpu_idx, for_eval=True)

        if isinstance(model_processor, str):
            self.model_processor = Wav2Vec2Processor.from_pretrained(model_processor)
        else:
            self.model_processor = model_processor

    def get_features(self, audio_inp, sampling_rate=16000):

        input_values = self.model_processor(
            audio_inp, sampling_rate=sampling_rate, return_tensors="pt"
        ).input_values.to(self.model.device)

        with torch.no_grad():
            features = self.model(input_values).last_hidden_state

        return features

    def streaming_featurize(
        self, sampling_rate=16000, callback=print, **callback_kwargs
    ):

        with self._audio_streamer as audio_streamer:
            sampling_rate = getattr(audio_streamer, "processing_rate", sampling_rate)

            for i, stream in enumerate(audio_streamer.stream()):

                if stream is not None:
                    audio_inp = np.frombuffer(stream, np.float32)
                    features = self.get_features(audio_inp, sampling_rate=sampling_rate)

                    if features:
                        callback(features, **callback_kwargs)
