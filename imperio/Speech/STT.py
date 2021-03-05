from AudioStreamer import AudioStreamer
from BaseSTT import BaseSTT, SAMPLE_RATE, CHUNK, PA_FORMAT

import numpy as np
import torch
from transformers import Wav2Vec2Tokenizer, Wav2Vec2ForCTC

MODEL_IDENTIFIER = "facebook/wav2vec2-base-960h"
STREAM_COUNT = 50
STREAM_OVERLAP = 5
STREAM_SILENCE = 5


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
        stream_count=STREAM_COUNT,
        stream_overlap=STREAM_OVERLAP,
        stream_silence=STREAM_SILENCE,
    ):

        super(STT, self).__init__(
            rate, chunk, pa_format, lang, text_batcher, text_batch_processor
        )

        self._model = model
        if model == "default":
            self._model = Wav2Vec2ForCTC.from_pretrained(MODEL_IDENTIFIER)

        self._tokenizer = tokenizer
        if tokenizer == "default":
            self._tokenizer = Wav2Vec2Tokenizer.from_pretrained(MODEL_IDENTIFIER)

        self._stream_count = stream_count
        self._stream_overlap = stream_overlap
        self._stream_silence = stream_silence

    def transcribe(self, audio_inp, model=None, tokenizer=None):
        model = self._model if model is None else model
        tokenizer = self._tokenizer if tokenizer is None else tokenizer

        input_values = tokenizer(audio_inp, return_tensors="pt").input_values
        logits = model(input_values).logits
        predicted_ids = torch.argmax(logits, dim=-1)
        transcription = tokenizer.batch_decode(predicted_ids)[0]

        return transcription

    def streaming_transcribe(self, model=None, tokenizer=None):

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
                        audio_inp = audio_inp[-self._stream_overlap * len(audio_inp) :]

                    else:
                        audio_inp = np.array([], dtype=np.float32)
                        if text:
                            self._process_text(text, reset=True)
                        text = ""

    def _append_transcriptions(self, text, transcription):
        transcription = transcription.strip()
        if transcription:
            return text + " " + transcription
        return text


if __name__ == "__main__":
    from TextBatcher import TextBatcher
    from TextBatchPublisher import TextBatchPublisher

    import rospy

    rospy.init_node("STT")

    lang = "en-US"
    # lang = "en-IN"

    text_batcher = None
    # text_batcher = TextBatcher()

    # text_batch_processor = None
    text_batch_processor = TextBatchPublisher(lang=lang)

    STT(
        text_batcher=text_batcher,
        text_batch_processor=text_batch_processor,
        stream_overlap=0,
    ).streaming_transcribe()

    # rospy.spin()
