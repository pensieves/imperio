from abc import ABCMeta, abstractmethod
from ..sonorus.audio import VADAudioInputStreamer
from .TextBatchProcessor import TextBatchProcessor


class BaseSTT(object, metaclass=ABCMeta):
    def __init__(
        self,
        lang="en-US",
        audio_streamer=None,
        text_batcher=None,
        text_batch_processor=None,
    ):

        self._lang = lang

        self._audio_streamer = (
            VADAudioInputStreamer() if audio_streamer is None else audio_streamer
        )

        self._text_batcher = text_batcher

        self._text_batch_processor = text_batch_processor
        if text_batch_processor is None:
            self._text_batch_processor = TextBatchProcessor(lang=lang)

    @abstractmethod
    def streaming_transcribe(self, **kwargs):
        r""" Code to transcribe specific to model or service."""
        pass

    def _process_text(self, text, reset=True):
        batch = None

        if self._text_batcher:
            batch = self._text_batcher.get_batch(text, reset=reset)

        elif reset:
            batch = [text]

        if batch:
            self._text_batch_processor.process(batch, reset=reset)

        if reset:
            print("--Final--\n", text, "\n---\n")
