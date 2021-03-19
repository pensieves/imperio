import pyaudio
from abc import ABCMeta, abstractmethod
from .TextBatchProcessor import TextBatchProcessor

# Audio recording parameters
SAMPLE_RATE = 16000
CHUNK = int(SAMPLE_RATE / 10)
PA_FORMAT = pyaudio.paFloat32


class BaseSTT(object, metaclass=ABCMeta):
    def __init__(
        self,
        rate=SAMPLE_RATE,
        chunk=CHUNK,
        pa_format=PA_FORMAT,
        lang="en-US",
        text_batcher=None,
        text_batch_processor=None,
    ):

        self._rate = rate
        self._chunk = chunk
        self._pa_format = pa_format

        self._lang = lang
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
