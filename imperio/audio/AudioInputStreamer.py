from six.moves import queue
import numpy as np
import pyaudio
import wave

from .utils import audio_resample


class AudioInputStreamer(object):
    r"""Streams raw audio from microphone/wav file. Data is received in a separate thread, 
    and stored in a buffer, to be read from."""

    # Audio recording parameters, subject to VAD/subsequent proceesing step
    SAMPLE_RATE = 16000
    CHANNELS = 1
    BLOCKS_PER_SECOND = 50
    PA_FORMAT = pyaudio.paFloat32
    FILE_CHUNK = 320

    FMT2TYPE = {
        pyaudio.paInt8: np.int8,
        pyaudio.paInt16: np.int16,
        pyaudio.paInt32: np.int32,
        pyaudio.paFloat32: np.float32,
    }

    def __init__(
        self,
        sample_rate=SAMPLE_RATE,
        blocks_per_second=BLOCKS_PER_SECOND,
        pa_format=PA_FORMAT,
        channels=CHANNELS,
        processing_rate=SAMPLE_RATE,
        device=None,
        file_path=None,
        file_chunk=FILE_CHUNK,
        callback=None,
    ):

        self.sample_rate = sample_rate
        self.processing_rate = processing_rate
        self.blocks_per_second = blocks_per_second
        self.channels = channels

        self.sample_block_size = sample_rate // blocks_per_second
        self.processing_block_size = processing_rate // blocks_per_second

        self.pa_format = pa_format
        self._buff = queue.Queue()
        self._callback = callback if callback is not None else self._fill_buffer

        self.device = device
        if file_path is not None:
            self.file_chunk = file_chunk
            self.file = wave.open(file_path, "rb")

        self.closed = True

    def _start(self):

        self._pa = pyaudio.PyAudio()

        kwargs = {
            "format": self.pa_format,
            "channels": self.channels,
            "rate": self.sample_rate,
            "input": True,
            "input_device_index": self.device,
            "frames_per_buffer": self.sample_block_size,
            "stream_callback": self._stream_callback,
        }

        self._audio_stream = self._pa.open(**kwargs)
        self.closed = False
        self._audio_stream.start_stream()

        return self

    def __enter__(self):
        return self._start()

    def _stop(self):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        """Signal the generator to terminate so that in case of web-based clients
        such as Google cloud speech to text, the streaming recognize method 
        will not block the process termination."""
        self._buff.put(None)
        self._pa.terminate()

        return self

    def __exit__(self, exception_type, exception_value, traceback):
        return self._stop()

    def _fill_buffer(self, in_data):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)

    def _stream_callback(self, in_data, frame_count, time_info, status_flags):
        r"""Callback to be passed to pyaudio"""
        if getattr(self, "file", None):
            in_data = self.file.readframes(self.file_chunk)
        self._callback(in_data)
        return None, pyaudio.paContinue

    def read_resampled(self):
        """Return a block of audio data resampled to 16000hz, blocking if necessary."""
        try:
            data = audio_resample(
                data=self._buff.get(),
                sample_rate=self.sample_rate,
                resample_rate=self.processing_rate,
                dtype=FMT2TYPE[self.pa_format],
            )
        except queue.Empty:
            data = None
        return data

    def read(self):
        """Return a block of audio data, blocking if necessary."""
        try:
            data = self._buff.get()
        except queue.Empty:
            data = None
        return data

    def _stream(self):
        while not self.closed:
            read_fn = (
                self.read
                if self.sample_rate == self.processing_rate
                else self.read_resampled
            )
            while True:
                yield read_fn()

    def stream(self):
        r"""Generator that yields series of consecutive audio frames."""
        yield from self._stream()
