import threading
from six.moves import queue
import pyaudio
import time


class AudioOutputStreamer(object):

    SAMPLE_RATE = 16000
    CHUNK = 8000

    def __init__(
        self,
        sample_rate=SAMPLE_RATE,
        chunk=CHUNK,
        device=None,
        pre_process=lambda x: x,
    ):

        self.sample_rate = sample_rate
        self.chunk = chunk
        self.left_bytes = b""

        self._buff = queue.Queue()
        self.device = device

        self.pre_process = pre_process

    def _start(self):
        self._pa = pyaudio.PyAudio()

        kwargs = {
            "format": self._pa.get_format_from_width(2),
            # "format": pyaudio.paInt16,
            "channels": 1,
            "rate": self.sample_rate,
            "output": True,
            "output_device_index": self.device,
            "start": False,
        }

        self._audio_stream = self._pa.open(**kwargs)

        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()
        self.thread_stop = threading.Event()

        return self

    def __enter__(self):
        return self._start()

    def _stop(self):
        self.thread_stop.set()
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self._pa.terminate()

        return self

    def __exit__(self, exception_type, exception_value, traceback):
        return self._stop()

    def append_buff(self, out_data):
        """Appends the audio chunk to data queue"""
        out_data = self.pre_process(out_data)
        out_data = self.left_bytes + out_data

        if len(out_data) < self.chunk:
            self.left_bytes = out_data

        else:
            self.left_bytes = out_data[self.chunk :]
            out_data = out_data[: self.chunk]
            self._buff.put(out_data)

    def stream(self, in_streamer):

        self.processed_buff = False

        for in_data in in_streamer:
            self.append_buff(in_data)

        while not self.processed_buff:
            time.sleep(0.01)

    def run(self):
        self._audio_stream.start_stream()
        while not self.thread_stop.is_set():
            self._audio_stream.write(self._buff.get())
            if self._buff.empty():
                self.processed_buff = True
