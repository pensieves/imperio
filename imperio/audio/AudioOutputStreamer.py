import threading

from six.moves.queue import Queue

import pyaudio

from std_msgs.msg import UInt8MultiArray


class AudioOutputStreamer(object):

    CHUNK_SIZE = 8000
    SAMPLE_RATE = 16000
    CHANNELS = 1
    PA_FORMAT = pyaudio.paFloat32

    def __init__(
        self,
        chunk_size=CHUNK_SIZE,
        sample_rate=SAMPLE_RATE,
        channels=CHANNELS,
        pa_format=PA_FORMAT,
        device=None,
    ):

        self.chunk_size = chunk_size
        self.sample_rate = sample_rate

        self.left_chunk = b""
        self.buffer = Queue()

        self.channels = channels
        self.pa_format = pa_format
        self.device = device
        self.pa = pyaudio.PyAudio()

        self.audio_stream = self.pa.open(
            format=self.pa_format,
            channels=self.channels,
            rate=self.sample_rate,
            output=True,
            start=False,
            output_device_index=self.device,
        )

        self.job = threading.Thread(target=self.run)
        self.job.daemon = True
        self.job.start()

        # audio_stream_topic = rospy.get_param('audio_stream_topic', 'speech_audio')
        audio_stream_topic = rospy.get_param(
            "audio_stream_topic", "/hr/sensors/audio/speech_recognizer"
        )
        rospy.Subscriber(audio_stream_topic, UInt8MultiArray, self.append_data)

    def append_data(self, msg):
        """Appends the audio chunk to data queue"""
        chunk = msg.data
        chunk = self.left_chunk + chunk
        if len(chunk) < self.CHUNK_SIZE:
            self.left_chunk = chunk
            return
        else:
            self.left_chunk = chunk[self.CHUNK_SIZE :]
            chunk = chunk[: self.CHUNK_SIZE]
            self.buffer.put(chunk)

    def run(self):
        try:
            self.stream.start_stream()
            while True:
                data = self.buffer.get()
                self.stream.write(data)
        finally:
            self.close()

    def close(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
