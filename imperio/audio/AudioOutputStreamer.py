import threading

from six.moves.queue import Queue

import pyaudio
import rospy

from std_msgs.msg import UInt8MultiArray


class AudioOutputStreamer(object):
    def __init__(
        self,
        sample_rate=16000,
        channels=1,
        chunk_size=8000,
        topic="/hr/sensors/audio/speech_recognizer",
    ):
        r"""specify /binaural_audio as topic for binaural audio"""

        self.chunk_size = chunk_size
        self.left_chunk = b""
        self.sample_rate = rospy.get_param("audio_rate", sample_rate)

        self.buffer = Queue()
        self.p = pyaudio.PyAudio()

        self.stream = self.p.open(
            format=self.p.get_format_from_width(2),
            channels=channels,
            rate=self.sample_rate,
            output=True,
            start=False,
        )
        self.job = threading.Thread(target=self.run)
        self.job.daemon = True
        self.job.start()

        # audio_stream_topic = rospy.get_param('audio_stream_topic', 'speech_audio')
        audio_stream_topic = rospy.get_param("audio_stream_topic", topic)
        rospy.Subscriber(audio_stream_topic, UInt8MultiArray, self.append_data)

    def append_data(self, msg):
        """Appends the audio chunk to data queue"""
        chunk = msg.data
        chunk = self.left_chunk + chunk
        if len(chunk) < self.chunk_size:
            self.left_chunk = chunk
            return
        else:
            self.left_chunk = chunk[self.chunk_size :]
            chunk = chunk[: self.chunk_size]
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
