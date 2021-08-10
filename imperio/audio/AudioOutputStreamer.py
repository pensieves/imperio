import threading

from six.moves.queue import Queue

import pyaudio
import rospy

from std_msgs.msg import UInt8MultiArray


class AudioOutputStreamer(object):
    def __init__(self, chunk_size=8000, sample_rate=16000):
        self.chunk_size = chunk_size
        self.left_chunk = b""
        self.sample_rate = rospy.get_param("audio_rate", sample_rate)

        self.buffer = Queue()
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=self.p.get_format_from_width(2),
            channels=1,
            rate=self.sample_rate,
            output=True,
            start=False,
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
