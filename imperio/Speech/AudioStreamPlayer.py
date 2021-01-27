import logging
import threading

from six.moves.queue import Queue

import pyaudio
import rospy

from std_msgs.msg import UInt8MultiArray

logger = logging.getLogger("hr.audio_stream.stream_player")

class AudioStreamPlayer(object):

    def __init__(self):
        self.CHUNK_SIZE = 8000
        self.left_chunk = b''
        self.sample_rate = rospy.get_param('audio_rate', 16000)

        self.buffer = Queue()
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.p.get_format_from_width(2),
                        channels=1,
                        rate=self.sample_rate,
                        output=True,
                        start=False)
        self.job = threading.Thread(target=self.run)
        self.job.daemon = True
        self.job.start()

        #audio_stream_topic = rospy.get_param('audio_stream_topic', 'speech_audio')
        audio_stream_topic = rospy.get_param('audio_stream_topic', '/hr/sensors/audio/speech_recognizer')
        rospy.Subscriber(audio_stream_topic, UInt8MultiArray, self.append_data)


    def append_data(self, msg):
        """Appends the audio chunk to data queue"""
        chunk = msg.data
        chunk = self.left_chunk+chunk
        if len(chunk) < self.CHUNK_SIZE:
            self.left_chunk = chunk
            return
        else:
            self.left_chunk = chunk[self.CHUNK_SIZE:]
            chunk = chunk[:self.CHUNK_SIZE]
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
        logger.info('closing audio stream')
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

if __name__ == '__main__':
    rospy.init_node('audio_stream_player')
    AudioStreamPlayer()
    rospy.spin()