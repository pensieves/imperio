from SpeechRecognizer import SpeechRecognizer
from TextStreamer import TextStreamer
from TextStreamPublisher import TextStreamPublisher

import rospy

rospy.init_node("speech_recognizer")
while True:
    try:
        # text_streamer = None
        text_streamer = TextStreamer()
        SpeechRecognizer(text_streamer=text_streamer).transcribe()
    except Exception as exception:
        print(exception)
# rospy.spin()