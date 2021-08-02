import rospy

import sys
from pathlib import Path

path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

from imperio.audio import AudioOutputStreamer

rospy.init_node("audio_output")
AudioOutputStreamer()
rospy.spin()
