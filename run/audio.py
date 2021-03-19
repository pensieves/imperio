import rospy

import sys
from pathlib import Path
path = str(Path(__file__).parents[1].resolve())
sys.path.append(path)

from imperio.audio import AudioStreamPlayer

rospy.init_node("audio_stream_player")
AudioStreamPlayer()
rospy.spin()