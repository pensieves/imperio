# imperio
Named after a charm in the Harry Potter Universe. When cast successfully, it places the other organism completely under the caster's control. In muggles' terminology, this is a repository of modules for interacting to and through an avatar or a tele-operated robot, including using Speech and Natural Language for action execution on robots.

## Getting Started:

### Installation:
*Install python modules mentioned in requirements.txt:*
`pip install -r requirements.txt`
or install using conda in a conda environment.

*Install other external dependencies:*
- hr_msgs (from Hanson Robotics)
- Other, if any, vendor dependent installations for the installed ROS distribution.

### Environment set up:

*Source the ROS setup file:*
`source /opt/ros/noetic/setup.bash`

Sourcing ROS is required to access rospy in ROS 1 (or rclpy in ROS 2) and other ROS modules.

*Update the environment exporting file and source it:*
`source .env`

OR set environment variables explicitly by exporting e.g.: 
```
export GOOGLE_APPLICATION_CREDENTIALS=/path/to/google-cloud-credentials.json
export ROS_MASTER_URI=http://<ros_master_ip>:<ros_master_port>
export ROS_IP=<local_machine_ros_ip>
```

Refer .env file to see examples of environment values and format.

### Running instructions:

*Robot/Avatar (Imperiused) to Operator (Caster) audio reception:*
`python3 AudioStreamPlayer.py`

*Operator (Caster) to Robot/Avatar (Imperiused) speech relay or execution:*
`python3 SpeechRecognizer.py`

### ToDo:
- [ ] Edge based speech-to-text (STT) implementation using facebook's word2vec2 model from Huggingface.
- [ ] Audio to Lip sync on robot/avatar.
- [ ] Speech Style Transfer for emotive and expressive text-to-speech (TTS) on robot/avatar based on operator's speech.