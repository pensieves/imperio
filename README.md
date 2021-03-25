# imperio
Named after a charm in the Harry Potter Universe. When cast successfully, it places the other organism completely under the caster's control. In muggles' terminology, this is a repository of modules for interacting to and through an avatar or a tele-operated robot, including using Speech and Natural Language for action execution on robots.

## Getting Started:

### Installation:
*Install python modules mentioned in requirements.txt:*

`pyaudio` has a dependency on `portaudio`. If not using conda, make sure `portaudio` is installed. For example, for Ubuntu, the same can be installed by executing:

`sudo apt install portaudio19-dev`

Then install requirements by executing:

`pip install -r requirements.txt`

or install using conda in a conda environment.

*Install other external dependencies:*
- hr_msgs (from Hanson Robotics)
- Other, if any, vendor dependent installations for the installed ROS distribution.

### Environment set up:

*Source the ROS setup file:*

`source /opt/ros/noetic/setup.bash`

Sourcing ROS is required to access rospy in ROS 1 (or rclpy in ROS 2) and other ROS modules.

*Note:* Google Application Credentials is only required to be provided when using Google Cloud's speech to text api.

*Update the environment exporting file and source it:*

`source .env`

Or set environment variables explicitly by exporting e.g.: 
```
export GOOGLE_APPLICATION_CREDENTIALS=/path/to/google-cloud-credentials.json
export ROS_MASTER_URI=http://<ros_master_ip>:<ros_master_port>
export ROS_IP=<local_machine_ros_ip>
```

Refer .env file to see examples of environment values and format.

### Running instructions:

- *Robot/Avatar (Imperiused) to Operator (Caster) audio reception:* Receives audio input from robot/avatar captured through its microphone and plays it back on speaker on operator's machine.

`python3 run/audio.py`

- *Operator (Caster) to Robot/Avatar (Imperiused) speech relay or execution:* Receives speech input from operator and either plays it back on robot's/avatar's side, or if wake/context words such as `Asha` is recognized then execute the subsequent action specified.

`python3 run/speech.py --ros_init`

for on-device Facebook's Wav2Vec2 model made available by Hugging Face. For using Google cloud's speech to text execute:

`python3 run/google-speech.py --ros_init`

To modify the execution parameters of the on-device model such as stream_overlap and word_overlap for intermediate transcribed text correction and for providing GPU device index in case of availability, the program can be run as:

`python3 run/speech.py --ros_init --stream_count 100 --stream_overlap 10 --word_overlap 6 --gpu_idx 0`

### ToDo:
- [ ] Audio to Lip sync on robot/avatar.
- [ ] Speech Style Transfer for emotive and expressive text-to-speech (TTS) on robot/avatar based on operator's speech.