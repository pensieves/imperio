# imperio
Named after a charm in the Harry Potter Universe. When cast successfully, it places the other organism completely under the caster's control. In muggles' terminology, this is a repository of modules for interacting to and through an avatar or a tele-operated robot, including using Speech and Natural Language for action execution on robots.

## Getting Started:

### Installation:
*Install python modules mentioned in requirements.txt:*

`pyaudio` and `librosa`/`soundfile` have dependencies on `portaudio` and `libsndfile1`. For Ubuntu, the same can be installed by executing:

`sudo apt install portaudio19-dev libsndfile1`

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

### Using Docker:

Ensure that external installation files are present in the `docker/externals` folder. Execute the following commands from the project root directory i.e. current directory:

Make the build and run shell scripts executable by:
```
chmod +x docker/docker_build.sh
chmod +x docker/docker_run.sh
```

Now, to build the `imperio` docker image, execute:

`docker/docker_build.sh`

If required, update the ROS_IP and ROS_MASTER_URI environment variables in the `docker/run.env`. Finally, to run the `imperio` docker container, execute:

`docker/docker_run.sh`

Follow the running instructions for the required operation, e.g. `streaming-speech` for *operator (Caster)* to *robot/avatar (Imperiused)* direct speech relay.

### Running instructions:

- *Operator (Caster) to Robot/Avatar (Imperiused) direct speech relay or execution:* Receives speech input from operator and plays it back on robot's/avatar's side directly maintaining audio content as is, with gender conversion if specified. To relay speech without gender voice manipulation execute:

`python3 examples/streaming-speech.py`

To relay speech with gender voice manipulation, two praat voice conversion functions are supported - change_gender and change_pitch, with change_pitch being preferable. *To convert male voice to female, specify multiplier values > 1 and vice-versa for female to male voice conversion.* A sample command to execute is:

`python3 examples/streaming-speech.py --voice_conv_fn change_pitch --multiplier 1.5`

- *Operator (Caster) to Robot/Avatar (Imperiused) indirect speech relay or execution:* Receives speech input from operator and either plays it back on robot's/avatar's side, or if wake/context words such as `Asha` is recognized then execute the subsequent action specified.

`python3 examples/speech.py --ros_init`

for on-device Facebook's Wav2Vec2 model made available by Hugging Face. For using Google cloud's speech to text execute:

`python3 examples/google-speech.py --ros_init`

To modify the execution parameters of the on-device model such as providing GPU device index in case of availability, the program can be run as:

`python3 examples/speech.py --ros_init --gpu_idx 0`

- *Robot/Avatar (Imperiused) to Operator (Caster) audio reception:* Receives audio input from robot/avatar captured through its microphone and plays it back on speaker on operator's machine.

`python3 examples/audio.py`
