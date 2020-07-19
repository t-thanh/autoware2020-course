# Autoware.auto Lecture 11: LGSVL Simulation

## Self-Driving Cars w/ROS & Autoware (hosted by Apex.AI)

![](Autoware.Auto_Lecture_11_LGSVL_Simulation.png)

## 0. <a name="introduction">Introduction</a>

_Refer to related video or slides for additional information_

### Intro
Presented by Steve Lemke, Principal Engineer @ LG America R&D Lab

### Outline
0. [Introduction and features of the Simulator](#introduction) (slides 2-13)
1. [Installation of the Simulator](#installation) (slides 14-16)
2. [Getting Started, User Interface, Configuration](#ui) (slides 17-24 and hands-on)
3. [Running Simulation with Autoware.Auto](#autoware) (hands-on)
4. [Automation and Python API](#pythonapi) (hands-on)
5. [Advanced Topics](#advanced) (tutorial videos)
6. [Additional Information](#moreinfo) (slide 25)


### Key Features
* Unity 3D engine
	* High Definition Render Pipeline
	* Physics-based modeling of multiple lighting sources
* Out of the box integration with Autoware.AI and Autoware.Auto (and other AD stacks)
* ROS, ROS2, CyberRT bridges
	* Focus of this lecture is Autoware.Auto and thus ROS2 
* Physical Sensors: Cameras, LiDAR, Radar, GPS, IMU
* Virtual Sensors: 2D and 3D Ground Truth, Depth and Segmentation Cameras
* Road/map annotation editor
	* Import of HD maps from: Lanelet2, OpenDRIVE, Apollo
	* Export of HD maps to: Lanelet2, Autoware Vector, OpenDRIVE, Apollo

## 1. <a name="installation">Installation of the Simulator</a>

### Goal: Understand simulator requirements and installation

1. [System Requirements](https://www.lgsvlsimulator.com/docs/getting-started/#getting-started)
	* 4GHz Quad core CPU
	* Nvidia GTX-1080 (8GB memory)
	* Windows 10 64-bit (or Ubuntu ~18.04)

2. [GPU drivers and libraries](https://www.lgsvlsimulator.com/docs/getting-started/#downloading-and-starting-simulator)
	* Install latest Nvidia drivers
	* Install libvulkan (on Linux) 
	* Install nvidia-smi (on Linux)

3. [Downloading and installing](https://www.lgsvlsimulator.com/docs/getting-started/#downloading-and-starting-simulator)
	* Click Download button on [LGSVLSimulator.com](https://lgsvlsimulator.com)
	* Or find release notes at [https://github.com/lgsvl/simulator/releases/latest](https://github.com/lgsvl/simulator/releases/latest)

4. [Building from source](https://www.lgsvlsimulator.com/docs/build-instructions/)
	* Download and install Unity Hub
	* Download and install Unity 2019.3.3f1
	* Download and install Node.js
	* Verify git-lfs installation
	* Clone simulator source from GitHub
	* More on this later in "Advanced Topics"


## 2. <a name="ui">Getting Started</a>

### Goal: Understand simulator user interface for configuring maps, vehicles, and other settings

1. Basic simulator concepts: Refer to [Getting Started doc](https://www.lgsvlsimulator.com/docs/getting-started/#simulator-instructions) or the lecture slides

	1. [Maps](https://www.lgsvlsimulator.com/docs/maps-tab/)
		* There are several pre-loaded reference maps including:
			* BorregasAve
			* AutonomouStuff
			* GoMentum
		* Other maps available at [https://content.lgsvlsimulator.com/](https://content.lgsvlsimulator.com/)

	2. [Vehicles](https://www.lgsvlsimulator.com/docs/vehicles-tab/)
		* There are several pre-loaded reference vehicles including:
			* Jaguar2015XE (Autoware)
			* Lexus2016RXHybrid (Autoware)
				* We could just change ROS to ROS2 but instead let's create a new vehicle:
		* Navigate to content site: [https://content.lgsvlsimulator.com/](https://content.lgsvlsimulator.com/)
			* Scroll to "Vehicles" and click "View All"
			* Click on [Lexus RX 2016](https://content.lgsvlsimulator.com/vehicles/lexusrx2016/)
			* Right-click to copy `Asset Bundle` link location
			* Close Lexus browser window
			* Return to LGSVL Vehicles web UI
		* Click "Add New" vehicle
			* Enter Vehicle Name: `Lexus2016RXHybrid (Autoware.Auto/ROS2)`
			* Paste copied Lexus URL
			* Click "Submit" to save new vehicle
		* Navigate to example [Autoware.Auto json config](https://www.lgsvlsimulator.com/docs/autoware-auto-json-example/)
			* Scroll to "Complete JSON Configuration" field
			* Click "Copy" to copy JSON Configuration text
		* Click wrench icon to edit vehicle configuration
			* Select Bridge Type: `ROS2`
			* Paste copied JSON Configuration text into "Sensors" field
			* Click "Submit" to save vehicle configuration
	3. [Clusters](https://www.lgsvlsimulator.com/docs/clusters-tab/)
		* Local Machine: Default (single machine simulation)
		* Local Cluster: See [Cluster Simulation Introduction](https://www.lgsvlsimulator.com/docs/cluster-simulation-introduction/)
	4. [Simulations](https://www.lgsvlsimulator.com/docs/simulations-tab/)
		* BorregasAve (with Autoware)
			* We could just select our new vehicle but let's create a new simulation:
		* Click "Add New" Simulation
			* General tab:
				* Enter Simulation Name: `BorregasAve (Autoware.Auto/ROS2)`
				* Select Cluster: `Local Machine`
				* API Only: leave unchecked
				* Headless Mode: leave unchecked
			* Map & Vehicles tab:
				* Interactive Mode: check this for interactive controls
				* Select Map: `BorregasAve`
				* Select Vehicle: `Lexus2016RXHybrid (Autoware.Auto/ROS2)`
				* Enter "Bridge Connection String": `localhost:9090`
				* We could click "+" to add additional ego vehicles/bridges
			* Traffic tab:
				* Use Predetermined Seed: leave unchecked
				* Random Traffic: check to have NPC vehicles spawn at beginning of simulation
				* Random Pedestrians: check to have pedestrians spawn at beginning of simulation
			* Weather tab:
				* Set time of day during simulation
				* Set rain, wetness, fog, and cloudiness
			* Click "Submit" to save new simulation


2. [How to start simulation](https://www.lgsvlsimulator.com/docs/simulation-menu/)

	* Click in `BorregasAve (Autoware.Auto/ROS2)` to select simulation (indicated by check mark)
	* Click "Play" icon to begin simulation

3. [Controlling the simulator](https://www.lgsvlsimulator.com/docs/simulation-menu/#controls-menu)

	1. Start/Pause Simulation (only necessary in Interactive Mode)
		* Click "Play" button
	2. Show Menu Layer
		* Click "Menu" button (if NON-Interactive Mode)
	3. Information
		* Build information, and logged warnings and errors
	4. Controls
		* [Keyboard shortcuts](https://www.lgsvlsimulator.com/docs/keyboard-shortcuts/) 
	5. Bridge Info
		* Bridge address, status, and topic list
	6. Environment (only in Interactive Mode)
		* [Simulation parameters](https://www.lgsvlsimulator.com/docs/simulation-menu/#interactive-menu)
	7. Visualize
		* Color, Depth, Semantic Cameras
		* LiDAR, RADAR
		* 2D and 3D Ground Truth


## 3. <a name="autoware">Running Simulation with Autoware.Auto</a>

### Goal: Use the simulator with ADE, ROS2 bridge, and RViz

1. [Set up Docker, ADE, and ROS2](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/)

	* Should already have installed latest Nvidia drivers (above) before installing Simulator

	* Install Docker CE and Nvidia Docker as described in [Autoware.Auto with LGSVL Simulator: Setup](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#setup)

	* Make sure `nvidia-smi` works on host and in docker:
		* Confirm Nvidia drivers with:

			`nvidia-smi`

		* Confirm nvidia-docker with:

			`nvidia-docker run nvidia/cuda:10.0-base nvidia-smi`
			
		* To monitor GPU stats (including memory usage), type:

			`watch -n1 nvidia-smi`

	* Install ADE (and ROS2 dashing)
		* Refer to [Autoware Class 2020 Lecture 1](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/01_DevelopmentEnvironment/devenv.md) if needed

	* At this point ade and ros2 should be installed and working
		* Refer to [Autoware Class 2020 Lecture 2](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/02_ROS2_101/Lesson2Basics.rst) and [Autoware Class 2020 Lecture 3](https://gitlab.com/ApexAI/autowareclass2020/-/blob/master/lectures/03_ROS_Tooling/Lesson3CLI.rst) if needed


2. [Set up ROS2 bridge](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#install-ros2-web-bridge)

	* Clone the [ROS2 web bridge](https://github.com/RobotWebTools/ros2-web-bridge):

		```bash
		cd ~/adehome/AutowareAuto
		ade start -- --net=host --privileged # to allow connect to rosbridge
		ade enter
		git clone -b 0.2.7 https://github.com/RobotWebTools/ros2-web-bridge.git
		```
		
	* Install nodejs v10:
	
		```bash
		curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
		sudo apt install -y nodejs
		cd ros2-web-bridge
		npm install    # If node.js packages are not installed, run this.
		```
	
	* Install [lgsvl_msgs](https://www.lgsvlsimulator.com/docs/lgsvl-msgs/)

		* `lgsvl_msgs` is a ROS / ROS2 hybrid package that provides AD stack-agnostic message definitions for interfacing with the LGSVL Simulator

		* Clone repository and build with colcon:

			```bash
			git clone https://github.com/lgsvl/lgsvl_msgs.git
			cd lgsvl_msgs
			colcon build
			```

		* Source in terminal where rosnodes who publish / subscribe to the messages are running:

			```bash
			source install/setup.bash
			```

3. [Run Simulator alongside Autoware.Auto](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/#run-simulator-alongside-autoware-auto)

	* Start the Autoware.Auto containers:

		```bash
		cd ~/adehome/AutowareAuto
		ade start -- --net=host --privileged # to allow connect to rosbridge
		```

	* Enter the container and start rviz2:

		```bash
		ade enter
		cd ~/AutowareAuto
		colcon build    # If you want to use autoware_auto_msgs, ros2-web-bridge needs to compile them
		export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/nvidia/lib64/
		source ~/AutowareAuto/install/local_setup.bash
		rviz2 -d /home/"${USER}"/AutowareAuto/install/autoware_auto_examples/share/autoware_auto_examples/rviz2/autoware.rviz
		```

	* Start (resume) the LGSVL Simulator:
		* Note this is LGSVL Simulator, downloaded from [LGSVLSimulator.com](LGSVLSimulator.com), version 2020.05
		* Launch the executable and click on the button to open the web UI
		* Click "Simulations" to view the available/configured simulations
		* Click in `BorregasAve (Autoware.Auto/ROS2)` simulation to select it (indicated by check mark)
		* Click "Play" icon to begin simulation

	* Launch ROS2 web bridge (in a new shell)

		**NOTE:** [Node.js](#node-js) will need to be re-installed (above) in the container after ADE is started

		```bash
		ade enter      # ros2 web bridge should be run in ade environment
		cd ros2-web-bridge
		source ~/AutowareAuto/install/local_setup.bash
		node bin/rosbridge.js
		```

	* In RViz window:

		* Under "Global Options", set "Fixed Frame" to `lidar_front`
		* Expand "Tranformed Points" and set "Topic" to `/lidar_front/points_raw`

		You should now be able to see the lidar point cloud in RViz

3. [Different sensor configurations](https://www.lgsvlsimulator.com/docs/sensor-json-options/)

	* What is a sensor?

		* A `SENSOR` is defined in the JSON configuration in the following format:
		
			```output
			{
			    "type": STRING,
			    "name": STRING,
			    "params": {PARAMS},
			    "parent": STRING,
			    "transform": {
			      "x": FLOAT,
			      "y": FLOAT,
			      "z": FLOAT,
			      "pitch": FLOAT,
			      "yaw": FLOAT,
			      "roll": FLOAT,
			    }
			}
			```

	* Sensor fields are explained in [How to specify a sensor](https://www.lgsvlsimulator.com/docs/sensor-json-options/#how-to-specify-a-sensor)
	* Supported sensor list:

		* Color Camera, Depth Camera, Segmentation Camera
		* Lidar
		* Radar
		* 2D Ground Truth, 2D Ground Truth Visualizer
		* 3D Ground Truth, 3D Ground Truth Visualizer
		* CAN-Bus
		* GPS Device, GPS Odometry, GPS-INS Status
		* Vehicle Control, Keyboard Control, Wheel Control, Manual Control, Cruise Control
		* IMU, Clock, Control Calibration
		* Transform Sensor, Signal Sensor

	* Examine [Autoware.Auto example sensor file](https://www.lgsvlsimulator.com/docs/autoware-auto-json-example/) from [Example Configurations](https://www.lgsvlsimulator.com/docs/sensor-json-options/#examples)

		* Transform Sensor
		* CAN-Bus
		* GPS Device, GPS Odometry
		* IMU
		* Lidar
		* Vehicle Control, Vehicle State, Vehicle Odometry
		* Keyboard Control

	* Review sensor configuration for Autoware.Auto/ROS2 vehicle in Simulator UI


4. Add camera viewer

	* Install and launch camera decompression transport (in a new ADE shell)

		```bash
		ade enter
		sudo apt update
		sudo apt install ros-dashing-compressed-image-transport
		source ~/AutowareAuto/install/local_setup.bash
		ros2 run image_transport republish compressed in/compressed:=/simulator/camera_node/image/compressed raw out:=/image_raw
		```

	* In RViz window:

		* Add image viewer on the `/image_raw` topic

	* Why don't we see anything in the image viewer?
		* Our car doesn't have a camera sensor
		* Let's add one!

			```JSON
			{				"type": "Color Camera",				"name": "Main Camera",				"params": {					"Width": 1920,					"Height": 1080,					"Frequency": 15,					"JpegQuality": 75,					"FieldOfView": 50,					"MinDistance": 0.1,					"MaxDistance": 1000,					"Topic": "/simulator/camera_node/image/compressed",					"Frame": "camera"				},				"transform": { "x": 0, "y": 1.7, "z": -0.2, "pitch": 0, "yaw": 0, "roll": 0 }
			},
			```

		* Stop and re-start the simulator from the web UI

		* Now we should see live camera images in the in RViz image viewer


5. [Visualizing sensor information](https://www.lgsvlsimulator.com/docs/sensor-visualizers/)

	* Adding JSON sensor parameters (for ground truth or other sensors)

		* Depth Camera and Segmentation Camera
		* 2D and 3D Ground Truth
		* Color Camera, LiDAR, Radar

	* Visualizing ground truth sensors
		* Add 3D ground truth sensor

			```JSON
			{
			    "type": "3D Ground Truth",
			    "name": "3D Ground Truth",
			    "params": {
			      "Frequency": 10,
			      "Topic": "/simulator/ground_truth/3d_detections"
			    },
			    "transform": { "x": 0, "y": 1.975314, "z": -0.3679201, "pitch": 0, "yaw": 0, "roll": 0 }
			}
			```

		* Stop and re-start the simulator from the web UI

		* We should be able to enable sensor visualizations including 3D Ground Truth


6. Troubleshooting tips and [FAQ](https://www.lgsvlsimulator.com/docs/faq/)

	* Simulator shows "disconnected" from bridge?
		* Try `telnet localhost 9090` from shell (with and without bridge running)
			* Send a character (e.g. "5"); should return error
		* Check "Player.log" file and search for "bridge"
			* `gedit "$HOME/.config/unity3d/LG Silicon Valley Lab/LGSVL Simulator/Player.log"`
		* Remove old ".db" files, re-install Simulator, re-launch, and re-configure

			```bash
			rm "$HOME/.config/unity3d/LG Silicon Valley Lab/LGSVL Simulator/data.db"
			rm "$HOME/.config/unity3d/LG Silicon Valley Lab/LGSVL Simulator/backup.db"			```

		* Reboot system and restart ADE (and remember to reinstall node.js)

		* If all else fails, post an issue on the LGSVL GitHub issues page:
			* [https://github.com/lgsvl/simulator/issues/](https://github.com/lgsvl/simulator/issues/)


## 4. <a name="pythonapi">Automation and Python API</a>

### Goal: Learn how to control the simulator using the Python API

1. [Controlling environment using Python API](https://www.lgsvlsimulator.com/docs/python-api/)

	1. [Quickstart](https://www.lgsvlsimulator.com/docs/python-api/#quickstart)

		* Select pythonapi (virtual environment)
			* Check out [virtualenvwrapper](https://virtualenvwrapper.readthedocs.io/en/latest/)
			* Create virtualenv with: ```mkvirtualenv -p python3 pythonapi```
			* Activate with: ```workon pythonapi```
	
		* Download and Install Python API
	
			```bash
			cd ~
			git clone https://github.com/lgsvl/PythonAPI.git
			cd PythonAPI
			pip3 install --user -e .
			```
			
			(Leave out `--user` if working in a python virtual environment)
	
		* Launch Simulator
		* Review "API Only" Simulation settings
		* Start "API Only" mode simulation
			* "Open Browser..." changes to "API Ready!"
		* Let's try one of the quickstart scripts to confirm that everything is working:
	
			```bash
			./quickstart/05-ego-drive-in-circle.py
			```

	2. [Core Concepts](https://www.lgsvlsimulator.com/docs/python-api/#core-concepts): Simulator and Agents
		* Show doc page; intro to Python API concepts

2. [Quickstart Example Scripts](https://www.lgsvlsimulator.com/docs/api-quickstart-descriptions/)

	1. 	[Simulation](https://www.lgsvlsimulator.com/docs/python-api/#simulation): Connection, execution flow, and time scale
		* Run script 1: connecting-to-simulator
		* Run script 22: connecting-bridge
		* Run script 24: ego-drive-straight-non-realtime

	2. 	[Agents](https://www.lgsvlsimulator.com/docs/python-api/#agents): Ego vehicle, NPC vehicles, and Pedestrians
		* Run script 5: ego-drive-in-circle
		* Run script 10: npc-follow-the-lane
		* Run script 12: create-nps-on-lane

	3. 	[Callbacks](https://www.lgsvlsimulator.com/docs/python-api/#callbacks): Ego vehicle, NPC vehicles, and Pedestrians
		* Run script 11: collision-callbacks

	4. 	[Sensors](https://www.lgsvlsimulator.com/docs/python-api/#sensors): Enabling/disabling sensors and saving sensor data
		* Run script 6: save-camera-image
		* Run script 7: save-lidar-point-cloud
		* Run script 20: enable-sensors

	5. 	[Weather and Time of Day](https://www.lgsvlsimulator.com/docs/python-api/#weather-and-time-of-day-control): Modifying weather parameters and time of day
		* Run script 18: weather-effects
		* Run script 19: time-of-day

	6. [Controllable Objects](https://www.lgsvlsimulator.com/docs/python-api/#controllable-objects)
		* See [Controllable Plugins](https://www.lgsvlsimulator.com/docs/controllable-plugins/) to build your own controllable objects
		* Run script 28: control-traffic-cone

	7. [Helper Functions](https://www.lgsvlsimulator.com/docs/python-api/#helper-functions)
		* Simulator class offers several useful helper functions:
			* `version`
			* `current_scene`, `current_frame`, `current_time` 
			* `get_spawn` and `get_agents`
		* Simulator class also offers functions to map points in Unity coordinates to GPS coordinates:
			* `map_to_gps`, `map_from_gps`, and `raycast`


3. [Python API Use Case Examples](https://www.lgsvlsimulator.com/docs/api-example-descriptions/)

	* Several provided examples of e.g. NHTSA test scenarios include:
		* Basic sample scenarios
		* [Vehicle following](https://www.lgsvlsimulator.com/docs/api-example-descriptions/#vehicle-following)
		* Encroaching Oncoming Vehicle

4. [Sensor plug-ins](https://www.lgsvlsimulator.com/docs/sensor-plugins/)

	* Refer to CustomCameraSensor example in [sensor-plugins](https://www.lgsvlsimulator.com/docs/sensor-plugins/) documentation

5. [Custom Sensor Plugin example: Judging ride comfort](https://github.com/lgsvl/ComfortSensor)
 
	* Refer to ComfortSensor example on [github](https://github.com/lgsvl/ComfortSensor)


## 5. <a name="advanced">Advanced Topics</a>

### Goal: Learn how to build the simulator from source, and create new environments, vehicles, and sensors

1. Building Simulator from source in Unity

	* [Build Instructions](https://www.lgsvlsimulator.com/docs/build-instructions/)
	* New Video Tutorial on YouTube: [How to build LGSVL Simulator from source](https://youtu.be/EsCVYI6mhL4)
	* See also [Adding and Building Assets](https://www.lgsvlsimulator.com/docs/assets/)

2. New environment creation

	* Refer to [Adding and Building Assets](https://www.lgsvlsimulator.com/docs/assets/)
	* Video Tutorial on YouTube: [How to create environments for LGSVL Simulator](https://youtu.be/S0w2ahhEPbE)

3. New vehicle creation

	* Refer to [Adding and Building Assets](https://www.lgsvlsimulator.com/docs/assets/)
	* Video Tutorial on YouTube: [How to create a new vehicle asset for LGSVL Simulator](https://youtu.be/R_Z65kimCII)

4. New sensor creation

	* Stay tuned for this (and other) new video tutorials (watch the blog)

5. Map annotation

	* Refer to [https://www.lgsvlsimulator.com/docs/map-annotation/](https://www.lgsvlsimulator.com/docs/map-annotation/)
	* Stay tuned for this (and other) new video tutorials (watch the blog)


## 6. <a name="moreinfo">Helpful Links</a>
* LGSVL Simulator website: [LGSVLSimulator.com](https://www.lgsvlsimulator.com/)
* LGSVL Simulator blog: [https://www.lgsvlsimulator.com/blog/](https://www.lgsvlsimulator.com/blog/)
* LGSVL Simulator on GitHub: [https://github.com/lgsvl/simulator](https://github.com/lgsvl/simulator)
* LGSVL Simulator on YouTube: [https://www.youtube.com/c/LGSVLSimulator/videos](https://www.youtube.com/c/LGSVLSimulator/videos)
* LGSVL Simulator with Autoware.Auto: [https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/](https://www.lgsvlsimulator.com/docs/autoware-auto-instructions/)