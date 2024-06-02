# :rocket: Nicla Vision ROS2 package :rocket: 
A ROS2 interface for the Arduino Nicla Vision board

![Alt Text](assets/Nicla_ROSpkg_Architecture.png)

-------------------

## Description

This ROS package enables the [Arduino Nicla Vision](https://docs.arduino.cc/hardware/nicla-vision/) board to be ready-to-use in the ROS world! :boom:

The implemented architecture is described in the above image: the Arduino Nicla Vision board streams the sensors data to a ROS-running machine through TCP/UDP socket. This package will be running on the ROS-running machine, allowing to deserialize the received info, and stream it in the corresponding ROS topics 

Here a list of the available sensors with their respective ROS topics:
- **2MP color camera** streams on
    - `/nicla/camera/camera_info` 
    - `/nicla/camera/image_raw`
    - `/nicla/camera/image_raw/compressed` 
- **Time-of-Flight (distance) sensor** streams on:
    - `/nicla/tof`
- **Microphone** streams on:
    - `/nicla/audio` 
    - `/nicla/audio_info`
    - `/nicla/audio_stamped`
- **Imu** streams on:
    - `/nicla/imu`

The user can choose if this package should receive the sensors data by UDP or TCP socket connection, providing just an ip address. Moreover, the user can decide which sensor to stream within the ROS environment. 
In this repository you can find the Python code optimised for receiving the data by the board, and subsequently publishing them through ROS topics.

## Table of Contents 
1. [Installation](#installation)
2. [Usage](#usage)
3. [Package List](#package-list)
4. [Video](#video-demonstration)
5. [License](#license)
6. [Cite](#citation)
   
-------------------

# Installation
Step-by-step instructions on how to get the ROS package running (tested on ROS Noetic).

```bash
$ cd <your_workpace>/src
$ git clone https://github.com/ADVRHumanoids/nicla_vision_ros2.git
$ cd <your_workpace>
$ colcon build
$ source <your_workpace>/devel/setup.bash
```

### Note
We rely on the [audio_common_msgs](https://github.com/ros-drivers/audio_common/tree/master/audio_common_msgs) for the microphone data. Nevertheless, such package is not available as binary for ros2 yet. Hence, for now, we have copy-pasted the messages in this repo itself.

**Additional Note:** binary package will be released soon for ROS2!

# Usage 
Follow the following two steps for enjoying your Arduino Nicla Vision board with ROS.
### 1. Run the ROS package
- For receiving sensors data from the board and 
    ```bash
    $ ros2 launch nicla_vision_ros2 nicla_receiver.launch receiver_ip:="x.x.x.x" <optional arguments>
    ```
    Set the `receiver_ip` with the ip address of your ROS-running machine.
    You can get it by running the command:
    ```bash
    $ ifconfig
    ```
    and taking the inet address under the enp voice.
  
    Using the `<optional arguments>`, you can decide which sensor to simulate and which socket type, TCP or UDP,  to use (connection_type:="tcp" or "udp").
    
- For simulating the Arduino Nicla Vision in Gazebo and Rviz:
     ```bash
    $ ros2 launch nicla_vision_ros nicla_sim.py <optional arguments>
    ```
    Using the `<optional arguments>`, you can decide if to run the simulation in gazebo or in rviz, and which sensor to simulate (everything set to true as default). 
    
### 2. Arduino Nicla Vision setup
Turn on your Arduino Nicla Vision, after having completed its setup following the steps in the [Nicla Vision Drivers repository](https://github.com/ADVRHumanoids/nicla_vision_drivers.git). 
When you power on your Arduino Nicla Vision, it will automatically connect to the network and it will start streaming to your ROS-running machine.

# Video Demonstration

https://github.com/ADVRHumanoids/nicla_vision_ros/assets/63496571/699b265c-3f6a-4a9d-8d6c-fccf6bd39227

# Package List
Here some useful links:

- [Nicla Vision Drivers repository](https://github.com/ADVRHumanoids/nicla_vision_drivers.git)
- [Nicla Vision ROS repository](https://github.com/ADVRHumanoids/nicla_vision_ros.git)
- [Nicla Vision ROS2 repository](https://github.com/ADVRHumanoids/nicla_vision_ros2.git)

# License
Distributed under the Apache-2.0 License. See LICENSE for more information.

# Citation 
:raised_hands: If you use this work or take inspiration from it, please cite (publication in progress):
```bash
@inproceedings{delbianco2024dagana,
  title={},
  author={},
  booktitle={},
  year={}
}
```
