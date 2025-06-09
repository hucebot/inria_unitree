#UNITREE_INRIA


[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-red)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)

ROS2 Package to read the information from the unitree robots using the SDK and convert them into ROS2 information.

## Table of Contents

## Pre-requisites
- For visualization, you need to install the [g1_description](https://github.com/hucebot/g1_description) package in the same directory as this package.
- Be connected to the robot via WiFi or Ethernet. **It's important to know which interdace you are using.**

## Installation
We prepare a docker image to build the package. You can use the following command to build the package, go the `docker` folder and run the following command:

```bash
sh build.sh
```

Then, you can run the docker image with the following command:

```bash
sh run.sh
```

## Usage
Once you have the docker image running, you can run the following command to start the unitree node:

```bash
colcon build --symlink-install --packages-select inria_unitree g1_description
````

Then, source the workspace:

```bash
source install/setup.bash
```
Now you can run the unitree node:

```bash
ros2 run inria_unitree ros_bridge --ros-args -p interface:=<your_interface>
```