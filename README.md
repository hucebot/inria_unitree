# UNITREE_INRIA


[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-red)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)

ROS2 Package to control the  Unitree G1 robot. This package provides different ways to control the robot.


## Visual Overview
| **Joint Controller** | **Cartesian Controller** |
|---------------------|--------------------|
| <img src="https://github.com/hucebot/inria_unitreeblob/main/images/joint_controller.gif" alt="Static Sensors" width="400"> | <img src="https://github.com/hucebot/inria_unitreeblob/main/images/cartesian_controller.gif" alt="Moving Sensors" width="400"> |

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
To visualize the real robot in RViz, you can run the following command:

```bash
ros2 run inria_unitree ros_bridge --ros-args -p interface:=<your_interface>
```

To control the robot, using the joint controller, you can run the following command:
```bash
ros2 run inria_unitree joint_controller --ros-args -p interface:=<your_interface>
```
To control the robot, using the cartesian controller, you can run the following command:
```bash
ros2 run inria_unitree cartesian_controller --ros-args -p interface:=<your_interface>
```
```bash
ros2 run inria_unitree interactive_marker
```