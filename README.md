# G1Pilot

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-red)](
https://docs.ros.org/en/noetic/index.html)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)

G1Pilot is an open-source ROS2 package designed to provide full control of the Unitree G1 humanoid robot.
It offers two flexible control modes:

    Joint Controller – for precise low-level joint control

    Cartesian Controller – for intuitive Cartesian-space manipulation

In addition to control capabilities, G1Pilot continuously publishes essential real-time robot states, including:

    IMU data – for orientation and acceleration monitoring

    Odometry – for accurate localization

    Motor states – with temperature, voltage, position, and velocity feedback

With native RViz visualization support and a ready-to-use Docker environment, G1Pilot is ideal for teleoperation, robotics research, and motion analysis, enabling seamless interaction between ROS2 and the Unitree G1.


## Visual Overview
| **Joint Controller** | **Cartesian Controller** |
|---------------------|--------------------|
| <img src="https://github.com/hucebot/g1pilot/blob/main/images/joint_controller.gif" alt="Static Sensors" width="400"> | <img src="https://github.com/hucebot/g1pilot/blob/main/images/cartesian_controller.gif" alt="Moving Sensors" width="400"> |

## 🚀 Features
- 🎮 Dual Control Modes – Switch seamlessly between Joint and Cartesian controllers for maximum flexibility.

- 📡 Real-Time State Publishing – Continuously publishes essential robot data:

    - IMU orientation and acceleration

    - Odometry for localization

    - Detailed motor states (temperature, voltage, position, velocity)

- 🐳 Docker Ready – Quickly build and run the package using provided Docker scripts.

- 🔄 ROS2 Humble Support – Fully compatible with ROS2 Humble, ensuring smooth integration with modern robotics stacks.

- 🛠️ Open-Source & Extensible – Designed for researchers and developers to easily customize and extend functionality.

## Table of Contents
- [Pre-requisites](#pre-requisites)
- [Installation](#installation)
- [Nodes Overview](#-nodes-overview)
- [Usage](#usage)
- [Visual Overview](#visual-overview)
- [Features](#-features)
- [Contributing](#contributing)
- [License](#license)

## Pre-requisites
- For visualization, you need to install the [g1_description](https://github.com/hucebot/g1_description) package in the same directory as this package.
- Be connected to the robot via WiFi or Ethernet. **It's important to know which interface you are using.**

## Installation
We prepare a docker image to build the package. You can use the following command to build the package, go the `docker` folder and run the following command:

```bash
sh build.sh
```

Then, you can run the docker image with the following command:

```bash
sh run.sh
```

## 🧠 Nodes Overview

**G1Pilot** provides multiple ROS2 nodes to control and monitor the Unitree G1 robot.  

### 1️⃣ Cartesian Controller (`cartesian_controller`)  
- 🎯 **Purpose:** Controls the **right arm end-effector** of the robot.  
- 🔄 **Behavior:** Sends high-level Cartesian commands that are converted into joint movements via the **Joint Controller**.  
- 🛠️ **Use case:** Ideal for precise end-effector manipulation in Cartesian space.  

### 2️⃣ Joint Controller (`joint_controller`)  
- 🎯 **Purpose:** Directly controls the robot’s individual joints.  
- 🔄 **Behavior:** Receives low-level joint commands and sends them to the robot hardware for execution.  
- 🛠️ **Use case:** Required for fine-grained motion control and as a backend for Cartesian control.  

### 3️⃣ Interactive Marker (`interactive_marker`)  
- 🎯 **Purpose:** Publishes a **movable marker in RViz** to interactively control the robot’s end-effector.  
- 🛠️ **Use case:** Intuitive GUI control of the robot without manual command-line inputs.  

### 4️⃣ Robot State (`robot_state`)  
- 🎯 **Purpose:** Publishes the **complete state of the robot**, including:  
  - IMU readings  
  - Odometry  
  - Detailed motor states (temperature, voltage, position, velocity)  
- 🛠️ **Use case:** Used for monitoring and visualization in RViz or other tools.  

### 5️⃣ Loco Client (`loco_client`)  
- 🎯 **Purpose:** Enables **whole-body control and locomotion**, allowing the robot to move using **Unitree’s built-in policy**.  
- 🛠️ **Use case:** Autonomous or manual locomotion commands for walking and navigation.  

### 6️⃣ Joystick (`joystick`)  
- 🎯 **Purpose:** Integrates a **game controller (joystick)** to manually control the robot.  
- 🛠️ **Use case:** Remote teleoperation for research or demonstrations.  

#### **Basic Controls**
- **L1** → Emergency Stop → The robot enters **Damp Mode** (safe state).
- **⬆️ D-Pad Up** → Switches the robot to **FSM 4**, making it ready to receive commands.
- **R1** → Activates **Balance Mode**.
- **Left Joystick** → Controls **linear movements** (forward, backward, sideways).
- **Right Joystick** → Controls **angular rotation** (turning).

## Usage
Once you have the docker image running, you can run the following command to start the unitree node:

```bash
colcon build --symlink-install --packages-select g1pilot g1_description
````

Then, source the workspace:

```bash
source install/setup.bash
```
To visualize the real robot in RViz, you can run the following command:

```bash
ros2 run g1pilot ros_bridge --ros-args -p interface:=<your_interface>
```

To control the robot, using the joint controller, you can run the following command:
```bash
ros2 run g1pilot joint_controller --ros-args -p interface:=<your_interface>
```
To control the robot, using the cartesian controller, you can run the following command:
```bash
ros2 run g1pilot cartesian_controller --ros-args -p interface:=<your_interface>
```
```bash
ros2 run g1pilot interactive_marker
```

