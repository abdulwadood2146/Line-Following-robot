# Line Following Robot (ROS 2)

This project implements a **Line Following Robot** using **ROS 2**, designed to autonomously follow a path using a camera.
The robot captures images of the ground in real-time, detects a line, and adjusts its steering and speed to stay on course.

## Features

- Real-time line detection using camera input
- ROS 2 node-based architecture for modularity
- Smooth line-following control logic
- Compatible with simulation and real-world deployment

## Requirements

- ROS 2 (Humble or later)
- Python 3
- OpenCV
- Gazebo (for simulation)
- A robot with a camera (for real-world use)

## Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/abdulwadood2146/Line-Following-robot.git
cd ..
colcon build
source install/setup.bash

## Launch Gazebo Simulation

To launch the Gazebo simulation, use the following command:

```bash
ros2 launch simple_robot_description gazebo.launch.py
```
## View Camera Feed

To view the camera feed, run the following command:

```bash
ros2 run image_tools showimage --ros-args --remap image:=/camera1/image_raw
```
## Run Line Follower Script

To execute the line follower script, use the following command:

```bash
ros2 run simple_robot_description joint_commander
```
