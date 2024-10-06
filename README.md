# Trajectory Tracking and Replication

This project aims to track the trajectory of a moving object using a depth camera and replicate its motion using a drone. The system combines real-time depth sensing, object detection, trajectory estimation, and drone control to achieve this goal.

## Introduction

In this project, I used a depth camera to capture the real-time movement of an object and estimate its trajectory. The captured trajectory is then used to command a drone to replicate the same movement path in a 3D space. This system can be applied in various fields such as object following, surveillance, and industrial automation.

## Features

- Real-time object tracking using depth camera technology
- Trajectory estimation to track the precise movement path of the object
- Drone path replication based on the tracked object’s movement
- Modular architecture with components for object detection, trajectory tracking, and drone control

## Hardware Requirements

- **Depth Camera** - Intel RealSense D435
- **Drone** - compatible with external control like PX4 and ROS
- **Computer** - Rapsberry pi 4

## Software Requirements

- **Operating System**: Ubuntu 20.04 
- **Libraries and Tools**:
  - [Depth Camera SDK](https://www.intelrealsense.com/developers/)
  - [OpenCV](https://opencv.org/) for image processing
  - [ROS (Robot Operating System)](https://www.ros.org/) for drone control
  - [MAVSDK](https://mavsdk.mavlink.io/)
  - Python 3x
  - NumPy for trajectory computation

## Installation

### Clone the Repository

```bash
git clone https://github.com/sohamsarpotdar2001/Trajectory_tracking_and_replication.git
cd Trajectory_tracking_and_replication
```
### Launch the turtlebot and drone
```bash
roslaunch drone_project project.launch
```
### Tracking the turtlebot as a moving object
```bash
python drone_hold.py && python centroid_track_new.py
```
### Replicating the trajectory
```bash
python replicate.py
```
