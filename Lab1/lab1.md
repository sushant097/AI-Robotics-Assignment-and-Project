# Lab 1 Assignment - Setup and Introduction to ROS

**The full assignment instruction pdf is in [here](docs/Lab1.pdf)**.

# Lab 1 - Setup and Introduction to ROS

## Overview

This folder contains the solution code for Lab 1 of the AI Robotics course, which focuses on setting up the ROS (Robot Operating System) environment and introducing basic ROS functionalities. The lab guides through creating ROS packages, writing Python scripts for controlling a simulated robot (see in rviz), and subscribing to ROS topics to monitor the robot's state.

## Lab Objectives

The main objectives of this lab are:
1. **Set Up ROS Noetic**: Install and configure ROS Noetic on Ubuntu 20.04, ensuring a working development environment.
2. **Learn Basic ROS Commands**: Work through introductory ROS tutorials to gain familiarity with ROS concepts and commands.
3. **Create a ROS Package**: Set up a ROS package to house code for robotic simulations.
4. **Control a Simulated Robot**: Write Python scripts to control the movement of a robot in a simulation, making it move in predefined patterns.
5. **Subscribe to ROS Topics**: Implement a subscriber node to monitor and print the robot's position.

## Contents

- `square.py`: Script to move the robot in a square trajectory.
- `initials.py`: Script to move the robot in a trajectory that spells out the initials "SG".
- `subscribe_pose.py`: Script to subscribe to the robot's pose and print its position.
- `Lab1.pdf`: The full assignment instructions.

## Setup Instructions

1. **ROS Noetic Installation**:
   - Follow the [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu) to install the Desktop-Full version on Ubuntu 20.04.
   - Complete the beginner-level ROS tutorials to understand the basic concepts.

2. **Creating a Workspace**:
   - Set up a catkin workspace by following the [catkin workspace tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
   - Create a ROS package using the following commands:
     ```bash
     cd ~/catkin_ws/src
     catkin_create_pkg ai_labs std_msgs rospy roscpp sensor_msgs geometry_msgs
     ```
   - Unzip the provided programming exercise files into the `ai_labs` folder and organize the scripts into the `scripts` directory:
     ```bash
     mkdir ~/catkin_ws/src/ai_labs/scripts
     mv *.py ~/catkin_ws/src/ai_labs/scripts/
     chmod +x ~/catkin_ws/src/ai_labs/scripts/*
     ```

3. **Building the Workspace**:
   - Navigate to the catkin workspace and build the workspace:
     ```bash
     cd ~/catkin_ws
     catkin_make
     source devel/setup.bash
     ```

## Script Descriptions

### 1. `square.py`

This script makes the robot move in a square trajectory using ROS. The robot moves forward and then turns 90 degrees, repeating this process to form a square.

**Run Command**:
```bash
roslaunch ai_labs lab1.launch
```

Solution is in [square.py](ai_labs/scripts/square.py)

### 2. `initials.py`

This script extends the functionality of `square.py` by making the robot move in a trajectory that spells out the initials "SG". The script uses a combination of linear and angular movements to trace out each letter.

**Key Sections**:
- **Move Forward and Backward**: Controls the linear movement of the robot.
- **Turn at 45 and 90 Degrees**: Controls the angular movement to trace out the initials.

**Run Command**:
```bash
rosrun ai_labs initials.py
```

Access the code: [initials.py](ai_labs/scripts/initials.py)

### 3. `subscribe_pose.py`

This script subscribes to the `/turtle1/pose` topic to monitor the robot’s position and prints the pose (x, y, theta) to the console.

**Key Sections**:
- **pose_callback**: A callback function that logs the robot's current position and orientation.
- **Subscriber Initialization**: Subscribes to the `/turtle1/pose` topic and continuously listens for updates.

Access the code: [subscribe_pose.py](ai_labs/scripts/subscribe_pose.py) 

**Run Command**:
```bash
rosrun ai_labs subscribe_pose.py
```

## Conclusion

This lab provides a foundational understanding of ROS, focusing on setting up a ROS environment, controlling a simulated robot, and subscribing to topics to monitor the robot's state. The scripts provided demonstrate basic robot movement and interaction with ROS topics, forming the basis for more advanced robotics tasks in future labs.