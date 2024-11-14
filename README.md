# Open-Manipulator-LLM

## Overview

The Open-Manipulator-LLM package is designed to control an Open Manipulator using a combination of computer vision and speech recognition. The manipulator can detect objects of specific colors, pick them up, and place them in predefined positions based on voice commands.

## Prerequisites
- **Operating System:** Ubuntu 20.04
- **ROS Version:** Noetic

## Installation of Open Manipulator-X Package

1. **Install the Open Manipulator X package** by following the installation steps provided in the official e-manual. 
   - Reference: [Official E-Manual](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/).
   - Follow steps 4.1.1 to 4.1.3 of the manual to complete the installation.
   - Follow steps 5.4.2 of the manual to setup for gravity compensation.

# Open-Manipulator LLM (Pick and Place with Object Detection and Voice Commands)

This project implements a manipulator control system integrated with object detection and voice commands for performing pick-and-place operations. It is built using ROS, Python, and OpenCV for the object detection and manipulation tasks. The system can detect objects based on color, interpret voice commands, and control a manipulator to pick and place objects in a specified workspace.

## Table of Contents

- [Project Structure](#project-structure)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Setup Instructions](#setup-instructions)
- [Usage](#usage)
  - [Object Detection and Manipulator Control](#object-detection-and-manipulator-control)
  - [Color Range Definition](#color-range-definition)
  - [Voice Commands](#voice-commands)
  - [Calibration](#calibration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Project Structure

The project is organized into the following directories and files:




### Directory Breakdown

- **calibration/**: Contains scripts for camera calibration and workspace setup.
  - `camera.py`: Handles camera setup and object detection using OpenCV.
  - `README.md`: Documentation for calibration-related files and instructions.
  
- **src/**: Contains the main Python scripts for manipulator control and pick-and-place tasks.
  - `calibrate_workspace.py`: Used for calibrating the workspace and defining the operational area.
  - `cartesian_control.py`: Controls the manipulator in Cartesian space for accurate positioning.
  - `define_color_range.py`: Defines color ranges for object detection based on HSV values.
  - `get_coordinates.py`: Extracts the coordinates of detected objects from the camera feed.
  - `get_joint_states.py`: Retrieves the current joint states of the manipulator.
  - `llm_pick_and_place.py`: Main script that integrates object detection, manipulator control, and voice command processing for pick-and-place tasks.

- **CMakeLists.txt**: Defines the build configuration for the project (if applicable).
- **package.xml**: ROS package configuration that includes dependencies and setup.

## Installation

### Prerequisites

Before setting up the project, ensure you have the following prerequisites installed:

- **ROS**: Robot Operating System (ROS Noetic is recommended).
- **Python 3.x**: Python 3.6 or later.
- **OpenCV**: For image processing and object detection.
- **SpeechRecognition**: For processing voice commands.
- **NumPy**: For numerical operations.
- **Other Python dependencies**: `cv2`, `numpy`, `speechrecognition`.

### Setup Instructions

1. Clone the repository into your ROS workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone <repository-url>

2. Install the required Python dependencies:

   ```bash
   pip install opencv-python numpy speechrecognition

Build the ROS workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

Source the workspace:

   ```bash
   source devel/setup.bash
   ```

Launch the ROS nodes for manipulator control and object detection:

   ```bash
   rosrun <package_name> llm_pick_and_place.py
   ```

## Usage

### Object Detection and Manipulator Control

The system uses object detection based on predefined color ranges to detect objects within the camera feed. Once an object is detected, the manipulator performs the corresponding pick-and-place action based on voice commands.

#### How it Works:

1. **Object Detection**: The camera continuously captures the feed and processes the image to identify objects based on their color. The color ranges for detection are predefined in the `define_color_range.py` script. Each object is detected based on its color (e.g., red, green, yellow), and the position of the object is extracted using the `get_coordinates.py` script.

2. **Voice Commands**: The system listens for voice commands using the `speech_recognition` library. Once a command is detected, the robot proceeds with the pick-and-place action. Common commands include:
   - "Pick up the red object"
   - "Place the red object"

   The voice recognition functionality is implemented in the `llm_pick_and_place.py` script. The system uses speech recognition to trigger the pick-and-place operation based on the user's verbal instructions.

3. **Manipulator Control**: Upon detecting an object and receiving a voice command, the system calculates the joint angles needed to move the manipulator to the object's location. The `cartesian_control.py` script is responsible for controlling the manipulator in Cartesian space to perform the movement accurately.

#### Workflow:
- **Step 1**: The system listens for a voice command.
- **Step 2**: The object is detected by the camera, and its coordinates are extracted.
- **Step 3**: The `cartesian_control.py` computes the necessary joint angles to pick or place the object.
- **Step 4**: The manipulator moves to the target position and performs the action (pick or place).

Ensure that the camera feed is active, and the robot is properly calibrated for the pick-and-place task. The robot will respond to the following voice commands to perform the respective actions:
- "Pick up the [color] object"
- "Place the [color] object"

#### Example Usage:
1. Launch the ROS node:

   ```bash
   rosrun <package_name> llm_pick_and_place.py
   ```

2. Once the system is running, speak the following commands to the robot:
   - "Pick up the red object" → The robot will detect the red object, compute the joint angles, and pick it up.
   - "Place the red object" → The robot will place the red object at a predefined location.
   Make sure that the color range settings in define_color_range.py match the color of the object you are trying to interact with.

