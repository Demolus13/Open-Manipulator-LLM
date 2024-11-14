# Open-Manipulator-LLM
[https://github.com/user-attachments/assets/8915d633-6475-4254-8f0a-fd39ddcc58e0](https://github.com/user-attachments/assets/8915d633-6475-4254-8f0a-fd39ddcc58e0
)
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
   cd ~/catkin_ws
   git clone https://github.com/Demolus13/Open-Manipulator-LLM.git

2. Install the required Python dependencies:

   ```bash
   pip install opencv-python numpy speechrecognition

3. Build the ROS workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
4. Create Executable Files inside the `~/catkin_ws/src` and `~/catkin_ws/calibration` directory. (using `chmod +x <python file>` .
    
Source the workspace:

   ```bash
   source devel/setup.bash
   ```
## Connect to the Open Manipulator Hardware
### To Connect
- Connect power source cable to the port on board. 
- Connect the USB cable from the open manipulator to your PC's USB port. (Do not use too many USB extensions)
- Connect the camera setup and plug its USB in your PC's port.
- Use [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) to configure the robot.

### To Launch
1. **Terminal 1**
    ```bash
    roslaunch open_manipulator_controller open_manipulator_controller.launch
    ```
2. **Terminal 2**
    ```bash
    rosrun <package name> llm_pick_and_place.py
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

2. Once the system is running, speak the following commands to the robot:
   - "Pick up the red object" → The robot will detect the red object, compute the joint angles, and pick it up.
   - "Place the red object" → The robot will place the red object at a predefined location.
   Make sure that the color range settings in define_color_range.py match the color of the object you are trying to interact with.



## Color Range Definition
Objects are detected based on their colors using predefined HSV (Hue, Saturation, Value) ranges. These ranges are set in the define_color_range.py script and can be customized based on the objects you are working with.

   ```python
   color_ranges = {
       'red': ([0, 0, 88], [105, 28, 155]),
       'green': ([26, 40, 0], [75, 255, 31]),
       'purple': ([81, 25, 0], [126, 58, 255]),
       'yellow': ([60, 115, 114], [127, 178, 185]),
       'orange': ([0, 36, 89], [75, 255, 190])
   }
   ```

## Voice Commands
The system is integrated with the speech_recognition library to process voice commands. After launching the system, the robot will listen for commands, such as:

"Pick up the red object"
"Place the red object"
Ensure that your microphone is configured and working correctly for voice recognition. If no command is recognized, ensure you're speaking clearly and the microphone input is properly set up.

## Calibration
To calibrate the workspace and the camera, you can use the scripts under the calibration/ directory:

- camera.py: This script handles the camera setup and calibration to detect objects accurately.
calibrate_workspace.py: This script is used to define the workspace limits and map them to real-world coordinates for the manipulator to interact within the designated area.

## Troubleshooting
 - No object detected: Ensure the object is within the camera's field of view and that the color ranges in define_color_range.py match the color of the object.
Manipulator not moving: Check if the manipulator is correctly connected and configured in ROS. Verify that the joint states are being properly received.
 - Voice command not recognized: Ensure the microphone is set up and that background noise is minimized for better accuracy. You can also try adjusting the speech recognition library settings or speaking more clearly.
 - No movement after calibration: Double-check the calibration process and make sure the workspace boundaries are defined correctly in calibrate_workspace.py.

## Contributing
We welcome contributions to this project! If you have suggestions or improvements, feel free to open an issue or submit a pull request.

 - How to Contribute
   1. Fork the repository.
   2. Create a new branch for your feature or bug fix.
   3.Make your changes.
   4. Commit your changes with meaningful messages.
   5. Push your changes to your forked repository.
   6. Open a pull request to the main repository.
