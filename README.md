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

## Features

- **Object Detection**: Uses a camera to detect objects of specific colors.
- **Speech Recognition**: Listens for voice commands to pick up objects of a specified color.
- **Manipulator Control**: Controls the Open Manipulator to pick up and place objects.

## Package Structure
```
Open-Manipulator-LLM/
├── calibration/
│   ├── datasets/
│   ├── models/
│   ├── camera.py
│   └── model.py
├── CMakeLists.txt
├── include/
│   └── open_manipulator_llm/
├── package.xml
├── README.md
└── src/
    ├── calibrate_workspace.py
    ├── define_color_range.py
    └── llm_pick_and_place.py
```

### Key Files

- [**calibration/camera.py**](./calibration/camera.py): Handles capturing images and detecting objects of specific colors.
- [**calibration/model.py**](./calibration/model.py): Contains the model for converting image coordinates to manipulator joint positions.
- [**src/llm_pick_and_place.py**](./src/llm_pick_and_place.py): Main script that integrates object detection, speech recognition, and manipulator control.

## Installation

1. **Clone the repository**: Run these commands in your terminal to create a ROS workspace and clone the repository into the src directory of your workspace.
    ```sh
    mkdir -p ~/<ros_workspace>/src
    cd ~/<ros_workspace>/src
    git clone https://github.com/Demolus13/Open-Manipulator-LLM.git
    ```

2. **Install dependencies**: Run this command in the terminal from the root of your ROS workspace.
    ```sh
    cd ~/<ros_workspace>
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **Build the package**: Run this command in the terminal from the root of your ROS workspace.
    ```sh
    cd ~/<ros_workspace>
    catkin_make
    ```

4. **Source the setup file**: Run this command in the terminal from the root of your ROS workspace to source the setup file and set up the environment.
    ```sh
    cd ~/<ros_workspace>
    source devel/setup.bash
    ```

## Usage

### Workspace calibration
1. Define the color range for your selected colors:
    
    - The color ranges for object detection are defined in `src/llm_pick_and_place.py`.
        ```py
        color_ranges = {
            'red': ([0, 0, 100], [50, 50, 255]),
            'blue': ([100, 0, 0], [255, 50, 50])
        }
        ```
    - To change the color ranges or define new colors run the script `src/define_color_range.py`.

2. Collect the data for workspace calibration:

    - Define for which color are you calibrating the workspace for using the script `src/calibrate_workspace.py`.
        ```py
        color_ranges = {
            'red': ([0, 0, 100], [50, 50, 255])
        }
        dataset_path = 'calibration/datasets/DATASET_0.csv'
        frequency = 10
        ```
    - Make use of Open Manipulator-X Gravity compensation package to move the manipulator to the desired positions.
    - Move the object with the specified color to various pickup positions in the workspace, in this case it will collect data at a frequency of 10Hz.
    - It will store the pixel and joint space data collected in the [`calibration/datasets`](./calibration/) folder.

3. Train the model to convert 2D pixel to Joint States:

    - Specify the location of your dataset and model save path in the script [`calibration/model.py`](./calibration/model.py).
        ```py
        # Load data from .csv file
        data_path = "calibration/datasets/DATASET_0.csv"
        data = pd.read_csv(data_path)

        ...

        # Save the trained model
        os.makedirs("calibration/models", exist_ok=True)
        model_path = "calibration/models/MODEL_0.pth"
        torch.save(model.state_dict(), model_path)
        print(f"Model saved to {model_path}")
        ```
    - Run the script `calibration/model.py` to train the model.
    - Trained model will be stored in [`calibration/models`] folder.

4. Configure your drop-off positions:

    - Edit the drop-off position according the colors specified in [`src/llm_pick_and_place.py`](./src/llm_pick_and_place.py).
        ```py
        self.drop_off_positions = {
            'red': [0.5, 0.5, 0.1, 0.0],
            'blue': [0.5, -0.5, 0.1, 0.0]
        }
        ```

### Start your Open Manipulator

#### Launch 
1. Launch the ROS master:
    ```sh
    roscore
    ```

2. Run the pick and place node:
    ```sh
    cd ~/<ros_workspace>
    rosrun open_manipulator_llm llm_pick_and_place.py
    ```

3. Give voice command to pick up your specified color objects.

#### Nodes

- `llm_pick_and_place.py`: This node integrates object detection, speech recognition, and manipulator control.

#### Services

- `/goal_joint_space_path` (open_manipulator_msgs/SetJointPosition): Service to control the manipulator's joint positions.
- `/goal_tool_control` (open_manipulator_msgs/SetJointPosition): Service to control the manipulator's gripper.
