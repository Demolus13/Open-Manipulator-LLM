import os
import sys
import cv2
import time
import math

import rospy
import threading
import numpy as np

from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest, SetKinematicsPose, SetKinematicsPoseRequest
import speech_recognition as sr

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from calibration.camera import Camera

class ManipulatorController:
    def __init__(self, color_ranges, workspace_points):
        # Initialize camera, speech recognizer, and microphone
        self.camera = Camera(color_ranges)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Define manipulator positions
        self.start_position = [0.022643, 0.157449, 0.091056, -0.447205, 0.478455, 0.516029, 0.552089]
        self.home_position = [0.170184, -0.017297, 0.086728, 0.036180, 0.663726, -0.040664, 0.745993]
        self.drop_off_position = [0.011012, 0.161025, 0.093839, 0.454568, 0.451787, -0.544460, 0.541129]
        self.drop_off_positions = {
            'default': [0.024607, 0.216026, 0.116800, -0.306642, 0.325058, 0.613874, 0.650743],
            'red': [0.024607, 0.216026, 0.116800, -0.306642, 0.325058, 0.613874, 0.650743],
            'green': [0.00264, -0.226742, 0.126346, 0.281248, 0.269835, -0.664532, 0.637563],
            'purple': [0.177435, 0.196612, 0.181032, -0.086134, 0.185044, 0.413116, 0.887510],
            'orange': [0.125398, -0.195257, 0.118291, 0.208797, 0.362717, -0.453098, 0.787111]
        }

        # Setup workspace transformation
        self.base_position_x = -0.030385
        self.base_position_y = -0.154077
        self.base_z = 0.045792
        self.real_workspace_points = [
            (self.base_position_x, self.base_position_y + 0.31), 
            (self.base_position_x, self.base_position_y), 
            (self.base_position_x + 0.31, self.base_position_y), 
            (self.base_position_x + 0.31, self.base_position_y + 0.31)
        ]
        self.M = cv2.getPerspectiveTransform(np.float32(workspace_points), np.float32(self.real_workspace_points))
        
        # Variables for joint state, threading, and object coordinates
        self.current_joint_states = None
        self.running = False
        self.image_thread = None
        self.command_thread = None
        self.object_coordinates = {}

        # Subscribe to joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # Initialize manipulator to start position
        self.reset_to_start()
        self.control_gripper(0.01)  # Open gripper at the start

    def joint_states_callback(self, data):
        """Callback to update current joint states."""
        self.current_joint_states = data.position

    def control_gripper(self, position, path_time=1.0):
        """Control the gripper position."""
        try:
            rospy.wait_for_service('/goal_tool_control', timeout=5)
            move_service = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
            request = SetJointPositionRequest()
            request.joint_position.joint_name = ["gripper"]
            request.joint_position.position = [position]
            request.path_time = path_time
            response = move_service(request)

            if response.is_planned:
                rospy.loginfo("Gripper action executed successfully!")
            else:
                rospy.logwarn("Failed to execute gripper action.")
            
            time.sleep(path_time)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def control_single_joint_position(self, joint_name, joint_position, path_time=1.0):
        """Move a single joint to the specified position."""
        if self.current_joint_states is None:
            rospy.logwarn("Joint states not received yet.")
            return False

        joint_names = ["joint1", "joint2", "joint3", "joint4"]
        if joint_name not in joint_names:
            rospy.logwarn(f"Invalid joint name: {joint_name}")
            return False

        # Update target position for the specified joint
        self.intermediate_position = list(self.current_joint_states)
        joint_index = joint_names.index(joint_name)
        self.intermediate_position[joint_index] = joint_position

        try:
            rospy.wait_for_service('/goal_joint_space_path', timeout=5)
            move_joint_service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
            request = SetJointPositionRequest()
            request.joint_position.joint_name = joint_names
            request.joint_position.position = self.intermediate_position
            request.path_time = path_time
            response = move_joint_service(request)

            if response.is_planned:
                time.sleep(path_time)
                rospy.loginfo(f"Joint {joint_name} moved to position {joint_position} successfully!")
            else:
                rospy.logwarn(f"Failed to move joint {joint_name} to position {joint_position}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

        return response.is_planned

    def control_all_joint_position(self, joint_positions, path_time=1.0):
        """Move all joints to specified positions."""
        try:
            rospy.wait_for_service('/goal_joint_space_path', timeout=5)
            move_joint_service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
            request = SetJointPositionRequest()
            request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
            request.joint_position.position = joint_positions
            request.path_time = path_time
            response = move_joint_service(request)

            if response.is_planned:
                time.sleep(path_time)
                rospy.loginfo("Joints moved to target positions successfully!")
            else:
                rospy.logwarn("Failed to move joints to target positions.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
        return response.is_planned

    def move_end_effector(self, x, y, z, q1=0.0, q2=0.0, q3=0.0, q4=0.0, path_time=1.0):
        """Move the end effector to a specific position and orientation."""
        rospy.wait_for_service('/goal_task_space_path')
        try:
            move_service = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
            request = SetKinematicsPoseRequest()
            request.planning_group = "arm"
            request.end_effector_name = "gripper"
            request.kinematics_pose.pose.position.x = x
            request.kinematics_pose.pose.position.y = y
            request.kinematics_pose.pose.position.z = z
            request.kinematics_pose.pose.orientation.x = q1
            request.kinematics_pose.pose.orientation.y = q2
            request.kinematics_pose.pose.orientation.z = q3
            request.kinematics_pose.pose.orientation.w = q4
            request.path_time = path_time
            rospy.loginfo(f"Moving end effector to (x={x}, y={y}, z={z}) over {path_time} seconds")
            response = move_service(request)

            time.sleep(path_time)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def execute_pick_and_place(self, color, coordinates):
        """Perform a pick-and-place task for detected objects of a given color."""
        for (x, y, w, h) in coordinates:
            object_center_x, object_center_y = cv2.perspectiveTransform(
                np.float32([[x + w//2, y + h//2]]).reshape(-1, 1, 2), self.M
            ).flatten()

            # Pick object
            self.set_to_home_position()
            self.control_gripper(0.01)  # Open gripper
            joint_angle = math.atan2(object_center_y, object_center_x)
            if not self.control_single_joint_position("joint1", joint_angle, 1.0):
                self.reset_to_start()
                continue
            if not self.move_end_effector(object_center_x + 0.1 * (0.31 - object_center_x), object_center_y, self.base_z, 0.140060, 0.671755, -0.148471, 0.712099):
                self.reset_to_start()
                continue

            self.control_gripper(-0.01)  # Close gripper to pick the object
            if not self.control_all_joint_position(self.intermediate_position, 1.0):
                self.reset_to_start()
                continue

            # Place object
            self.set_to_home_position()
            if not self.drop_off(color):
                print(f"Failed to drop off {color} object.")
                self.reset_to_start()
                continue

            self.control_gripper(0.01)  # Open gripper to release the object
            self.reset_to_start()

    def capture_images(self):
        """Continuously capture images to detect objects."""
        while not rospy.is_shutdown():
            self.camera.start(self.object_coordinates, show_masked_image=False)

    def listen_for_commands(self):
        """Listen for voice commands to identify objects and initiate pick-and-place."""
        while not rospy.is_shutdown():
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source)
                print("Listening for commands...")
                audio = self.recognizer.listen(source)

            try:
                command = self.recognizer.recognize_google(audio).lower()
                print(f"Command received: {command}")

                for color in self.camera.color_ranges.keys():
                    if color in command:
                        if color in self.object_coordinates:
                            self.execute_pick_and_place(color, self.object_coordinates[color])
                            self.object_coordinates.pop(color)
                        else:
                            rospy.logwarn(f"No {color} objects detected.")
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand the audio.")
            except sr.RequestError as e:
                rospy.logerr(f"Could not request results from Google Speech Recognition service; {e}")

    def set_to_start_position(self, path_time=1.0):
        """Move the manipulator to the start position."""
        self.move_end_effector(*self.start_position, path_time)

    def set_to_home_position(self, path_time=1.0):
        """Move the manipulator to the home position."""
        self.move_end_effector(*self.home_position, path_time)

    def drop_off(self, color):
        """Move the end effector to the drop-off position based on color."""

        if color in self.drop_off_positions:
            if(color == 'red'):
                self.drop_off_position = [0.022643, 0.157449, 0.091056, -0.447205, 0.478455, 0.516029, 0.552089]
            elif(color == 'green'):
                self.drop_off_position = [0.011012, -0.161025, 0.093839, 0.454568, 0.451787, -0.544460, 0.541129]
            elif(color == 'purple'):
                self.drop_off_position = [0.119341, 0.130795, 0.109074, -0.263562, 0.557255, 0.336658, 0.711803]
            elif(color == 'orange'):
                self.drop_off_position = [0.090887, -0.145967, 0.093733, 0.328219, 0.550468, -0.393125, 0.659325]
            else:
                self.drop_off_position = [0.022643, 0.157449, 0.091056, -0.447205, 0.478455, 0.516029, 0.552089]
                print('No drop-off position defined for {color} object.')
            
            
            if not self.move_end_effector(*self.drop_off_position):
                return False
            cartesian_position = self.drop_off_positions[color]
            return self.move_end_effector(*cartesian_position)
        else:
            print(f"No drop-off position defined for {color} object.")
            cartesian_position = self.drop_off_positions['default']
            return self.move_end_effector(*cartesian_position)

    def reset_to_start(self):
        """Reset manipulator to start position in case of error during task."""
        self.set_to_home_position()
        self.set_to_start_position()

    def start_controller(self):
        """Start image capture and voice command listener threads."""
        self.running = True
        self.image_thread = threading.Thread(target=self.capture_images)
        self.command_thread = threading.Thread(target=self.listen_for_commands)
        self.image_thread.start()
        self.command_thread.start()

    def stop_controller(self):
        """Stop image capture and command listener threads."""
        self.running = False
        print("Stopping manipulator controller...")

        if self.image_thread is not None:
            self.camera.stop()
            self.image_thread.join()
            print("Image capture stopped.")
        
        if self.command_thread is not None:
            self.command_thread.join()
            print("Command listener stopped.")
        
        print("Manipulator controller stopped.")

    def run(self):
        """Main function to initialize and start the ROS node."""
        rospy.init_node('manipulator_pick_and_place')
        self.start_controller()
        rospy.spin()
        self.stop_controller()


if __name__ == "__main__":
    color_ranges = {
        'red': ([0, 0, 88], [105, 28, 155]),
        'green': ([26, 40, 0], [75, 255, 31]),
        'purple': ([48, 0, 0], [85, 23, 67]),
        'orange': ([0, 35, 93], [38, 84, 155])
    }
    workspace_points = [(522, 24), (119, 21), (78, 447), (560, 446)]
    controller = ManipulatorController(color_ranges, workspace_points)

    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop_controller()
