import os
import sys
import time
import math

import rospy
import threading
import numpy as np

import cv2

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from calibration.camera import Camera

from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest, SetKinematicsPose, SetKinematicsPoseRequest
import speech_recognition as sr

class ManipulatorController:
    def __init__(self, color_ranges, workspace_points):
        self.camera = Camera(color_ranges)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(device_index=1)

        self.start_positions = [0.022643, 0.157449, 0.091056, -0.447205, 0.478455, 0.516029, 0.552089]
        self.home_positions = [0.170184, -0.017297, 0.086728, 0.036180, 0.663726, -0.040664, 0.745993]
        self.drop_off_positions = {
            'red': [0.5, 0.5, 0.1, 0.0],
            'blue': [0.5, -0.5, 0.1, 0.0]
        }
        xx = -0.025548
        yy = -0.159262
        self.base = 0.040792
        self.real_workspace_points = [(xx, yy+0.31), (xx, yy), (xx+0.31,yy), (xx+0.31, yy+0.31)]
        self.M = cv2.getPerspectiveTransform(np.float32(workspace_points), np.float32(self.real_workspace_points))
        self.current_joint_states = None

        self.running = False
        self.image_thread = None
        self.command_thread = None
        self.coordinates = {}

        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        self.set_start()
        self.control_gripper(0.01)  

    def control_gripper(self, position, path_time=2.0):
        try:
            service_name = '/goal_tool_control'
            rospy.wait_for_service(service_name, timeout=5)
            move_service = rospy.ServiceProxy(service_name, SetJointPosition)
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

    def control_joint_positions(self, joint_name, joint_position, path_time=3.0):
        if self.current_joint_states is None:
            rospy.logwarn("Joint states not received yet.")
            return False
        
        joint_names = ["joint1", "joint2", "joint3", "joint4"]
        if joint_name not in joint_names:
            rospy.logwarn(f"Invalid joint name: {joint_name}")
            return False
        
        joint_positions = list(self.current_joint_states)
        joint_index = joint_names.index(joint_name)
        joint_positions[joint_index] = joint_position

        try:
            rospy.wait_for_service('/goal_joint_space_path', timeout=5)
            move_joint_service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
            request = SetJointPositionRequest()
            request.joint_position.joint_name = joint_names
            request.joint_position.position = joint_positions
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

    def move_end_effector(self, x, y, z, q1 = 0.0, q2 = 0.0, q3 = 0.0, q4 = 0.0, path_time=3.0):
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
            rospy.loginfo("Sending request to move end effector to x=%f, y=%f, z=%f over %f seconds" % (x, y, z, path_time))
            response = move_service(request)
            rospy.loginfo("Service response: %s" % response)

            time.sleep(path_time)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def pick_and_place(self, color, coordinates):
        for (x, y, w, h) in coordinates:
            # Convert (x, y) to joint positions using the model
            robot_x, robot_y = cv2.perspectiveTransform(np.float32([[x + w//2, y + h//2]]).reshape(-1, 1, 2), self.M).flatten()

            # Move to the object's position
            self.set_home()
            self.control_gripper(0.01)
            response = self.control_joint_positions("joint1", math.atan2(robot_y, robot_x))
            if not response:
                self.set_home()
                self.set_start()
                continue
            response = self.move_end_effector(robot_x, robot_y, 0.034792, 0.140060, 0.671755, -0.148471, 0.712099)
            if not response:
                self.set_home()
                self.set_start()
                continue
            self.control_gripper(-0.01)
            self.set_home()
            response = self.set_drop_off(color)
            if not response:
                self.set_home()
                self.set_start()
                continue
            self.control_gripper(0.01)

            self.set_start(1.0)

    def set_home(self, path_time=3.0):
        # Define home position for the manipulator
        self.move_end_effector(*self.home_positions, path_time)

    def set_start(self, path_time=3.0):
        # Define start position for the manipulator
        self.move_end_effector(*self.start_positions, path_time)

    def set_drop_off(self, color):
        # Define drop-off position for the manipulator
        response = self.move_end_effector(0.022643, 0.157449, 0.091056, -0.447205, 0.478455, 0.516029, 0.552089)
        if not response:
            return False

        response = self.move_end_effector(0.024607, 0.216026, 0.116800, -0.306642, 0.325058, 0.613874, 0.650743)
        if not response:
            return False
        
        return True
    
    def joint_states_callback(self, data):
        self.current_joint_states = data.position

    def capture_images(self):
        while not rospy.is_shutdown():
            self.camera.start(self.coordinates, show_masked_image=False)

    def listen_for_commands(self):
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
                        if color in self.coordinates:
                            self.pick_and_place(color, self.coordinates[color])
                            self.coordinates.pop(color)
                        else:
                            rospy.logwarn(f"No {color} objects detected.")
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand the audio.")
            except sr.RequestError as e:
                rospy.logerr(f"Could not request results from Google Speech Recognition service; {e}")

    def start(self):
        self.running = True
        self.image_thread = threading.Thread(target=self.capture_images)
        self.command_thread = threading.Thread(target=self.listen_for_commands)
        self.image_thread.start()
        self.command_thread.start()

    def stop(self):
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
        rospy.init_node('llm_pick_and_place')
        self.start()
        rospy.spin()
        self.stop()

if __name__ == "__main__":
    color_ranges = {
        'red': ([0, 0, 88], [105, 28, 155]),
        'purple': ([81, 25, 0], [126, 58, 255]),
    }
    workspace_points = [(532, 32), (149, 29), (113, 429), (571, 433)]
    controller = ManipulatorController(color_ranges, workspace_points)
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()