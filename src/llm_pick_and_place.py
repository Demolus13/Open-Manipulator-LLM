import os
import sys
import time

import rospy
import threading
import numpy as np

import cv2

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from calibration.camera import Camera

from std_srvs.srv import Empty
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest, SetKinematicsPose, SetKinematicsPoseRequest
import speech_recognition as sr

class ManipulatorController:
    def __init__(self, color_ranges, workspace_points):
        self.camera = Camera(color_ranges)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.home_positions = [-0.4356505572795868, -0.6044466686248779, -0.13038836419582367, 1.306951642036438]
        self.drop_off_positions = {
            'red': [0.5, 0.5, 0.1, 0.0],
            'blue': [0.5, -0.5, 0.1, 0.0]
        }

        self.real_workspace_points = [(-0.015955, 0.152749), (-0.020716, -0.149990), (0.264617, -0.152468), (0.272969, 0.142075)]
        self.M = cv2.getPerspectiveTransform(np.float32(workspace_points), np.float32(self.real_workspace_points))

        self.running = False
        self.image_thread = None
        self.command_thread = None

    def control_manipulator(self, joint_names, positions, path_time=2.0):
        try:
            service_name = '/goal_tool_control' if 'gripper' in joint_names else '/goal_joint_space_path'
            rospy.wait_for_service(service_name, timeout=5)
            move_service = rospy.ServiceProxy(service_name, SetJointPosition)
            request = SetJointPositionRequest()
            request.joint_position.joint_name = joint_names
            request.joint_position.position = positions
            request.path_time = path_time
            response = move_service(request)
            if response.is_planned:
                rospy.loginfo("Action executed successfully!")
            else:
                rospy.logwarn("Failed to execute the action.")

            time.sleep(path_time)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def move_end_effector(self, x, y, z, path_time):
        rospy.wait_for_service('/goal_task_space_path')
        x, y, z = (0.264617, -0.152468, 0.056600)
        try:
            move_service = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
            request = SetKinematicsPoseRequest()
            request.planning_group = "arm"
            request.end_effector_name = "gripper"
            request.kinematics_pose.pose.position.x = x
            request.kinematics_pose.pose.position.y = y
            request.kinematics_pose.pose.position.z = z
            request.kinematics_pose.pose.orientation.x = 0.0
            request.kinematics_pose.pose.orientation.y = 0.0
            request.kinematics_pose.pose.orientation.z = 0.0
            request.kinematics_pose.pose.orientation.w = 1.0
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
            # robot_x, robot_y = cv2.perspectiveTransform(np.float32([[x + w//2, y + h//2]]).reshape(-1, 1, 2), self.M).flatten()
            robot_x, robot_y = cv2.perspectiveTransform(np.float32([[560, 426]]).reshape(-1, 1, 2), self.M).flatten()

            # Move to the object's position
            self.set_home()
            self.control_manipulator(["gripper"], [0.01])
            self.move_end_effector(robot_x, robot_y, 0.066, 3.0)
            self.control_manipulator(["gripper"], [-0.01])
            self.set_home()
            self.control_manipulator(["joint1", "joint2", "joint3", "joint4"], [-0.647339940071106, 0.1026602154970169, 0.15033012628555298, 0.9341943264007568])
            self.control_manipulator(["gripper"], [0.01])
            self.set_home()

    def set_home(self):
        # Define home position for the manipulator
        self.control_manipulator(["joint1", "joint2", "joint3", "joint4"], self.home_positions)

    def set_drop_off(self, color):
        # Define drop-off position for the manipulator
        self.control_manipulator(["joint1", "joint2", "joint3", "joint4"], self.drop_off_positions[color])

    def capture_images(self):
        while not rospy.is_shutdown():
            # self.camera.start(show_masked_image=False)
            # self.coordinates = self.camera.get_coordinates()
            pass

    def listen_for_commands(self):
        move = False
        while not rospy.is_shutdown():
            # with self.microphone as source:
            #     print("Listening for commands...")
            #     audio = self.recognizer.listen(source)

            # try:
            #     command = self.recognizer.recognize_google(audio).lower()
            #     print(f"Command received: {command}")

            #     for color in self.camera.color_ranges.keys():
            #         if f"pick up {color}" in command:
            #             if color in self.coordinates:
            #                 self.pick_and_place(color, self.coordinates[color])
            #             else:
            #                 rospy.logwarn(f"No {color} objects detected.")

            self.coordinates = {
                'red': [(0, 0, 0, 0)]
            }
            if not move:
                self.pick_and_place('red', self.coordinates['red'])
                move = True
            # except sr.UnknownValueError:
            #     rospy.logwarn("Could not understand the audio.")
            # except sr.RequestError as e:
            #     rospy.logerr(f"Could not request results from Google Speech Recognition service; {e}")

    def start(self):
        self.running = True
        self.image_thread = threading.Thread(target=self.capture_images)
        self.command_thread = threading.Thread(target=self.listen_for_commands)
        self.image_thread.start()
        self.command_thread.start()

    def stop(self):
        self.running = False
        if self.image_thread is not None:
            self.image_thread.join()
        if self.command_thread is not None:
            self.command_thread.join()

    def run(self):
        rospy.init_node('llm_pick_and_place')
        self.start()
        rospy.spin()
        self.stop()

if __name__ == "__main__":
    color_ranges = {
        'red': ([37, 0, 0], [255, 25, 56]),
        'orange': ([0, 36, 83], [66, 103, 170]),
        'yellow': ([43, 107, 106], [123, 172, 173]),
    }
    workspace_points = [(524, 27), (140, 21), (104, 423), (560, 426)]
    controller = ManipulatorController(color_ranges, workspace_points)
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()