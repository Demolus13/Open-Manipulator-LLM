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
        self.microphone = sr.Microphone(device_index=1)

        self.home_positions = [0.160545, -0.013251, 0.087827, 0.030292, 0.680489, -0.032558, 0.731408]
        self.drop_off_positions = {
            'red': [0.5, 0.5, 0.1, 0.0],
            'blue': [0.5, -0.5, 0.1, 0.0]
        }
        xx = -0.025548
        yy = -0.159262
        self.real_workspace_points = [(xx, yy+0.31), (xx, yy), (xx+0.31,yy), (xx+0.31, yy+0.31)]
        self.M = cv2.getPerspectiveTransform(np.float32(workspace_points), np.float32(self.real_workspace_points))

        self.running = False
        self.image_thread = None
        self.command_thread = None
        self.coordinates = {}

        self.set_home()

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
            self.move_end_effector(robot_x, robot_y, 0.034792, 0.140060, 0.671755, -0.148471, 0.712099)
            self.control_gripper(-0.01)
            self.set_home()
            self.set_drop_off(color)
            self.control_gripper(0.01)
            self.set_home()


    def set_home(self):
        # Define home position for the manipulator
        self.move_end_effector(*self.home_positions, 3.0)

    def set_drop_off(self, color):
        # Define drop-off position for the manipulator
        self.move_end_effector(0.285860, -0.003361, 0.210861, -0.000132, -0.021474, -0.006134, 0.999751)
        self.move_end_effector(0.030401, 0.260340, 0.221987, 0.036584, -0.039261, 0.680742, 0.730555)
        self.move_end_effector(0.037506, 0.187797, 0.135732, -0.303393, 0.347384, 0.583663, 0.668292)

    def capture_images(self):
        while not rospy.is_shutdown():
            self.camera.start(self.coordinates, show_masked_image=True)

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
        'red': ([37, 0, 0], [255, 25, 56])
    }
    workspace_points = [(531, 20), (129, 18), (97, 438), (560, 440)]
    controller = ManipulatorController(color_ranges, workspace_points)
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()