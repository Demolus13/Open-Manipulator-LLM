import rospy
import torch
import threading

from calibration.camera import Camera
from calibration.model import CalibrationModel

from std_srvs.srv import Empty
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
import speech_recognition as sr

class ManipulatorController:
    def __init__(self, model_path, color_ranges):
        self.model = torch.load(model_path)
        self.model.eval()
        self.camera = Camera(color_ranges)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.home_positions = [0.0, 0.0, 0.0, 0.0]
        self.drop_off_positions = {
            'red': [0.5, 0.5, 0.1, 0.0],
            'blue': [0.5, -0.5, 0.1, 0.0]
        }

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
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def pick_and_place(self, color, coordinates):
        for (x, y, w, h) in coordinates:
            # Convert (x, y) to joint positions using the model
            input_tensor = torch.tensor([x, y], dtype=torch.float32).unsqueeze(0)
            joint_positions = self.model(input_tensor).detach().numpy().flatten()

            # Move to the object's position
            self.control_manipulator(["grasper"], [0.01])
            self.control_manipulator(["joint1", "joint2", "joint3", "joint4"], joint_positions)
            self.control_manipulator(["gripper"], [-0.01])
            self.set_home()
            self.control_manipulator(["joint1", "joint2", "joint3", "joint4"], [0.5, 0.5, 0.1, 0.0])
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
            self.camera.capture_image()
            self.coordinates = self.camera.get_coordinates()

    def listen_for_commands(self):
        while not rospy.is_shutdown():
            with self.microphone as source:
                print("Listening for commands...")
                audio = self.recognizer.listen(source)

            try:
                command = self.recognizer.recognize_google(audio).lower()
                print(f"Command received: {command}")

                for color in self.camera.color_ranges.keys():
                    if f"pick up {color}" in command:
                        if color in self.coordinates:
                            self.pick_and_place(color, self.coordinates[color])
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
    model_path = "calibration/models/MODEL_0.pth"
    color_ranges = {
        'red': ([0, 0, 0], [255, 30, 68])
    }
    controller = ManipulatorController(model_path, color_ranges)
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()