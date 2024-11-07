import os
import csv
import cv2
import sys
import threading
import rospy
from sensor_msgs.msg import JointState

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from calibration.camera import Camera

class DataCollector:
    def __init__(self, color_ranges, dataset_path, interval=0.1):
        self.color_ranges = color_ranges
        self.dataset_path = dataset_path
        self.interval = interval
        self.camera = Camera(color_ranges)
        self.running = False
        self.paused = True
        self.camera_thread = None
        self.joint_thread = None
        self.joint_positions = [0, 0, 0, 0]

        # Initialize ROS node
        rospy.init_node('data_collector', anonymous=False)

        # Write the header to the CSV file
        os.makedirs(os.path.dirname(dataset_path), exist_ok=True)
        with open(dataset_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x_midpoint', 'y_midpoint', 'joint1', 'joint2', 'joint3', 'joint4'])

    def capture_camera_data(self):
        while self.running and not rospy.is_shutdown():
            self.camera.start(show_masked_image=False)
            
    def capture_joint_data(self):
        while self.running and not rospy.is_shutdown():
            joint_state = rospy.wait_for_message('/joint_states', JointState)
            self.joint_positions = joint_state.position[1:]

    def store_camera_and_joint_data(self):
        coordinates = self.camera.get_coordinates()
        data = []
        for color, points in coordinates.items():
            for x, y, w, h in points:
                data.append([x + w//2, y + h//2] + list(self.joint_positions))

        with open(self.dataset_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(data)

    def start(self):
        self.running = True
        self.camera_thread = threading.Thread(target=self.capture_camera_data)
        self.joint_thread = threading.Thread(target=self.capture_joint_data)
        self.camera_thread.start()
        self.joint_thread.start()
        
        while self.running and not rospy.is_shutdown():
            if not self.paused:
                self.store_camera_and_joint_data()
            rospy.sleep(self.interval)

    def stop(self):
        self.running = False
        if self.camera_thread is not None:
            self.camera_thread.join()
        if self.joint_thread is not None:
            self.joint_thread.join()
        self.camera.stop()
        cv2.destroyAllWindows()

    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            print("Data collection paused.")
        else:
            print("Data collection resumed.")

if __name__ == "__main__":
    color_ranges = {
        'red': ([37, 0, 0], [255, 25, 56]),
        # 'orange': ([0, 36, 83], [66, 103, 170]),
        # 'yellow': ([43, 107, 106], [123, 172, 173]),
    }
    dataset_path = 'calibration/datasets/DATASET_0.csv'
    frequency = 10
    collector = DataCollector(color_ranges, dataset_path, 1/frequency)

    # Start the data collector in a separate thread
    collector_thread = threading.Thread(target=collector.start)
    collector_thread.start()

    try:
        while True:
            user_input = input("Press Enter to toggle pause/resume, or 'q' to quit: ").strip().lower()
            if user_input == 'q':
                collector.stop()
                break
            else:
                collector.toggle_pause()
    except KeyboardInterrupt:
        collector.stop()