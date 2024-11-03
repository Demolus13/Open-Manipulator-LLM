import os
import cv2
import numpy as np

class Camera:
    def __init__(self, color_ranges):
        self.cap = None
        for i in range(1, 10):
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                print(f"Camera {i} is active")
                break
        else:
            raise Exception("No camera is available")
        
        self.color_ranges = {}
        for color, (lower_bound, upper_bound) in color_ranges.items():
            self.color_ranges[color] = (
                np.array(lower_bound[::-1]),
                np.array(upper_bound[::-1]),
                tuple((np.array(lower_bound[::-1]) + np.array(upper_bound[::-1]))//2)
            )

        self.coordinates = {}

    def capture_image(self):
        ret, frame = self.cap.read()
        if ret:
            coordinates = {}

            for color, (lower_bound, upper_bound, color_value) in self.color_ranges.items():
                mask = cv2.inRange(frame, lower_bound, upper_bound)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                coordinates[color] = []

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 100:
                        x, y, w, h = cv2.boundingRect(contour)
                        coordinates[color].append((x, y, w, h))

                        cv2.rectangle(frame, (x, y), (x + w, y + h), color_value, 2)
                        cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color_value, 2)

            self.coordinates = coordinates

            cv2.imshow('Webcam Feed', frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            raise Exception("Failed to capture image")
        
    def get_coordinates(self):
        return self.coordinates

    def release(self):
        if self.cap.isOpened():
            self.cap.release()
