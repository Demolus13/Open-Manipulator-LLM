import os
import cv2
import numpy as np

class Camera:
    def __init__(self, color_ranges):
        self.cap = None
        for i in range(0, 10):
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                print(f"Camera {i} is active")
                break
        else:
            raise Exception("No camera is available")

        # Store color ranges in HSV
        self.color_ranges = color_ranges

        self.coordinates = {}

    def capture_image(self, show_masked_image=False):
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to HSV
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            coordinates = {}

            for color, (lower_bound, upper_bound) in self.color_ranges.items():
                mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                coordinates[color] = []

                if show_masked_image:
                    masked_image = cv2.bitwise_and(frame, frame, mask=mask)
                    cv2.imshow(f'{color} Masked Image', masked_image)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 500:  # Filter out small objects
                        x, y, w, h = cv2.boundingRect(contour)
                        coordinates[color].append((x, y, w, h))

                        # Draw rectangle and label the detected color
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            self.coordinates = coordinates

            # Show the main webcam feed with detections
            cv2.imshow('Webcam Feed', frame)
        else:
            raise Exception("Failed to capture image")

    def get_coordinates(self):
        return self.coordinates

    def start(self, coordinates, show_masked_image=False):
        while True:
            self.capture_image(show_masked_image=show_masked_image)
            coordinates.clear()
            coordinates.update(self.get_coordinates())

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.stop()

    def stop(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Define the color ranges in HSV
    color_ranges = {
        'red': ([0, 100, 100], [10, 255, 255]),  # Lower and upper bounds for red
        'green': ([35, 50, 50], [85, 255, 255]), # Lower and upper bounds for green
        'purple': ([130, 50, 50], [160, 255, 255]), # Lower and upper bounds for purple
        'orange': ([10, 100, 100], [25, 255, 255]) # Lower and upper bounds for orange
    }

    camera = Camera(color_ranges)
    coordinates = {}
    camera.start(coordinates, show_masked_image=True)
