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
        
        # Convert RGB input to BGR and store in color_ranges
        self.color_ranges = {}
        for color, (lower_bound, upper_bound) in color_ranges.items():
            self.color_ranges[color] = (
                np.array(lower_bound[::-1]),  # Reverse RGB to BGR
                np.array(upper_bound[::-1])
            )

        self.coordinates = {}

    def capture_image(self, show_masked_image=False):
        ret, frame = self.cap.read()
        if ret:
            coordinates = {}

            for color, (lower_bound, upper_bound) in self.color_ranges.items():
                mask = cv2.inRange(frame, lower_bound, upper_bound)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                coordinates[color] = []

                if show_masked_image:
                    masked_image = cv2.bitwise_and(frame, frame, mask=mask)
                    cv2.imshow(f'{color} Masked Image', masked_image)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 500:
                        x, y, w, h = cv2.boundingRect(contour)
                        coordinates[color].append((x, y, w, h))

                        # midpoint_x = x + w // 2
                        # midpoint_y = y + h // 2
                        # print(f"{color} object detected at ({midpoint_x}, {midpoint_y})")

                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            self.coordinates = coordinates

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

        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    color_ranges = {
        'red': ([37, 0, 0], [255, 25, 56]),
    }
    camera = Camera(color_ranges)
    coordinates = {}
    camera.start(coordinates, show_masked_image=True)
