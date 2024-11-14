import cv2

class Calibration:
    """Class for webcam-based point calibration to capture four user-defined points on a video feed."""

    def __init__(self, start_index=0):
        """
        Initialize the Calibration class with webcam setup and point configuration.
        """
        self.start_index = start_index
        self.cap = None
        self.webcam_index = -1
        self.points = []
        self.point_names = ["upper-right", "upper-left", "bottom-left", "bottom-right"]

        self._initialize_webcam()

    def _initialize_webcam(self):
        """Attempt to find and initialize a working webcam starting from the given index."""
        for i in range(self.start_index, 10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Webcam found at index {i}")
                self.cap = cap
                self.webcam_index = i
                return
            cap.release()  # Release if not working to continue search
        print("No working webcam found.")

    def mark_point(self, event, x, y, flags, param):
        """
        Mouse callback to capture a point on left-click and prompt for the next point.
        """
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            # Add the point and print its name
            self.points.append((x, y))
            current_point = self.point_names[len(self.points) - 1]
            print(f"Marked {current_point} point at ({x}, {y})")
            
            # Prompt for the next point or inform completion
            if len(self.points) < 4:
                print(f"Please mark the {self.point_names[len(self.points)]} point.")
            else:
                print("All points marked. Press Enter to finish or Backspace to delete the last point.")

    def display_webcam(self):
        """Display the webcam feed and allow the user to mark points on the frame."""
        if not self.cap:
            print("No webcam available for display.")
            return

        cv2.namedWindow("Webcam")
        cv2.setMouseCallback("Webcam", self.mark_point)
        print("Please mark the upper-left point.")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Can't receive frame. Exiting...")
                break

            # Draw points and connecting lines on the frame
            self._draw_points_and_lines(frame)
            cv2.imshow("Webcam", frame)
            
            # Handle key presses for marking and navigating
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Exiting on 'q' key press.")
                break
            elif key == 8:  # Backspace key
                self._remove_last_point()
            elif key == 13:  # Enter key
                print("Calibration completed.")
                break

        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()

        if self.points:
            print("Final marked points:", self.points)

    def _draw_points_and_lines(self, frame):
        """Draw marked points and connecting lines on the video frame."""
        for i, point in enumerate(self.points):
            cv2.circle(frame, point, 5, (0, 255, 0), -1)
            if i > 0:
                cv2.line(frame, self.points[i - 1], point, (0, 255, 0), 2)
        if len(self.points) == 4:
            cv2.line(frame, self.points[-1], self.points[0], (0, 255, 0), 2)

    def _remove_last_point(self):
        """Remove the last marked point and prompt for the next available point."""
        if self.points:
            removed_point = self.points.pop()
            print(f"Removed {self.point_names[len(self.points)]} point at {removed_point}")
            if len(self.points) < 4:
                next_point = self.point_names[len(self.points)]
                print(f"Please mark the {next_point} point.")

# Usage example
if __name__ == "__main__":
    calibration = Calibration(start_index=1)
    calibration.display_webcam()
