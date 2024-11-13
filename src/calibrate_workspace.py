import cv2

class Calibration:
    def __init__(self, start_index=0):
        self.start_index = start_index
        self.cap = None
        self.index = -1
        self.points = []
        self.point_names = ["upper-right", "upper-left", "bottom-left", "bottom-right"]
        self._initialize_webcam()
    
    def _initialize_webcam(self):
        # Loop through indices to find a working webcam during initialization
        for i in range(self.start_index, 10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Webcam found at index {i}")
                self.cap = cap
                self.index = i
                return
            else:
                cap.release()
        print("No working webcam found.")
    
    def mark_point(self, event, x, y, flags, param):
        # Capture points on mouse click and prompt for the next point
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            # Store the current point
            self.points.append((x, y))
            print(f"Marked {self.point_names[len(self.points) - 1]} point at ({x}, {y})")
            
            # Prompt for the next point if there are more to mark
            if len(self.points) < 4:
                next_point = self.point_names[len(self.points)]
                print(f"Please mark the {next_point} point.")
            else:
                print("All points marked. Press Enter to finish or Backspace to delete the last point.")

    def display_webcam(self):
        # Display the webcam feed and handle user marking of points
        if not self.cap:
            print("No webcam to display.")
            return

        # Set up mouse callback to mark points
        cv2.namedWindow("Webcam")
        cv2.setMouseCallback("Webcam", self.mark_point)

        print("Please mark the upper-left point.")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Can't receive frame. Exiting...")
                break

            # Draw points and lines on the frame
            for i, point in enumerate(self.points):
                cv2.circle(frame, point, 5, (0, 255, 0), -1)
                if i > 0:
                    # Draw a line from the previous point to the current point
                    cv2.line(frame, self.points[i - 1], point, (0, 255, 0), 2)
            # Connect the last point back to the first, if 4 points are marked
            if len(self.points) == 4:
                cv2.line(frame, self.points[-1], self.points[0], (0, 255, 0), 2)

            cv2.imshow("Webcam", frame)
            
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Exit display on 'q' key press
                break
            elif key == 8:  # Backspace key to delete the last marked point
                if self.points:
                    removed_point = self.points.pop()
                    print(f"Removed {self.point_names[len(self.points)]} point at {removed_point}")
                    if len(self.points) < 4:
                        next_point = self.point_names[len(self.points)]
                        print(f"Please mark the {next_point} point.")
            elif key == 13:  # Enter key to exit
                break

        # Release resources after loop exits
        self.cap.release()
        cv2.destroyAllWindows()

        if self.points:
            print("Final marked points:", self.points)


# Usage example:
if __name__ == "__main__":
    webcam_display = Calibration(start_index=0)
    webcam_display.display_webcam()
