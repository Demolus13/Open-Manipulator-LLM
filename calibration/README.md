# Color Object Detection with OpenCV

This project is a Python program that uses OpenCV to detect specific color objects in a live video feed from a camera. The program captures video frames, applies color filters to isolate specified colors, and marks detected objects with bounding boxes. This setup could be useful for applications like color-based object tracking, simple color recognition, or robotics projects involving color-coded objects.

---

## Features

- **Automatic Camera Connection**: Connects to a camera, automatically selecting an active video source.
- **Color Filtering**: Filters objects by predefined colors using BGR color ranges.
- **Bounding Boxes and Labels**: Draws bounding boxes around detected objects and displays their color names.
- **Real-Time Display**: Displays masked images and the full camera feed with bounding boxes in real-time.
- **Coordinate Capture**: Allows you to capture specific object coordinates for further processing.

---

## Requirements

- Python 3.x
- OpenCV
- NumPy

To install the required packages, use:

```bash
pip install opencv-python-headless numpy
```

## How It Works

1. **Camera Initialization**  
   The program attempts to connect to a camera device, iterating through possible indices (1-9) until it finds an active camera. Once connected, it captures frames from the active video source for processing.

2. **Color Detection**  
   A dictionary of color ranges in BGR format is predefined to detect specific colors. For each frame, the program applies binary masks based on these color ranges, isolating areas of the image that match the specified colors.

3. **Contour Detection**  
   For each color mask, contours are extracted to identify regions that match the specified color range. Contours allow the program to identify the boundaries of colored areas within each frame.

4. **Bounding Box and Labeling**  
   Objects with a contour area larger than a defined threshold (500 pixels) are marked with a red bounding box. The bounding box includes a label that displays the name of the detected color.

5. **Real-Time Display**  
   The processed video feed, with bounding boxes and color names, is displayed in real time. Additionally, individual color masks (optional) can be shown alongside the main feed for better visualization.

6. **Coordinate Logging**  
   The program logs the coordinates of detected objects in each frame. These coordinates are continuously updated, allowing for further processing or data collection based on object positions.

---

## Example Color Ranges

Modify the `color_ranges` dictionary to detect different colors. The format is:

```python
color_ranges = {
    'red': ([0, 0, 88], [105, 28, 155]),
    'purple': ([81, 25, 0], [126, 58, 255]),
    'yellow': ([60, 115, 114], [127, 178, 185]),
    'orange': ([0, 36, 89], [75, 255, 190])
}

## Usage

### Running the Program

1. **Clone or Save the Code**  
   Clone this repository or save the code file locally.

2. **Define Color Ranges**  
   Modify the `color_ranges` dictionary in the code to define the color ranges you want to detect. The colors are specified in BGR format (Blue, Green, Red).

3. **Run the Program**  
   Once the color ranges are defined, run the program with the following command:

   ```bash
   python color_object_detection.py

