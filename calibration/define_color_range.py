import cv2
import numpy as np

def nothing(x):
    pass

# Create a window
cv2.namedWindow('Masked Image')

# Create trackbars for lower bound color change
cv2.createTrackbar('Lower H', 'Masked Image', 0, 179, nothing)  # Hue range is 0-179 in OpenCV
cv2.createTrackbar('Lower S', 'Masked Image', 0, 255, nothing)
cv2.createTrackbar('Lower V', 'Masked Image', 0, 255, nothing)

# Create trackbars for upper bound color change
cv2.createTrackbar('Upper H', 'Masked Image', 179, 179, nothing)
cv2.createTrackbar('Upper S', 'Masked Image', 255, 255, nothing)
cv2.createTrackbar('Upper V', 'Masked Image', 255, 255, nothing)

# Capture video from the webcam
for i in range(1, 10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        break
else:
    print("Cannot open the webcam")
    exit()

while True:
    # Read the frame
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get current positions of the trackbars for lower bound
    lower_h = cv2.getTrackbarPos('Lower H', 'Masked Image')
    lower_s = cv2.getTrackbarPos('Lower S', 'Masked Image')
    lower_v = cv2.getTrackbarPos('Lower V', 'Masked Image')

    # Get current positions of the trackbars for upper bound
    upper_h = cv2.getTrackbarPos('Upper H', 'Masked Image')
    upper_s = cv2.getTrackbarPos('Upper S', 'Masked Image')
    upper_v = cv2.getTrackbarPos('Upper V', 'Masked Image')

    # Define the lower and upper bounds for the color
    lower_bound = np.array([lower_h, lower_s, lower_v])
    upper_bound = np.array([upper_h, upper_s, upper_v])

    # Create a mask
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    masked_image = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the masked image
    cv2.imshow('Masked Image', masked_image)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and destroy all windows
cap.release()
cv2.destroyAllWindows()

# Print the selected color ranges
print(f"Selected color range:\nLower bound: [{lower_h}, {lower_s}, {lower_v}]\nUpper bound: [{upper_h}, {upper_s}, {upper_v}]")
