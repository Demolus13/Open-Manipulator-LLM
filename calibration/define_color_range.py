import cv2
import numpy as np

def nothing(x):
    pass

# Create a window
cv2.namedWindow('Masked Image')

# Create trackbars for lower bound color change
cv2.createTrackbar('Lower R', 'Masked Image', 0, 255, nothing)
cv2.createTrackbar('Lower G', 'Masked Image', 0, 255, nothing)
cv2.createTrackbar('Lower B', 'Masked Image', 0, 255, nothing)

# Create trackbars for upper bound color change
cv2.createTrackbar('Upper R', 'Masked Image', 255, 255, nothing)
cv2.createTrackbar('Upper G', 'Masked Image', 255, 255, nothing)
cv2.createTrackbar('Upper B', 'Masked Image', 255, 255, nothing)

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

    # Get current positions of the trackbars for lower bound
    lower_r = cv2.getTrackbarPos('Lower R', 'Masked Image')
    lower_g = cv2.getTrackbarPos('Lower G', 'Masked Image')
    lower_b = cv2.getTrackbarPos('Lower B', 'Masked Image')

    # Get current positions of the trackbars for upper bound
    upper_r = cv2.getTrackbarPos('Upper R', 'Masked Image')
    upper_g = cv2.getTrackbarPos('Upper G', 'Masked Image')
    upper_b = cv2.getTrackbarPos('Upper B', 'Masked Image')

    # Define the lower and upper bounds for the color
    lower_bound = np.array([lower_r, lower_g, lower_b])
    upper_bound = np.array([upper_r, upper_g, upper_b])

    # Create a mask
    mask = cv2.inRange(frame, lower_bound, upper_bound)
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
print(f"Selected color range:\nLower bound: {lower_bound}\nUpper bound: {upper_bound}")