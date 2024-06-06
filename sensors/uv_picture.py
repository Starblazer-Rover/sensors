import cv2
import numpy as np

# Open a connection to the first video device (typically the webcam)
video_capture = cv2.VideoCapture(8)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Check if the video capture is opened successfully
if not video_capture.isOpened():
    print("Error: Could not open video device.")
else:
    # Capture a single frame
    ret, frame = video_capture.read()

    adjusted_image = np.clip(frame * 0.8, 0, 255).astype(np.uint8)

    if ret:
        # Save the captured frame as a PNG file in the current working directory
        cv2.imwrite("captured_image.png", adjusted_image)
        print("Image saved as captured_image.png")
    else:
        print("Error: Could not read frame from video device.")

    # Release the video capture object
    video_capture.release()

# Clean up any OpenCV windows
cv2.destroyAllWindows()
