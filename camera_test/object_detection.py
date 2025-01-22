# MASLAB 2025
# Object Detection
# 
# Developed by Joseph Hobbs
# This code is open-source
#   under the MIT License

import cv2
import numpy as np

# Capture video from webcam
capture = cv2.VideoCapture(0)

# Calibration parameter (focal length over sensor width)
CALIBRATION = 1

# Object width (meters)
OBJECT_WIDTH = 8.48e-2

def compute_distance(image):
    """
    Compute the distance of the block, given an image
    """
    # Convert the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply a mask to the image
    mask = cv2.inRange(
        hsv_image,
        np.array([60, 100, 100]),
        np.array([90, 200, 200])
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Check contours
    if contours:
        # Get the largest contour
        max_contour = max(contours, key=cv2.contourArea)

        # Find the bounding rectangle
        x, y, w, h = cv2.boundingRect(max_contour)

        # Calculate the contour width, in pixels
        width = np.sqrt(cv2.contourArea(max_contour))

        # Get the frame width, in pixels
        frame_width = bgr_image.shape[1]
    
        # Calculate the object's distance from its width (centimeters)
        if width != 0:
            distance = OBJECT_WIDTH * frame_width / width * CALIBRATION * 100
        else:
            distance = None

        return distance, x, y, w, h
    
    return None, None, None, None, None

if __name__ == "__main__":
    while True:
        # Read a frame
        _, bgr_image = capture.read()

        # Calculate the distance
        distance, x, y, w, h = compute_distance(bgr_image)
        
        # Make sure that the distance is a meaningful number
        if distance is None or distance == float("inf") or distance == float("-inf") or distance != distance:
            pass
        else:
            # Display the bounding box
            cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # Display the contour size
            cv2.putText(
                bgr_image,
                f"{round(distance)} cm",
                (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        # Display the frame in the video feed
        # NOTE: `cv2.imshow` takes images in BGR
        cv2.imshow("Video Feed", bgr_image)

        # Wait for the user to press Q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Quit the program
            break