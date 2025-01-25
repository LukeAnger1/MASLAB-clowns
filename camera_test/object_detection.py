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

    green_lower = np.array([40, 40, 40])  # Lower bound for green color
    green_upper = np.array([80, 255, 255])  # Upper bound for green color

    # IMPORTANT TODO: We need to fine tune this to hit the red/green cubes, this mask isnt hitting the contours
    # Apply a mask to the image
    mask_green = cv2.inRange(
        hsv_image,
        green_lower,
        green_upper
    )

    # Define the lower and upper bounds for red color
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Create masks for the two red ranges
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    # Combine the masks to detect both ranges of red
    mask_red = mask1 + mask2

    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    answer_green, answer_red = [], []

    # Go through the contours and get them added to the array
    for contour in contours_green:

        # Find the bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        # Calculate the contour width, in pixels
        width = np.sqrt(cv2.contourArea(contour))

        # Get the frame width, in pixels
        frame_width = bgr_image.shape[1]
    
        # Get the location of the pixel in the bottom middle
        answer_green.append((x+w//2, y))

    # Go through the contours and get them added to the array
    for contour in contours_red:

        # Find the bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        # Calculate the contour width, in pixels
        width = np.sqrt(cv2.contourArea(contour))

        # Get the frame width, in pixels
        frame_width = bgr_image.shape[1]
    
        # Get the location of the pixel in the bottom middle
        answer_red.append((x+w//2, y))
    
    return (answer_green, answer_red)

if __name__ == "__main__":
    while True:
        # Read a frame
        _, bgr_image = capture.read()

        green_pixels, red_pixels = compute_distance(bgr_image)

        # Calculate the distance
        for x, y in green_pixels:
        
            # Display the bounding box
            cv2.rectangle(bgr_image, (x, y), (x, y), (0, 255, 0), 2)

            # Display the contour size
            cv2.putText(
                bgr_image,
                f'green pixel',
                (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

        # Calculate the distance
        for x, y in red_pixels:
        
            # Display the bounding box
            cv2.rectangle(bgr_image, (x, y), (x, y), (0, 0, 255), 2)

            # Display the contour size
            cv2.putText(
                bgr_image,
                f'red pixel',
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