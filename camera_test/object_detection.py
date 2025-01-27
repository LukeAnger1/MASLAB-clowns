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

if not capture.isOpened():
    print("Error: Could not access the webcam.")
    exit()

# Calibration parameter (focal length over sensor width)
CALIBRATION = 1

# Object width (meters)
OBJECT_WIDTH = 8.48e-2

def get_possible_pixel_locations_with_blur(image):
    """
    Detect green and red objects in the image while reducing noise.
    IMPORTANT: The lists it returns are (x, y, w, h), not (x, y)
        Use on the display with boxes
    """
    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply Gaussian blur to reduce noise
    blur_factor = 7
    blurred_image = cv2.GaussianBlur(hsv_image, (blur_factor, blur_factor), 0)

    # Define color ranges
    green_lower = np.array([40, 40, 40])
    green_upper = np.array([80, 255, 255])
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Create masks
    mask_green = cv2.inRange(blurred_image, green_lower, green_upper)
    mask_red = cv2.inRange(blurred_image, lower_red1, upper_red1) + cv2.inRange(blurred_image, lower_red2, upper_red2)

    # Apply morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask_green = cv2.erode(mask_green, kernel, iterations=1)
    mask_green = cv2.dilate(mask_green, kernel, iterations=2)
    mask_red = cv2.erode(mask_red, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=2)

    # Find contours
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    filtered_green = filter_overlapping_contours(contours_green)
    filtered_red = filter_overlapping_contours(contours_red)

    answer_green, answer_red = [], []

    # Filter contours by size
    for contour in filtered_green:
        area = cv2.contourArea(contour)
        if area < 500:  # Filter out small contours
            continue
        x, y, w, h = cv2.boundingRect(contour)
        answer_green.append((x, y, w, h))

    for contour in filtered_red:
        area = cv2.contourArea(contour)
        if area < 500:  # Filter out small contours
            continue
        x, y, w, h = cv2.boundingRect(contour)
        answer_red.append((x, y, w, h))


    return answer_green, answer_red

   

def filter_overlapping_contours(contours, overlap_threshold=0.7):
    """
    Filters out contours that overlap more than a specified percentage with larger contours.
    The overlap threshold determines the percentage of the smaller box that must overlap to be removed.
    
    Parameters:
    - contours: List of contours to filter.
    - overlap_threshold: Fraction (0.0 to 1.0) of overlap required to discard a box.
    
    Returns:
    - A list of filtered contours.
    """
    filtered_contours = []
    bounding_boxes = [cv2.boundingRect(c) for c in contours]

    # Loop through each bounding box
    for i, bbox in enumerate(bounding_boxes):
        x1, y1, w1, h1 = bbox
        keep = True

        for j, other_bbox in enumerate(bounding_boxes):
            if i == j:
                continue

            x2, y2, w2, h2 = other_bbox

            # Calculate the intersection area
            intersection_x = max(0, min(x1 + w1, x2 + w2) - max(x1, x2))
            intersection_y = max(0, min(y1 + h1, y2 + h2) - max(y1, y2))
            intersection_area = intersection_x * intersection_y

            # Calculate the area of the smaller box
            smaller_area = w1 * h1

            # Check if the overlap exceeds the threshold
            if intersection_area / smaller_area > overlap_threshold:
                keep = False
                break

        if keep:
            filtered_contours.append(contours[i])

    return filtered_contours

def display_pixels_with_boxes(pixels, color):
    '''
    Displays the pixels on the screen.
    Pixels: list of pixels in output from get_possible_pixel_locations_with_blur
    IMPORTANT: pixels include (x, y, w, h)! not (x,y)
    Color: string of 'green' or 'red' for now
    '''
    if color == 'green':
        color_tuple = (0, 255, 0)
    else:    
        color_tuple = (0, 0, 255)
    # Calculate the distance
    for x, y, w, h in pixels:
    
        # Display the bounding box
        cv2.rectangle(bgr_image, (x, y), (x + w, y + h), color_tuple, 2)

        # Display the contour size
        cv2.putText(
            bgr_image,
            color + ' pixel',
            (x, y-10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            color_tuple,
            2
        )

if __name__ == "__main__":
    while True:
        # Read a frame
        _, bgr_image = capture.read()

        green_pixels, red_pixels = get_possible_pixel_locations_with_blur(bgr_image)

        # display
        display_pixels_with_boxes(green_pixels, 'green')
        display_pixels_with_boxes(red_pixels, 'red')


        # Display the frame in the video feed
        # NOTE: `cv2.imshow` takes images in BGR
        cv2.imshow("Video Feed", bgr_image)

        # Wait for the user to press Q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Quit the program
            break