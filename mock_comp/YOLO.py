from ultralytics import YOLO
import cv2
import numpy as np
import math

from camera import get_camera_shape

camera_width, _ = get_camera_shape()

# make sure distance calculation accounts for camera height being off the floor where the blocks are

# Load an image
# image_path = "image.png"  # Replace with your image path
# image = cv2.imread(image_path)

def get_block_placement(image):
    """
    This will return a tuple of (distance, angle) where distance is the distance to whatever YOLO detects and the angle is the angle from the center of the camera while the right is the positive direction in radians
    """

    # If the image is none return none
    if image is None:
        return None

    # Load a YOLO model (pre-trained YOLOv8 nano model)
    model = YOLO('yolov8n.pt')  # Replace with your model if needed
    
    # Run object detection
    results = model(image)
    FOCAL_LENGTH = 1920 # 1920 pixels for a 1080p camera
    REAL_WORLD_SIZE = 2 # assuming straight, front-facing cube, not tilted/diagonal

    # HSV color ranges for red and green (tune these values as needed)
    LOWER_RED_1 = np.array([0, 50, 50])
    UPPER_RED_1 = np.array([10, 255, 255])
    LOWER_RED_2 = np.array([170, 50, 50])  # Handle red wrapping around in HSV
    UPPER_RED_2 = np.array([180, 255, 255])

    LOWER_GREEN = np.array([35, 50, 50])
    UPPER_GREEN = np.array([85, 255, 255])

    # Iterate over the results
    for result in results:
        boxes = result.boxes      # Access bounding box outputs
        masks = result.masks      # Access segmentation masks (if applicable)
        keypoints = result.keypoints  # Access keypoints (if applicable)
        probs = result.probs      # Access classification probabilities (if applicable)
        obb = result.obb          # Access oriented bounding boxes (if applicable)

    # Extract the first result (since we're working with one image)
    result = results[0]

    # Iterate through the bounding boxes
    for box in result.boxes:
        # Extract coordinates (x1, y1, x2, y2) and confidence
        x1, y1, x2, y2 = box.xyxy[0]  # Get the top-left and bottom-right coordinates
        int_x1, int_y1, int_x2, int_y2 = map(int, box.xyxy[0])
        confidence = box.conf[0]      # Get the confidence score

        # Print the values
        # print(f"Box Coordinates: ({x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f})")
        print(f"Confidence: {confidence:.2f}")

        # Bounding box dimensions
        box_width = x2 - x1           # Width of the box in pixels

        # DISTANCE CALCULATION
        distance = (FOCAL_LENGTH * REAL_WORLD_SIZE) / box_width

        # Print block details
        box_width = x2 - x1  # Width of the box in pixels
        distance = (FOCAL_LENGTH * REAL_WORLD_SIZE) / box_width

        print(f"Box Coordinates: ({x1}, {y1}, {x2}, {y2})")
        print(f"Distance to Object: {distance:.2f} inches")

        # find the midpoint of the box
        mid_point = (x1 + x2) // 2

        global camera_width

        angle_estimate = (mid_point - camera_width // 2) / camera_width * math.pi

        print(f'the angle is {angle_estimate}')

        return (distance, angle_estimate)