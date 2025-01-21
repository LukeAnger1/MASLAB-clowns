from ultralytics import YOLO
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage

# Calibration parameter (focal length over sensor width)
CALIBRATION = 1

# Object width (meters)
OBJECT_WIDTH = 8.48e-2

def compute_distance(image: CompressedImage):
    # Load a YOLO model (pre-trained YOLOv8 nano model)
    model = YOLO('yolov8n.pt')  # Replace with your model if needed
    
    # make sure distance calculation accounts for camera height being off the floor where the blocks are
    
    # if image is None:
        # print(f"Failed to load image from {image_path}. Please check the file path and format.")
        # exit(1)

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

        # # Display or save the results
        # result.show()             # Display the image with annotations
        # result.save("output.jpg") # Save the annotated image

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
        # print(f"Confidence: {confidence:.2f}")

        # Bounding box dimensions
        box_width = x2 - x1           # Width of the box in pixels

        # DISTANCE CALCULATION
        distance = (FOCAL_LENGTH * REAL_WORLD_SIZE) / box_width
        # print(f"Box Width (pixels): {box_width:.2f}")
        # print(f"Distance to Object: {distance:.2f} inches")

        # COLOR RECOGNITION

        # # Crop the bounding box region from the image
        # # CANNOT HANDLE ZERO COORDINATES
        # roi = image[int_y1:int_y2, int_x1:int_x2]

        # # Convert ROI to HSV color space
        # hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # # Create masks for red and green
        # red_mask1 = cv2.inRange(hsv_roi, LOWER_RED_1, UPPER_RED_1)
        # red_mask2 = cv2.inRange(hsv_roi, LOWER_RED_2, UPPER_RED_2)
        # red_mask = red_mask1 | red_mask2  # Combine both red masks
        # green_mask = cv2.inRange(hsv_roi, LOWER_GREEN, UPPER_GREEN)

        # # Count non-zero pixels in each mask
        # red_pixels = cv2.countNonZero(red_mask)
        # green_pixels = cv2.countNonZero(green_mask)

        #     # Determine the dominant color
        # if red_pixels > green_pixels:
        #     block_color = "Red"
        # elif green_pixels > red_pixels:
        #     block_color = "Green"
        # else:
        #     block_color = "Unknown"

        # Print block details
        box_width = x2 - x1  # Width of the box in pixels
        distance = (FOCAL_LENGTH * REAL_WORLD_SIZE) / box_width
        # print(f"Box Coordinates: ({x1}, {y1}, {x2}, {y2})")
        # print(f"Distance to Object: {distance:.2f} inches")
        # print(f"Block Color: {block_color}")

        # distance from camera
        # angle? from
        return (distance, x1, y1, x2, y2)

def compute_distances_from_class(image):
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
        frame_width = image.shape[1]
    
        # Calculate the object's distance from its width (centimeters)
        distance = self.OBJECT_WIDTH * frame_width / width * self.CALIBRATION * 100

        return distance, x, y, w, h
    
    return None, None, None, None, None

if __name__ == '__main__':

    # Load an image
    image_path = "image.png"  # Replace with your image path
    image = cv2.imread(image_path)