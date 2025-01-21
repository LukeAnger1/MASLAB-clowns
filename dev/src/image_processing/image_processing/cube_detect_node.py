from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from ultralytics import YOLO

class CubeDetect(Node):

    def __init__(self):
        self.bridge = CvBridge()
        super().__init__('cube_detect')

        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        self.cube_image_pub = self.create_publisher(Float32, "cube_data/grey", 10)

    def print_predicted_distances(self, frame):
        # Note: This is what we should fine tune to make our block detection better

        cube_info = compute_distance(frame)

        distance, x1, y1, x2, y2 = cube_info

        self.get_logger().info(f'the distance is {distance} the x1 is {x1} the x2 is {x2} the y1 is {y1} the y2 is {y2}\n')

        # Note: This is what we should fine tune to make our block detection better
        return cube_info

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  

        cube_info = self.print_predicted_distances(frame)

        distance, x1, y1, x2, y2 = cube_info

        msg = Float32()

        msg.data = distance

        # cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.cube_image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    cube_detect = CubeDetect()

    rclpy.spin(cube_detect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cube_detect.destroy_node()
    rclpy.shutdown()








def compute_distance(image: CompressedImage):
    # make sure distance calculation accounts for camera height being off the floor where the blocks are
    
    # Load a YOLO model (pre-trained YOLOv8 nano model)
    model = YOLO('yolov8n.pt')  # Replace with your model if needed

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