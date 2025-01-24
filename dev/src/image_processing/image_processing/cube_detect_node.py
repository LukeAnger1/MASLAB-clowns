from sensor_msgs.msg import CompressedImage
from pixels_interfaces.msg import PixelLocations

import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Calibration parameter (focal length over sensor width)
CALIBRATION = 1

# Object width (meters)
OBJECT_WIDTH = 8.48e-2

class CubeDetect(Node):

    def __init__(self):
        self.bridge = CvBridge()
        super().__init__('cube_detect')

        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        # self.cube_image_pub = self.create_publisher(CompressedImage, "cube_image/compressed", 10)
        self.cube_image_pub = self.create_publisher(PixelLocations, "cube_locations/pixel_locations", 10)

    def compute_distance(self, image):
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
            # np.array([60, 100, 100]),
            # np.array([90, 200, 200])
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

        # Apply the mask to the image
        result = cv2.bitwise_and(image, image, mask=mask_red)

        contours, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.get_logger().info(f'right before the contours')

        # Check contours
        if contours:
            # Get the largest contour
            max_contour = max(contours, key=cv2.contourArea)

            self.get_logger().info(f'getting the bounding rectanlges')
            # Find the bounding rectangle
            x, y, w, h = cv2.boundingRect(max_contour)
            self.get_logger().info(f'after getting the bounding recctangles')
            # Calculate the contour width, in pixels
            width = np.sqrt(cv2.contourArea(max_contour))

            # Get the frame width, in pixels
            frame_width = image.shape[1]
        
            # Calculate the object's distance from its width (centimeters)
            if width != 0:
                self.get_logger().info(f'the width is not zero')
                distance = OBJECT_WIDTH * frame_width / width * CALIBRATION * 100
            else:
                self.get_logger().info(f'the width is zero')
                distance = None

            return distance, x, y, w, h
        
        return None, None, None, None, None

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  

        holder = self.compute_distance(frame)

        # cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)

        pixels_msg = PixelLocations()
        
        pixels_msg.green_pixel_locations = [7]
        pixels_msg.red_pixel_locations = [8]

        self.cube_image_pub.publish(pixels_msg)

def main(args=None):
    rclpy.init(args=args)

    cube_detect = CubeDetect()

    rclpy.spin(cube_detect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cube_detect.destroy_node()
    rclpy.shutdown()