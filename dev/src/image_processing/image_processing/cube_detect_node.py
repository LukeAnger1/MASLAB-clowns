from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class CubeDetect(Node):

    # Calibration parameter (focal length over sensor width)
    CALIBRATION = 1

    # Object width (meters)
    OBJECT_WIDTH = 8.48e-2

    last_frame = None

    def __init__(self):
        self.bridge = CvBridge()
        super().__init__('cube_detect')

        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        self.cube_image_pub = self.create_publisher(CompressedImage, "cube_image/compressed", 10)

    def print_predicted_distances(self):
        # Note: This is what we should fine tune to make our block detection better

        if self.last_frame is None:
            return

        distance, x, y, w, h = self.compute_distance(self.last_frame)

        self.get_logger().info(f'the distance is {distance} the x is {x} the y is {y} the w is {w} the h is {h}\n')

        # Note: This is what we should fine tune to make our block detection better  

    def compute_distance(self, image):
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

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  

        self.get_logger().info("calling in here")

        self.print_predicted_distances()

        cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.cube_image_pub.publish(cube_image_msg)

def main(args=None):
    rclpy.init(args=args)

    cube_detect = CubeDetect()

    rclpy.spin(cube_detect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cube_detect.destroy_node()
    rclpy.shutdown()