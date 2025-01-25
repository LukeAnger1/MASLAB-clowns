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

    def get_possible_pixel_locations_with_blur(self, image):
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

        # filtered_green = filter_overlapping_contours(contours_green)
        # filtered_red = filter_overlapping_contours(contours_red)

        answer_green, answer_red = [], []

        # Filter contours by size
        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area < 500:  # Filter out small contours
                continue
            x, y, w, h = cv2.boundingRect(contour)
            answer_green.append((x + w//2, y+h))

        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area < 500:  # Filter out small contours
                continue
            x, y, w, h = cv2.boundingRect(contour)
            answer_red.append((x+w//2, y+h))


        return answer_green, answer_red

    def convert_pixel_location_to_int32(self, x, y):
        # Add this number to gaurantee they are always positive when doing the encoding
        shift = 1 << 15
        shift = np.int32(shift)
        x, y = x+shift, y+shift

        return np.int32((x << 16) + y)

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  

        green_pixels, red_pixels = self.get_possible_pixel_locations_with_blur(frame)

        # cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)

        pixels_msg = PixelLocations()
        
        pixels_msg.green_pixel_locations = [self.convert_pixel_location_to_int32(*pixel) for pixel in green_pixels]
        pixels_msg.red_pixel_locations = [self.convert_pixel_location_to_int32(*pixel) for pixel in red_pixels]

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