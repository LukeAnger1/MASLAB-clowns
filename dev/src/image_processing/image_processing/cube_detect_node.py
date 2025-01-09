from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class CubeDetect(Node):

    def __init__(self):
        self.bridge = CvBridge()
        super().__init__('cube_detect')

        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        self.cube_image_pub = self.create_publisher(CompressedImage, "cube_image/compressed", 10)

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Note: This is what we should fine tune to make our block detection better

        

        # Note: This is what we should fine tune to make our block detection better    

        cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.cube_image_pub.publish(cube_image_msg)

def main():
    pass