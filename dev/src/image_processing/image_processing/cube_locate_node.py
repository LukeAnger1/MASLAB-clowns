import cv2
import numpy as np
from pixels_interfaces.msg import PixelLocations, MapLocations

import rclpy
from rclpy.node import Node

# This is the image plain directly from rqt pixels
PTS_IMAGE_PLANE = [[488, 329],
                   [316, 333],
                   [483, 213],
                   [307, 52],
                   [314, 275]
                   ]

# This is the measured plain in inches
PTS_GROUND_PLANE = [[3, 1],
                    [0, 1],
                    [3, 3],
                    [0, 6],
                    [0, 2]
                    ]

class CubeLocate(Node):

    def __init__(self):
        super().__init__('cube_locate')

        #Initialize data into a homography matrix
        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        self.image_sub = self.create_subscription(PixelLocations, "cube_locations/pixel_locations", self.transform_callback, 10)
        self.cube_image_pub = self.create_publisher(MapLocations, "cube_locations/map_locations", 10)

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def convert_int32_to_pixel_location(self, value):

        x = value >> 16
        y = value & ((1<<16)-1)

        # Undo the shift here
        shift = 1 << 15
        shift = np.int32(shift)
        x, y = x-shift, y-shift

        # IMPORTANT NOTE: CHECHY BS
        if x < -300:
            x += 65536

        return (x, y)

    def transform_callback(self, msg: PixelLocations):
        # Calculate pixel of the center of the bottom of the cube.
        # NOTE: This is already done in the previous node

        # Use transformUvToXy to calculate the world coordinate of the bottom of the cube.
        green_pixels = [self.convert_int32_to_pixel_location(value) for value in msg.green_pixel_locations]
        red_pixels = [self.convert_int32_to_pixel_location(value)for value in msg.red_pixel_locations]

        # Use cv2.circle to add the center bottom point to the image for debugging.
        # Use cv2.putText to add text of the world coordinate to the image for debugging.
        # NOTE: This is currently being done in the previous node

        # This is test code
        # self.get_logger().info(f'the green pixel locations are {green_pixels}\nthe red pixel locations are {red_pixels}')

        green_map_locations = [self.transformUvToXy(pixel[0], pixel[1]) for pixel in green_pixels]
        red_map_locations = [self.transformUvToXy(pixel[0], pixel[1]) for pixel in red_pixels]

        # This is test code
        self.get_logger().info(f'G: {[value for value in green_map_locations]}')
        self.get_logger().info(f'R: {[value for value in red_map_locations]}')

        msg = MapLocations()
        msg.green_x_locations = (map_location[0] for map_location in green_map_locations)
        msg.green_y_locations = (map_location[1] for map_location in green_map_locations)
        msg.red_x_locations = (map_location[0] for map_location in red_map_locations)
        msg.red_y_locations = (map_location[1] for map_location in red_map_locations)

        # self.get_logger().info(f'in the locate the green x locations are {msg.green_x_locations} while the actual ones are {(map_location[0] for map_location in green_map_locations)}')

        self.cube_image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    cube_detect = CubeLocate()

    rclpy.spin(cube_detect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cube_detect.destroy_node()
    rclpy.shutdown()