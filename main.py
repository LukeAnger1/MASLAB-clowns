
import cv2
from cv_bridge import CvBridge
import numpy as np

# Calibration parameter (focal length over sensor width)
CALIBRATION = 1

# Object width (meters)
OBJECT_WIDTH = 8.48e-2

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


class CubeDetect():

    def __init__(self):
        self.bridge = CvBridge()

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

        filtered_green = filter_overlapping_contours(contours_green)
        filtered_red = filter_overlapping_contours(contours_red)

        answer_green, answer_red = [], []

        # Filter contours by size
        for contour in filtered_green:
            area = cv2.contourArea(contour)
            if area < 500:  # Filter out small contours
                continue
            x, y, w, h = cv2.boundingRect(contour)
            answer_green.append((x + w//2, y+h))

        for contour in filtered_red:
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

    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  

        green_pixels, red_pixels = self.get_possible_pixel_locations_with_blur(frame)

        return (green_pixels, red_pixels)

##########################3 The above is the detect code

import cv2
import numpy as np

# This is the image plain directly from rqt pixels
PTS_IMAGE_PLANE = [[371, 424],
                    [366, 326],
                    [222, 379],
                    [217, 74],
                    [345, 21]
                   ]

# This is the measured plain in inches
PTS_GROUND_PLANE = [[0, 1],
                    [0, 3],
                    [-3, 2],
                    [-3, 9],
                    [0, 10]
                    ]

class CubeLocate():

    def __init__(self):
        #Initialize data into a homography matrix
        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

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

    def transform_callback(self, msg):
        # Calculate pixel of the center of the bottom of the cube.
        # NOTE: This is already done in the previous node

        # Use transformUvToXy to calculate the world coordinate of the bottom of the cube.
        green_pixels = [self.convert_int32_to_pixel_location(value) for value in msg[0]]
        red_pixels = [self.convert_int32_to_pixel_location(value)for value in msg[1]]

        # Use cv2.circle to add the center bottom point to the image for debugging.
        # Use cv2.putText to add text of the world coordinate to the image for debugging.
        # NOTE: This is currently being done in the previous node

        # This is test code
        # self.get_logger().info(f'the green pixel locations are {green_pixels}\nthe red pixel locations are {red_pixels}')

        green_map_locations = [self.transformUvToXy(pixel[0], pixel[1]) for pixel in green_pixels]
        red_map_locations = [self.transformUvToXy(pixel[0], pixel[1]) for pixel in red_pixels]

        # # This is test code
        print(f'G: {[value for value in green_map_locations]}')
        print(f'R: {[value for value in red_map_locations]}')

        # msg = MapLocations()
        green_x_locations = (map_location[0] for map_location in green_map_locations)
        green_y_locations = (map_location[1] for map_location in green_map_locations)
        red_x_locations = (map_location[0] for map_location in red_map_locations)
        red_y_locations = (map_location[1] for map_location in red_map_locations)

        # # self.get_logger().info(f'in the locate the green x locations are {msg.green_x_locations} while the actual ones are {(map_location[0] for map_location in green_map_locations)}')

        # self.cube_image_pub.publish(msg)
        return (green_x_locations, green_y_locations, red_x_locations, red_y_locations)
    
############## The above is the locate 

import time

class DecisionNode():
    def __init__(self):

        # NOTE: This is so it drives forward on the start to find a green block
        self.closest_green = (float(0), float(69))

        # This is the start time
        self.start_time = int(time.time())

    def distance_squared(self, x1, y1, x2=0, y2=0):
        """
        This function returns the distance squared between these 2 points
        """
        return (x1-x2)**2+(y1-y2)**2

    def map_generator(self, msg):

        # Check if the game if over with the time
        if (time.time() - self.start_time) > 150:
            print(f'We hit the 230 mark on the time')
            return

        # After updating the map this will update the goal destination for the motors

        # self.get_logger().info(f'the green x locations {msg.green_x_locations} of type {type(msg.green_x_locations)}')

        green_x_locations = msg[0]
        green_y_locations = msg[1]
        red_x_locations = msg[2]
        red_y_locations = msg[3]

        # For now we are only going to go to the closest green block
        current_min_dist = 100000000
        # self.get_logger().info(f'the green x locations {green_x_locations}')
        for green_x, green_y in zip(green_x_locations, green_y_locations):
            # self.get_logger().info(f'the green possible location is ({green_x}, {green_y})')
            possible_min_dist = self.distance_squared(green_x, green_y)
            if (possible_min_dist < current_min_dist):
                current_min_dist = possible_min_dist
                self.closest_green = (green_x, green_y)

        # msg = GoalDestination()

        x = self.closest_green[0]
        y = self.closest_green[1]

        # self.get_logger().info(f'publishing the goal destination. x: {msg.x}, y: {msg.y} ')
        # publish the message
        # self.motor_destination.publish(msg)
        return (x, y)

############################# This is the decision making node

import numpy as np

from raven.raven import Raven
raven_board = Raven()

import math
from time import sleep

# This constant is how much to scale the angles by
RADIANS_MULTIPLIER = 5

# This is a constant to check if we are turning or driving forward
EPISOLON_ANGLE = 0.7

class DriveNode():
    def __init__(self):

        # These are values to control driving at basic level
        self.drive = False
        self.angle = 0
        self.goal_speed = 25

        # PID control parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain
        self.previous_error = 0
        self.integral = 0

    def update_motors(self, msg):
        goal_x = msg[0]
        goal_y = msg[1]
        # self.get_logger().info(f"Updating Motor Values. x: {goal_x}, y: {goal_y}")

        # Safety check to avoid division by zero
        if np.absolute(goal_y) < 0.05:
            # self.get_logger().warning("Goal y is zero. Setting angle to 0.")
            self.angle = 0
        else:
            self.angle = math.atan(goal_x / goal_y)
            # self.get_logger().info(f"the angle is {self.angle}")

        self.drive = True
        self.run_motors()

    def run_motors(self):
        # Apply PID control to dynamically adjust speed
        error = self.goal_speed# - self.current_speed
        self.integral += error
        derivative = error - self.previous_error

        # adjustment = (
        #     self.kp * error +
        #     self.ki * self.integral +
        #     self.kd * derivative
        # )
        self.previous_error = error

        adjustment = 0

        print(f'the angle is {self.angle}')

        # If angle is straight enough
        if (abs(self.angle) < EPISOLON_ANGLE):
            # Go straighter
            print("Going stragiht ish")

            # Calculate angle-adjusted speeds
            angle_measurement = self.angle * RADIANS_MULTIPLIER
            motor1_speed = max(0, self.goal_speed - angle_measurement + adjustment)
            motor2_speed = max(0, self.goal_speed + angle_measurement + adjustment)

            # Configure motor 1
            raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)
            raven_board.set_motor_torque_factor(Raven.MotorChannel.CH4, 85)
            raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, motor1_speed)

            # Configure motor 2
            raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.DIRECT)
            raven_board.set_motor_torque_factor(Raven.MotorChannel.CH5, 85)
            raven_board.set_motor_speed_factor(Raven.MotorChannel.CH5, motor2_speed, reverse=True)

        # If angle is not straight enough
        else:

            # We do hard turn
            print("Doing hard turn ish")

            # Calculate angle-adjusted speeds
            # angle_measurement = self.angle * RADIANS_MULTIPLIER
            # motor1_speed = max(0, self.goal_speed - angle_measurement + adjustment)
            # motor2_speed = max(0, self.goal_speed + angle_measurement + adjustment)

            # This is a sign to be able to flip the motors faster
            sign = self.angle < 0

            turn_goal_speed =35

            # Configure motor 1
            raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)
            raven_board.set_motor_torque_factor(Raven.MotorChannel.CH4, 85)
            raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, turn_goal_speed, reverse=sign)

            # Configure motor 2
            raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.DIRECT)
            raven_board.set_motor_torque_factor(Raven.MotorChannel.CH5, 85)
            raven_board.set_motor_speed_factor(Raven.MotorChannel.CH5, turn_goal_speed, reverse=(sign))

        # self.get_logger().info(f"Running Motors: Motor1 Speed={motor1_speed}, Motor2 Speed={motor2_speed}")
        sleep(.1)  # Pause for stability

    def shutdown_motors(self):
        print("Shutting down motors...")

        # Set both motors to 0 speed
        raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)
        raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, 0)

        raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.DIRECT)
        raven_board.set_motor_speed_factor(Raven.MotorChannel.CH5, 0)

        print("Motors have been shut down.")

def main():
    cube_detect = CubeDetect()
    cube_locate = CubeLocate()
    decision_node = DecisionNode()
    drive_control = DriveNode()

    while True:
        
        hmm = cube_detect.image_callback(img)
        hmm = cube_locate.transform_callback(hmm)
        hmm = decision_node.map_generator(hmm)
        hmm = drive_control.update_motors(hmm)

if __name__ == '__main__':
    main()