import cv2
from cv_bridge import CvBridge
import numpy as np

# This is the start time
start_time = int(time.time())

# Calibration parameter (focal length over sensor width)
CALIBRATION = 1

# Object width (meters)
OBJECT_WIDTH = 8.48e-2

bridge = CvBridge()

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
        answer_green.append((x + w//2, y+h))

    for contour in filtered_red:
        area = cv2.contourArea(contour)
        if area < 500:  # Filter out small contours
            continue
        x, y, w, h = cv2.boundingRect(contour)
        answer_red.append((x+w//2, y+h))


    return answer_green, answer_red

def image_callback(msg):
    global bridge

    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  

    green_pixels, red_pixels = get_possible_pixel_locations_with_blur(frame)

    return (green_pixels, red_pixels)

####################################33 cube detect above

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

#Initialize data into a homography matrix
np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

h, err = cv2.findHomography(np_pts_image, np_pts_ground)

def transformUvToXy(u, v):
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
    xy = np.dot(h, homogeneous_point)
    scaling_factor = 1.0 / xy[2, 0]
    homogeneous_xy = xy * scaling_factor
    x = homogeneous_xy[0, 0]
    y = homogeneous_xy[1, 0]
    return x, y

def convert_int32_to_pixel_location(value):

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

def transform_callback(msg):
    # Calculate pixel of the center of the bottom of the cube.
    # NOTE: This is already done in the previous node

    # Use transformUvToXy to calculate the world coordinate of the bottom of the cube.
    green_pixels = msg[0]
    red_pixels = msg[1]

    # Use cv2.circle to add the center bottom point to the image for debugging.
    # Use cv2.putText to add text of the world coordinate to the image for debugging.
    # NOTE: This is currently being done in the previous node

    # This is test code
    # self.get_logger().info(f'the green pixel locations are {green_pixels}\nthe red pixel locations are {red_pixels}')

    green_map_locations = [transformUvToXy(pixel[0], pixel[1]) for pixel in green_pixels]
    red_map_locations = [transformUvToXy(pixel[0], pixel[1]) for pixel in red_pixels]

    # This is test code
    print(f'G: {[value for value in green_map_locations]}')
    print(f'R: {[value for value in red_map_locations]}')

    # msg = MapLocations()
    green_x_locations = (map_location[0] for map_location in green_map_locations)
    green_y_locations = (map_location[1] for map_location in green_map_locations)
    red_x_locations = (map_location[0] for map_location in red_map_locations)
    red_y_locations = (map_location[1] for map_location in red_map_locations)

    return (green_x_locations, green_y_locations, red_x_locations, red_y_locations)

######################## The above is the cube locate command

import time

# NOTE: This is so it drives forward on the start to find a green block
# self.closest_green = (float(0), float(69))

def distance_squared(self, x1, y1, x2=0, y2=0):
    """
    This function returns the distance squared between these 2 points
    """
    return (x1-x2)**2+(y1-y2)**2

def map_generator(self, msg):

    # Check if the game if over with the time
    if (time.time() - self.start_time) > 150:
        self.get_logger().info(f'We hit the 230 mark on the time')
        return

    # After updating the map this will update the goal destination for the motors

    # self.get_logger().info(f'the green x locations {msg.green_x_locations} of type {type(msg.green_x_locations)}')

    green_x_locations = msg.green_x_locations
    green_y_locations = msg.green_y_locations
    red_x_locations = msg.red_x_locations
    red_y_locations = msg.red_y_locations

    # For now we are only going to go to the closest green block
    current_min_dist = 100000000
    closest_green = None
    # self.get_logger().info(f'the green x locations {green_x_locations}')
    for green_x, green_y in zip(green_x_locations, green_y_locations):
        # self.get_logger().info(f'the green possible location is ({green_x}, {green_y})')
        possible_min_dist = distance_squared(green_x, green_y)
        if (possible_min_dist < current_min_dist):
            current_min_dist = possible_min_dist
            closest_green = (green_x, green_y)

    msg = GoalDestination()

    if closest_green is not None:
        # msg.x = closest_green[0]
        # msg.y = closest_green[1]
        return 
    else:
        # This is so it turns to search
        msg.x = float(69)
        msg.y = float(0)

    # self.get_logger().info(f'publishing the goal destination. x: {msg.x}, y: {msg.y} ')
    # publish the message
    self.motor_destination.publish(msg)
