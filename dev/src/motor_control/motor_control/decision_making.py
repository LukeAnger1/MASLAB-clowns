import rclpy
from rclpy.node import Node

from pixels_interfaces.msg import MapLocations
from decision_interfaces.msg import GoalDestination

import time

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.get_logger().info("Starting the decision node")

        # create a publisher object to send data
        self.motor_destination = self.create_publisher(GoalDestination, "motor_control/goal_destination", 1)

        self.optimized_map_sub = self.create_subscription(MapLocations, "cube_locations/optimize_map_locations", self.map_generator, 1)

        # NOTE: This is so it drives forward on the start to find a green block
        # self.closest_green = (float(0), float(69))

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
            possible_min_dist = self.distance_squared(green_x, green_y)
            if (possible_min_dist < current_min_dist):
                current_min_dist = possible_min_dist
                closest_green = (green_x, green_y)

        msg = GoalDestination()

        if closest_green is not None:
            msg.x = closest_green[0]
            msg.y = closest_green[1]
            msg.is_block = True
        else:
            # This is so it turns to search
            msg.x = 0
            msg.y = 0
            msg.is_block = False

        # self.get_logger().info(f'publishing the goal destination. x: {msg.x}, y: {msg.y} ')
        # publish the message
        self.motor_destination.publish(msg)

def main(args=None):
    rclpy.init()
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()