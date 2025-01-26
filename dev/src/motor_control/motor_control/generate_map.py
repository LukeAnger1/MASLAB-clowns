import rclpy
from rclpy.node import Node

from pixels_interfaces.msg import MapLocations

# TODO import the number message used for the numbers topic

class MapGeneratorNode(Node):
    def __init__(self):
        super().__init__('map_generator')
        self.get_logger().info("Starting the map generator")

        # create a publisher object to send data
        self.optimal_map_location_pub = self.create_publisher(MapLocations, "cube_locations/optimize_map_locations", 10)

        self.map_location_sub = self.create_subscription(MapLocations, "cube_locations/map_locations", self.map_callback, 10)

    def map_callback(self, msg):
        # IMPORTANT TODO: Keep track of previous maps and use that data to remove cubes that are misidentified and use previous readings average for a better prediction

        green_x_locations = msg.green_x_locations
        green_y_locations = msg.green_y_locations
        red_x_locations = msg.red_x_locations
        red_y_locations = msg.red_y_locations

        # IMPORTANT TODO: Code here to add in blocks that are tmeporarily not seen but remembered or misidentifications

        # IMPORTANT TODO: Code here to add in blocks that are tmeporarily not seen but remembered or misidentifications

        msg = MapLocations()
        msg.green_x_locations = green_x_locations
        msg.green_y_locations = green_y_locations
        msg.red_x_locations = red_x_locations
        msg.red_y_locations = red_y_locations

        # publish the message
        self.optimal_map_location_pub.publish(msg)

def main(args=None):
    rclpy.init()
    node = MapGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()