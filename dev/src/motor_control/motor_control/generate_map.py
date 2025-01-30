import rclpy
from rclpy.node import Node

from pixels_interfaces.msg import MapLocations

# TODO import the number message used for the numbers topic

class MapGeneratorNode(Node):
    def __init__(self):
        super().__init__('map_generator')
        self.get_logger().info("Starting the map generator")

        # create a publisher object to send data
        self.optimal_map_location_pub = self.create_publisher(MapLocations, "cube_locations/optimize_map_locations", 1)

        self.map_location_sub = self.create_subscription(MapLocations, "cube_locations/map_locations", self.map_callback, 1)
        self.map_location_sub = self.create_subscription(MapLocations, "cube_locations/map_locations2", self.map_callbacky, 1)

        self.map_location1 = None
        self.map_location2 = None

    def map_callbacky(self, msg):
        # IMPORTANT TODO: Keep track of previous maps and use that data to remove cubes that are misidentified and use previous readings average for a better prediction

        green_x_locations = msg.green_x_locations
        green_y_locations = msg.green_y_locations
        red_x_locations = msg.red_x_locations
        red_y_locations = msg.red_y_locations

        # IMPORTANT TODO: Code here to add in blocks that are tmeporarily not seen but remembered or misidentifications

        # Save the map location 2 for reuse
        self.map_location2 = (green_x_locations, green_y_locations, red_x_locations, red_y_locations)

        # Check if the other one is None and dont move on if so
        if self.map_location1 is None:
            return
        
        # Add together the information
        green_x_locations = green_x_locations + self.map_location1[0]
        green_y_locations = green_y_locations + self.map_location1[1]
        red_x_locations = red_x_locations + self.map_location1[2]
        red_y_locations = red_y_locations + self.map_location1[3]

        # IMPORTANT TODO: Code here to add in blocks that are tmeporarily not seen but remembered or misidentifications
        # self.get_logger().info(f'the green x locations about to be published are {green_x_locations[0]}')
        msg = MapLocations()
        msg.green_x_locations = green_x_locations
        msg.green_y_locations = green_y_locations
        msg.red_x_locations = red_x_locations
        msg.red_y_locations = red_y_locations

        # publish the message
        self.get_logger().info("Publishing the optimized map")
        self.optimal_map_location_pub.publish(msg)

    def map_callback(self, msg):
        # IMPORTANT TODO: Keep track of previous maps and use that data to remove cubes that are misidentified and use previous readings average for a better prediction

        green_x_locations = msg.green_x_locations
        green_y_locations = msg.green_y_locations
        red_x_locations = msg.red_x_locations
        red_y_locations = msg.red_y_locations

        # IMPORTANT TODO: Code here to add in blocks that are tmeporarily not seen but remembered or misidentifications

        # Save the map location 2 for reuse
        self.map_location1 = (green_x_locations, green_y_locations, red_x_locations, red_y_locations)

        # Check if the other one is None and dont move on if so
        if self.map_location2 is None:
            return
        
        # Add together the information
        green_x_locations = green_x_locations + self.map_location2[0]
        green_y_locations = green_y_locations + self.map_location2[1]
        red_x_locations = red_x_locations + self.map_location2[2]
        red_y_locations = red_y_locations + self.map_location2[3]

        # IMPORTANT TODO: Code here to add in blocks that are tmeporarily not seen but remembered or misidentifications
        # self.get_logger().info(f'the green x locations about to be published are {green_x_locations[0]}')
        msg = MapLocations()
        msg.green_x_locations = green_x_locations
        msg.green_y_locations = green_y_locations
        msg.red_x_locations = red_x_locations
        msg.red_y_locations = red_y_locations

        # publish the message
        self.get_logger().info("Publishing the optimized map")
        self.optimal_map_location_pub.publish(msg)

def main(args=None):
    rclpy.init()
    node = MapGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()