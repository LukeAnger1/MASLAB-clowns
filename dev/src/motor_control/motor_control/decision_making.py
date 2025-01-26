import rclpy
from rclpy.node import Node

from pixels_interfaces.msg import MapLocations
from decision_interfaces.msg import GoalDestination

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.get_logger().info("Starting the decision node")

        # create a publisher object to send data
        self.motor_destination = self.create_publisher(GoalDestination, "motor_control/goal_destination", 10)

        self.number_sub = self.create_subscription(MapLocations, "cube_locations/optimize_map_locations", self.map_generator, 10)

    def map_generator(self, msg):
        # # this function is called whenever a number is received.

        # number = msg.data 

        # fizzbuzz_str = self.fizzbuzz(number)
        # # loginfo to print the string to the terminal
        # self.get_logger().info(fizzbuzz_str)

        # fizzbuzz_msg = FizzBuzz()
        # fizzbuzz_msg.fizzbuzz = fizzbuzz_str
        # fizzbuzz_msg.fizz_ratio = 0 # TODO fill in this value
        # fizzbuzz_msg.buzz_ratio = 0 # TODO fill in this value
        # fizzbuzz_msg.fizzbuzz_ratio = 0 # TODO fill in this value
        # fizzbuzz_msg.number_total = 0 # TODO fill in this value

        # # publish the message
        # self.fizzbuzz_pub.publish(fizzbuzz_msg)
        pass

def main(args=None):
    rclpy.init()
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()