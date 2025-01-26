import rclpy
from rclpy.node import Node

from pixels_interfaces.msg import MapLocations

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.get_logger().info("Starting the drive node")

        # IMPROTANT TODO: Change this message type
        self.number_sub = self.create_subscription(MapLocations, "motor_control/goal_destination", self.update_motors, 10)

    def update_motors(self, msg):
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
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()