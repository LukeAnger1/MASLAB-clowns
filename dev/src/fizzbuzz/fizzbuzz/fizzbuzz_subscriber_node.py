import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

# import our new fizzbuzz message type
from fizzbuzz_interfaces.msg import FizzBuzz 

class FizzBuzzNode(Node):
    def __init__(self):
        super().__init__('fizzbuzz')
        self.get_logger().info("Starting fizzbuzz node")

        self.total_numbers = 0
        self.total_fizz = 0
        self.total_buzz = 0
        self.total_fizzbuzz = 0

        # create a publisher object to send data
        self.fizzbuzz_pub = self.create_publisher(FizzBuzz, "fizzbuzz_stats", 10)

        self.number_sub = self.create_subscription(Int64, "numbers", self.number_callback, 10)

    def number_callback(self, msg):
        # this function is called whenever a number is recived.

        number = msg.data 

        fizzbuzz_str = self.fizzbuzz(number)
        # loginfo to print the string to the terminal
        self.get_logger().info(fizzbuzz_str)

        if fizzbuzz_str == "fizz":
            self.total_fizz += 1
        elif fizzbuzz_str == "buzz":
            self.total_buzz += 1
        elif fizzbuzz_str == "fizzbuzz":
            self.total_fizzbuzz += 1
        self.total_numbers += 1

        fizzbuzz_msg = FizzBuzz()
        fizzbuzz_msg.fizzbuzz = fizzbuzz_str
        fizzbuzz_msg.fizz_ratio = self.total_fizz / self.total_numbers
        fizzbuzz_msg.buzz_ratio = self.total_buzz / self.total_numbers
        fizzbuzz_msg.fizzbuzz_ratio = self.total_fizzbuzz / self.total_numbers
        fizzbuzz_msg.number_total = self.total_numbers

        # publish the message
        self.fizzbuzz_pub.publish(fizzbuzz_msg)

    def fizzbuzz(self, number):
        result = ""
        if number % 3 == 0:
            result += "fizz"
        if number % 5 == 0:
            result += "buzz"
        return result


def main(args=None):
    rclpy.init()
    node = FizzBuzzNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()