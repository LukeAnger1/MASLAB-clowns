import rclpy
from rclpy.node import Node

from decision_interfaces.msg import GoalDestination

from raven.raven import Raven
raven_board = Raven()

import math
from time import sleep

# This constant is how much to scale the angles by
RADIANS_MULTIPLIER = 0

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.get_logger().info("Starting the drive node")

        self.motor_destination_sub = self.create_subscription(GoalDestination, "motor_control/goal_destination", self.update_motors, 10)

        # These are values to control driving at basic level
        self.drive = False
        self.angle = 0
        self.goal_speed = 20

    def update_motors(self, msg):

        self.get_logger().info("Updating Motor Values")
        goal_x = msg.x
        goal_y = msg.y
        self.drive = True
        self.angle = math.atan(goal_x/goal_y)

        self.run_motors()

    def run_motors(self):

        angle_measurement = self.angle * RADIANS_MULTIPLIER

        # Set one motor
        raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)

        raven_board.set_motor_torque_factor(Raven.MotorChannel.CH4, 50)

        raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, self.goal_speed - angle_measurement)

        # Set the other motor
        raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.DIRECT)

        raven_board.set_motor_torque_factor(Raven.MotorChannel.CH5, 50)

        raven_board.set_motor_speed_factor(Raven.MotorChannel.CH5, self.goal_speed + angle_measurement, reverse=True)

        self.get_logger().info("Running the Motor: " + str(self.drive))

        sleep(1)

def main(args=None):
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()