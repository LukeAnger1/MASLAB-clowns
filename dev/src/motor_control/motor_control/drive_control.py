import rclpy
from rclpy.node import Node
import numpy as np

from decision_interfaces.msg import GoalDestination

from raven.raven import Raven
raven_board = Raven()

import math
from time import sleep

# This constant is how much to scale the angles by
RADIANS_MULTIPLIER = 5

# This is a constant to check if we are turning or driving forward
EPISOLON_ANGLE = 0.7

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.get_logger().info("Starting the drive node")

        # NOTE: The last number parameter is the size of the queue for the motors so we are going to have to work on this
        self.motor_destination_sub = self.create_subscription(GoalDestination, "motor_control/goal_destination", self.update_motors, 1)

        # These are values to control driving at basic level
        self.drive = False
        self.angle = 0
        self.goal_speed = 35

        # PID control parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain
        self.previous_error = 0
        self.integral = 0

    def update_motors(self, msg):
        goal_x = msg.x
        goal_y = msg.y
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

        self.get_logger().warning(f'the angle is {self.angle}')

        # If angle is straight enough
        if (abs(self.angle) < EPISOLON_ANGLE):
            # Go straighter
            self.get_logger().warning("Going stragiht ish")

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
            self.get_logger().warning("Doing hard turn ish")

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
        self.get_logger().info("Shutting down motors...")

        # Set both motors to 0 speed
        raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)
        raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, 0)

        raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.DIRECT)
        raven_board.set_motor_speed_factor(Raven.MotorChannel.CH5, 0)

        self.get_logger().info("Motors have been shut down.")

def main(args=None):
    rclpy.init()
    node = DriveNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (CTRL+C) detected.")
        # Shut down motors before exiting
        node.shutdown_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()