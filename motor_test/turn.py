from raven.raven import Raven

raven_board = Raven()

# Do angles in radians
radians = 0

# This constant is how much to scale the angles by
RADIANS_MULTIPLIER = 5

# This constant is the goal speed
GOAL_SPEED = 10

ANGLE_MEASUREMENT = radians * RADIANS_MULTIPLIER

# Set one motor
raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)

raven_board.set_motor_torque_factor(Raven.MotorChannel.CH4, 50)
raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, GOAL_SPEED - ANGLE_MEASUREMENT)

# Set the other motor
raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.DIRECT)

raven_board.set_motor_torque_factor(Raven.MotorChannel.CH5, 50)
raven_board.set_motor_speed_factor(Raven.MotorChannel.CH5, GOAL_SPEED + ANGLE_MEASUREMENT, reverse=True)

while True:
    pass