import time
from raven.raven import Raven

# Initialize Raven board
raven_board = Raven()

# Define encoder limits
START_POSITION = 0
TARGET_POSITION = 2000  # 2000 is good for dropping, 1450 is good for throwing
REVERSE_SPEED = 30       # Fixed slow speed for reversing
RECOVERY_OFFSET = 100   #OFFSET for returning to original position

# Reset encoder at the start
raven_board.set_motor_encoder(Raven.MotorChannel.CH1, 0)

# Function to set motor speed and direction
def move_motor(speed, reverse=False):
    raven_board.set_motor_mode(Raven.MotorChannel.CH1, Raven.MotorMode.DIRECT)
    raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)  # Fixed torque
    raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, speed, reverse= not reverse)

# Main control loop
def chechy_bs():

    speed_percent = int(50)
    if 0 <= speed_percent <= 100:
        motor_speed = speed_percent / 100 * 100  # Scale speed to 0-100%
        move_motor(motor_speed)

        # Move motor until it reaches the target position
        while raven_board.get_motor_encoder(Raven.MotorChannel.CH1) < TARGET_POSITION:
            time.sleep(0.1)

        move_motor(0)  # Stop motor at target

    move_motor(REVERSE_SPEED, reverse=True)

        # Move motor down until it reaches the start position
    while raven_board.get_motor_encoder(Raven.MotorChannel.CH1) > (START_POSITION + 200):
        time.sleep(0.1)

    # Set a wait to return to position
    time.sleep(.5)

    # Reset the motor encoder
    raven_board.set_motor_encoder(Raven.MotorChannel.CH1, 0)

    move_motor(0)  # Stop motor

chechy_bs()