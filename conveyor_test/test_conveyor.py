from raven.raven import Raven

raven_board = Raven()

raven_board.set_motor_encoder(Raven.MotorChannel.CH1, 0) # Reset encoder
# raven_board.set_motor_mode(Raven.MotorChannel.CH1, Raven.MotorMode.POSITION) # Set motor mode to POSITION
# raven_board.set_motor_pid(Raven.MotorChannel.CH1, p_gain = 20, i_gain = 0, d_gain = 0) # Set PID values

# Make the motor spin until 4400 counts (10 rev of wheel motor)
# raven_board.set_motor_target(Raven.MotorChannel.CH1, -50)

raven_board.set_motor_mode(Raven.MotorChannel.CH1, Raven.MotorMode.DIRECT)

raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 50)
raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, 50, reverse=True)

while abs(raven_board.get_motor_encoder(Raven.MotorChannel.CH1)) < 2000:
    print(f'the encoder is {raven_board.get_motor_encoder(Raven.MotorChannel.CH1)}')
    pass  # Keep looping until the motor reaches the desired position

