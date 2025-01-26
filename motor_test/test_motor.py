from raven.raven import Raven

raven_board = Raven()

channel = Raven.MotorChannel.CH5


raven_board.set_motor_mode(channel, Raven.MotorMode.DIRECT)

raven_board.set_motor_torque_factor(channel, 50)
raven_board.set_motor_speed_factor(channel, 50)

while True:
    pass