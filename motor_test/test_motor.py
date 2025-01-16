from raven.raven import Raven

raven_board = Raven()

raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)

raven_board.set_motor_torque_factor(Raven.MotorChannel.CH4, 100)
raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, 100)

while True:
    pass