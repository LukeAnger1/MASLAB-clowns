from raven.raven import Raven

raven_board = Raven()

# Set the servo 1 to -75 degrees with custom pulse microseconds
raven_board.set_servo_position(Raven.ServoChannel.CH1, -10, min_us=500, max_us=2500)