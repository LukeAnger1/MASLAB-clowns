
from icm42688 import ICM42688
import board

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

while not spi.try_lock():
    pass

spi.configure(baudrate=5000000)

imu = ICM42688(spi)
imu.begin()

##### When running in ROS seperate these as two sepearte nodes

# Reference: https://github.com/MASLAB/software-how-to?tab=readme-ov-file

from raven.raven import Raven

raven_board = Raven()

# IMPORTANT TODO: Make sure to set the channel later

# Set one motor
raven_board.set_motor_encoder(Raven.MotorChannel.CH?1, 0) # Reset encoder
raven_board.set_motor_mode(Raven.MotorChannel.CH?1, Raven.MotorMode.VELOCITY) # Set motor mode to VELOCITY
raven_board.set_motor_pid(Raven.MotorChannel.CH?1, p_gain = 10, i_gain = 0, d_gain = 0) # Set PID values

# Make the motor spin at -4400 counts/second (-10 rev/sec of wheel motor)
raven_board.set_motor_target(Raven.MotorChannel.CH?1, -4400)

# Set the other motor
raven_board.set_motor_encoder(Raven.MotorChannel.CH?2, 0) # Reset encoder
raven_board.set_motor_mode(Raven.MotorChannel.CH?2, Raven.MotorMode.VELOCITY) # Set motor mode to VELOCITY
raven_board.set_motor_pid(Raven.MotorChannel.CH?2, p_gain = 10, i_gain = 0, d_gain = 0) # Set PID values

# Make the motor spin at -4400 counts/second (-10 rev/sec of wheel motor)
raven_board.set_motor_target(Raven.MotorChannel.CH?2, -4400)

while True:
  accel, gyro = imu.get_data()
  # Returned linear acceleration is a tuple of (X, Y, Z) m/s^2
  # Returned gyroscope reading is a tuple of (X, Y, Z) radian/s

  # I coded PID in c before but I like this repo for PID better. I saw we just copy their code give them credit then cook my dudes
  # https://github.com/BurakDmb/DifferentialDrivePathTracking/blob/master/main.py
  # Clown Penis out
