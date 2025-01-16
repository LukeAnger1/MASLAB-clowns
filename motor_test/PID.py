
from icm42688 import ICM42688
import board
import busio

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
raven_board.set_motor_encoder(Raven.MotorChannel.CH4, 0) # Reset encoder
raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.VELOCITY) # Set motor mode to VELOCITY
raven_board.set_motor_pid(Raven.MotorChannel.CH4, p_gain = 10, i_gain = 0, d_gain = 0) # Set PID values

# Make the motor spin at -4400 counts/second (-10 rev/sec of wheel motor)
raven_board.set_motor_target(Raven.MotorChannel.CH4, -44)

# Set the other motor
raven_board.set_motor_encoder(Raven.MotorChannel.CH5, 0) # Reset encoder
raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.VELOCITY) # Set motor mode to VELOCITY
raven_board.set_motor_pid(Raven.MotorChannel.CH5, p_gain = 10, i_gain = 0, d_gain = 0) # Set PID values

# Make the motor spin at -4400 counts/second (-10 rev/sec of wheel motor)
raven_board.set_motor_target(Raven.MotorChannel.CH5, 44)







# raven_board.set_motor_mode(Raven.MotorChannel.CH5, Raven.MotorMode.DIRECT)

# raven_board.set_motor_torque_factor(Raven.MotorChannel.CH5, 10)
# raven_board.set_motor_speed_factor(Raven.MotorChannel.CH5, 10)

# raven_board.set_motor_mode(Raven.MotorChannel.CH4, Raven.MotorMode.DIRECT)

# raven_board.set_motor_torque_factor(Raven.MotorChannel.CH4, 10)
# raven_board.set_motor_speed_factor(Raven.MotorChannel.CH4, 10, reverse=True)




while True:
   pass

print("made it here")
exit()

while True:
  accel, gyro = imu.get_data()
  # Returned linear acceleration is a tuple of (X, Y, Z) m/s^2
  # Returned gyroscope reading is a tuple of (X, Y, Z) radian/s

  # I coded PID in c before but I like this repo for PID better. I saw we just copy their code give them credit then cook my dudes
  # https://github.com/BurakDmb/DifferentialDrivePathTracking/blob/master/main.py
  # Clown Penis out
