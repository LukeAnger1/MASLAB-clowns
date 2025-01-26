from icm42688 import ICM42688
import board

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

while not spi.try_lock():
    pass

spi.configure(baudrate=5000000)

imu = ICM42688(spi)
imu.begin()

accel, gyro = imu.get_data()
# Returned linear acceleration is a tuple of (X, Y, Z) m/s^2
# Returned gyroscope reading is a tuple of (X, Y, Z) radian/s