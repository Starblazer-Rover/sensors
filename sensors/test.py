import time
import board
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX

i2c = board.I2C()
ism = ISM330DHCX(i2c)

acceleration = ism.acceleration
gyro = ism.gyro

while True:
    print(f"Accel_X: {acceleration[0]}, Accel_Y: {acceleration[1]}, Accel_Z: {acceleration[2]}")
    print(f"Gyro_X: {gyro[0]}, Gyro_Y: {gyro[1]}, Gyro_Z: {gyro[2]}")