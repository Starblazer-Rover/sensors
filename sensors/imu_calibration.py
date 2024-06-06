# SPDX-FileCopyrightText: 2023 JG for Cedar Grove Maker Studios
# SPDX-License-Identifier: MIT

"""
`bno055_calibrator.py`
===============================================================================
A CircuitPython module for calibrating the BNo055 9-DoF sensor. After manually
calibrating the sensor, the module produces calibration offset tuples for use
in project code.

* Author(s): JG for Cedar Grove Maker Studios

Implementation Notes
--------------------
**Hardware:**
* Adafruit BNo055 9-DoF sensor
**Software and Dependencies:**
* Driver library for the sensor in the Adafruit CircuitPython Library Bundle
* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
"""

import time
import board
import adafruit_bno055


# pylint: disable=too-few-public-methods
class Mode:
    CONFIG_MODE = 0x00
    ACCONLY_MODE = 0x01
    MAGONLY_MODE = 0x02
    GYRONLY_MODE = 0x03
    ACCMAG_MODE = 0x04
    ACCGYRO_MODE = 0x05
    MAGGYRO_MODE = 0x06
    AMG_MODE = 0x07
    IMUPLUS_MODE = 0x08
    COMPASS_MODE = 0x09
    M4G_MODE = 0x0A
    NDOF_FMC_OFF_MODE = 0x0B
    NDOF_MODE = 0x0C


def _write_register_data(register, data):
    write_buffer = bytearray(1)
    write_buffer[0] = register & 0xFF
    write_buffer[1:len(data)+1]=data
    with sensor.i2c_device as i2c:
        i2c.write(write_buffer, start=0, end=len(write_buffer))

def _read_registers(register, length):
    read_buffer = bytearray(23)
    read_buffer[0] = register & 0xFF
    with sensor.i2c_device as i2c:
        i2c.write(read_buffer, start=0, end=1)
        i2c.readinto(read_buffer, start=0, end=length)
        return read_buffer[0:length]
    
def get_calibration():
    """Return the sensor's calibration data and return it as an array of
    22 bytes. Can be saved and then reloaded with the set_calibration function
    to quickly calibrate from a previously calculated set of calibration data.
    """
    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    sensor.mode = adafruit_bno055.CONFIG_MODE
    # Read the 22 bytes of calibration data and convert it to a list (from
    # a bytearray) so it's more easily serialized should the caller want to
    # store it.
    cal_data = list(_read_registers(0X55, 22))
    # Go back to normal operation mode.
    mode = adafruit_bno055.NDOF_MODE
    return cal_data


# Uncomment these lines for UART interface connection
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

# Instantiate I2C interface connection
i2c = board.I2C()  # For board.SCL and board.SDA
#i2c = board.STEMMA_I2C()  # For the built-in STEMMA QT connection
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = Mode.NDOF_MODE

print("Magnetometer: Perform the figure-eight calibration dance.")
while sensor.calibration_status[3] != 3 or sensor.calibration_status[2] != 3 or sensor.calibration_status[1] != 3:
    
    # Calibration Dance Step One: Magnetometer
    #   Move sensor away from magnetic interference or shields
    #   Perform the figure-eight until calibrated
    print(f"Mag Calib Status: {100 / 3 * sensor.calibration_status[3]:3.0f}%")

    # Calibration Dance Step Two: Accelerometer
    #   Place sensor board into six stable positions for a few seconds each:
    #    1) x-axis right, y-axis up,    z-axis away
    #    2) x-axis up,    y-axis left,  z-axis away
    #    3) x-axis left,  y-axis down,  z-axis away
    #    4) x-axis down,  y-axis right, z-axis away
    #    5) x-axis left,  y-axis right, z-axis up
    #    6) x-axis right, y-axis left,  z-axis down
    #   Repeat the steps until calibrated
    print(f"Accel Calib Status: {100 / 3 * sensor.calibration_status[2]:3.0f}%")

    # Calibration Dance Step Three: Gyroscope
    #  Place sensor in any stable position for a few seconds
    #  (Accelerometer calibration may also calibrate the gyro)
    print(f"Gyro Calib Status: {100 / 3 * sensor.calibration_status[1]:3.0f}%")
    print('------------------------------')


    time.sleep(1)
print("... CALIBRATED")
time.sleep(1)

print(get_calibration())

print("\nCALIBRATION COMPLETED")
print("Insert these preset offset values into project code:")
print(f"  Offsets_Magnetometer:  {sensor.offsets_magnetometer}")
print(f"  Offsets_Gyroscope:     {sensor.offsets_gyroscope}")
print(f"  Offsets_Accelerometer: {sensor.offsets_accelerometer}")

path = '/home/billee/billee_ws/src/sensors/resource/imu_calibration.dat'
with open(path, 'w') as file:
    file.write(f'{sensor.offsets_magnetometer[0]}, {sensor.offsets_magnetometer[1]}, {sensor.offsets_magnetometer[2]}\n')
    file.write(f'{sensor.offsets_gyroscope[0]}, {sensor.offsets_gyroscope[1]}, {sensor.offsets_gyroscope[2]}\n')
    file.write(f'{sensor.offsets_accelerometer[0]}, {sensor.offsets_accelerometer[1]}, {sensor.offsets_accelerometer[2]}\n')

