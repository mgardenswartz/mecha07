# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import gc
import time
gc.collect()
import machine
gc.collect()
import micropython
gc.collect()
import os
gc.collect()
from BNO055 import BNO055
gc.collect()

micropython.alloc_emergency_exception_buf(300)

#i2c = machine.I2C(1, freq=400_000)
# Read device ID which should be 0xA0
#print(i2c.readfrom(0x28, 1))

#i2c = machine.I2C(1)  # uses board.SCL and board.SDA
sensor = BNO055()

# If you are going to use UART uncomment these lines
# uart = board.UART()
# sensor = adafruit_bno055.BNO055_UART(uart)

# last_val = 0xFFFF


# def temperature():
#     global last_val  # pylint: disable=global-statement
#     result = sensor.temperature
#     if abs(result - last_val) == 128:
#         result = sensor.temperature
#         if abs(result - last_val) == 128:
#             return 0b00111111 & result
#     last_val = result
#     return result


sensor.begin()

# print(sensor.getRevInfo())

sensor.set_axis_remap(x=BNO055.AXIS_REMAP_Y, y=BNO055.AXIS_REMAP_X, z=BNO055.AXIS_REMAP_Z, x_sign=BNO055.AXIS_REMAP_NEGATIVE, y_sign=BNO055.AXIS_REMAP_POSITIVE, z_sign=BNO055.AXIS_REMAP_POSITIVE)

#sensor.setCalibrationData([0, 0, 0, 0, 0, 0, 97, 9, 176, 2, 122, 6, 2, 0, 255, 255, 0, 0, 232, 3, 21, 2])

while True:
    try:

        print("Calibration Status: {}".format(sensor.getCalibrationStatus()))
        # print("Temperature: {} degrees C".format(sensor.getTemp()))
        # print("Accelerometer (m/s^2): {}".format(sensor.getVector(BNO055.VECTOR_ACCELEROMETER)))
        # print("Magnetometer (microteslas): {}".format(sensor.getVector(BNO055.VECTOR_MAGNETOMETER)))
        print("Angular Velocity (rad/sec): {}".format(sensor.getVector(BNO055.VECTOR_GYROSCOPE)))
        print("Euler angle: {}".format(sensor.getVector(BNO055.VECTOR_EULER)))
        # print("Quaternion: {}".format(sensor.getQuat()))
        # print("Linear acceleration (m/s^2): {}".format(sensor.getVector(BNO055.VECTOR_LINEARACCEL)))
        # print("Gravity (m/s^2): {}".format(sensor.getVector(BNO055.VECTOR_GRAVITY)))
        print()
    
    except OSError:
        continue

    time.sleep(1)