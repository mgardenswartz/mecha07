# Imports
import pyb
from BNO055_Driver import *
from csvread import *
from time import sleep_ms

myIMU = BNO055_Driver(bus=1,baudrate=400_000)

# Grab the calibration coefficients.
filedata = csvread('IMU_cal_coeffs.txt') #Reads the CSV
first_row = filedata[0]
cal_coeffs = first_row
# This line would turn hex value into signed integers:
# cal_coeffs = [(int(hex_val,16) & 0x7F) - (int(hex_val,16) & 0x80) for hex_val in first_row]
myIMU.write_coefficients(cal_coeffs)

first_loop = True
while True:
   sleep_ms(30)
   Euler_angles = myIMU.read_Euler_angles()
   omegas = myIMU.read_angular_velocities()
   print(f"Heading: {Euler_angles[0]} deg, Roll: {Euler_angles[1]} deg, Yaw: {Euler_angles[2]} deg")
   print(f"W_x = {omegas[0]} dps, W_y = {omegas[1]} dps, W_z = {omegas[2]} dps")