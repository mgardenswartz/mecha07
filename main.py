# Imports
import pyb
from BNO055_Driver import *

myIMU = BNO055_Driver(bus=1,baudrate=400_000)
