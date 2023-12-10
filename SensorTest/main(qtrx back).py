# Micropython Imports
import pyb
import micropython
import gc 

# Ridgely Libaries
# from task_share import *
# from cotask import *

# Our Drivers
# from closedLoopControl import *
# from encoderDriver import *
# from motorDriver import *
from sensorDriver import *

# Our Tasks
# from motorControlTask import *
# from garbageCollectTask import *

# Troubleshoots PuTTy! -- Thank you, Jack Miller.
micropython.alloc_emergency_exception_buf(100)

from time import sleep_ms

if __name__ == "__main__":
   secondSensorArray = sensorDriver(Pins=[ pyb.Pin.cpu.C4, pyb.Pin.cpu.C3, pyb.Pin.cpu.C2, pyb.Pin.cpu.B1, pyb.Pin.cpu.C5, pyb.Pin.cpu.C0 ],
                                     whiteCalibration = [2604, 2436, 1145, 1935, 2009, 2447],
                                     blackCalibration = [3564, 3389, 2815, 3031, 3394, 3544]) 

   while True:
      readings = secondSensorArray.read_raw()
      readings = readings[::-1]
      colors = secondSensorArray.read_color()
      colors = colors[::-1]
      percentBrightness = secondSensorArray.read_brightness()
      percentBrightness = percentBrightness[::-1]
      percentBrightness = ', '.join(map(str,percentBrightness))
      #print(f"{percentBrightness},{colors}")
      #print(readings)
      print(colors)
      sleep_ms(250)