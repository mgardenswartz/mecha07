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
   frontSensorArray = sensorDriver(Pins=[ pyb.Pin.cpu.C4, pyb.Pin.cpu.C3, pyb.Pin.cpu.C2, pyb.Pin.cpu.B1, pyb.Pin.cpu.C5, pyb.Pin.cpu.C0 ],
                                     whiteCalibration = [3055, 3037, 2173, 2692, 2681, 3145],
                                     blackCalibration = [3956, 3976, 3409, 3636, 3619, 3801]) 

   while True:
      readings = frontSensorArray.read_raw()
      readings = readings[::-1]
      colors = frontSensorArray.read_color()
      colors = colors[::-1]
      percentBrightness = frontSensorArray.read_brightness()
      percentBrightness = percentBrightness[::-1]
      percentBrightness = ', '.join(map(str,percentBrightness))
      print(f"{percentBrightness},{colors}")
      print(readings)
      print(colors)
      sleep_ms(30)