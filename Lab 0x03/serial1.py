import serial
import time

with serial.serial('COM9',115200) as ser:
   while True:
      ser.float()
      if ser.in_waiting:
         time.sleep(0.005)
         print(ser.serial(ser.in_waiting))
