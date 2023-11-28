from pyb import Pin, ADC
class sensorDriver:
   def __init__(self,
      Pins: list(Pin),
      whiteCalibration: list(int),
      blackCalibration: list(int)):

      # Constants
      self.numberOfPins = len(Pins)
      self.blackCalibration = [0]*self.numberOfPins
      self.whiteCalibration = [0]*self.numberOfPins
       # Error Checking
      if len(whiteCalibration) != self.numberOfPins or len(blackCalibration) != self.numberOfPins:
         raise ValueError("Initialization Failed. Number of pins must match calibration")
      for index in range(self.numberOfPins):
         self.blackCalibration[index] = 4095 - blackCalibration[index]
         self.whiteCalibration[index] = 4095 - whiteCalibration[index]

      # Pin Attributes
      self.Pins = [None]*self.numberOfPins
      for index in range(self.numberOfPins):
         self.Pins[index] = Pin(Pins[index], mode=Pin.ANALOG)

      # Timer
      self.adc = [None]*self.numberOfPins
      for index in range(self.numberOfPins):
         self.adc[index] = ADC(self.Pins[index])
      
   def read_raw(self):
      self.readings = [None]*self.numberOfPins
      for index in range(self.numberOfPins):
         self.readings[index] = self.adc[index].read()

      return self.readings
   
   def read_brightness(self):
      self.readings = self.read_raw()

      # Rescale
      self.analogRange = [0]*self.numberOfPins
      self.percentBrightness = [0]*self.numberOfPins
      for index in range(self.numberOfPins):
         self.readings[index] = 4095 - self.readings[index]
         self.analogRange[index] = self.whiteCalibration[index] - self.blackCalibration[index]
         self.percentBrightness[index] = (self.readings[index] - self.blackCalibration[index])/self.analogRange[index]*100
         self.percentBrightness[index] = round(self.percentBrightness[index])
      return self.percentBrightness

   def read_color(self):
      self.percentBrightness = self.read_brightness()

      # Convert brightness to color.
      self.colors = [None]*self.numberOfPins
      for index in range(self.numberOfPins):
         if self.percentBrightness[index] >= 50:
            self.colors[index] = "White"
         else:
            self.colors[index] = "Black"

      return self.colors
   
   def recalibrate_white(self):
      self.readings = self.read_raw()
      for index in range(self.numberOfPins):
         self.whiteCalibration[index] = 4095 - self.whiteCalibration[index]

      return self.readings

   def recalibrate_black(self):
      self.readings = self.read_raw()
      for index in range(self.numberOfPins):
         self.blackCalibration[index] = 4095 - self.blackCalibration[index]

      return self.readings