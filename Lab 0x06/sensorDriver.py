"""!
@file motorControlTask.py
This file creates a driver for Pololu QTR Analog Reflectance Sensor
of any pitch or number of channels.

@author Max Gardenswartz
@date   2023-Nov-17 MLG Approximate date of creation of file
@date   2023-Dec-15 MLG Latest itteration. 
            Note that recalibration does not persist between reboot.

It is intended for educational use only, but its use is not limited thereto.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from pyb import Pin, ADC
class sensorDriver:
   def __init__(self,
      Pins: list(Pin), # type: ignore
      whiteCalibration: list(int), # type: ignore
      blackCalibration: list(int)): # type: ignore

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
         self.readings[index] = self.adc[index].read() # type: ignore

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
            self.colors[index] = "White" # type: ignore
         else:
            self.colors[index] = "Black" # type: ignore

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