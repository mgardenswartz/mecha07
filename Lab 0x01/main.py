# Created on Tue Oct  3 15:18:06 2023
# @author: maxga

# Imports
from pyb import Pin, Timer
#from time import sleep_ms

class L6206:
   '''!@brief A driver class for one channel of the L6206.
   @details Objects of this class can be used to apply PWM to a given
   DC motor on one channel of the L6206 from ST Microelectronics.
   '''

   def __init__(self, _PWM_tim, _IN1_pin, _IN2_pin, _EN_pin):
      '''!@brief Initializes and returns an object associated with a DC motor.
      '''
      # Creates a pin object from the enable pin specified
      # as an input parameter, and then stores the pin 
      # object in a class attrbiute
      
      #self.enable = Pin(_EN_pin,mode=Pin.OUT_PP) doesn't work if place this.
      self._IN1_pin = Pin(_IN1_pin,mode=Pin.OUT_PP)
      self._IN2_pin = Pin(_IN2_pin,mode=Pin.OUT_PP)

      self._EN_pin = Pin(_EN_pin,mode=Pin.OUT_PP)
      # Creates timer channels
      self.pwm_ch_1 = _PWM_tim.channel(1, pin=Pin(_IN1_pin,mode=Pin.OUT_PP), mode=Timer.PWM_INVERTED)
      self.pwm_ch_2 = _PWM_tim.channel(2, pin=Pin(_IN2_pin,mode=Pin.OUT_PP), mode=Timer.PWM_INVERTED)
      

   def set_duty (self, duty):
      '''!@brief Set the PWM duty cycle for the DC motor.
      @details This method sets the duty cycle to be sent
      to the L6206 to a given level. Positive values
      cause effort in one direction, negative values
      in the opposite direction.
      @param duty A signed number holding the duty
      cycle of the PWM signal sent to the L6206'''
      
      if duty >= 0: # Toggling IN1
         # Set IN2 pin low.
         # self._IN2_pin.low()

         # Make the motor go forward
         self.pwm_ch_1.pulse_width_percent(duty)
         self.pwm_ch_2.pulse_width_percent(0)
      elif duty < 0: # Toggling IN2
         # Set IN1 pin low.
         # self._IN1_pin.low()

         # Make the motor go backward
         self.pwm_ch_1.pulse_width_percent(0)
         self.pwm_ch_2.pulse_width_percent(-duty)

   def enable (self):
      '''!@brief Enable one channel of the L6206.
      @details This method sets the enable pin associated with one
      channel of the L6206 high in order to enable that
      channel of the motor driver.
      '''
      #self.enable.high() doesn't work
      self._EN_pin.high()

if __name__ == '__main__':
   # Adjust the following code to write a test program for your L6206 class. Any
   # code within the if __name__ == '__main__' block will only run when the
   # script is executed as a standalone program. If the script is imported as
   # a module the code block will not run.

   # Create a timer object to use for motor control
   
   tim_A = Timer(3, freq = 20000)

   # Create an L6206 driver object. You will need to modify the code to facilitate
   # passing in the pins and timer objects needed to run the motors.
   mot_A = L6206(tim_A, Pin.cpu.B4, Pin.cpu.B5, Pin.cpu.A10 )

   # Enable the L6206 driver
   mot_A.enable()

   # Set the duty cycle of the first L6206 channel to 100 percent
   mot_A.set_duty(50)
  

   tim_B = Timer(5, freq = 20000)

   # Create an L6206 driver object. You will need to modify the code to facilitate
   # passing in the pins and timer objects needed to run the motors.
   mot_B = L6206(tim_B, Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.C1)

   # Enable the L6206 driver
   mot_B.enable()

   # Set the duty cycle of the first L6206 channel to 30 percent
   mot_B.set_duty(0)
  
