import pyb

class motorDriver:
   '''!@brief A driver class for one channel of the L6206.
   @details Objects of this class can be used to apply PWM to a given
   DC motor on one channel of the L6206 from ST Microelectronics.
   '''

   def __init__(self, PWM_timer, Dir_pin, EFF_pin, EN_pin, PWM_channel: int):
      '''!@brief Initializes and returns an object associated with a DC motor.
      '''
      # Creates a pin object from the enable pin specified
      # as an input parameter, and then stores the pin 
      # object in a class attrbiute
      self.Dir_pin = pyb.Pin(Dir_pin,mode=pyb.Pin.OUT_PP)
      self.EFF_pin = pyb.Pin(EFF_pin,mode=pyb.Pin.OUT_PP)
      self.EN_pin = pyb.Pin(EN_pin,mode=pyb.Pin.OUT_PP)
      self.PWM_channel = int(PWM_channel)

      # Create a timer channel
      self.pwm_ch_1 = PWM_timer.channel(self.PWM_channel, 
                                        pin=pyb.Pin(EFF_pin,mode=pyb.Pin.OUT_PP), 
                                        mode=pyb.Timer.PWM)

   def set_duty (self, duty):
      '''!@brief Set the PWM duty cycle for the DC motor.
      @details This method sets the duty cycle to be sent
      to the L6206 to a given level. Positive values
      cause effort in one direction, negative values
      in the opposite direction.
      @param duty A signed number holding the duty
      cycle of the PWM signal sent to the L6206'''
      
      if duty >= 0: # Toggling IN1
         # Make the motor go forward
         self.pwm_ch_1.pulse_width_percent(duty)
         self.Dir_pin.low()
      elif duty < 0: # Toggling IN2
         # Make the motor go backward
         self.pwm_ch_1.pulse_width_percent(-duty)
         self.Dir_pin.high()

   def enable (self):
      '''!@brief Enable one channel of the L6206.
      @details This method sets the enable pin associated with one
      channel of the L6206 high in order to enable that
      channel of the motor driver.
      '''
      #self.enable.high() doesn't work
      self.EN_pin.high()
   
   def disable(self):
      self.EN_pin.low()