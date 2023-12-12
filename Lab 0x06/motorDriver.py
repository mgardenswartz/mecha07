"""!
@file motorControlTask.py
This file creates a driver for a Romi motor.

@author Max Gardenswartz and Jonathan Lam
@date   2023-Nov-17 MLG Approximate date of creation of file
@date   2023-Dec-15 MLG Latest itteration.

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

import pyb

class motorDriver:
   '''!@brief A driver class for the Romi motor.
   @details Objects of this class can be used to apply PWM to a given
   DC motor on one channel of the Romi motor from ST Microelectronics.
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

      # Create a timer channel.
      # Do not use INVERTED mode!
      self.pwm_ch = PWM_timer.channel(self.PWM_channel, 
                                        pin=pyb.Pin(EFF_pin,mode=pyb.Pin.OUT_PP), 
                                        mode=pyb.Timer.PWM)

   def set_duty (self, duty):
        '''!@brief Set the PWM duty cycle for the DC motor.
        @details This method sets the duty cycle to be sent
        to the Romi motor to a given level. Positive values
        cause effort in one direction, negative values
        in the opposite direction.
        @param duty A signed number holding the duty
        cycle of the PWM signal sent to the Romi motor'''
        
        self.duty = duty
        if self.duty >= 0: # Toggling IN1
            # Make the motor go forward
            self.pwm_ch.pulse_width_percent(self.duty)
            self.Dir_pin.low()
        elif self.duty < 0: # Toggling IN2
            # Make the motor go backward
            self.pwm_ch.pulse_width_percent(-self.duty)
            self.Dir_pin.high()
        else:
            raise ValueError(f"Invalid duty cycle provided: {self.duty}.")

   def enable (self):
        '''!@brief Enable one channel of the Romi motor.
        @details This method sets the enable pin associated with one
        channel of the Romi motor to make it run.
        '''

        self.EN_pin.high()
   
   def disable(self):
        '''!@brief Disable one channel of the Romi motor.
        @details This method lowers the enable pin associated with one
        channel of the Romi motor to make the motor free spin.
        '''
        self.EN_pin.low()