"""!
@file closedLoopControl.py
This file implements a PI controller for closed-loop control of a given system,
like a motor.
Not for use with state space.
No derivative control available.

@author Max Gardenswartz
@date   2023-Sep-17 MLG Approximate date of creation of file
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

class closedLoopControl:
    """
    @brief Class for reading encoder values using a timer.
    @details This class takes in an input speed and outputs a PWM.

    @param target_RPM: Target speed, in RPM.
    @
    """

    def __init__(self,controlFrequency: float,Kp: float,Ki: float,toggle: bool):
        '''!@brief Initializes and returns an object associated with a PI controller.
        @param Kp Proportional Control Constant
        @param Ki 1000 times Integral Control Constant
        @param satMode True/False for saturation on get_effort function
        '''
        self.controlFrequency = controlFrequency
        self.error=0
        self.effort_sat=0
        self.running_error=0
        self.Kp=Kp
        self.Ki=Ki
        self.controlPeriod = 1/self.controlFrequency
        self.toggle = toggle

    def get_effort_sat(self,ref: float,meas: float,satLimit: float):
        '''!@brief Returns a control output.
        @details Returns an saturated control output
        based on the controller constants and error.
        @param ref The desired performance value for system's output
        @param meas The system's current output
        '''

        # Store control constants for later use.
        self.ref = ref
        self.meas = meas 
        self.satLimit = abs(satLimit)   

        if self.toggle == False:
            self.meas = self.meas
        else: 
            self.meas = -self.meas

        # Error
        self.error = self.ref - self.meas# Debug

        # Running Effort (for Integral Control)
        self.running_error += self.error*self.controlPeriod 

        # Controller output
        self.P_effort = self.error*self.Kp
        self.I_effort = self.running_error*self.Ki
        self.effort_unsat = self.P_effort + self.I_effort

        # Saturation
        if self.effort_unsat > self.satLimit:
            self.effort_sat = self.satLimit
        elif self.effort_unsat < -self.satLimit:
            self.effort_sat = -self.satLimit
        else:
            self.effort_sat = self.effort_unsat
    
        return self.effort_sat
    
    def update_Kp(self,new_Kp):
        '''!@brief Sets a new value of Kp.
        @param new_Kp New value of Kp.'''
        
        self.Kp = new_Kp

    def update_Ki(self,new_Ki):
        '''!@brief Sets a new value of Ki.
        @param new_Ki New value of Ki.'''
        
        self.Ki = new_Ki
