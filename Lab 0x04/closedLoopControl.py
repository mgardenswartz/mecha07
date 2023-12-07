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
          self.Kp = new_Kp
