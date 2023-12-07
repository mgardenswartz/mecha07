class motorControlTask:
    """
    @brief Class for controlling a motor with feedback from an encoder.
    @details This class enables open-loop and closed-loop control of a motor using a specified motor, motor control, and encoder object. It supports both open-loop control (using duty cycle) and closed-loop control (maintaining a desired RPM).

    @param motor: Motor object for controlling the physical motor.
    @param motorControl: Motor control object enabling closed-loop control.
    @param encoder: Encoder object for reading motor speed feedback.
    @param controlMode: Control mode (0 for open-loop, 1 for closed-loop control).
    @param encoderCPR: Counts per revolution for the encoder.
    @param max_duty: Maximum allowable duty cycle percentage for the motor.
    @param motor_RPM_wanted: Desired motor speed in RPM for closed-loop control.
    @param motor_RPM: Current motor speed in RPM (updated during operation).
    @param motor_duty_wanted: Desired motor duty cycle for open-loop control.
    @param flip_Speed: Boolean flag to ensure correct sign for closed-loop control.
    @param debug: Boolean flag for printing debug information over UART.

    @note The class includes internal variables, attributes, and shared variables for managing the motor control task.
    """
    def __init__(self,
                 motor,
                 motorControl,
                 encoder: Encoder,
                 controlMode: Share,
                 encoderCPR: int,
                 max_duty: int,
                 motor_RPM_wanted: Share,
                 motor_RPM: Share,
                 motor_duty_wanted: Share,
                 flip_Speed: bool,
                 debug: bool):

        # Internal Variables
        self.counter = 0
        self.state = 0
        self.print_flag = True

        # Attributes
        self.motor = motor
        self.motorControl = motorControl
        self.encoder = encoder
        self.encoderCPR = encoderCPR
        self.max_duty = max_duty
        self.flip_Speed = flip_Speed
        self.debug = debug

        # Shares
        self.controlMode = controlMode
        self.motor_RPM_wanted = motor_RPM_wanted
        self.motor_RPM = motor_RPM
        self.motor_duty_wanted = motor_duty_wanted

    def run(self):
        while True:
            if self.state == 0:
                # State 0 
                if self.print_flag == True:
                    print(f"Motor Control is in State {self.state}.\r")
                    self.print_flag = False

                # Motor
                self.motor.enable()
                self.encoder.zero()

                # State Transition
                self.print_flag = True
                self.state = 1

            elif self.state == 1:
                # State 1
                if self.print_flag == True:
                    print(f"Motor Control is in State {self.state}.\r")
                    self.motorControl.error=0
                    self.motorControl.effort_sat=0
                    self.motorControl.running_error=0
                    self.print_flag = False
                
                # Turns off motor
                self.motor.set_duty(0)
                self.encoder.zero()

                if self.controlMode.get() == 0: 
                    # Transition
                    self.print_flag = True    
                    self.state = 3
                
                elif self.controlMode.get() == 1: 
                    #Transition
                    self.print_flag = True
                    self.state = 2
                
            elif self.state == 2:
                # State 2: Closed Loop
                if self.print_flag == True:
                    print(f"Motor Control is in State {self.state}.\r")
                    self.print_flag = False

                # Grab latest encoder data.
                self.encoder.update()

                # Measure motor speed
                self.encoderDelta = self.encoder.get_delta()
                self.motor_RPM.put( self.encoderDelta*(self.motorControl.controlFrequency*60)/self.encoderCPR) #RPM
                if self.flip_Speed == True:
                    self.motor_RPM.put( -self.motor_RPM.get() )

                # Grab the effort.
                self.motor_effort = self.motorControl.get_effort_sat(ref = self.motor_RPM_wanted.get(),
                                                                     meas = self.motor_RPM.get(),
                                                                     satLimit = self.max_duty)
                
                #print(f"line 97 the encoder delta is: {self.encoderDelta}")
                #print(f"Line 98 of the Motor Control Task says RPM: {self.motor_RPM.get()}, Want: {self.motor_RPM_wanted.get()}, Error: {self.motorControl.error}, Effort: {self.motor_effort}") 
                if self.debug == True:
                    print(f"{self.motor_RPM.get()}")

                # Update the duty cycle
                self.motor.set_duty(self.motor_effort)

                # State Transition Conditionss
                if self.controlMode.get() == 0:
                    self.print_flag = True
                    self.state = 1

            elif self.state == 3:
                # State 3: Open Loop
                if self.print_flag == True:
                    print(f"Motor Control is in State {self.state}, Open Loop.\r")
                    self.print_flag = False
                    
                self.motor.set_duty(self.motor_duty_wanted.get())

                # Measure motor speed
                self.encoderDelta = self.encoder.get_delta()
                self.motor_RPM.put( self.encoderDelta*(self.motorControl.controlFrequency*60)/self.encoderCPR) #RPM
                if self.flip_Speed == True:
                    self.motor_RPM.put( -self.motor_RPM.get() )
                print(f"Line 110 of the Motor Control Task states we're going {self.motor_RPM.get()} RPM.")

                # State Transition Conditions
                if self.controlMode.get() == 1:
                    self.print_flag = True
                    self.state = 1            
            
            else:
                # Invalid state
                raise ValueError(f"Invalid State: {self.state}\r")
            
            yield self.state

                