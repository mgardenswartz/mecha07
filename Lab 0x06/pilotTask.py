from multiprocessing import Value


class pilotTask:
    def __init__(self,
                 cruiseSpeed: int,
                 deltaSpeedforTurn: int,
                 encoder_LEFT,
                 encoder_RIGHT,
                 motor_RPM_wanted_LEFT,
                 motor_RPM_wanted_RIGHT,
                 encoderCPR: int,
                 revolutionLimit: int,
                 IMU,
                 print_flag: bool):
        
        # Attributes
        self.cruiseSpeed = cruiseSpeed
        self.deltaSpeedforTurn = deltaSpeedforTurn
        self.encoder_LEFT = encoder_LEFT
        self.encoder_RIGHT = encoder_RIGHT
        self.encoderCPR = encoderCPR
        self.revolutionLimit = revolutionLimit
        self.IMU = IMU

        # Variables
        self.state = 0
        self.counter = 0
        self.print_flag = print_flag
        self.max_turn_speed = 60
        self.one_time = True

        # Shares
        self.motor_RPM_wanted_LEFT = motor_RPM_wanted_LEFT
        self.motor_RPM_wanted_RIGHT = motor_RPM_wanted_RIGHT
        self.full_stop()

    def run(self):    
        while True:
            if self.state == 0:
<<<<<<< HEAD
                self.
=======
                self.motor_RPM_wanted_LEFT.put(0)
                self.motor_RPM_wanted_RIGHT.put(0)
                
>>>>>>> 9b874c78bef02e96277eba8b07c4b7b665919089
            elif self.state == 1:
                pass
            else:
                raise ValueError(f"Invalid state: {self.state}.")
            
            yield self.state

    def full_stop(self):
        self.motor_RPM_wanted_LEFT.put(0)
        self.motor_RPM_wanted_RIGHT.put(0)

    def turn_speed(self,turnSpeed,direction):
        # Convert Turn speed from dps to rad/s
        self.turnSpeed = turnSpeed/180*3.14159 #rad/s 

        # Determine direction.
        if direction in ["cc","CC","Counterclockwise","Left","left","L"]:
            self.directionSign = -1
        else:
            self.directionSign = 1
        
        # Convert information to a maneuver of the wheels.
        self.omega_left = self.turnSpeed*145/70

        # This allow driving forward motion to turn while 
        self.motor_RPM_wanted_LEFT.put(   self.motor_RPM_wanted_LEFT.get()+self.directionSign*self.omega_left)
        self.motor_RPM_wanted_RIGHT.put( self.motor_RPM_wanted_RIGHT.get()-self.directionSign*self.omega_left)

    def drive_forward(self,speed,direction):
        # Convert speed from mm/s to RPM
        self.speed = speed/35/6.282*60 # RPM

        # Determine direction.
        if direction in ["Reverse","Backward","R","r","reverse","backward"]:
            self.directionSign = -1
        else:
            self.directionSign = 1

        # Set speed
        self.motor_RPM_wanted_LEFT.put(   self.motor_RPM_wanted_LEFT.get()+self.directionSign*self.speed)
        self.motor_RPM_wanted_RIGHT.put( self.motor_RPM_wanted_RIGHT.get()+self.directionSign*self.speed)
        
    # def turn_heading(self,
    #                  heading_wanted: float,
    #                  turn_speed: float):
    #     while True:
    #         if self.state == 0: # Initialization
    #             # Turn off motors
    #             self.motor_RPM_wanted_LEFT.put(0)
    #             self.motor_RPM_wanted_RIGHT.put(0)

    #             # Zero encoders
    #             self.encoder_LEFT.zero()
    #             self.encoder_RIGHT.zero()

    #             # Get current heading
    #             self.Heading,self.Roll,self.Pitch = self.IMU.read_Euler_angles()

    #             # Constants
    #             self.turn_speed = turn_speed #RPM
    #             self.heading_wanted = heading_wanted

    #             # Raise an error if the entry was out of range.
    #             if self.turn_speed >= self.max_turn_speed:
    #                 self.turn_speed = self.max_turn_speed
    #                 print(f"Turn speed of {self.turn_speed} was too big, so it was reduced to {self.max_turn_speed}.")

    #             if self.heading_wanted >= 360 or self.heading_wanted < 0:
    #                 raise ValueError(f"Invalid Heading: {self.heading_wanted}.")

    #             # Decide which way to turn.
    #             self.half_way_heading = self.heading_wanted + 180
    #             if self.half_way_heading >= 360:
    #                 self.half_way_heading -= 360

    #             if self.Heading < self.half_way_heading:
    #                 self.sign = -1
    #                 if self.print_flag == True:
    #                     print("Romi will turn LEFT.")
    #             else:
    #                 self.sign = 1
    #                 if self.print_flag == True:
    #                     print("Romi will turn RIGHT.")

    #             # State Transition
    #             self.state = 1

    #         elif self.state == 1: # Turn!

    #             # Get current heading
    #             self.Heading,self.Roll,self.Pitch = self.IMU.read_Euler_angles()

    #             if  270 <= self.Heading <= 360 and self.sign == -1:
    #                 if self.print_flag == True:
    #                     print("We were turning LEFT and have reached north!")
    #                 self.motor_RPM_wanted_LEFT.put( 0 )
    #                 self.motor_RPM_wanted_RIGHT.put(0 ) 

    #                 # State Transition
    #                 self.state = 2

    #             elif 0 <= self.Heading <= 90 and self.sign == 1:
    #                 if self.print_flag == True:
    #                     print("We were turning RIGHT and have reached north!")
    #                 self.motor_RPM_wanted_LEFT.put( 0 )
    #                 self.motor_RPM_wanted_RIGHT.put(0 ) 

    #                 # State Transition
    #                 self.state = 2

    #             else:
    #                 # Keep Turning!
    #                 if self.print_flag == True:
    #                     print(f"Current heading: {self.Heading} deg. Romi will keep turning.")
    #                 self.motor_RPM_wanted_LEFT.put(  self.sign*self.turn_speed)
    #                 self.motor_RPM_wanted_RIGHT.put(-self.sign*self.turn_speed)

    #         elif self.state == 2:    
    #             # Do nothing.
    #             if self.one_time == True:
    #                 if self.print_flag == True:
    #                     print("Finished turn. Reboot to rerun face_north program.")
    #                 self.one_time = False
    #             self.motor_RPM_wanted_LEFT.put( 0 )
    #             self.motor_RPM_wanted_RIGHT.put(0 )

    #         else:
    #             # Invalid state
    #             raise ValueError(f"Invalid State: {self.state}\r")    
            
    #         yield self.state 
