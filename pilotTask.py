class pilotTask:
    def __init__(self,
                 cruiseSpeed: int,
                 deltaSpeedforTurn: int,
                 sensor,
                 encoder_LEFT,
                 encoder_RIGHT,
                 motor_RPM_wanted_LEFT,
                 motor_RPM_wanted_RIGHT,
                 encoderCPR: int,
                 debug: bool,
                 revolutionLimit: int,
                 IMU):
        
        # Attributes
        self.cruiseSpeed = cruiseSpeed
        self.deltaSpeedforTurn = deltaSpeedforTurn
        self.sensor = sensor
        self.encoder_LEFT = encoder_LEFT
        self.encoder_RIGHT = encoder_RIGHT
        self.encoderCPR = encoderCPR
        self.debug = debug
        self.revolutionLimit = revolutionLimit
        self.IMU = IMU

        # Variables
        self.state = 0

        # Shares
        self.motor_RPM_wanted_LEFT = motor_RPM_wanted_LEFT
        self.motor_RPM_wanted_RIGHT = motor_RPM_wanted_RIGHT

    def face_north(self):
        while True:
            if self.state == 0: # Initialization
                # Turn off motors
                self.motor_RPM_wanted_LEFT.put(0)
                self.motor_RPM_wanted_RIGHT.put(0)

                # Zero encoders
                self.encoder_LEFT.zero()
                self.encoder_RIGHT.zero()

                # Get current heading
                self.Heading,self.Roll,self.Pitch = self.IMU.read_Euler_angles()

                # Set turn speeds
                self.turn_speed = 30 #RPM

                # Decide which way to turn.
                if self.Heading < 180:
                    self.sign = -1
                    print("Romi will turn LEFT.")
                else:
                    self.sign = 1
                    print("Romi will turn RIGHT.")

                # State Transition
                self.state = 1

            elif self.state == 1: # Turn!

                # Get current heading
                self.Heading,self.Roll,self.Pitch = self.IMU.read_Euler_angles()
   
                if  270 <= self.Heading <= 360 and self.sign == -1:
                    print("We were turning LEFT and have reached north!")
                    self.motor_RPM_wanted_LEFT.put( 0 )
                    self.motor_RPM_wanted_RIGHT.put(0 ) 

                    # State Transition
                    self.state = 2

                elif 0 <= self.Heading <= 90 and self.sign == 1:
                    print("We were turning RIGHT and have reached north!")
                    self.motor_RPM_wanted_LEFT.put( 0 )
                    self.motor_RPM_wanted_RIGHT.put(0 ) 

                    # State Transition
                    self.state = 2

                else:
                    # Keep Turning!
                    print(f"Current heading: {self.Heading} deg. Romi will keep turning.")
                    self.motor_RPM_wanted_LEFT.put(  self.sign*self.turn_speed)
                    self.motor_RPM_wanted_RIGHT.put(-self.sign*self.turn_speed)

            elif self.state == 2:    
                # Do nothing.
                print("Finished turn. Reboot to rerun face_north program.")
                self.motor_RPM_wanted_LEFT.put( 0 )
                self.motor_RPM_wanted_RIGHT.put(0 )

            else:
                # Invalid state
                raise ValueError(f"Invalid State: {self.state}\r")    
            
            yield self.state 