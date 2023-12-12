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
                 print_flag: bool,
                 firstRow,
                 secondRow,
                 bumpers):
        
        # Attributes
        self.cruiseSpeed = cruiseSpeed
        self.deltaSpeedforTurn = deltaSpeedforTurn
        self.encoder_LEFT = encoder_LEFT
        self.encoder_RIGHT = encoder_RIGHT
        self.encoderCPR = encoderCPR
        self.revolutionLimit = revolutionLimit
        self.IMU = IMU
        self.firstRow = firstRow
        self.secondRow = secondRow
        self.bumpers = bumpers

        # Variables
        self.state = 3
        self.counter = 0
        self.print_flag = print_flag
        self.max_turn_speed = 60
        self.one_time = True

        # Share
        self.motor_RPM_wanted_LEFT = motor_RPM_wanted_LEFT
        self.motor_RPM_wanted_RIGHT = motor_RPM_wanted_RIGHT
        self.stop()

    def run(self):    
        while True:
            if self.state == 0:
                # Cruise
                self.drive(speed = 10, # mm/s
                           direction = "forward")
                
            elif self.state == 1:
                # Turn left in place
                self.turn(turnSpeed = 5, #deg/s
                          direction = "left")
            elif self.state == 2:
                # Turn right in place
                self.turn(turnSpeed = 10, #deg/s
                          direction = "right")
            elif self.state == 3:
                self.stop()
            else:
                raise ValueError(f"Invalid state: {self.state}.")
            
            # Read sensors
            # self.colorsFirstRaw = self.firstRow.read_line_color()
            # self.colorsSecondRaw = self.secondRow.read_color()[::1]
            # self.colorsFirst = [1 if color == "Black" else 0 for color in self.colorsFirstRaw]
            # self.colorsSecond = [1 if color == "Black" else 0 for color in self.colorsSecondRaw]
            
            print("-"*50)
            print(self.firstRow.read_line_color())
            print(self.secondRow.read_raw()[::1])

            # One of the sensors is all blank.
            # if self.colorsFirst == [0]*6 or self.colorsSecond == [0,0,1,0,0,0]:
            #     # 

            # if self.colorsFirst[1] == 1 and self.colorsFirst[4] == 0: 
            #     self.state = 1
            # if self.colorsFirst[1] == 0 and self.colorsFirst[4] == 1:
            #     self.state = 2
            # else:
            #     self.state = 0
            
            self.bumperStates = [not(bumper.value()) for bumper in self.bumpers]
            if any(self.bumperStates):
                print("A bumper was pressed!")

            yield self.state

        

    def stop(self):
        self.motor_RPM_wanted_LEFT.put(0)
        self.motor_RPM_wanted_RIGHT.put(0)

    def turn(self,turnSpeed,direction):
        # Convert Turn speed from dps to rad/s
        self.turnSpeed = turnSpeed/180*3.14159 #rad/s 

        # Determine direction.
        if direction in ["cc","CC","Counterclockwise","Left","left","L"]:
            self.directionSign = -1
        else:
            self.directionSign = 1
        
        # Convert information to a maneuver of the wheels.
        self.omega_left = self.turnSpeed*145/70 # rad/s
        self.omega_left *= 60/(2*3.14159)       # RPM

        # This allows driving forward motion to turn while 
        # self.motor_RPM_wanted_LEFT.put(   self.motor_RPM_wanted_LEFT.get()+self.directionSign*self.omega_left)
        # self.motor_RPM_wanted_RIGHT.put( self.motor_RPM_wanted_RIGHT.get()-self.directionSign*self.omega_left)
        self.motor_RPM_wanted_LEFT.put(  self.directionSign*self.omega_left)
        self.motor_RPM_wanted_RIGHT.put( -self.directionSign*self.omega_left)

    def drive(self,speed,direction):
        # Convert speed from mm/s to RPM
        self.speed = speed/35*60/(2*3.14159)  # RPM

        # Determine direction.
        if direction in ["Reverse","Backward","R","r","reverse","backward"]:
            self.directionSign = -1
        else:
            self.directionSign = 1

        # Set speed
        # self.motor_RPM_wanted_LEFT.put(   self.motor_RPM_wanted_LEFT.get()+self.directionSign*self.speed)
        # self.motor_RPM_wanted_RIGHT.put( self.motor_RPM_wanted_RIGHT.get()+self.directionSign*self.speed)
        self.motor_RPM_wanted_LEFT.put(   self.directionSign*self.speed)
        self.motor_RPM_wanted_RIGHT.put( self.directionSign*self.speed)