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
                 revolutionLimit: int):
        
        # Attributes
        self.cruiseSpeed = cruiseSpeed
        self.deltaSpeedforTurn = deltaSpeedforTurn
        self.sensor = sensor
        self.encoder_LEFT = encoder_LEFT
        self.encoder_RIGHT = encoder_RIGHT
        self.encoderCPR = encoderCPR
        self.debug = debug
        self.revolutionLimit = revolutionLimit
        
        # Variables
        self.state = 0

        # Shares
        self.motor_RPM_wanted_LEFT = motor_RPM_wanted_LEFT
        self.motor_RPM_wanted_RIGHT = motor_RPM_wanted_RIGHT

    def turn_left_heading(self,
                          turn_speed: int,
                          headingWanted: float):
        self.turn_speed = turn_speed
        self.headingWanted = headingWanted

        while heading <= headingWanted:
            self.motor_RPM_wanted_LEFT.put(-self.turn_speed)
            self.motor_RPM_wanted_RIGHT.put(self.turn_speed)
        
    def run(self):
        while True:
            if self.state == 0:
               # Turn off motors
               self.motor_RPM_wanted_LEFT.put(0)
               self.motor_RPM_wanted_RIGHT.put(0)

               # Zero encoders
               self.encoder_LEFT.zero()
               self.encoder_RIGHT.zero()

               # State Transition 
               self.state = 1

            elif self.state == 1:
               # Search for line by cruising
               self.motor_RPM_wanted_LEFT.put(self.cruiseSpeed)
               self.motor_RPM_wanted_RIGHT.put(self.cruiseSpeed)
               
            elif self.state == 2:
               # Turn left
               self.motor_RPM_wanted_LEFT.put(self.cruiseSpeed+self.deltaSpeedforTurn)
               self.motor_RPM_wanted_RIGHT.put(self.cruiseSpeed)

            elif self.state == 3:
               # Turn left
               self.motor_RPM_wanted_LEFT.put(self.cruiseSpeed)
               self.motor_RPM_wanted_RIGHT.put(self.cruiseSpeed+self.deltaSpeedforTurn)

            elif self.state == 4:
               # Turn off motors
               self.motor_RPM_wanted_LEFT.put(0)
               self.motor_RPM_wanted_RIGHT.put(0)

               # State Transition
               self.state = 4
            else:
                # Invalid state
                raise ValueError(f"Invalid State: {self.state}\r")
            
            # Poll sensors
            self.colors = self.sensor.read_color()
            self.colors = self.colors[::-1]
            if self.debug == True:
               print(f"State {self.state}...... {self.colors}")

            # Full Circle?
            self.encoderTheta_LEFT = self.encoder_LEFT.get_position()/self.encoderCPR # rev
            self.encoderTheta_RIGHT = self.encoder_RIGHT.get_position()/self.encoderCPR # rev
            self.distanceTurned = 70/(2*145)*(self.encoderTheta_LEFT - self.encoderTheta_RIGHT)
            if self.debug == True:
               print(f"You've turned {round(self.distanceTurned,2)} rev.")
            if self.revolutionLimit >0:
               if abs(float(self.distanceTurned)) >= float(self.revolutionLimit): 
                  self.state = 4
                  yield

            # State Transition
            if self.state != 4:   
               if self.colors[0] == "Black" and self.colors[2] == "White":
                  self.state = 2
               elif self.colors[0] == "White" and self.colors[2] == "Black":
                  self.state = 3
               else:
                  self.state = 1
            
            yield self.state 