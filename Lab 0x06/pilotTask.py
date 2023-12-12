"""!
@file pilotTask.py
This file interacts with sensors, bumpers, encoders, and the IMU to 
intelligently make decisions to pilot the Romi through the course.
A new PI controller attempts to keep the Romi on the line. 

@author Max Gardenswartz and Jonathan Lam
@date   2023-Nov-17 MLG Approximate date of creation of file
@date   2023-Dec-15 MLG Latest itteration for Term Project.

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

class pilotTask:
    def __init__(self,
                 cruiseSpeed: int,
                 encoder_LEFT,
                 encoder_RIGHT,
                 motor_RPM_wanted_LEFT,
                 motor_RPM_wanted_RIGHT,
                 encoderCPR: int,
                 IMU,
                 print_flag: bool,
                 firstLeftRow,
                 firstRightRow,
                 secondRow,
                 bumpers,
                 debug: bool,
                 controller,
                 max_spin: float):
        
        # Attributes
        self.cruiseSpeed = cruiseSpeed
        self.encoder_LEFT = encoder_LEFT
        self.encoder_RIGHT = encoder_RIGHT
        self.encoderCPR = encoderCPR
        self.IMU = IMU
        self.firstLeftRow = firstLeftRow
        self.firstRightRow = firstRightRow
        self.secondRow = secondRow
        self.bumpers = bumpers
        self.debug = debug
        self.controller = controller
        self.max_spin = max_spin

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
            # if self.state == 0:
            #     # Cruise
            #     self.drive(speed = 10, # mm/s
            #                direction = "forward")
            # elif self.state == 1:
            #     # Turn left in place
            #     self.turn(turnSpeed = 5, #deg/s
            #               direction = "left")
            # elif self.state == 2:
            #     # Turn right in place
            #     self.turn(turnSpeed = 10, #deg/s
            #               direction = "right")
            # elif self.state == 3:
            #     self.stop()
            # else:
            #     raise ValueError(f"Invalid state: {self.state}.")
            
            # Read Sensors
            self.firstLeftColors = self.firstLeftRow.read_color()[::1]
            self.firstRightColors = self.firstRightRow.read_color()[::1]
            self.firstColors = self.firstLeftColors.extend(self.firstRightColors)
            self.secondColors = self.secondRow.read_color()[::1]
            self.firstLeftValues = self.firstLeftRow.read_raw()[::1]
            self.firstRightValues = self.firstRightRow.read_raw()[::1]
            self.firstValues = self.firstLeftValues.extend(self.firstRightValues)
            self.secondValues = self.secondRow.read_raw()[::1]

            # Remove Sensor 0 
            self.firstColors = self.firstColors[1:]

            # Line Position
            self.firstTerms = [0]*len(self.firstColors)
            self.secondTerms = [0]*len(self.secondColors)
            for i in range(len(self.firstTerms)):
                self.firstTerms[i] = self.firstValues*(i+1+1)/len(self.firstValues) 
            for i in range(len(self.secondTerms)):
                self.secondTerms[i] = self.secondValues*(i+1)/len(self.secondValues)

            # Error for PI controller.
            self.firstAverage = sum(self.firstTerms)
            self.secondAverage = sum(self.secondTerms)
            self.askewness = self.firstAverage - self.secondAverage
            self.spin_effort = self.controller.get_effort_sat(ref = 0,
                                                         meas = self.askewness,
                                                         satLimit = self.max_spin)   

            # Controller
            if abs(self.spin_effort) < 10:
                self.drive(speed = 10, # mm/s
                           direction = "forward")
            else: 
                self.turn( turnSpeed = self.spin_effort, # Dps
                          direction = "right")
            
            # Bumpers
            self.bumperStates = [not(bumper.value()) for bumper in self.bumpers]
            if any(self.bumperStates):
                print("A bumper was pressed!")

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