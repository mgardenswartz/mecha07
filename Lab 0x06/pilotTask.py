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
from time import sleep, sleep_ms

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
                 firstLeftRow,
                 firstRightRow,
                 secondRow,
                 bumpers,
                 debug: bool,
                 max_spin: float):
        
        # Attributes
        self.cruiseSpeed = cruiseSpeed
        self.deltaSpeedforTurn = deltaSpeedforTurn
        self.encoder_LEFT = encoder_LEFT
        self.encoder_RIGHT = encoder_RIGHT
        self.encoderCPR = encoderCPR
        self.revolutionLimit = revolutionLimit
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
            
            # Read Sensors (Colors)
            self.firstLeftColors = self.firstLeftRow.read_color()
            self.firstRightColors = self.firstRightRow.read_color()
            self.firstColors = self.firstLeftColors + self.firstRightColors
            self.firstColors.reverse()
            self.secondColors = self.secondRow.read_color()
            self.secondColors.reverse()

            # Read Sensors (Brightness)
            self.firstLeftValues = self.firstLeftRow.read_brightness()
            self.firstRightValues = self.firstRightRow.read_brightness()
            self.firstValues = self.firstLeftValues + self.firstRightValues
            self.firstValues.reverse()
            self.secondValues = self.secondRow.read_brightness()
            self.secondValues.reverse()

            # Read Sensors (Raw)
            self.firstLeftValuesRaw = self.firstLeftRow.read_raw()
            self.firstRightValuesRaw = self.firstRightRow.read_raw()
            self.firstValuesRaw = self.firstLeftValuesRaw + self.firstRightValuesRaw
            self.firstValuesRaw.reverse()
            self.secondValuesRaw = self.secondRow.read_raw()
            self.secondValuesRaw.reverse()
            
            # Reverse brightness scale
            self.firstValues = [100-value for value in self.firstValues]
            self.secondValues = [100-value for value in self.secondValues]


            # Pick Sensor one column in from the sides.
            self.firstValues =[ self.firstValues[1], self.firstValues[4] ] 
            self.secondValues =[ self.secondValues[1], self.secondValues[4] ] 
            self.firstColors =[ self.firstColors[1], self.firstColors[4] ] 
            self.secondColors =[ self.secondColors[1], self.secondColors[4] ] 


            print("Bright Values 1 array Sensors:", self.firstValues)
            print("Bright Values 2 array Sensors:", self.secondValues) #now should be only 2 array

            # Calculate overall average
            # overall_average = (first_average + second_average) / 2
            # print("Overall Average Raw Value:", overall_average)

            #if overall_average > 1700:  Some values I found if both are white this around 1700 or more.
                #self.motor_RPM_wanted_LEFT.put(0)
               # self.motor_RPM_wanted_RIGHT.put(0)
            #elif overall_average <500 is going straight
                #self.motor_RPM_wanted_LEFT.put(60)
               # self.motor_RPM_wanted_RIGHT.put(60)
            #elif overall_average > 500 and overall_average < 1100 needs to turn right this is around 1086 the problem is that also applies for the left turn
               #self.motor_RPM_wanted_LEFT.put(0)
               # self.motor_RPM_wanted_RIGHT.put(60)
            #elif overall_average < 500 (for sharp turns
               #self.motor_RPM_wanted_LEFT.put(0)
               # self.motor_RPM_wanted_RIGHT.put(60)

            # Line Position
            self.firstTerms = [0]*len(self.firstColors)
            self.secondTerms = [0]*len(self.secondColors)
            for i in range(len(self.firstTerms)):
                self.firstTerms[i] = self.firstValues[i] * (i + 1)
            for i in range(len(self.secondTerms)):
                self.secondTerms[i] = self.secondValues[i] * (i + 1) 

            # Error for PI controller.
            self.firstAverage = sum(self.firstTerms)  / len(self.firstTerms)
            self.secondAverage = sum(self.secondTerms) / len(self.secondTerms)
            self.askewness = self.firstAverage - self.secondAverage

            # PI Controller
            #self.spin_effort = self.controller.get_effort_sat(ref = 0,
                                                         #meas = self.askewness,
                                                         #satLimit = self.max_spin)   

            # Debug Printing
            if self.debug:
                # print(f"Position of line under the first row is {self.firstAverage}.")
                # print(f"Position of line under the second row is {self.secondAverage}.")
                #print(f"Askewness if {self.askewness}.")
                #print(f"Spin effort is {self.spin_effort}.")

            # Controller


            if self.firstValues[1] > 50 and self.firstValues[4] < 50: #turn to left so higher in right wheel a right wheel and then left little
                 self.turn(turnSpeed=self.spin_effort, direction="left")
            elif self.secondValues[1] > 50 and self.secondValues[4] < 50: #second row left turn so higher speed in right wheel
                self.turn(turnSpeed=self.spin_effort, direction="left")
            elif self.firstValues[1] < 50 and self.firstValues[4] > 50: #  first row left speed but still we need a little right
                self.turn(turnSpeed=self.spin_effort, direction="right")
            elif self.secondValues[1] < 50 and self.secondValues[4] > 50:  #second row right turn so higher left speed
                self.turn(turnSpeed=self.spin_effort, direction="right") 
            elif self.firstValues[1] > 50 and self.firstValues[4] > 50: #first row all dark  go foward
                self.drive(speed=50, direction="forward") 
            elif self.secondValues[1] > 50 and self.secondValues[4] > 50: #also keep going foward if second row is dark
                self.drive(speed=50, direction="forward")   
            else:
                self.drive(speed=50, direction="forward") # only white and white and keep going foward.

            # Bumpers
            self.bumperStates = [not(bumper.value()) for bumper in self.bumpers]
            if any(self.bumperStates):
                print("A bumper was pressed!")

            yield

    def stop(self):
        self.motor_RPM_wanted_LEFT.put(0)
        self.motor_RPM_wanted_RIGHT.put(0)

    def turn(self,direction): 
        # Determine direction.
        if direction in ["cc","CC","Counterclockwise","Left","left","L"]:
            self.motor_RPM_wanted_RIGHT.put(  39 )
            self.motor_RPM_wanted_LEFT.put(  6.5 )


    def turn_in_place(self,turnSpeed,direction):
        # Convert Turn speed from dps to rad/s
        self.turnSpeed = abs(turnSpeed/180*3.14159) #rad/s 

        # Convert information to a maneuver of the wheels.
        self.wheelSpeed  = self.turnSpeed*145/70 # rad/s
        self.wheelSpeed *= 60/(2*3.14159)       # RPM

        # Determine direction.
        if direction in ["cc","CC","Counterclockwise","Left","left","L"]:
            self.motor_RPM_wanted_RIGHT.put(  self.wheelSpeed )
            self.motor_RPM_wanted_LEFT.put(  -self.wheelSpeed )
        else:
            self.motor_RPM_wanted_RIGHT.put(  -self.wheelSpeed )
            self.motor_RPM_wanted_LEFT.put(   self.wheelSpeed )

    def drive(self,speed,direction):
        # Convert speed from mm/s to RPM
        self.speed = abs(speed/35*60/(2*3.14159))  # RPM

        # Determine direction.
        if direction in ["Reverse","Backward","R","r","reverse","backward"]:
            self.motor_RPM_wanted_LEFT.put(  -self.speed)
            self.motor_RPM_wanted_RIGHT.put( -self.speed)
        else:
            self.motor_RPM_wanted_LEFT.put(  self.speed)
            self.motor_RPM_wanted_RIGHT.put( self.speed)
