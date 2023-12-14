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
                 encoder_LEFT,
                 encoder_RIGHT,
                 motor_RPM_wanted_LEFT,
                 motor_RPM_wanted_RIGHT,
                 encoderCPR: int,
                 revolutionLimit: int,
                 IMU,
                 firstLeftRow,
                 firstRightRow,
                 secondRow,
                 bumpers,
                 debug: bool):
        
        # Attributes
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

        # Variables
        self.state = 0

        # Share
        self.motor_RPM_wanted_LEFT = motor_RPM_wanted_LEFT
        self.motor_RPM_wanted_RIGHT = motor_RPM_wanted_RIGHT
        self.stop()

        # Create a dictionary to map colors to values
        self.color_mapping = {"White": 0, "Black": 1}

        # Dictionary for possible manuevers.
        self.manuevers = {
                    ((0, 0), 
                     (0, 0)): 'straight',

                    ((1, 1), 
                     (1, 1)): 'straight',

                    ((0, 0), 
                     (1, 1)): 'straight',

                    ((1, 1), 
                     (0, 0)): 'straight',

                    ((1, 1), 
                     (0, 1)): 'slight_right',

                    ((0, 1), 
                     (0, 0)): 'slight_right',

                    ((0, 1), 
                     (0, 1)): 'slight_right',

                    ((0, 1), 
                     (1, 1)): 'slight_right',

                    ((0, 1), 
                     (1, 0)): 'sharp_right',

                    ((0, 0), 
                     (0, 1)): 'sharp_right',

                    ((1, 0), 
                     (1, 0)): 'slight_left',

                    ((1, 0), 
                     (0, 0)): 'slight_left',

                    ((1, 1), 
                     (1, 0)): 'slight_left',

                    ((1, 0), 
                     (1, 1)): 'slight_left',

                    ((1, 0), 
                     (0, 1)): 'sharp_left',

                    ((0, 0), 
                     (1, 0)): 'sharp_left'
                    }

    def run(self):    
        while True:
            if self.state == 0:
                # Read sensors
                self.read_sensors()
                
                # Reverse brightness scale so that 100 is black and 0 is white. 
                self.firstValues = [100-value for value in self.firstValues]
                self.secondValues = [100-value for value in self.secondValues]

                # Pick Sensors.
                self.firstValues =[ self.firstValues[1], self.firstValues[4] ] 
                self.secondValues =[ self.secondValues[1], self.secondValues[4] ] 
                self.firstColors =[ self.firstColors[1], self.firstColors[4] ] 
                self.secondColors =[ self.secondColors[1], self.secondColors[4] ] 

                # Control Logic

                # Replace colors with values and convert to tuple
                self.firstColorsTuple = tuple(self.color_mapping[color] for color in self.firstColors)
                self.secondColorsTuple= tuple(self.color_mapping[color] for color in self.secondColors)
                self.colorsTuple = (self.firstColorsTuple, self.secondColorsTuple)
                
                # Printing for debugging.
                if self.debug:
                    print(f"Bright Values 1 array Sensors {self.firstValues}.")
                    print(f"Bright Values 2 array Sensors: {self.secondValues}.")
                    #print(f"Tuple reads: {self.colorsTuple}")
                
                # What should the robot do?
                self.whatToDo = self.manuevers[self.colorsTuple]

                if self.whatToDo == "straight":
                    if self.debug:
                        print("Performing straight maneuver.")
                    self.drive(speed=80, direction="Forward")

                elif self.whatToDo == "slight_right":
                    if self.debug:
                        print("Performing slight right turn.")
                    self.slight_turn(direction="right")

                elif self.whatToDo == "sharp_right":
                    if self.debug:
                        print("Performing sharp right turn.")
                    self.sharp_turn(direction="right")

                elif self.whatToDo == "slight_left":
                    if self.debug:
                        print("Performing slight left turn.")
                    self.slight_turn(direction="left")

                elif self.whatToDo == "sharp_left":
                    if self.debug:
                        print("Performing sharp left turn.")
                    self.sharp_turn(direction="left")

                else:
                    raise Exception("Something went wrong.")

                # if self.firstValues[1] > 50 and self.firstValues[4] < 50: #turn to left so higher in right wheel a right wheel and then left little
                #      self.turn(turnSpeed=self.spinSpeed, direction="left")
                # elif self.secondValues[1] > 50 and self.secondValues[4] < 50: #second row left turn so higher speed in right wheel
                #     self.turn(turnSpeed=self.spinSpeed, direction="left")
                # elif self.firstValues[1] < 50 and self.firstValues[4] > 50: #  first row left speed but still we need a little right
                #     self.turn(turnSpeed=self.spinSpeed, direction="right")
                # elif self.secondValues[1] < 50 and self.secondValues[4] > 50:  #second row right turn so higher left speed
                #     self.turn(turnSpeed=self.spinSpeed, direction="right") 
                # elif self.firstValues[1] > 50 and self.firstValues[4] > 50: #first row all dark  go foward
                #     self.drive(speed=50, direction="forward") 
                # elif self.secondValues[1] > 50 and self.secondValues[4] > 50: #also keep going foward if second row is dark
                #     self.drive(speed=50, direction="forward")   
                # else:
                #     self.drive(speed=50, direction="forward") # only white and white and keep going foward.

                # Bumpers
                self.bumperStates = [not(bumper.value()) for bumper in self.bumpers]
                if any(self.bumperStates):
                    print("A bumper was pressed!")
                    self.state = 1

            elif self.state == 1:
                self.stop()
            else:
                raise ValueError(f"Invalid state: {self.state}.")
            
            yield self.state

    def stop(self):
        self.motor_RPM_wanted_LEFT.put(0)
        self.motor_RPM_wanted_RIGHT.put(0)

    def slight_turn(self,direction): 
        # Determine direction.
        if direction in ["cc","CC","Counterclockwise","Left","left","L"]:
            self.motor_RPM_wanted_RIGHT.put(  45 )
            self.motor_RPM_wanted_LEFT.put(  6.5 )
        else: 
            self.motor_RPM_wanted_RIGHT.put(  6.5 )
            self.motor_RPM_wanted_LEFT.put(  45 )

    def sharp_turn(self,direction):
        # Determine direction.
        if direction in ["cc","CC","Counterclockwise","Left","left","L"]:
            self.motor_RPM_wanted_RIGHT.put(  60 )
            self.motor_RPM_wanted_LEFT.put(  6 )
        else:
            self.motor_RPM_wanted_RIGHT.put(  6 )
            self.motor_RPM_wanted_LEFT.put(  60 )

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

    def read_sensors(self):
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
