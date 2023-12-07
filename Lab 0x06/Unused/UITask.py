from pyb import UART,repl_uart,USB_VCP
from task_share import Queue
import cotask
import pyb

class UITask:
    def __init__(self,
                 q_times, q_pos, q_vel, q_deltas,
                 controlMode, 
                 zeroEncoder_flag_A, zeroEncoder_flag_B,
                 getPosition_flag_A, getVelocity_flag_A,
                 getPosition_flag_B, getVelocity_flag_B,
                 getDelta_flag_A, updateDuty_flag_A,
                 getDelta_flag_B, updateDuty_flag_B,
                 updateGains_flag_A, updateVelocity_flag_A,
                 updateGains_flag_B, updateVelocity_flag_B,
                 collectTrial_flag_A, newRunStep_flag_A,
                 collectTrial_flag_B, newRunStep_flag_B,
                 encPosition_A, encDelta_A, mot_RPM_A, new_Kp_A,
                 encPosition_B, encDelta_B, mot_RPM_B, new_Kp_B,
                 motor_ref_A,motor_ref_B,maxTrialPoints, 
                 uart:UART, 
                 vcp: USB_VCP
                 ):

        # Constants
        self._state = 0
        self.buffer = ""
        self.doneEnteringNumber = False
        self.print_flag = True
        self.maxTrialPoints = maxTrialPoints

        # Write to object.
        self.motor_ref_A = motor_ref_A
        self.motor_ref_B = motor_ref_B
        self.q_times = q_times
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_deltas = q_deltas
        self.controlMode = controlMode

        self.zeroEncoder_flag_A = zeroEncoder_flag_A
        self.getPosition_flag_A = getPosition_flag_A
        self.getDelta_flag_A = getDelta_flag_A
        self.getVelocity_flag_A = getVelocity_flag_A
        self.updateDuty_flag_A = updateDuty_flag_A
        self.updateGains_flag_A = updateGains_flag_A
        self.updateVelocity_flag_A = updateVelocity_flag_A
        self.collectTrial_flag_A = collectTrial_flag_A
        self.newRunStep_flag_A = newRunStep_flag_A
        self.encPosition_A = encPosition_A
        self.encDelta_A = encDelta_A
        self.mot_RPM_A = mot_RPM_A
        self.new_Kp_A = new_Kp_A

        self.zeroEncoder_flag_B = zeroEncoder_flag_B
        self.getPosition_flag_B = getPosition_flag_B
        self.getDelta_flag_B = getDelta_flag_B
        self.getVelocity_flag_B = getVelocity_flag_B
        self.updateDuty_flag_B = updateDuty_flag_B
        self.updateGains_flag_B = updateGains_flag_B
        self.updateVelocity_flag_B = updateVelocity_flag_B
        self.collectTrial_flag_B = collectTrial_flag_B
        self.newRunStep_flag_B = newRunStep_flag_B
        self.encPosition_B = encPosition_B
        self.encDelta_B = encDelta_B
        self.mot_RPM_B = mot_RPM_B
        self.new_Kp_B = new_Kp_B

        #self.uart = uart
        self.vcp = vcp
        self.uart = uart # Debug
        # self.serial_stream = self.vcp
        # self.nb_in = NB_Input(self.serial_stream, echo=True)

        # Dictionaries
        self.strings = {'z': 'Zeroing Motor A Position\r',
                        'Z': 'Zeroing Motor B Position\r',
                        'p': 'Sending Motor A Position\r',
                        'P': 'Sending Motor B Position\r',
                        'd': 'Sending Motor A Delta\r',
                        'D': 'Sending Motor B Delta\r', 
                        'v': 'Print Out Velocity in Encoder A\r',
                        'V': 'Print Out Velocity in Encoder B\r' ,
                        'm': 'Enter a duty cycle for Motor A...\r',
                        'M': 'Enter a duty cycle for Motor B...\r',
                        'g': 'Collecting data for Motor A for the next 3 seconds...\r',
                        'G': 'Collecting data for Motor B for the next 3 seconds...\r',
                        'c': 'Switching to Closed Loop Control\r',
                        'C': 'Switching to Closed Loop Control\r',
                        'k': 'Enter new gains for Motor A...\r',
                        'K': 'Enter new gains for Motor B...\r',
                        's': 'Enter a new velocity for Motor A...\r',
                        'S': 'Enter a new velocity for Motor B...\r',
                        'r': 'Collecting data for a Motor A step response...\r',
                        'R': 'Collecting data for a Motor B step response...\r',
                        'o': 'Switching to Open Loop Control\r',
                        'O': 'Switching to Open Loop Control\r',                                           
                        }
        
    def run(self):
        while True:
            if self._state == 0:
                # Print the prompt once.
                if self.print_flag == True:
                    print("Please choose...\r")
                    print("   'O' for open-loop control.\r")
                    print("   'C' for closed-loop control.\r")
                    self.print_flag = False #change this JL since should be not == (set to false)
                
                # Char ready?
                if self.vcp.any():
                    # Read one character.
                    self.char = self.vcp.read(1).decode()

                    # State Transition
                    self.print_flag = True
                    
                    # Debug
                    #print("You pressed "+str(self.char)+".\r")
                    
                    # Set the next state accordingly and raise a flag.
                    if self.char in 'cC':
                        self.controlMode.put(1)
                        #print("Control Mode Variable is "+str(self.controlMode.get())+".\r")
                        self._state = 1

                    elif self.char in 'oO':
                        # Update controlMode share
                        self.controlMode.put(0)
                        #print("Control Mode Variable is "+str(self.controlMode.get())+".\r")
                        self._state = 2

            elif self._state == 1:
                '''@brief State 1: Closed Loop UI
                '''
                if self.print_flag == True:
                    print('Closed Loop Mode:\r')  
                    print('     Press (k/K) to update Kp.\r') 
                    print('     Press (s/S) to setpoint a velocity.\r')
                    print('     Press (r/R) to trigger set response and print.\r')
                    print('     Press (o/O) to switch to open loop.\r')
                    self.print_flag = False 
                    
                # Char ready?
                if self.vcp.any():

                    # Read one character
                    self.char = self.vcp.read(1).decode()

                    # Debug
                    #print("You pressed "+str(self.char)+".\r")
                                   
                    if self.char in 'kKsSrRoO':
                        # Print a message
                        print(self.strings[self.char])

                        # State Transition
                        self.print_flag = True

                        # Which character?
                        if self.char == 'k':
                            self.updateGains_flag_A.put(1)
                            
                            # Transition
                            self._nextState = self._state
                            self.doneEnteringNumber = False
                            self._state = 4

                        elif self.char == 'K':
                            self.updateGains_flag_B.put(1) 
                            
                            # Transition
                            self._nextState = self._state
                            self.doneEnteringNumber = False
                            self._state = 4

                        elif self.char == 's':
                            self.updateVelocity_flag_A.put(1)

                            # Transition
                            self._nextState = self._state
                            self.doneEnteringNumber = False
                            self._state = 4

                        elif self.char == 'S':
                            self.updateVelocity_flag_B.put(1)

                            # Transition
                            self._nextState = self._state
                            self.doneEnteringNumber = False
                            self._state = 4

                        elif self.char == 'r':
                            self.newRunStep_flag_A.put(5)
                            

                            # Transition
                            self._state = 3
    
                        elif self.char == 'R':
                            self.newRunStep_flag_B.put(5)   
                            self.uart.write('~~~\r\n')
                            
                            # Transition
                            self._state = 3

                        else: 
                            self.controlMode.put(0)
                            self.controlMode_Value = self.controlMode.get()
                            #print("Control Mode Variable is "+str(self.controlMode_Value)+".\r")
                            self._state = 2
                        
            elif self._state == 2:
                '''@brief State 2: Open Loop UI
                '''
                # Prompt
                if self.print_flag == True:
                    print('Open Loop Mode:\r')  
                    print('     Press (z/Z) to zero the encoder.\r') 
                    print('     Press (p/P) to print current encoder position.\r')
                    print('     Press (d/D) to print current delta.\r')
                    print('     Press (v/V) to velocity for encoder.\r')
                    print('     Press (m/M) to enter a new duty cycle.\r')
                    print('     Press (g/G) to collect data for 30 seconds and send to PC.\r')
                    print('     Press (c/C) to switch to closed loop mode.\r')
                    self.print_flag = False 

                # Char ready?
                if self.vcp.any():
                    # Read one character
                    self.char = self.vcp.read(1).decode()

                    # Debug
                    #print("You pressed "+str(self.char)+".\r")

                    if self.char in 'zZpPdDvVmMgGcC':
                        # Print a message
                        print(self.strings[self.char])

                        # Print flag
                        self.print_flag = True
                        
                        if self.char == 'z':
                            self.zeroEncoder_flag_A.put(1) 

                            # Transition
                            self._state = 3
                            
                        elif self.char =='Z':
                            self.zeroEncoder_flag_B.put(1) 
                                                        
                            # Transition
                            self._state = 3
                            
                        elif self.char =='p':
                            self.getPosition_flag_A.put(1) 
                            
                            # Transition
                            self._state = 3
                            
                        elif self.char =='P':
                            self.getPosition_flag_B.put(1) 
                            
                            # Transition
                            self._state = 3
                            
                        elif self.char =='d':
                            self.getDelta_flag_A.put(1) 
                            
                            # Transition
                            self._state = 3

                        elif self.char =='D':
                            self.getDelta_flag_B.put(1) 

                            # Transition
                            self._state = 3

                        elif self.char =='v':
                            self.getVelocity_flag_A.put(1) 
                               
                            # Transition    
                            self._state = 3

                        elif self.char =='V':
                            self.getVelocity_flag_B.put(1)

                            # Transition 
                            self._state = 3

                        elif self.char =='m':
                            self.updateDuty_flag_A.put(1)

                            # Transition
                            self._nextState = self._state
                            self.doneEnteringNumber = False
                            self._state = 4

                        elif self.char =='M':
                            self.updateDuty_flag_B.put(1)
                            
                            # Transition
                            self._nextState = self._state
                            self.doneEnteringNumber = False
                            self._state = 4
                            
                        elif self.char =='g':
                            self.collectTrial_flag_A.put(1)
                            print('You pressed g.\r')
                            # Transition
                            self._state = 3

                        elif self.char =='G':
                            self.collectTrial_flag_B.put(1)
                            print('You pressed G.\r')
                            # Transition
                            self._state = 3

                        else: 
                            self.controlMode.put(1)
                            #print("Control Mode Variable is "+str(self.controlMode.get())+".\r")
                            self._state = 1
                
            elif self._state == 3:
                '''@brief State 3: Printing, No Input Allowed
                '''
                if self.print_flag == True:
                    #print('UI is in State 3: Printing.\r')
                    self.print_flag = False

                # Oepn Loop printing
                if self.zeroEncoder_flag_A.get() == 2:
                    print("Encoder A was zeroed.\r")
                    self.zeroEncoder_flag_A.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 2

                if self.zeroEncoder_flag_B.get() == 2:
                    print("Encoder B was zeroed.\r")
                    self.zeroEncoder_flag_B.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 2

                if  self.getPosition_flag_A.get() == 2:
                    print("Encoder A's position is "+str(self.encPosition_A.get())+".\r")
                    self.getPosition_flag_A.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 2

                if  self.getPosition_flag_B.get() == 2:
                    print("Encoder B's position is "+str(self.encPosition_B.get())+".\r")
                    self.getPosition_flag_B.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 2   

                if  self.getDelta_flag_A.get() == 2:
                    print("Encoder A's delta is "+str(self.encDelta_A.get())+".\r")
                    self.getDelta_flag_A.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 2   

                if  self.getDelta_flag_B.get() == 2:
                    print("Encoder B's delta is "+str(self.encDelta_B.get())+".\r")
                    self.getDelta_flag_B.put(0) 

                    # Transition
                    self.print_flag = True
                    self._state = 2   
  
                if  self.getVelocity_flag_A.get() == 2:
                    print("The Encoder A's velocity is "+str(self.mot_RPM_A.get())+" RPM.\r")
                    self.getVelocity_flag_A.put(0) 

                    # Transition
                    self.print_flag = True
                    self._state = 2   

                if  self.getVelocity_flag_B.get() == 2:
                    print("The Encoder B's velocity is "+str(self.mot_RPM_B.get())+" RPM.\r")
                    self.getVelocity_flag_B.put(0) 

                    # Transition
                    self.print_flag = True
                    self._state = 2   
                    
                if self.updateDuty_flag_A.get() == 3:
                    print("The duty cycle in A has been updated to "+str(self.motor_ref_A.get())+"%.\r")
                    self.updateDuty_flag_A.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 2   
                        
                if self.updateDuty_flag_B.get() == 3:
                    print("The duty cycle in B has been updated to "+str(self.motor_ref_B.get())+"%.\r")
                    self.updateDuty_flag_B.put(0) 

                    # Transition
                    self.print_flag = True
                    self._state = 2  

                #print("Line 411 of UI TASK: collectTrial_flag_A is "+str(self.collectTrial_flag_A.get())+".\r")
                if self.collectTrial_flag_A.get() ==1:
                    print("UI Task Line 413\r")
                    try: 
                        #print('it is trying at least.\r')
                        self.uart.write(f"{self.q_times.get()},{self.q_deltas.get()},{self.q_pos.get()},{self.q_vel.get()}\r\n")
                    except:
                        print('A line of Collectrial was passed in line 415.\r')
                    
                if self.collectTrial_flag_B.get() ==1:
                    try: 
                        self.uart.write(f"{self.q_times.get()},{self.q_deltas.get()},{self.q_pos.get()},{self.q_vel.get()}\r\n")
                    except:
                        print('A line of Collectrial was passed in line 421.\r')

                if self.collectTrial_flag_A.get()==2:
                    self.collectTrial_flag_A.put(0)
                    self.uart.write('~~~\r\n')
                    print("collecTrialflag was reset.\r")

                    # Transition
                    self.print_flag = True
                    self._state = 2
                
                if self.collectTrial_flag_B.get()==2:
                    self.collectTrial_flag_B.put(0)
                    self.uart.write('~~~\r\n')
                    print("collecTrialflag was reset.\r")

                    # Transition
                    self.print_flag = True
                    self._state = 2

                # Closed Loop printing
                if self.updateGains_flag_A.get() == 3:
                    print("Motor A's Kp has been updated to "+str(self.new_Kp_A.get())+".\r")
                    self.updateGains_flag_A.put(0) 

                    # Transition
                    self.print_flag = True
                    self._state = 1

                if self.updateGains_flag_B.get() == 3:
                    print("Motor B's Kp has been updated to "+str(self.new_Kp_B.get())+".\r")
                    self.updateGains_flag_B.put(0)
                    
                    # Transition
                    self.print_flag = True
                    self._state = 1

                if self.updateVelocity_flag_A.get() == 3:
                    print("Motor A's velocity has been updated to "+str(self.motor_ref_A.get())+" RPM.\r")
                    self.updateVelocity_flag_A.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 1

                if self.updateVelocity_flag_B.get() == 3:
                    print("Motor B's velocity has been updated to "+str(self.motor_ref_B.get())+" RPM.\r")
                    self.updateVelocity_flag_B.put(0)

                    # Transition
                    self.print_flag = True
                    self._state = 1

                if  self.newRunStep_flag_A.get()==5:
                    #print("You're in line 451 of UI\r")
                    if self.q_times.any() == True:
                        self.uart.write(f"{self.q_times.get()},{self.q_deltas.get()},{self.q_pos.get()},{self.q_vel.get()}\r\n")
                        self.q_times.clear()
                        self.q_deltas.clear()
                        self.q_pos.clear()
                        self.q_vel.clear()
                        
                if  self.newRunStep_flag_B.get()==5:
                    #print("You're in line 456 of UI\r")
                    if self.q_times.any() == True:
                        self.uart.write(f"{self.q_times.get()},{self.q_deltas.get()},{self.q_pos.get()},{self.q_vel.get()}\r\n")
                        self.q_times.clear()
                        self.q_deltas.clear()
                        self.q_pos.clear()
                        self.q_vel.clear()

                if  self.newRunStep_flag_A.get()==2:
                    self.newRunStep_flag_A.put(0)
                    self.uart.write('~~~\r\n')
                    print("newRunStepflag was reset.\r")

                    # Transition
                    self.print_flag = True
                    self._state = 1
                
                if  self.newRunStep_flag_B.get()==2:
                    self.newRunStep_flag_B.put(0)
                    self.uart.write('~~~\r\n')
                    print("newRunStepflag was reset.\r")

                    # Transition
                    self.print_flag = True
                    self._state = 1
                
            elif self._state == 4:
                # State 4: Digit Entry Handler
                if self.print_flag == True:
                    print('UI is in State 4: Digit Entry.\r')
                    self.print_flag = False

                if self.vcp.any(): 
                    # Read one character
                    self.char = self.vcp.read(1).decode()

                    # If it's a digit, append it to the buffer
                    if self.char.isdigit():
                        self.buffer += self.char
                        print("Entry: "+str(self.buffer)+".\r")

                    # Negative sign
                    # If it's a negative sign and the buffer is empty, append it
                    elif self.char == "-" and len(self.buffer)==0: 
                        self.buffer += self.char
                        print("Entry: "+str(self.buffer)+".\r")

                    elif self.char == ".":
                        self.buffer += self.char
                        print("Entry: "+str(self.buffer)+".\r")
                        
                    # Handle backspace -- untested
                    elif self.char == "\x7f" and len(self.buffer)!=0: 
                        # Remove the last character
                        self.buffer = self.buffer[:-1]
                        print("Entry: "+str(self.buffer)+".\r") 

                    # Check for Enter key
                    elif self.char == "\r": #Debug
                        if len(self.buffer)==0:
                            print("No digits entered. Please enter a valid value...\r")
                        else:
                            try: 
                                self.floatOfUserEntry = float(self.buffer)

                                if  self.updateDuty_flag_A.get() == 1:
                                    self.motor_ref_A.put(self.floatOfUserEntry)
                                    self.updateDuty_flag_A.put(2)

                                    # Transition
                                    self.conversionFailed = False

                                elif self.updateDuty_flag_B.get() == 1:
                                    self.motor_ref_B.put(self.floatOfUserEntry)
                                    self.updateDuty_flag_B.put(2)

                                    # Transition
                                    self.conversionFailed = False
                                
                                elif self.updateVelocity_flag_A.get() == 1:
                                    self.motor_ref_A.put(self.floatOfUserEntry)
                                    self.updateVelocity_flag_A.put(2)

                                    # Transition
                                    self.conversionFailed = False
                                
                                elif self.updateVelocity_flag_B.get() == 1:
                                    self.motor_ref_B.put(self.floatOfUserEntry) 
                                    self.updateVelocity_flag_B.put(2) 
                                    
                                    # Transition
                                    self.conversionFailed = False

                                elif self.updateGains_flag_A.get() == 1:
                                    self.updateGains_flag_A.put(2) 
                                    self.new_Kp_A.put(self.floatOfUserEntry)
                                    
                                    # Transition
                                    self.conversionFailed = False

                                elif self.updateGains_flag_B.get() == 1:
                                    self.updateGains_flag_B.put(2)
                                    self.new_Kp_B.put(self.floatOfUserEntry)
                                    
                                    # Transition
                                    self.conversionFailed = False
                                else:
                                    print("Uh oh.\r")
                            except Exception as e:
                                print(f"An exception occured due to invalid entry. Exiting. {e}\r")
                                self.conversionFailed = True
                                self.updateGains_flag_B.put(0)
                                self.updateGains_flag_A.put(0) 
                                self.updateVelocity_flag_B.put(0) 
                                self.updateVelocity_flag_A.put(0)
                                self.updateDuty_flag_B.put(0)
                                self.updateDuty_flag_A.put(0)
                            finally: 
                                self.buffer = ""
                                self.print_flag = True
                                self._state = 3
     
            else:
                # Invalid state
                raise ValueError(f"Invalid State: {self._state}\r")
            yield self._state