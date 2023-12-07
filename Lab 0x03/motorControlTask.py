from pyb import UART,repl_uart,USB_VCP
from task_share import Queue
import cotask
import pyb

class motorControlTask:
    '''!@brief A class implementing motor contro
    '''
    
    def __init__(self, ref, max_duty,
                 q_times, q_pos, q_vel, q_deltas,
                 mot_object, encoder_object, mot_ctrl_object,
                 controlMode, zeroEncoder_flag,
                 getPosition_flag, getVelocity_flag,
                 getDelta_flag, updateDuty_flag,
                 updateGains_flag, updateVelocity_flag,
                 collectTrial_flag, newRunStep_flag,
                 encPosition,encDelta,mot_RPM,new_Kp,
                 maxTrialPoints
                 ):
        '''!@brief          Initializes a data transfer task object
            @param ref      Speed (RPM) or duty cycle wanted.
            @param max_duty 
            @param q_times   A queue from which to get time values
            @param q_pos    A queue from which to get position values
            @param q_deltas  A queue from which to get delta values
            @param mot_object  Motor object
            @param encoder_object Encoder object
            @param mot_ctrl_object  Closed loop motor control object
            @param controlMode  A boolean for closed loop mode
            @param zeroEncoder_flag     
            @param getPosition_flag
            @param getDelta_flag
            @param getVelocity_flag
            @param updateDuty_flag
            @param updateGains_flag
            @param updateVelocity_flag
            @param collectTrial_flag
            @param runStep_flag
            @param encPosition 
            @param encDelta
            @param mot_RPM
            @param new_Kp
        '''
        
        ## The present state of the task
        self._state = 0
        self.conversionFailed = False
        self.want = 0

        # Shares
        self.ref = ref
        self.max_duty = max_duty
        self.q_times = q_times # Keep a reference to the queue holding time data
        self.q_pos = q_pos # Keep a reference to the queue holding pos. data
        self.q_vel = q_vel
        self.q_deltas = q_deltas
        self.mot = mot_object
        self.enc = encoder_object
        self.mot_ctrl = mot_ctrl_object
        self.controlMode = controlMode
        self.zeroEncoder_flag = zeroEncoder_flag
        self.getPosition_flag = getPosition_flag
        self.getDelta_flag = getDelta_flag
        self.getVelocity_flag = getVelocity_flag 
        self.updateDuty_flag = updateDuty_flag
        self.updateGains_flag = updateGains_flag
        self.updateVelocity_flag = updateVelocity_flag
        self.collectTrial_flag = collectTrial_flag
        self.newRunStep_flag = newRunStep_flag
        self.encPosition = encPosition
        self.encDelta = encDelta
        self.mot_RPM = mot_RPM
        self.new_Kp = new_Kp
        self.maxTrialPoints = maxTrialPoints

        #Debug 
        self.print_flag = True
        self.counter = 0
        
    def run(self):
        '''!@brief          The task implementation as a generator function
        '''
        while True:
            if self._state == 0:
                # Run state 0 code:
                if self.print_flag == True:
                    print("Motor Control is in State "+str(self._state)+".\r")
                    self.print_flag = False
                # Motor
                self.mot.enable()
                self.enc.zero()
                # State Transition Conditions
                self.print_flag = True
                self._state = 1

            elif self._state == 1:
                
                if self.print_flag == True:
                    print(f"Motor Control is in State {str(self._state)}.\r")
                    self.mot_ctrl.error=0
                    self.mot_ctrl.effort_sat=0
                    self.mot_ctrl.running_error=0
                    self.print_flag = False
                
                # Turns off motor
                self.mot.set_duty(0)
                self.want = 0 

                if self.controlMode.get() == 0: # duty cycle goes into self.want

                    # Transition
                    self.print_flag = True    
                    self._state = 3
                
                elif self.controlMode.get() == 1: # Convert self.ref from RPM to ticks*controlHz.
                    #self.want = self.ref.get()*16384/(60*self.mot_ctrl.controlFrequency)
                    #self.want = 0 

                    #Transition
                    self.print_flag = True
                    self._state = 2
                
            elif self._state == 2:
                '''@brief Closed Loop state
                '''
                if self.print_flag == True:
                    print("Motor Control is in State "+str(self._state)+", Closed Loop.\r")
                    self.mot_ctrl.running_error = 0
                    self.print_flag = False

                # UI Handler
                if self.updateGains_flag.get() == 2:
                    self.mot_ctrl.update_Kp(self.new_Kp.get())
                    self.updateGains_flag.put(3)

                if self.updateVelocity_flag.get() == 2:
                    self.want = self.ref.get() # .get()*16384/(60*self.mot_ctrl.controlFrequency) #RPM
                    self.updateVelocity_flag.put(3)

                self.newRunStep_value = self.newRunStep_flag.get()

                # Grab latest encoder data.
                self.enc.update()

                # Measure motor speed
                self.encDelta.put( self.enc.get_delta() )
                self.mot_RPM.put( self.encDelta.get()*(self.mot_ctrl.controlFrequency*60)/16384 )#RPM

                # Grab the effort.
                self.mot_eff = self.mot_ctrl.get_effort_sat(ref=self.want,meas=self.mot_RPM.get(),satLimit=self.max_duty)
                # Update the duty cycle
                self.mot.set_duty(self.mot_eff)
                #print("The current motor effort is "+str(self.mot_eff)+" and the error is"+str(self.mot_RPM.get())+".\r")

                 # print("self.newRunStep_value is "+str(self.newRunStep_value))
                if self.newRunStep_value == 5:
                    if self.counter == 50:
                        self.want = 120 #RPM

                    if self.counter == 0:
                        #self.mot.set_duty(0) 
                        self.want = 0
                        self.mot_ctrl.Kp = 0.4

                    elif (self.counter >= 50) and (self.counter<=250): 
                        self.encPosition.put( self.enc.get_position() )
                        self.q_times.put( (self.counter)/10 )
                        self.q_pos.put(self.encPosition.get())
                        self.q_deltas.put(self.encDelta.get())
                        self.q_vel.put(self.mot_RPM.get())
                        #print("Running motor.\r")

                    elif (self.counter > 250) and (self.counter<300):
                        #self.mot.set_duty(0)
                        
                        self.want = 0

                    elif self.counter >= 300:
                        self.newRunStep_flag.put(2)
                        self.counter = -1
                        print("Flag was reset.\r")
                    self.counter += 1







                # State Transition Conditions
                if self.controlMode.get() == 0:
                    self.print_flag = True
                    self.ref.put(0) #Debug
                    self._state = 1

            elif self._state == 3:
                '''@brief Open Loop State'''
                if self.print_flag == True:
                    print(f"Motor Control is in State {self._state}, Open Loop.\r")

                    self.print_flag = False
                    
                #print(f"Motor Control is in State {self._state}, Open Loop.\r")

                # UI handler
                if self.zeroEncoder_flag.get() == 1:
                    self.enc.zero()
                    self.zeroEncoder_flag.put(2)
                
                self.enc.update()    
                if self.getPosition_flag.get() == 1:
                    self.encPosition.put(self.enc.get_position())
                    self.getPosition_flag.put(2)

                if self.getDelta_flag.get() == 1:
                    self.encDelta.put(self.enc.get_delta())
                    self.getDelta_flag.put(2)

                if self.getVelocity_flag.get() == 1:
                    self.encDelta.put(self.enc.get_delta())
                    self.mot_RPM.put(self.encDelta.get()*(1000*60)/16384) #RPM
                    self.getVelocity_flag.put(2)

                if self.updateDuty_flag.get() == 2: 
                    if self.ref.get() > self.max_duty:
                        self.want = self.max_duty
                    elif self.ref.get() < -self.max_duty:
                        self.want = -self.max_duty
                    else:
                        self.want = self.ref.get()

                    # Finally    
                    self.mot.set_duty(self.want)
                    self.updateDuty_flag.put(3)
                
                if self.collectTrial_flag.get() == 1:
                    print("Counter is at "+str(self.counter)+".\r")
                    if (self.counter >= 0) and (self.counter<300):
                        self.encPosition.put( self.enc.get_position() )
                        self.encDelta.put( self.enc.get_delta() )
                
                        self.q_times.put( self.counter)
                        self.q_pos.put(self.encPosition.get())
                        self.q_deltas.put(self.encDelta.get())
                        self.q_vel.put(self.mot_RPM.get())
                    else:
                        self.collectTrial_flag.put(2)
                        print("Motor Control Line 235: Collect Trial flag was successfully reset.\r")
                        self.counter = -1
                    self.counter += 1
                    
                # State Transition Conditions
                if self.controlMode.get() == 1:
                    self.print_flag = True
                    self._state = 1            
            
            else:
                # Invalid state
                raise ValueError(f"Invalid State: {self._state}\r")
            
            yield self._state