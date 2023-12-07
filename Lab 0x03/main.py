# Imports
import pyb
from pyb import UART,repl_uart,Pin,Timer
from task_share import Queue
import cotask
import L6206 as LM
from time import sleep_ms
import micropython
import ClosedLoop as CL
import encoder as EN #He capitalized it!
from array import array
import math as m
import time
import task_share
import UITask as UI
import motorControlTask as MCT

# Troubleshoots PuTTy! -- Thank you, Jack Miller.
micropython.alloc_emergency_exception_buf(100)

if __name__ == "__main__":

    # Disable REPL on UART2
    uart = pyb.UART(2, baudrate=115200)    
    vcp = pyb.USB_VCP()
    vcp.init()
    pyb.repl_uart(None) 
    #uart.init(115200, bits=8, parity=None, stop=1)
    
    # Create some queues to use for the task.
    queueLength= int(300)
    times = task_share.Queue('f', queueLength,name='times')
    positions = task_share.Queue('f', queueLength,name='positions')
    velocities = task_share.Queue('f',queueLength,name='velocities')
    deltas = task_share.Queue('f',queueLength,name='deltas')

    # Create some shares to use for the task.
    controlMode = task_share.Share('b',name='')
    zeroEncoder_flag_A = task_share.Share('b',name='')
    zeroEncoder_flag_B = task_share.Share('b',name='')
    getPosition_flag_A = task_share.Share('b',name='')
    getPosition_flag_B = task_share.Share('b',name='')
    getDelta_flag_A = task_share.Share('b',name='')
    getDelta_flag_B = task_share.Share('b',name='')
    getVelocity_flag_A = task_share.Share('b',name='')
    getVelocity_flag_B = task_share.Share('b',name='')
    updateDuty_flag_A = task_share.Share('b',name='')
    updateDuty_flag_B = task_share.Share('b',name='')
    updateGains_flag_A = task_share.Share('b',name='') 
    updateGains_flag_B = task_share.Share('b',name='') 
    updateVelocity_flag_A = task_share.Share('b',name='')
    updateVelocity_flag_B = task_share.Share('b',name='')
    collectTrial_flag_A = task_share.Share('b',name='')
    collectTrial_flag_B = task_share.Share('b',name='')
    runStep_flag_A = task_share.Share('b',name='')
    runStep_flag_B = task_share.Share('b',name='')
    newRunStep_flag_A = task_share.Share('f',name='')
    newRunStep_flag_B = task_share.Share('f',name='')

    # Buffers
    motor_ref_A = task_share.Share('f',name='Motor Speed or Duty for A')
    encPosition_A = task_share.Share('f',name='')
    encDelta_A = task_share.Share('f',name='')
    mot_RPM_A = task_share.Share('f',name='')
    new_Kp_A = task_share.Share('f',name='')
    motor_ref_B = task_share.Share('f',name='Motor Speed or Duty for B')
    encPosition_B = task_share.Share('f',name='')
    encDelta_B = task_share.Share('f',name='')
    mot_RPM_B = task_share.Share('f',name='')
    new_Kp_B = task_share.Share('f',name='')

    # Initialize Shares
    controlMode.put(900)
    zeroEncoder_flag_A.put(0)
    zeroEncoder_flag_B.put(0)
    getPosition_flag_A.put(0)
    getPosition_flag_B.put(0)
    getDelta_flag_A.put(0)
    getDelta_flag_B.put(0)
    getVelocity_flag_A.put(0)
    getVelocity_flag_B.put(0)
    updateDuty_flag_A.put(0)
    updateDuty_flag_B.put(0)
    updateGains_flag_A.put(0)
    updateGains_flag_B.put(0)
    updateVelocity_flag_A.put(0)
    updateVelocity_flag_B.put(0)
    collectTrial_flag_A.put(0)
    collectTrial_flag_B.put(0)
    #runStep_flag_A.put(0)
    #runStep_flag_B.put(0)
    newRunStep_flag_A.put(0)
    newRunStep_flag_B.put(0)
    motor_ref_A.put(0)
    encPosition_A.put(0)
    encDelta_A.put(0)
    mot_RPM_A.put(0)
    new_Kp_A.put(0)
    motor_ref_B.put(0)
    encPosition_B.put(0)
    encDelta_B.put(0)
    mot_RPM_B.put(0)
    new_Kp_B.put(0)

    # Object initialization
    Kp = 1
    Ki = 0
    controlFrequency = 100
    PWMfrequency = 20_000
    AutoReloadValue = 65535
    max_duty = 100 # percent
    trialTime = 0.8 # Times for step-response and collection trials
    #maxTrialPoints = 1+trialTime*controlFrequency
    maxTrialPoints = 250

    tim_A = Timer(3, freq=PWMfrequency)
    mot_A = LM.L6206(tim_A, Pin.cpu.B4, Pin.cpu.B5, Pin.cpu.A10)
    mot_A_ctrl = CL.ClosedLoop(controlFrequency=controlFrequency,Kp=Kp,Ki=Ki)
    timerencA= pyb.Timer(4, period=AutoReloadValue, prescaler=0) #Creating a timer object.
    #enc_A = EN.EncoderReader(timerencA, pyb.Pin.cpu.B6, pyb.Pin.cpu.B7, AutoReloadValue) #this is for A; had to search for pins
    enc_A = EN.EncoderReader(timerencA, pyb.Pin.cpu.B6, pyb.Pin.cpu.B7, AutoReloadValue) #this is for A; had to search for pins
    tim_B = Timer(5, freq=PWMfrequency)
    mot_B = LM.L6206(tim_B, Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.C1)
    mot_B_ctrl = CL.ClosedLoop(controlFrequency=controlFrequency,Kp=Kp,Ki=Ki)
    # timerancB = timerencA
    # enc_B = enc_A
    timerencB= pyb.Timer(8, period=AutoReloadValue, prescaler=0) 
    enc_B = EN.EncoderReader(timerencB, pyb.Pin.cpu.C6, pyb.Pin.cpu.C7, AutoReloadValue) #this is for B; had to search for pins   
    
    # Create task objects
    UI_Task = cotask.Task(UI.UITask(motor_ref_A = motor_ref_A, motor_ref_B = motor_ref_B,
                                        q_times = times, 
                                        uart = uart,
                                        vcp=vcp,
                                        q_pos = positions, q_vel = velocities, 
                                        q_deltas = deltas,  
                                        controlMode = controlMode, zeroEncoder_flag_A = zeroEncoder_flag_A, 
                                        getPosition_flag_A = getPosition_flag_A, 
                                        getDelta_flag_A = getDelta_flag_A, getVelocity_flag_A = getVelocity_flag_A, 
                                        updateDuty_flag_A = updateDuty_flag_A, updateGains_flag_A = updateGains_flag_A, 
                                        updateVelocity_flag_A = updateVelocity_flag_A, collectTrial_flag_A = collectTrial_flag_A, 
                                        newRunStep_flag_A = newRunStep_flag_A, encPosition_A = encPosition_A, 
                                        encDelta_A = encDelta_A, mot_RPM_A = mot_RPM_A, new_Kp_A = new_Kp_A, 
                                        zeroEncoder_flag_B = zeroEncoder_flag_B, getPosition_flag_B = getPosition_flag_B, 
                                        getDelta_flag_B = getDelta_flag_B, getVelocity_flag_B = getVelocity_flag_B, 
                                        updateDuty_flag_B = updateDuty_flag_B, updateGains_flag_B = updateGains_flag_B, 
                                        updateVelocity_flag_B = updateVelocity_flag_B, collectTrial_flag_B = collectTrial_flag_B, 
                                        newRunStep_flag_B = newRunStep_flag_B, encPosition_B = encPosition_B, encDelta_B = encDelta_B, 
                                        mot_RPM_B = mot_RPM_B, new_Kp_B = new_Kp_B, maxTrialPoints = maxTrialPoints).run,
                            name="UI",priority=1,period=5)

    mot_A_ctrl_task  = cotask.Task(MCT.motorControlTask(ref=motor_ref_A, max_duty=max_duty,
                                              q_times=times, q_pos=positions, q_vel=velocities, q_deltas=deltas,
                                              mot_object=mot_A, encoder_object=enc_A, mot_ctrl_object=mot_A_ctrl,
                                              controlMode=controlMode,zeroEncoder_flag=zeroEncoder_flag_A,
                                              getPosition_flag=getPosition_flag_A,getVelocity_flag=getVelocity_flag_A,
                                              getDelta_flag=getDelta_flag_A, updateDuty_flag = updateDuty_flag_A,
                                              updateGains_flag=updateGains_flag_A, 
                                              updateVelocity_flag=updateVelocity_flag_A,collectTrial_flag=collectTrial_flag_A,
                                              newRunStep_flag=newRunStep_flag_A, encPosition=encPosition_A,
                                              encDelta=encDelta_A,mot_RPM=mot_RPM_A,new_Kp=new_Kp_A,
                                              maxTrialPoints=maxTrialPoints).run,
                                    name="Motor A Control",priority=1, period=1)
    
    # mot_B_ctrl_task = cotask.Task(MCT.motorControlTask(ref=motor_ref_B, max_duty=max_duty,
    #                                                    q_times=times, q_pos=positions, q_deltas=deltas, q_vel=velocities,
    #                                                 mot_object=mot_B, encoder_object=enc_B, mot_ctrl_object=mot_B_ctrl,
    #                                                 controlMode=controlMode,zeroEncoder_flag=zeroEncoder_flag_B,
    #                                                 getPosition_flag=getPosition_flag_B,getVelocity_flag=getVelocity_flag_B,
    #                                                 getDelta_flag=getDelta_flag_B, updateDuty_flag = updateDuty_flag_B,
    #                                                 updateGains_flag=updateGains_flag_B,
    #                                                 updateVelocity_flag=updateVelocity_flag_B,collectTrial_flag=collectTrial_flag_B,
    #                                                 newRunStep_flag=newRunStep_flag_B,encPosition=encPosition_B,
    #                                                 encDelta=encDelta_B,mot_RPM=mot_RPM_B,new_Kp=new_Kp_B,
    #                                                 maxTrialPoints = maxTrialPoints).run,
    #                                 name="Motor B Control",priority=2, period=10)
    
    # Append the newly created task to the task list
    cotask.task_list.append(UI_Task)
    cotask.task_list.append(mot_A_ctrl_task )
    #cotask.task_list.append(mot_B_ctrl_task )

    # Run the scheduler
    while True:
        try: 
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            vcp.write("Exiting Program.\r\n")
            mot_A.set_duty(0)
            mot_B.set_duty(0)
            break

