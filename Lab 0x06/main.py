# Micropython Imports
import pyb
import micropython

# Ridgely Libaries
from task_share import *
from cotask import *

# Our Drivers
from closedLoopControl import *
from encoderDriver import *
from motorDriver import *
from sensorDriver import *
from BNO055_Driver import *

# Our Tasks
from motorControlTask import *
from garbageCollectTask import *
from pilotTask import *
from IMU_Task import *

# Troubleshoots PuTTy! -- Thank you, Jack Miller.
micropython.alloc_emergency_exception_buf(100)

if __name__ == "__main__":

    # Disable REPL on UART2
    uart = pyb.UART(2, baudrate=115200)    
    vcp = pyb.USB_VCP()
    vcp.init()
    pyb.repl_uart(None) 
    #uart.init(115200, bits=8, parity=None, stop=1)

    #  Shares
    controlMode = Share('B')
    motor_duty_wanted_LEFT = Share('f')
    motor_duty_wanted_RIGHT = Share('f')
    motor_RPM_wanted_LEFT = Share('f')
    motor_RPM_wanted_RIGHT = Share('f')
    motor_RPM_LEFT = Share('f')
    motor_RPM_RIGHT = Share('f')

    # Initialize Shares
    controlMode.put(1)
    motor_duty_wanted_LEFT.put(0)
    motor_duty_wanted_RIGHT.put(0)
    motor_RPM_wanted_LEFT.put(0)   # Max speed is 200 RPM
    motor_RPM_wanted_RIGHT.put(0)   # Max speed is 225 RPM
    motor_RPM_LEFT.put(0)
    motor_RPM_RIGHT.put(0)
    
    # Constants 
    debug = True
    Kp_LEFT = 0.6
    Ki_LEFT = 10
    Kp_RIGHT = 0.6
    Ki_RIGHT = 10
    cruiseSpeed = 60 # RPM
    deltaSpeedforTurn = 60 # RPM
    revolutionLimit = 3 # rev
    controlFrequency = 100 # Hz 
    PWMfrequency = 20_000 # Hz 
    AutoReloadValue = 65_535 # ticks
    max_duty = 100 # %
    encoderCPR = 1440 # Counter per revolution
    toggle_LEFT = False # Changes the sign of the summing junction 
    toggle_RIGHT = False # Changes the sign of the summing junction 
    flip_Speed_LEFT = True # Changes the speed printout's sign
    flip_Speed_RIGHT = True # Changes the speed printout's sign

    # Initialize sensors.
    # primarySensor = 
    # secondarySensor = 

    # Initializate PWM
    timerPWM = pyb.Timer(4, 
                         freq = PWMfrequency)

    # Initialize Left Motor
    motor_LEFT = motorDriver(PWM_timer = timerPWM, 
                             EN_pin = pyb.Pin.cpu.A8,
                             Dir_pin = pyb.Pin.cpu.A9,
                             EFF_pin= pyb.Pin.cpu.B6,
                             PWM_channel = 1)
    motorControl_LEFT = closedLoopControl(controlFrequency = controlFrequency,
                                   Kp = Kp_LEFT,
                                   Ki = Ki_LEFT,
                                   toggle=toggle_LEFT)
    
    # Initialize Left Encoder
    encoderTimer_LEFT= pyb.Timer(5, 
                                  period = AutoReloadValue, 
                                  prescaler = 0) 

    encoder_LEFT = encoderDriver(encoderTimer_LEFT, 
                        channel_a_pin = pyb.Pin.cpu.C13, 
                        channel_b_pin = pyb.Pin.cpu.C14, 
                        max_count = AutoReloadValue) 
    
    # Intialize Right Motor
    motor_RIGHT = motorDriver(PWM_timer = timerPWM, 
                             EN_pin = pyb.Pin.cpu.H1,
                             Dir_pin = pyb.Pin.cpu.H0,
                             EFF_pin= pyb.Pin.cpu.B7,
                             PWM_channel = 2)
    motorControl_RIGHT = closedLoopControl(controlFrequency = controlFrequency,
                                    Kp = Kp_RIGHT,
                                    Ki = Ki_RIGHT,
                                    toggle=toggle_RIGHT)

    # Initialize Right Encoder
    encoderTimer_RIGHT= pyb.Timer(3, 
                                 period = AutoReloadValue, 
                                 prescaler = 0) 
    encoder_RIGHT = encoderDriver(encoderTimer_RIGHT, 
                             channel_a_pin = pyb.Pin.cpu.B4, 
                             channel_b_pin = pyb.Pin.cpu.B5, 
                             max_count = AutoReloadValue) 
    
    # BNO055 IMU
    Pin_I2C1_SCL = pyb.Pin(pyb.Pin.cpu.B8, mode=pyb.Pin.ALT, alt=4)
    Pin_I2C1_SDA = pyb.Pin(pyb.Pin.cpu.B9, mode=pyb.Pin.ALT, alt=4)
    myIMU = BNO055_Driver(bus=1, baudrate=400_000)
    myIMU.begin_calibration()

    # Tasks
    motorControl_Task_LEFT = Task(motorControlTask(  motor = motor_LEFT,
                                                    motorControl = motorControl_LEFT,
                                                    encoder = encoder_LEFT,
                                                    controlMode = controlMode,
                                                    encoderCPR = encoderCPR,
                                                    max_duty = max_duty,
                                                    motor_RPM_wanted = motor_RPM_wanted_LEFT,
                                                    motor_RPM = motor_RPM_LEFT,
                                                    motor_duty_wanted = motor_duty_wanted_LEFT,
                                                    flip_Speed = flip_Speed_LEFT,
                                                    debug = not debug).run,
                                 priority = 1,
                                 period = 1000/controlFrequency)
    
    motorControl_Task_RIGHT = Task(motorControlTask(  motor = motor_RIGHT,
                                                    motorControl = motorControl_RIGHT,
                                                    encoder = encoder_RIGHT,
                                                    controlMode = controlMode,
                                                    encoderCPR = encoderCPR,
                                                    max_duty = max_duty,
                                                    motor_RPM_wanted = motor_RPM_wanted_RIGHT,
                                                    motor_RPM = motor_RPM_RIGHT,
                                                    motor_duty_wanted = motor_duty_wanted_RIGHT,
                                                    flip_Speed = flip_Speed_RIGHT,
                                                    debug = not debug).run,
                                 priority = 1,
                                 period = 1000/controlFrequency)

    garbageCollect_Task = Task(garbageCollectTask().run,
                              priority = 999,
                              period = 30)

    pilot_Task = Task(pilotTask(cruiseSpeed = cruiseSpeed,
                                deltaSpeedforTurn = deltaSpeedforTurn,
                                encoder_LEFT = encoder_LEFT,
                                encoder_RIGHT = encoder_RIGHT,
                                motor_RPM_wanted_LEFT=motor_RPM_wanted_LEFT,
                                motor_RPM_wanted_RIGHT=motor_RPM_wanted_RIGHT,
                                encoderCPR=encoderCPR,
                                revolutionLimit = revolutionLimit, IMU=myIMU,
                                print_flag= debug).run,
                      priority = 2,
                      period = 100)
    
    IMUTask = Task(IMU_Task(IMU=myIMU,
                            print_flag=debug).run,
                   priority = 2, 
                   period = 100)

    # Task Scheduler
    task_list.append(motorControl_Task_LEFT)
    task_list.append(motorControl_Task_RIGHT)
    task_list.append(garbageCollect_Task)
    task_list.append(pilot_Task)
    task_list.append(IMUTask)

    # Run the scheduler
    while True:
        try: 
            task_list.pri_sched()
        except KeyboardInterrupt:
            #vcp.write("Exiting Program.\r\n")
            motor_LEFT.set_duty(0)
            motor_RIGHT.set_duty(0)
            break

