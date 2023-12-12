# Micropython Imports
import pyb
import micropython
from time import sleep, sleep_ms

# Ridgely Libaries
from task_share import *
from cotask import *

# Our Drivers
from closedLoopControl import *
from encoderDriver import *
from motorDriver import *
from sensorDriver import *
from BNO055_Driver import *
from qtrsensors import QTRSensors, EMITTERS_NONE

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
    controlMode.put(1) # 1 is Closed-Loop, 0 is Open-Loop
    motor_duty_wanted_LEFT.put(0) # Only valid in Open-Loop
    motor_duty_wanted_RIGHT.put(0) # Only valid in Open-Loop
    motor_RPM_wanted_LEFT.put(0)  # Only valid in Closed-Loop  # Max speed is 200 RPM
    motor_RPM_wanted_RIGHT.put(0)   # Only valid in Closed-Loop  # Max speed is 225 RPM
    motor_RPM_LEFT.put(0) # Current Motor RPM as read from encoder
    motor_RPM_RIGHT.put(0) # Current Motor RPM as read from encoder
    
    # Constants 
    debug = True
    Kp_LEFT = 0.6 # Left Motor Proportional Control Constant
    Ki_LEFT = 10  # Left Motor Integral Control Constant
    Kp_RIGHT = 0.6 # Right Motor Proportional Control Constant
    Ki_RIGHT = 10  # Right Motor Integral Control Constant
    cruiseSpeed = 60 # RPM
    controlFrequency = 100 # Hz for motor controller
    PWMfrequency = 20_000 # Hz 
    AutoReloadValue = 65_535 # ticks
    max_duty = 100 # % # Cannot exceed 100. Maximum voltage fed to motor is 12 V at 100.
    encoderCPR = 1440 # Counter per revolution
    toggle_LEFT = False # Changes the sign of the summing junction 
    toggle_RIGHT = False # Changes the sign of the summing junction 
    flip_Speed_LEFT = True # Changes the speed printout's sign. Same as flipping motor leads
    flip_Speed_RIGHT = True # Changes the speed printout's sign. Same as flipping motor leads
    pilotTaskFrequency = 10 # Hz
    Kp_line = 0.6 # Line-Following Proportional Control Constant 
    Ki_line = 10 # Line-Following Proportional Control Constant 
    max_spin = 30 # dps for Line-Following PI Controller
    IMUcalibrationTime = 2 # seconds

    # PWM
    timerPWM = pyb.Timer(4, 
                         freq = PWMfrequency)

    # Initialize Left Motor
    motor_LEFT = motorDriver(PWM_timer = timerPWM, 
                             EN_pin = pyb.Pin.cpu.A8,
                             Dir_pin = pyb.Pin.cpu.A9,
                             EFF_pin= pyb.Pin.cpu.B6,
                             PWM_channel = 1)
    
    # Left Motor PI Controller
    motorControl_LEFT = closedLoopControl(controlFrequency = controlFrequency,
                                   Kp = Kp_LEFT,
                                   Ki = Ki_LEFT,
                                   toggle=toggle_LEFT)
    
    # Left Encoder
    encoderTimer_LEFT= pyb.Timer(5, 
                                  period = AutoReloadValue, 
                                  prescaler = 0) 

    encoder_LEFT = encoderDriver(encoderTimer_LEFT, 
                        channel_a_pin = pyb.Pin.cpu.A0, 
                        channel_b_pin = pyb.Pin.cpu.A1, 
                        max_count = AutoReloadValue) 
    
    # Right Motor
    motor_RIGHT = motorDriver(PWM_timer = timerPWM, 
                             EN_pin = pyb.Pin.cpu.H1,
                             Dir_pin = pyb.Pin.cpu.H0, 
                             EFF_pin= pyb.Pin.cpu.B7,
                             PWM_channel = 2)
    
    # Right Motor PI Controller
    motorControl_RIGHT = closedLoopControl(controlFrequency = controlFrequency,
                                    Kp = Kp_RIGHT,
                                    Ki = Ki_RIGHT,
                                    toggle=toggle_RIGHT)

    # Right Encoder
    encoderTimer_RIGHT= pyb.Timer(3, 
                                 period = AutoReloadValue, 
                                 prescaler = 0) 
    encoder_RIGHT = encoderDriver(encoderTimer_RIGHT, 
                             channel_a_pin = pyb.Pin.cpu.B4, 
                             channel_b_pin = pyb.Pin.cpu.B5, 
                             max_count = AutoReloadValue) 
    
    # Front Sensor Arrays (We have two three-channel analog arrays up front.)
    firstLeftSensorArray = sensorDriver(Pins = [pyb.Pin.cpu.A2, pyb.Pin.cpu.A6, pyb.Pin.cpu.A5],
                                        whiteCalibration = [1500]*3,
                                        blackCalibration = [3800]*3)
    firstRightSensorArray = sensorDriver(Pins = [pyb.Pin.cpu.A4, pyb.Pin.cpu.B0, pyb.Pin.cpu.C1],
                                        whiteCalibration = [1500]*3,
                                        blackCalibration = [3800]*3)

    # Secondary Sensor Array
    secondSensorArray = sensorDriver(Pins=[ pyb.Pin.cpu.C4, pyb.Pin.cpu.C3, pyb.Pin.cpu.C2, pyb.Pin.cpu.B1, pyb.Pin.cpu.C5, pyb.Pin.cpu.C0 ],
                                    whiteCalibration = [2383, 2159, 697, 1550, 1691, 2048],
                                    blackCalibration = [3898, 3677, 3215, 3485, 3483, 3815])

    # Line Sensor PI Controller
    lineSensorControl = closedLoopControl(controlFrequency = pilotTaskFrequency,
                                          toggle = False,
                                          Kp = Kp_line, 
                                          Ki = Ki_line)

    # # Define GPIO pins connected to the bumper sensors
    bumper_pins = [
    pyb.Pin.cpu.C10,  
    pyb.Pin.cpu.C11,  
    pyb.Pin.cpu.C12, 
    pyb.Pin.cpu.D2,  
    pyb.Pin.cpu.C8,  
    pyb.Pin.cpu.A15   
    ]

    # Configure the bumper pins as input with pull-up resistors
    bumpers = [pyb.Pin(pin, pyb.Pin.IN, pull=pyb.Pin.PULL_UP) for pin in bumper_pins]

    # BNO055 IMU
    Pin_I2C1_SCL = pyb.Pin(pyb.Pin.cpu.B8, mode=pyb.Pin.ALT, alt=4)
    Pin_I2C1_SDA = pyb.Pin(pyb.Pin.cpu.B9, mode=pyb.Pin.ALT, alt=4)
    myIMU = BNO055_Driver(bus=1, baudrate=400_000)
    # Allow time to calibrate IMU
    myIMU.begin_calibration()
    sleep(IMUcalibrationTime)

    # Tasks
    # Higher priorities values can run BEFORe those with lower priority vaues.
    # That is 5 will be prioritized over a value of 1.
    # Period is in milliseconds, NOT seconds. 
    
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

    myGarbageCollectTask = Task(garbageCollectTask().run,
                              priority = 999,
                              period = 30)

    myPilotTask = Task(pilotTask(cruiseSpeed = cruiseSpeed,
                                encoder_LEFT = encoder_LEFT,
                                encoder_RIGHT = encoder_RIGHT,
                                motor_RPM_wanted_LEFT=motor_RPM_wanted_LEFT,
                                motor_RPM_wanted_RIGHT=motor_RPM_wanted_RIGHT,
                                encoderCPR=encoderCPR,
                                IMU=myIMU,
                                print_flag= debug,
                                firstLeftRow = firstLeftSensorArray,
                                firstRightRow = firstRightSensorArray,
                                secondRow = secondSensorArray,
                                bumpers = bumpers,
                                debug = debug,
                                controller = lineSensorControl,
                                max_spin = max_spin).run,
                      priority = 2,
                      period = 1000/pilotTaskFrequency)
    
    myIMUTask = Task(IMU_Task(IMU=myIMU,
                            print_flag= not debug).run,
                   priority = 2, 
                   period = 1000/pilotTaskFrequency)

    # Task Scheduler
    task_list.append(motorControl_Task_LEFT)
    task_list.append(motorControl_Task_RIGHT)
    task_list.append(myGarbageCollectTask)
    task_list.append(myPilotTask)
    task_list.append(myIMUTask)

    # Run the scheduler
    while True:
        try: 
            task_list.pri_sched()
        except KeyboardInterrupt:
            print("Exiting Program.\r\n")
            motor_LEFT.set_duty(0)
            motor_RIGHT.set_duty(0)
            break

