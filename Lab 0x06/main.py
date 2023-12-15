"""!
@file main.py
This file creates object for all hardware on our Romi, necessary tasks with 
FSMs, and then runs a task scheduler in an infinite loop.
Most tuning can be achieved here!

@author Max Gardenswartz and Jonathan Lam
@date   2023-Sep-17 MLG Approximate date of creation of file
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

# Micropython Imports
import pyb
import micropython
from time import sleep

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

def main():
    # Disable REPL on UART2
    uart = pyb.UART(2, baudrate=115200)    
    vcp = pyb.USB_VCP()
    vcp.init()
    pyb.repl_uart(None) 

    # Shares
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
    
    # Constants -- Adjust your preferences/tuning here!!!
    debug = True
    Kp_LEFT = 0.6
    Ki_LEFT = 10
    Kp_RIGHT = 0.6
    Ki_RIGHT = 10
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
    pilotTaskFrequency = 10 # Hz
    IMUcalibrationTime = 2 # seconds

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
                        channel_a_pin = pyb.Pin.cpu.A0, 
                        channel_b_pin = pyb.Pin.cpu.A1, 
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
    
    # Front Sensor Array
    firstLeftSensorArray = sensorDriver(Pins = [pyb.Pin.cpu.A2, pyb.Pin.cpu.A6, pyb.Pin.cpu.A5],
                                        whiteCalibration = [1490, 1917, 1943],     
                                        blackCalibration = [1593, 3874, 2131])

    firstRightSensorArray = sensorDriver(Pins = [pyb.Pin.cpu.A4, pyb.Pin.cpu.B0, pyb.Pin.cpu.C1],
                                        whiteCalibration = [2338, 2123, 2735],
                                        blackCalibration = [4031, 4086, 4038])

    # Secondary Sensor Array
    secondSensorArray = sensorDriver(Pins=[ pyb.Pin.cpu.C4, pyb.Pin.cpu.C3, pyb.Pin.cpu.C2, pyb.Pin.cpu.B1, pyb.Pin.cpu.C0, pyb.Pin.cpu.C5 ],
                                    whiteCalibration = [2459, 2283, 702, 1523, 1780, 2223],
                                    blackCalibration = [3917, 3806, 3528, 3576, 3711, 3848])

    # # Define GPIO pins connected to the bumper sensors
    bumper_pins = [
    pyb.Pin.cpu.C10,  #  1
    pyb.Pin.cpu.C11,  #  2
    pyb.Pin.cpu.C12,  #  3
    pyb.Pin.cpu.D2,  #  4
    pyb.Pin.cpu.C8,  #  5
    pyb.Pin.cpu.A15   #  6
    ]
    # Configure the bumper pins as input with pull-up resistors
    bumpers = [pyb.Pin(pin, pyb.Pin.IN, pull=pyb.Pin.PULL_UP) for pin in bumper_pins]
    # Read the state of the bumper sensors with states = [bumper.value() for bumper in bumpers]

    # BNO055 IMU
    Pin_I2C1_SCL = pyb.Pin(pyb.Pin.cpu.B8, mode=pyb.Pin.ALT, alt=4)
    Pin_I2C1_SDA = pyb.Pin(pyb.Pin.cpu.B9, mode=pyb.Pin.ALT, alt=4)
    myIMU = BNO055_Driver(bus=1, baudrate=400_000)
    myIMU.begin_calibration()
    sleep(IMUcalibrationTime)

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

    myGarbageCollectTask = Task(garbageCollectTask().run,
                              priority = 999,
                              period = 30)

    myPilotTask = Task(pilotTask(
                                encoder_LEFT = encoder_LEFT,
                                encoder_RIGHT = encoder_RIGHT,
                                motor_RPM_wanted_LEFT=motor_RPM_wanted_LEFT,
                                motor_RPM_wanted_RIGHT=motor_RPM_wanted_RIGHT,
                                encoderCPR=encoderCPR,
                                revolutionLimit = revolutionLimit,
                                IMU=myIMU,
                                firstLeftRow = firstLeftSensorArray,
                                firstRightRow = firstRightSensorArray,
                                secondRow = secondSensorArray,
                                bumpers = bumpers,
                                debug = debug).run,
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
            # print("-"*50)
            # print(firstSensorArray.read_line_color())
            # print(secondSensorArray.read_color()[::1])
            # sleep_ms(100)
        except KeyboardInterrupt:
            #vcp.write("Exiting Program.\r\n")
            motor_LEFT.set_duty(0)
            motor_RIGHT.set_duty(0)
            #break

# Run the program.
if __name__ == "__main__":
    main()