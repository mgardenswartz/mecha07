#import pyb
from pyb import Pin, Timer
import L6206 as motor
from array import array
import encoder as Encoder #He capitalized it!
from time import sleep_ms
import micropython

# Troubleshoots PuTTy! -- Thank you, Jack Miller.
micropython.alloc_emergency_exception_buf(100)

# Initializae the Timer and Encoder
# Why can't we use Timer module 1 and 2?
timerencA= pyb.Timer(4, period=65535, prescaler=0) #Creating a timer object.
encoderA = Encoder.EncoderReader(timerencA, pyb.Pin.cpu.B6, pyb.Pin.cpu.B7, 65535) #this is for A; had to search for pins
timerencB= pyb.Timer(8, period=65535, prescaler=0) 
encoderB = Encoder.EncoderReader(timerencB, pyb.Pin.cpu.C6, pyb.Pin.cpu.C7, 65535) #this is for B; had to search for pins 

# Initialize the L6206 Motor Drivers
tim_A = Timer(3, freq=20000) #Create a timer object from module 3.
mot_A = motor.L6206(tim_A, Pin.cpu.B4, Pin.cpu.B5, Pin.cpu.A10)
mot_A.enable()
tim_B = Timer(5, freq = 20000) #Create a timer object from module 5
mot_B = motor.L6206(tim_B, Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.C1)
mot_B.enable()

# Set up for the callback timers. --Max needs clarification.
tim7 = Timer(7, freq=1000)     #it will be a timer 7 with 1 khz
tim6 = Timer(6, freq=1000)     # it will be timer 6 with 1khz

dataA = array('H', 3000*[0])    # make data for 3000 entries at 0 each and since there is a 3 index value it will print 1000 data.
dataB = array('H', 3000*[0])    # make data for 3000 entries at 0 each and since there is a 3 index value it will print 1000 data.
idx_A = 0
idx_B = 0

# Basically lab0 code again...
def tim_cbA(tim7):              #define the callback every time
   global dataA, idx_A            #make the data and index be global
   encoderA.update()

   if idx_A + 2 < 3000:  # Check if there is enough space to store the data
        dataA[idx_A] = idx_A // 3  # Store the index divided by 3
        dataA[idx_A + 1] = encoderA.get_position()  # Store the encoder position
        dataA[idx_A + 2] = encoderA.get_delta()  # Store the delta
        idx_A += 3  # Increase the array positioning by 3
   else:
        tim7.callback(None)  # Disable the callback if there's no more space
def tim_cbB(tim6):              #define the callback every time
   global dataB, idx_B            #make the data and index be global
   encoderB.update()

   if idx_B + 2 < 3000:  # Check if there is enough space to store the data
        dataB[idx_B] = idx_B // 3  # Store the index divided by 3
        dataB[idx_B + 1] = encoderB.get_position()  # Store the encoder position
        dataB[idx_B + 2] = encoderB.get_delta()  # Store the delta
        idx_B += 3  # Increase the array positioning by 3
   else:
        tim6.callback(None)  # Disable the callback if there's no more space

# Generate a callback. Turns on PuTTy printing for the upcoming test.
tim7.callback(tim_cbA)     
# Turn on Motor A for 1.2 s at full reverse.
mot_A.set_duty(-100)
sleep_ms(1200)
mot_A.set_duty(0)

# Check the length of idx_A. --Max needs help understanding.
for i in range(0, idx_A, 3):
    idx_A, pos_A, delta_A = dataA[i], dataA[i + 1], dataA[i + 2]

    # Calculate RPM for motor A
    frequency = 1000  # Hz (you may need to adjust this)
    
    # Convert delta value to RPM for motor A
    delta_rpm_A = delta_A * frequency*60 / 16384

    print(f"{idx_A},{pos_A},{delta_rpm_A}")

# Visually separate the tests in PuTTy.
print('-' * 30)

# Motor B test
tim6.callback(tim_cbB)     #generate a callback
mot_B.set_duty(-100)
sleep_ms(1200)
mot_B.set_duty(0)

for j in range(0, idx_B, 3): # Counting by threes, starting at 0, up to idx_B (exclusive)

    idx_B, pos_B, delta_B = dataB[j], dataB[j + 1], dataB[j + 2]

    # Calculate RPM for motor A
    
    frequency = 1000  # Hz (you may need to adjust this)
    
    # Convert delta value to RPM for motor A
    delta_rpm_B = delta_B * frequency*60 / 16384

    print(f"{idx_B},{pos_B},{delta_rpm_B}")