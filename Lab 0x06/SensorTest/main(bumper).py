import pyb
import time

# Define GPIO pins connected to the bumper sensors
bumper_pins = [
    pyb.Pin.cpu.C10,  # Change this to the actual pin for sensor 1
    pyb.Pin.cpu.C11,  # Change this to the actual pin for sensor 2
    pyb.Pin.cpu.C12,  # Change this to the actual pin for sensor 3
    pyb.Pin.cpu.D2,  # Change this to the actual pin for sensor 4
    pyb.Pin.cpu.C8,  # Change this to the actual pin for sensor 5
    pyb.Pin.cpu.A13   # Change this to the actual pin for sensor 6
]

# Configure the bumper pins as input with pull-up resistors
bumpers = [pyb.Pin(pin, pyb.Pin.IN, pull=pyb.Pin.PULL_UP) for pin in bumper_pins]

try:
    while True:
        # Read the state of the bumper sensors
        states = [bumper.value() for bumper in bumpers]

        # Display the state of each sensor
        for i, state in enumerate(states):
            if state:
                print(f"Sensor {i+1} ON")
            else:
                print(f"Sensor {i+1} OFF")

        # Add a short delay to avoid continuous printing (adjust as needed)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting program")
