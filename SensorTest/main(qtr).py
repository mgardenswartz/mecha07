import pyb
from qtrsensors import QTRSensors, EMITTERS_NONE
from time import sleep_ms

class LineColorReader:
    def __init__(self, sensor_pins):
        # Create QTRSensors instance with two emitter pins
        self.qtr_sensors = QTRSensors(sensor_pins, emitterPin=pyb.Pin.cpu.A5, evenEmitterPin=pyb.Pin.cpu.A5)

    def read_line_color(self):
        # Read sensor values
        self.qtr_sensors.read()

        # Read line position for each sensor
        output = []

        for i, pin in enumerate(self.qtr_sensors.sensorPins):
            raw_value = self.qtr_sensors.values[i]
            
            # Organize output for each sensor
            sensor_info = f"Sensor {i+1}: Raw Value: {raw_value}, Color: {'White' if raw_value > 800 else 'Black'}"
            
            output.append(sensor_info)

        # Print organized output horizontally
        print(" | ".join(output))

def main():
    # Define your sensor pins
    sensor_pins = [pyb.Pin.cpu.A5, pyb.Pin.cpu.A6, pyb.Pin.cpu.A2, pyb.Pin.cpu.A4, pyb.Pin.cpu.B0, pyb.Pin.cpu.C1]

    # Create LineColorReader instance
    line_color_reader = LineColorReader(sensor_pins)

    while True:
        # Example of reading sensor values and categorizing colors
        line_color_reader.read_line_color()
        sleep_ms(50)

if __name__ == "__main__":
    main()
