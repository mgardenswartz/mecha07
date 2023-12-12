class LineColorReader:
    def __init__(self, sensor_pins):
        # Create QTRSensors instance with two emitter pins
        self.qtr_sensors = QTRSensors(sensor_pins, emitterPin=pyb.Pin.cpu.A5, evenEmitterPin=pyb.Pin.cpu.A5)

        # Set the dimming level (adjust as needed)
        self.qtr_sensors.dimmingLevel = 5

    def read_line_color(self):
        # Read sensor values
        self.qtr_sensors.read()

        # Read line position for each sensor
        output = []

        for i, pin in enumerate(self.qtr_sensors.sensorPins):
            raw_value = self.qtr_sensors.values[i]
            
            # Organize output for each sensor
            #sensor_info = f"Sensor {i+1}: Raw Value: {raw_value}, Color: {'White' if raw_value < 800 else 'Black'}"
            sensor_info = f"{'White' if raw_value < 800 else 'Black'}"
            output.append(sensor_info)

        # Print organized output horizontally
        # print(" | ".join(output))
        return output

    def read_raw(self):
        # Read sensor values
        self.qtr_sensors.read()

        # Read line position for each sensor
        output = []

        for i, pin in enumerate(self.qtr_sensors.sensorPins):
            raw_value = self.qtr_sensors.values[i]
            output.append(raw_value)

        return output

    def read_brightness(self):
        self.readings = self.read_raw()
        self.blackCalibration = [1028, 1028, 1028, 1028, 1028, 1724]
        self.whiteCalibration = [285, 285, 285, 285, 285, 537]

        # Rescale
        self.analogRange = [0]*6
        self.percentBrightness = [0]*6
        for index in range(6):
            self.analogRange[index] = self.whiteCalibration[index] - self.blackCalibration[index]
            self.percentBrightness[index] = (self.readings[index] - self.blackCalibration[index])/self.analogRange[index]*100
            self.percentBrightness[index] = round(self.percentBrightness[index])
        return self.percentBrightness         

    def read_color(self):
      self.percentBrightness = self.read_brightness()

      # Convert brightness to color.
      self.colors = [""]*6
      for index in range(6):
         if self.percentBrightness[index] >= 50:
            self.colors[index] = "White" 
         else:
            self.colors[index] = "Black" 

      return self.colors