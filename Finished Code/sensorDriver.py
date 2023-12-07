from pyb import Pin, ADC
class sensorDriver:
    """
    @brief Class for driving Pololu QTR Reflectance Sensors, Analog Version of any number of channels.
    @details This class sets up and manages the QTR sensors, including pin configuration, calibration values, and ADC setup.

    @param Pins: List of Pin objects connecting to each channel in the sensor array.
    @param whiteCalibration: List of white calibration values for each sensor channel.
    @param blackCalibration: List of black calibration values for each sensor channel.
    """

    def __init__(self,
        Pins: list(Pin),
        whiteCalibration: list(int),
        blackCalibration: list(int)):

        # Constants
        self.numberOfPins = len(Pins)
        self.blackCalibration = [0]*self.numberOfPins
        self.whiteCalibration = [0]*self.numberOfPins

        # Error Checking
        if len(whiteCalibration) != self.numberOfPins or len(blackCalibration) != self.numberOfPins:
            raise ValueError("Initialization Failed. Number of pins must match calibration")
        
        # Calibration Values
        for index in range(self.numberOfPins):
            self.blackCalibration[index] = 4095 - blackCalibration[index]
            self.whiteCalibration[index] = 4095 - whiteCalibration[index]

        # Pin Attributes
        self.Pins = [None]*self.numberOfPins
        for index in range(self.numberOfPins):
            self.Pins[index] = Pin(Pins[index], mode=Pin.ANALOG)

        # Timer
        self.adc = [None]*self.numberOfPins
        for index in range(self.numberOfPins):
            self.adc[index] = ADC(self.Pins[index])

    def read_raw(self):
        """
        @brief Read and return raw analog values from all configured channels of the reflectance array.
        @details This method reads the raw analog values from each configured channel and returns a list of integers representing the readings.

        @return List of integers representing raw analog values from each channel.
        """
        self.readings = [None]*self.numberOfPins
        for index in range(self.numberOfPins):
            self.readings[index] = self.adc[index].read()

        return self.readings

    
    def read_brightness(self):
        """
        @brief Read brightness values after rescaling raw analog readings to a 0-100 percent scale.
        @details This method calls the read_raw method to get raw analog readings, rescales the values to a 0-100 percent brightness scale (where 0% represents Black and 100% represents White), and returns a list of brightness values.

        @return List of integers representing brightness values for each channel.
        """
        self.readings = self.read_raw()

        # Rescale
        self.analogRange = [0]*self.numberOfPins
        self.percentBrightness = [0]*self.numberOfPins
        for index in range(self.numberOfPins):
            self.readings[index] = 4095 - self.readings[index]
            self.analogRange[index] = self.whiteCalibration[index] - self.blackCalibration[index]
            self.percentBrightness[index] = (self.readings[index] - self.blackCalibration[index])/self.analogRange[index]*100
            self.percentBrightness[index] = round(self.percentBrightness[index])
        return self.percentBrightness


    def read_color(self):
        """
        @brief Read colors based on brightness values for each sensor channel.
        @details This method calls the read_brightness method to get brightness values for each channel and converts them to colors, returning a list of color labels ("White" or "Black") corresponding to each sensor channel.

        @return List of strings representing colors for each channel.
        """
        self.percentBrightness = self.read_brightness()

        # Convert brightness to color.
        self.colors = [None]*self.numberOfPins
        for index in range(self.numberOfPins):
            if self.percentBrightness[index] >= 50:
                self.colors[index] = "White"
            else:
                self.colors[index] = "Black"

        return self.colors
    
    def recalibrate_white(self):
        """
        @brief Recalibrate sensors to a new white reference.
        @details This method reads raw analog values from all channels, updates the white calibration values, and returns the raw readings.

        @return List of integers representing raw analog values after recalibration.
        """
        self.readings = self.read_raw()
        for index in range(self.numberOfPins):
            self.whiteCalibration[index] = 4095 - self.readings[index]

        return self.readings

    def recalibrate_black(self):
        """
        @brief Recalibrate sensors to a new black reference.
        @details This method reads raw analog values from all channels, updates the black calibration values, and returns the raw readings.

        @return List of integers representing raw analog values after recalibration.
        """
        self.readings = self.read_raw()
        for index in range(self.numberOfPins):
            self.blackCalibration[index] = 4095 - self.readings[index]

        return self.readings
