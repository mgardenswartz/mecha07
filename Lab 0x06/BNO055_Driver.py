import pyb # type: ignore
import time
import struct  # Add this import statement

class BNO055_Driver:
    """
    @brief Driver for the BNO055 IMU over I2C.
    @details This class provides functionality to interact with the BNO055 IMU sensor over I2C communication.

    @param baudrate: The baud rate for the I2C communication.
    @param bus: The I2C bus number to which the IMU is connected.
    """
    def __init__(self, baudrate: int, bus: int):

        # Initialization
        self.baudrate = baudrate
        self.bus = bus
        self.i2c = pyb.I2C(self.bus, pyb.I2C.MASTER, baudrate=self.baudrate)

        # Constants
        self.timeout = 10000  # ms
        self.axis_map_sign_addr = 0x42
        self.axis_map_config_addr = 0x41
        self.sys_calib_mask = 0b1100_0000
        self.gyr_calib_mask = 0b0011_0000
        self.acc_calib_mask = 0b0000_1100
        self.mag_calib_mask = 0b0000_0011
        self.axis_placement_mapping = {
            "P0": (0x21, 0x04),
            "P1": (0x24, 0x00),
            "P2": (0x24, 0x06),
            "P3": (0x21, 0x02),
            "P4": (0x24, 0x03),
            "P5": (0x21, 0x01),
            "P6": (0x21, 0x07),
            "P7": (0x24, 0x05)}

    def begin_calibration(self):
        """
        @brief Initiate the calibration process for the BNO055 IMU.
        @details This method performs the necessary steps to begin the calibration process:
            - Sets the sensor to configuration mode.
            - Triggers a reset of the sensor.
            - Waits for the sensor to complete the boot-up process.
            - Sets the sensor to normal power mode.
            - Configures the sensor to operate in the "NDOF" (Nine Degrees of Freedom) mode.
            - Configures the coordinate system for the sensor.

        @return True if calibration initiation is successful.
        """

        # Set to config mode
        self.set_mode("CONFIG")

        # Trigger a reset
        self.i2c.mem_write(0x20, addr=0x28, memaddr=0x3F)

        # Wait
        time.sleep(1)

        # Wait for device to be done booting up
        # Checks until device ID is correct
        while self.i2c.mem_read(1, addr=0x28, memaddr=0x00)[0] != 0xA0:
            time.sleep(0.01)
        time.sleep(0.05)

        # Set to normal power mode
        self.i2c.mem_write(0x00, addr=0x28, memaddr=0x3E)
        self.i2c.mem_write(0x00, addr=0x28, memaddr=0x07)
        self.i2c.mem_write(0x00, addr=0x28, memaddr=0x3F)
        
        # Change to NDOF mode.
        self.set_mode("NDOF")

        # Set up correct coordinate system for our Romi specifically.
        self.configure_coordinate_system() 

        # Wait
        time.sleep(0.02)

        return True

    def set_mode(self, mode: str):
        """
        @brief Set the fusion mode for the BNO055 IMU.
        @details This method sets the fusion mode for the IMU, where "CONFIG" corresponds to calibration mode and "NDOF" corresponds to Nine Degrees of Freedom mode.

        @param mode: Fusion mode to set ("CONFIG" for calibration, "NDOF" for Nine Degrees of Freedom).
        @throws Exception: Raised if an invalid fusion mode is provided.

        @note The method updates the internal mode attribute and writes the corresponding value to the sensor's register.
        """
        self.mode = mode

        if self.mode == "CONFIG":
            self.i2c.mem_write(0x00, addr=0x28, memaddr=0x3D, timeout=self.timeout)
        elif self.mode == "NDOF":
            self.i2c.mem_write(0x0C, addr=0x28, memaddr=0x3D, timeout=self.timeout)
        else:
            raise Exception(f"Invalid Fusion Mode Selected: {self.mode}.")
        
        time.sleep(0.02)

    def configure_coordinate_system(self, coord_sys: str = None):
        """
        @brief Configure the coordinate system for the BNO055 IMU.
        @details This method determines the correct orientation for the coordinate system on the IMU. If no specific coordinate system is provided, it uses default values. Otherwise, it checks for valid pre-defined coordinate systems.

        @param coord_sys: String representing a pre-defined coordinate system, if None, default values are used.
        @throws ValueError: Raised if an invalid coordinate system is provided.

        @note The method writes the necessary configuration values to the sensor's registers and sets the fusion mode to "NDOF."
        """
        if coord_sys is None:
            x = 0x01  # remap X to Y
            y = 0x00  # remap Y to X
            z = 0x02  # leave z alone
            x_dir = 0x01  # negative
            y_dir = 0x00  # positive
            z_dir = 0x00  # positive
            self.set_mode("CONFIG")
            map_config = 0x00
            map_config |= (z & 0x03) << 4
            map_config |= (y & 0x03) << 2
            map_config |= x & 0x03
            self.i2c.mem_write(map_config, addr=0x28, memaddr=0x41)

            sign_config = 0x00
            sign_config |= (x_dir & 0x01) << 2
            sign_config |= (y_dir & 0x01) << 1
            sign_config |= z_dir & 0x01
            self.i2c.mem_write(sign_config, addr=0x28, memaddr=0x42)

        elif coord_sys in self.axis_placement_mapping:
            sign, config = self.axis_placement_mapping[coord_sys]
            self.i2c.mem_write(sign, addr=0x28, memaddr=self.axis_map_sign_addr)
            self.i2c.mem_write(config, addr=0x28, memaddr=self.axis_map_config_addr)
        else:
            raise ValueError(f"Invalid coordinate system: {coord_sys}. Use one of {list(self.axis_placement_mapping.keys())}.")

        # Set mode to be used for this IMU on the Romi.
        self.set_mode("NDOF")

    def retrieve_calibration_status(self):
        """
        @brief Retrieve calibration status for the BNO055 IMU.
        @details This method reads the calibration status from the sensor and returns a tuple with four entries, where each entry represents the calibration status for the overall system, gyroscope, accelerometer, and magnetometer. Possible values for each entry are 0-3, where 3 indicates full calibration, and lower values indicate progression toward full calibration.

        @return Tuple with four integers representing calibration status (sys_calib, gyr_calib, acc_calib, mag_calib).
        """
        calib_data = self.i2c.mem_read(1, addr=0x28, memaddr=0x35)
        calib_value = calib_data[0]

        sys_calib = (calib_value >> 6) & 0x03
        gyr_calib = (calib_value >> 4) & 0x03
        acc_calib = (calib_value >> 2) & 0x03
        mag_calib = calib_value & 0x03

        return sys_calib, gyr_calib, acc_calib, mag_calib

    def retrieve_coefficients(self):
        """
        @brief Retrieve calibration coefficients for the BNO055 IMU.
        @details This method sets the sensor to configuration mode, reads 22 calibration coefficients, and returns them as a list. These coefficients are essential for correctly reading data from the IMU.

        @return List of 22 integers representing calibration coefficients.
        """
        self.set_mode("CONFIG")

        self.coefficients_to_read = bytearray(22)
        self.i2c.mem_read(self.coefficients_to_read, addr=0x28, memaddr=0x55)

        self.results = list(self.coefficients_to_read)

        self.set_mode("NDOF")

        return self.results
    
    def write_coefficients(self, coefficients: list[int]):
        """
        @brief Write a given set of 22 calibration coefficients to the BNO055 IMU.
        @details This method sets the sensor to configuration mode, writes the provided 22 calibration coefficients to the IMU, and then sets the sensor back to Nine Degrees of Freedom (NDOF) mode.

        @param coefficients: List of 22 integers representing calibration coefficients to be written.
        @throws ValueError: Raised if the input list does not contain 22 integers or if any element is out of the range of a 16-bit signed integer.
        """
        self.set_mode("CONFIG")

        coefficients_to_write = coefficients

        # Test for valid length.
        if len(coefficients_to_write) != 22:
            raise ValueError("Input does not contain 22 integers.")
        
        # Are all elements in the range of a 16-bit signed integer?
        if any(int(element) < -32768 or int(element) > 32767 for element in coefficients_to_write):
            raise ValueError("At least one element is out of range for a 16-bit number.")
         
        coefficients = [int(item) for item in coefficients] 
        data_to_write = bytearray(coefficients)
        self.i2c.mem_write(data_to_write, addr=0x28, memaddr=0x55)

        self.set_mode("NDOF")

    def read_Euler_angles(self):
        """
        @brief Read Euler angles from the BNO055 IMU.
        @details This method reads raw Euler angles from the sensor, unpacks the values, and returns them as a tuple of three angles (roll, pitch, yaw).

        @return Tuple of three floats representing Euler angles (roll, pitch, yaw) in degrees.
        """
        raw_Euler_angles = bytearray(6)
        self.i2c.mem_read(raw_Euler_angles, addr=0x28, memaddr=0x1A)
        Euler_angles = struct.unpack('hhh', struct.pack('BBBBBB', raw_Euler_angles[0], raw_Euler_angles[1], raw_Euler_angles[2], raw_Euler_angles[3], raw_Euler_angles[4], raw_Euler_angles[5]))

        return tuple([i/16 for i in Euler_angles])

    def read_angular_velocities(self):
        """
        @brief Read angular velocities from the BNO055 IMU gyroscope.
        @details This method reads raw angular velocities from the gyroscope, unpacks the values, and returns them as a tuple of three angular velocities (x, y, z).

        @return Tuple of three floats representing angular velocities (x, y, z) in degrees per second.
        """
        raw_gyr_rate = bytearray(6)
        self.i2c.mem_read(raw_gyr_rate, addr=0x28, memaddr=0x14)
        gyr_rate = struct.unpack('hhh', struct.pack('BBBBBB', raw_gyr_rate[0], raw_gyr_rate[1], raw_gyr_rate[2], raw_gyr_rate[3], raw_gyr_rate[4], raw_gyr_rate[5]))

        return tuple([i/900 for i in gyr_rate])

    
    def update_cal_coeffs(self):
        """
        @brief Update calibration coefficients for the BNO055 IMU from a file.
        @details This method reads a line from a file containing hexadecimal values, converts them to integers, and writes them as calibration coefficients to the IMU.

        @note The file path and format must be consistent with the expected input.

        @return Bytearray representing the updated calibration coefficients.
        """
        with open('/flash/IMU_cal_coeff.txt', 'r') as file:
            line = file.readline()
        line2 = line.split(",")
        coeff = [int(b, 16) for b in line2]
        byte_in = bytearray(coeff)
        self.i2c.mem_write(byte_in, addr=0x28, memaddr=0x55)
        return byte_in


    # def write_cal_coeffs(self):
    #     with open('/flash/IMU_cal_coeff.txt', 'w') as file:
    #         file.write(','.join(str(x) for x in self.coeffs))
