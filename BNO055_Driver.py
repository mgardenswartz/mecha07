from pyb import I2C
from csvread import *
import os
class BNO055_Driver:
    def __init__(self,
                 baudrate: int,
                 bus: int):
        
        # Initialization
        self.baudate = baudrate
        self.bus = bus
        self.i2c(self.bus, I2C.CONTROLLER, baudrate=baudrate, gencall=False, dma=False)

        # Constants
        self.timeout = 3000 #ms
        self.mode_addr = 0x3D
        self.calib_stat_addr = 0x35
        self.unit_sel_addr = 0x3B
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
        # Files
        self.file_name = "IMU_cal_coeffs.txt"
        self.files = os.listdir()
        if self.file_name in self.files:
            self.x_values,self.y_values,self.titles = csvread(self.file_name)
            
        else:
            raise FileNotFoundError(f"The file {self.file_name} was not found.")

        # Set units m/s^2, Dps, deg, degF, Windows 
        self.i2c.mem_write(0b0001_0000, addr = self.unit_sel_addr) 
        self.set_mode("IMU")
        self.configure_coordinate_system("P2")

    def set_mode(self,mode: str):
        self.mode = mode

        if self.i2c.is_ready(self.mode_addr):
            if self.mode == "IMU":
                self.i2c.mem_write(0b1000, addr = self.mode_addr, timeout=self.timeout)
            elif self.mode == "COMPASS":
                self.i2c.mem_write(0b1001, addr = self.mode_addr, timeout=self.timeout)
            elif self.mode == "M4G":
                self.i2c.mem_write(0b1010, addr = self.mode_addr, timeout=self.timeout)
            elif self.mode == "NDOF_FMC_OFF":
                self.i2c.mem_write(0b1011, addr = self.mode_addr, timeout=self.timeout)
            elif self.mode == "NDOF":
                self.i2c.mem_write(0b1100, addr = self.mode_addr, timeout=self.timeout)
            else:
                raise Exception(f"Invalid Fusion Mode Selected: {self.mode}.")
        else:
            raise Exception(f"Address {self.mode_addr} is not ready.")

    def retrieve_calibration_status(self):
        self.calib_data = bytearray(1)
        self.i2c.mem_read(self.calib_data,
                      addr = self.calib_stat_addr,
                      timeout = self.timeout)
        
        if self.calib_data & self.sys_calib_mask == self.sys_calib_mask:
            self.sys_cal = True 
        else:
            self.sys_cal = False

        if self.calib_data & self.gyr_calib_mask == self.gyr_calib_mask:
            self.gyr_cal = True
        else:
            self.gyr_cal = False

        if self.calib_data & self.acc_calib_mask == self.acc_calib_mask:
            self.acc_cal = True
        else:
            self.acc_cal = False
        
        if self.calib_data & self.mag_calib_mask == self.mag_calib_mask:
            self.mag_cal = True
        else:
            self.mag_cal = False

        return self.sys_cal,self.gyr_cal,self.acc_cal,self.mag_cal
    
    def retrieve_coefficients(self):
        self.coefficients_to_read = bytearray(22)
        self.results = [0]*11
        for index in range(22):
            self.i2c.mem_read(self.coefficients_to_read[index],
                                addr = 0x6A - index,
                                timeout = self.timeout)
            if index % 2 == 0: 
                self.results[index//2] = (self.coefficients_to_read[index] << 8) | self.coefficients_to_read[index+1]    
        
        return self.results
        
    def write_coefficients(self,coefficients: list[int]):
        self.coefficients_to_write = coefficients

        # Test for valid length.
        if len(self.coefficients_to_write) != 22:
            raise ValueError("Input does not contain 22 integers.")
        
        # Are all elements in range of a 16-bit sign integer?
        if any(element < -32768 or element > 32767 for element in self.coefficients_to_write):
            raise ValueError("At least one element is out of range for a 16-bit number.")
        
        # Convert to hex.
        self.coefficients_to_write_hex = [hex(element) for element in self.coefficients_to_write]

        # Separate into twice as many 8-bytes and write to registers.
        for index in range(0,11):
            self.MSB,self.LSB = self.separate_16bit_hex(self.coefficients_to_write_hex[index])
            self.i2c.mem_write(self.MSB, addr = 0x6A - 2*index, timeout = self.timeout)
            self.i2c.mem_write(self.LSB, addr = 0x69 - 2*index, timeout = self.timeout)

    def read_Euler_angles(self):
        self.raw_Euler_angles = bytearray(6)
        for index in range(6):
            self.i2c.mem_read( self.raw_Euler_angles[index], 
                              addr = 0x1A+index )
        self.EUL_Heading = (self.raw_Euler_angles[0] << 8) | self.raw_Euler_angles[1]
        self.EUL_Roll = (self.raw_Euler_angles[2] << 8) | self.raw_Euler_angles[3]
        self.EUL_Pitch = (self.raw_Euler_angles[4] << 8) | self.raw_Euler_angles[5]

        self.Euler_angles = [self.EUL_Heading, self.EUL_Roll, self.EUL_Pitch]
        self.Euler_angles = [int(hex_val & 0x7FFF) - int(hex_val & 0x8000) for hex_val in self.Euler_angles]
        self.Euler_angles = [val/16 for val in self.Euler_angles]
        
        return self.Euler_angles
    
    def read_angular_velocities(self):
        self.raw_angular_velocities = bytearray(6)
        for index in range(6):
            self.i2c.mem_read( self.raw_angular_velocities[index],
                              addr = 0x14 + index)
            
        self.gyr_data = [-1]*3
        self.gyr_data[0] = (self.raw_angular_velocities[0] << 8) | self.raw_angular_velocities[1]
        self.gyr_data[1] = (self.raw_angular_velocities[2] << 8) | self.raw_angular_velocities[3]
        self.gyr_data[2] = (self.raw_angular_velocities[4] << 8) | self.raw_angular_velocities[5]

        self.gyr_data = [value/16 for value in self.gyr_data] #Dps

        return self.gyr_data

    def configure_coordinate_system(self, axis_placement: str):
        self.axis_placement = axis_placement
        if self.axis_placement in self.axis_placement_mapping:
            self.axis_map_config, self.axis_map_sign = self.axis_placement_mapping[self.axis_placement]
        else:
            # Handle the case where the provided axis_placement is not in the mapping
            print(f"Invalid axis_placement: {self.axis_placement}")
            # You might want to raise an exception or handle it in some other way

        # Send info
        self.i2c.mem_write(self.axis_map_config,
                           addr = self.axis_map_config_addr,
                           timeout = self.timeout)
        self.i2c.mem_write(self.axis_map_sign,
                           addr = self.axis_map_sign_addr,
                           timeout = self.timeout)
        
    def separate_16bit_hex(hex_number):
        # Ensure the input is a valid hexadecimal string
        if not isinstance(hex_number, str) or not hex_number.startswith("0x"):
            raise ValueError("Input must be a 16-bit hexadecimal string starting with '0x'")

        # Remove '0x' prefix and convert to an integer
        num = int(hex_number, 16)

        # Separate into two 8-bit hex numbers
        high_byte = (num >> 8) & 0xFF
        low_byte = num & 0xFF

        return high_byte,low_byte