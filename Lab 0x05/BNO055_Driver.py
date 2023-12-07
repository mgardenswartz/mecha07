import pyb
from pyb import I2C
import time
import struct  # Add this import statement

class BNO055Driver:
    def __init__(self, baudrate: int, bus: int):
        # Initialization
        self.baudrate = baudrate
        self.bus = bus
        self.i2c = pyb.I2C(self.bus, I2C.MASTER, baudrate=self.baudrate)

        # Constants
        self.timeout = 10000  # ms
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

    def begin(self):
        # Set to config mode
        self.set_mode("CONFIG")

        # Trigger a reset
        self.i2c.mem_write(0x20, addr=0x28, memaddr=0x3F)

        # wait
        time.sleep(1)

        # wait for device to be done booting up
        # checks until device ID is correct
        while self.i2c.mem_read(1, addr=0x28, memaddr=0x00)[0] != 0xA0:
            time.sleep(0.01)
        time.sleep(0.05)

        # set to normal power mode
        self.i2c.mem_write(0x00, addr=0x28, memaddr=0x3E)
        self.i2c.mem_write(0x00, addr=0x28, memaddr=0x07)
        self.i2c.mem_write(0x00, addr=0x28, memaddr=0x3F)
        
        # Set units m/s^2, Dps, deg, degF, Windows
        #self.i2c.mem_write(0b0001_0000, addr=0x28, memaddr=self.unit_sel_addr)
        self.set_mode("NDOF")
        self.configure_coordinate_system()
        time.sleep(0.02)

        return True

    def set_mode(self, mode: str):
        self.mode = mode

        if self.mode == "CONFIG":
            self.i2c.mem_write(0x00, addr=0x28, memaddr=0x3D, timeout=self.timeout)
        elif self.mode == "NDOF":
            self.i2c.mem_write(0x0C, addr=0x28, memaddr=0x3D, timeout=self.timeout)
        else:
            raise Exception(f"Invalid Fusion Mode Selected: {self.mode}.")
        
        time.sleep(0.02)

    def configure_coordinate_system(self, coord_sys: str = None):
        if coord_sys == None:
            x = 0x01 # remap X to Y
            y = 0x00 # remap Y to X
            z = 0x02 # leave z alone
            x_dir = 0x01 # negative
            y_dir = 0x00 # positive
            z_dir = 0x00 # positive
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
        self.set_mode("NDOF")

    def retrieve_calibration_status(self):
        calib_data = self.i2c.mem_read(1, addr=0x28, memaddr=0x35)
        #calib_value = int.from_bytes(calib_data, "little")
        calib_value = calib_data[0]

        sys_calib = (calib_value >> 6) & 0x03
        gyr_calib = (calib_value >> 4) & 0x03
        acc_calib = (calib_value >> 2) & 0x03
        mag_calib = calib_value & 0x03

        return sys_calib, gyr_calib, acc_calib, mag_calib

    #
    
    def retrieve_coefficients(self):
        self.set_mode("CONFIG")
        self.coefficients_to_read = bytearray(22)
        self.i2c.mem_read(self.coefficients_to_read, addr=0x28, memaddr=0x55)

        self.results = list(self.coefficients_to_read)

        self.set_mode("NDOF")
        return self.results
    
    def write_coefficients(self, coefficients: list):
        
        self.set_mode("CONFIG")

        coefficients_to_write = coefficients

        # Test for valid length.
        if len(coefficients_to_write) != 22:
            raise ValueError("Input does not contain 22 integers.")
        
        # Are all elements in range of a 16-bit sign integer?
        if any(int(element) < -32768 or int(element) > 32767 for element in coefficients_to_write):
           raise ValueError("At least one element is out of range for a 16-bit number.")
         
        coefficients=[int(item) for item in coefficients] 
        data_to_write=bytearray(coefficients)
        self.i2c.mem_write(data_to_write, addr=0x28, memaddr=0x55)

        self.set_mode("NDOF")


    def read_Euler_angles(self):
        raw_Euler_angles = bytearray(6)
        self.i2c.mem_read(raw_Euler_angles, addr=0x28, memaddr=0x1A)
        Euler_angles = struct.unpack('hhh', struct.pack('BBBBBB', raw_Euler_angles[0], raw_Euler_angles[1], raw_Euler_angles[2], raw_Euler_angles[3], raw_Euler_angles[4], raw_Euler_angles[5]))

        return tuple([i/16 for i in Euler_angles])

    def read_angular_velocities(self):
        raw_gyr_rate = bytearray(6)
        self.i2c.mem_read(raw_gyr_rate, addr=0x28, memaddr=0x14)
        gyr_rate = struct.unpack('hhh', struct.pack('BBBBBB', raw_gyr_rate[0], raw_gyr_rate[1], raw_gyr_rate[2], raw_gyr_rate[3], raw_gyr_rate[4], raw_gyr_rate[5]))

        return tuple([i/900 for i in gyr_rate])
    
    def update_cal_coeffs(self):
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
