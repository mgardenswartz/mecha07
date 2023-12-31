from SMBus import SMBus
import time
import struct
from micropython import const

class BNO055:
    BNO055_ADDRESS_A = const(0x28)
    BNO055_ADDRESS_B = const(0x29)
    BNO055_ID = const(0xA0)

    # Power mode settings
    POWER_MODE_NORMAL = const(0X00)
    POWER_MODE_LOWPOWER = const(0X01)
    POWER_MODE_SUSPEND = const(0X02)

    # Operation mode settings
    OPERATION_MODE_CONFIG = const(0X00)
    OPERATION_MODE_ACCONLY = const(0X01)
    OPERATION_MODE_MAGONLY = const(0X02)
    OPERATION_MODE_GYRONLY = const(0X03)
    OPERATION_MODE_ACCMAG = const(0X04)
    OPERATION_MODE_ACCGYRO = const(0X05)
    OPERATION_MODE_MAGGYRO = const(0X06)
    OPERATION_MODE_AMG = const(0X07)
    OPERATION_MODE_IMUPLUS = const(0X08)
    OPERATION_MODE_COMPASS = const(0X09)
    OPERATION_MODE_M4G = const(0X0A)
    OPERATION_MODE_NDOF_FMC_OFF = const(0X0B)
    OPERATION_MODE_NDOF = const(0X0C)

    # Output vector type
    VECTOR_ACCELEROMETER = const(0x08)
    VECTOR_MAGNETOMETER = const(0x0E)
    VECTOR_GYROSCOPE = const(0x14)
    VECTOR_EULER = const(0x1A)
    VECTOR_LINEARACCEL = const(0x28)
    VECTOR_GRAVITY = const(0x2E)

    # REGISTER DEFINITION START
    BNO055_PAGE_ID_ADDR = const(0X07)

    BNO055_CHIP_ID_ADDR = const(0x00)
    BNO055_ACCEL_REV_ID_ADDR = const(0x01)
    BNO055_MAG_REV_ID_ADDR = const(0x02)
    BNO055_GYRO_REV_ID_ADDR = const(0x03)
    BNO055_SW_REV_ID_LSB_ADDR = const(0x04)
    BNO055_SW_REV_ID_MSB_ADDR = const(0x05)
    BNO055_BL_REV_ID_ADDR = const(0X06)

    # Accel data register
    BNO055_ACCEL_DATA_X_LSB_ADDR = const(0X08)
    BNO055_ACCEL_DATA_X_MSB_ADDR = const(0X09)
    BNO055_ACCEL_DATA_Y_LSB_ADDR = const(0X0A)
    BNO055_ACCEL_DATA_Y_MSB_ADDR = const(0X0B)
    BNO055_ACCEL_DATA_Z_LSB_ADDR = const(0X0C)
    BNO055_ACCEL_DATA_Z_MSB_ADDR = const(0X0D)

    # Mag data register
    BNO055_MAG_DATA_X_LSB_ADDR = const(0X0E)
    BNO055_MAG_DATA_X_MSB_ADDR = const(0X0F)
    BNO055_MAG_DATA_Y_LSB_ADDR = const(0X10)
    BNO055_MAG_DATA_Y_MSB_ADDR = const(0X11)
    BNO055_MAG_DATA_Z_LSB_ADDR = const(0X12)
    BNO055_MAG_DATA_Z_MSB_ADDR = const(0X13)

    # Gyro data registers
    BNO055_GYRO_DATA_X_LSB_ADDR = const(0X14)
    BNO055_GYRO_DATA_X_MSB_ADDR = const(0X15)
    BNO055_GYRO_DATA_Y_LSB_ADDR = const(0X16)
    BNO055_GYRO_DATA_Y_MSB_ADDR = const(0X17)
    BNO055_GYRO_DATA_Z_LSB_ADDR = const(0X18)
    BNO055_GYRO_DATA_Z_MSB_ADDR = const(0X19)
    
    # Euler data registers
    BNO055_EULER_H_LSB_ADDR = const(0X1A)
    BNO055_EULER_H_MSB_ADDR = const(0X1B)
    BNO055_EULER_R_LSB_ADDR = const(0X1C)
    BNO055_EULER_R_MSB_ADDR = const(0X1D)
    BNO055_EULER_P_LSB_ADDR = const(0X1E)
    BNO055_EULER_P_MSB_ADDR = const(0X1F)

    # Quaternion data registers
    BNO055_QUATERNION_DATA_W_LSB_ADDR = const(0X20)
    BNO055_QUATERNION_DATA_W_MSB_ADDR = const(0X21)
    BNO055_QUATERNION_DATA_X_LSB_ADDR = const(0X22)
    BNO055_QUATERNION_DATA_X_MSB_ADDR = const(0X23)
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = const(0X24)
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = const(0X25)
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = const(0X26)
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = const(0X27)

    # Linear acceleration data registers
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = const(0X28)
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = const(0X29)
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = const(0X2A)
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = const(0X2B)
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = const(0X2C)
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = const(0X2D)

    # Gravity data registers
    BNO055_GRAVITY_DATA_X_LSB_ADDR = const(0X2E)
    BNO055_GRAVITY_DATA_X_MSB_ADDR = const(0X2F)
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = const(0X30)
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = const(0X31)
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = const(0X32)
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = const(0X33)

    # Temperature data register
    BNO055_TEMP_ADDR = const(0X34)

    # Status registers
    BNO055_CALIB_STAT_ADDR = const(0X35)
    BNO055_SELFTEST_RESULT_ADDR = const(0X36)
    BNO055_INTR_STAT_ADDR = const(0X37)

    BNO055_SYS_CLK_STAT_ADDR = const(0X38)
    BNO055_SYS_STAT_ADDR = const(0X39)
    BNO055_SYS_ERR_ADDR = const(0X3A)

    # Unit selection register
    BNO055_UNIT_SEL_ADDR = const(0X3B)
    BNO055_DATA_SELECT_ADDR = const(0X3C)

    # Mode registers
    BNO055_OPR_MODE_ADDR = const(0X3D)
    BNO055_PWR_MODE_ADDR = const(0X3E)

    BNO055_SYS_TRIGGER_ADDR = const(0X3F)
    BNO055_TEMP_SOURCE_ADDR = const(0X40)

    # Axis remap registers
    BNO055_AXIS_MAP_CONFIG_ADDR = const(0X41)
    BNO055_AXIS_MAP_SIGN_ADDR = const(0X42)

    # Axis remap values
    AXIS_REMAP_X = const(0x00)
    AXIS_REMAP_Y = const(0x01)
    AXIS_REMAP_Z = const(0x02)
    AXIS_REMAP_POSITIVE = const(0x00)
    AXIS_REMAP_NEGATIVE = const(0x01)

    # SIC registers
    BNO055_SIC_MATRIX_0_LSB_ADDR = const(0X43)
    BNO055_SIC_MATRIX_0_MSB_ADDR = const(0X44)
    BNO055_SIC_MATRIX_1_LSB_ADDR = const(0X45)
    BNO055_SIC_MATRIX_1_MSB_ADDR = const(0X46)
    BNO055_SIC_MATRIX_2_LSB_ADDR = const(0X47)
    BNO055_SIC_MATRIX_2_MSB_ADDR = const(0X48)
    BNO055_SIC_MATRIX_3_LSB_ADDR = const(0X49)
    BNO055_SIC_MATRIX_3_MSB_ADDR = const(0X4A)
    BNO055_SIC_MATRIX_4_LSB_ADDR = const(0X4B)
    BNO055_SIC_MATRIX_4_MSB_ADDR = const(0X4C)
    BNO055_SIC_MATRIX_5_LSB_ADDR = const(0X4D)
    BNO055_SIC_MATRIX_5_MSB_ADDR = const(0X4E)
    BNO055_SIC_MATRIX_6_LSB_ADDR = const(0X4F)
    BNO055_SIC_MATRIX_6_MSB_ADDR = const(0X50)
    BNO055_SIC_MATRIX_7_LSB_ADDR = const(0X51)
    BNO055_SIC_MATRIX_7_MSB_ADDR = const(0X52)
    BNO055_SIC_MATRIX_8_LSB_ADDR = const(0X53)
    BNO055_SIC_MATRIX_8_MSB_ADDR = const(0X54)
    
    # Accelerometer Offset registers
    ACCEL_OFFSET_X_LSB_ADDR = const(0X55)
    ACCEL_OFFSET_X_MSB_ADDR = const(0X56)
    ACCEL_OFFSET_Y_LSB_ADDR = const(0X57)
    ACCEL_OFFSET_Y_MSB_ADDR = const(0X58)
    ACCEL_OFFSET_Z_LSB_ADDR = const(0X59)
    ACCEL_OFFSET_Z_MSB_ADDR = const(0X5A)

    # Magnetometer Offset registers
    MAG_OFFSET_X_LSB_ADDR = const(0X5B)
    MAG_OFFSET_X_MSB_ADDR = const(0X5C)
    MAG_OFFSET_Y_LSB_ADDR = const(0X5D)
    MAG_OFFSET_Y_MSB_ADDR = const(0X5E)
    MAG_OFFSET_Z_LSB_ADDR = const(0X5F)
    MAG_OFFSET_Z_MSB_ADDR = const(0X60)

    # Gyroscope Offset registers)
    GYRO_OFFSET_X_LSB_ADDR = const(0X61)
    GYRO_OFFSET_X_MSB_ADDR = const(0X62)
    GYRO_OFFSET_Y_LSB_ADDR = const(0X63)
    GYRO_OFFSET_Y_MSB_ADDR = const(0X64)
    GYRO_OFFSET_Z_LSB_ADDR = const(0X65)
    GYRO_OFFSET_Z_MSB_ADDR = const(0X66)

    # Radius registers
    ACCEL_RADIUS_LSB_ADDR = const(0X67)
    ACCEL_RADIUS_MSB_ADDR = const(0X68)
    MAG_RADIUS_LSB_ADDR = const(0X69)
    MAG_RADIUS_MSB_ADDR = const(0X6A)

    # REGISTER DEFINITION END


    def __init__(self, sensorId=-1, address=0x28):
        self._sensorId = sensorId
        self._address = address
        self._mode = BNO055.OPERATION_MODE_NDOF


    def begin(self, mode=None):
        if mode is None: mode = BNO055.OPERATION_MODE_NDOF
        # Open I2C bus
        self._bus = SMBus(1)

        # Make sure we have the right device
        if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(1)    # Wait for the device to boot up
            if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
                return False

        # Switch to config mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)

        # Trigger a reset and wait for the device to boot up again
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])
        time.sleep(1)
        while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(0.01)
        time.sleep(0.05)

        # Set to normal power mode
        self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])
        time.sleep(0.01)

        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])
        time.sleep(0.01)

        # Set the requested mode
        self.setMode(mode)
        time.sleep(0.02)

        return True

    def setMode(self, mode):
        self._mode = mode
        self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])
        time.sleep(0.03)

    def setExternalCrystalUse(self, useExternalCrystal = True):
        prevMode = self._mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        time.sleep(0.025)
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])
        time.sleep(0.01)
        self.setMode(prevMode)
        time.sleep(0.02)

    def getSystemStatus(self, run_self_test=False):
        """Return a tuple with status information.  Three values will be returned:
          - System status register value with the following meaning:
              0 = Idle
              1 = System Error
              2 = Initializing Peripherals
              3 = System Initialization
              4 = Executing Self-Test
              5 = Sensor fusion algorithm running
              6 = System running without fusion algorithms
          - Self test result register value with the following meaning:
              Bit value: 1 = test passed, 0 = test failed
              Bit 0 = Accelerometer self test
              Bit 1 = Magnetometer self test
              Bit 2 = Gyroscope self test
              Bit 3 = MCU self test
              Value of 0x0F = all good!
          - System error register value with the following meaning:
              0 = No error
              1 = Peripheral initialization error
              2 = System initialization error
              3 = Self test result failed
              4 = Register map value out of range
              5 = Register map address out of range
              6 = Register map write error
              7 = BNO low power mode not available for selected operation mode
              8 = Accelerometer power mode not available
              9 = Fusion algorithm configuration error
             10 = Sensor configuration error

        If run_self_test is passed in as False then no self test is performed and
        None will be returned for the self test result.  Note that running a
        self test requires going into config mode which will stop the fusion
        engine from running.
        """
        
        self_test = None
        if run_self_test:
            # store previous mode
            prevMode = self._mode
            # Switch to configuration mode if running self test.
            self.setMode(BNO055.OPERATION_MODE_CONFIG)
            # Perform a self test.
            sys_trigger = self.readBytes(BNO055.BNO055_SYS_TRIGGER_ADDR)[0]
            self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1)
            # Wait for self test to finish.
            time.sleep(1.0)
            # Read test result.
            self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)
            # Go back to operation mode.
            self.setMode(prevMode)

        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        (sys_stat, sys_err) = self.readBytes(BNO055.BNO055_SYS_STAT_ADDR, 2)
        self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)[0]
        return (sys_stat, self_test, sys_err)

    def getRevInfo(self):
        (accel_rev, mag_rev, gyro_rev) = self.readBytes(BNO055.BNO055_ACCEL_REV_ID_ADDR, 3)
        sw_rev = self.readBytes(BNO055.BNO055_SW_REV_ID_LSB_ADDR, 2)
        sw_rev = sw_rev[0] | sw_rev[1] << 8
        bl_rev = self.readBytes(BNO055.BNO055_BL_REV_ID_ADDR)[0]
        return (accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev)

    def getCalibrationStatus(self):
        """Read the calibration status of the sensors and return a 4 tuple with
        calibration status as follows:
          - System, 3=fully calibrated, 0=not calibrated
          - Gyroscope, 3=fully calibrated, 0=not calibrated
          - Accelerometer, 3=fully calibrated, 0=not calibrated
          - Magnetometer, 3=fully calibrated, 0=not calibrated
        """
        calData = self.readBytes(BNO055.BNO055_CALIB_STAT_ADDR)[0]
        return (calData >> 6 & 0x03, calData >> 4 & 0x03, calData >> 2 & 0x03, calData & 0x03)
    
    def getCalibrationData(self):
        """Return the sensor's calibration data and return it as an array of
        22 bytes. Can be saved and then reloaded with the setCalibration function
        to quickly calibrate from a previously calculated set of calibration data.
        """
        prevMode = self._mode
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        # Wait for mode switch       
        time.sleep(0.01)
        cal_data = list(self.readBytes(BNO055.ACCEL_OFFSET_X_LSB_ADDR, 22))
        # Go back to normal operation mode.
        self.setMode(prevMode)
        return cal_data
    
    def setCalibrationData(self, data: list[int]=None):
        """Set the sensor's calibration data using a list of 22 bytes that
        represent the sensor offsets and calibration data.  This data should be
        a value that was previously retrieved with get_calibration (and then
        perhaps persisted to disk or other location until needed again).
        """
        # Check that 22 bytes were passed in with calibration data.
        if data is None or len(data) != 22:
            raise ValueError('Expected a list of 22 bytes for calibration data.')
        prevMode = self._mode
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        # Set the 22 bytes of calibration data.
        self.writeBytes(BNO055.ACCEL_OFFSET_X_LSB_ADDR, data)
        # Go back to normal operation mode.
        self.setMode(prevMode)

    def getAxisRemap(self):
        """Return a tuple with the axis remap register values.  This will return
        6 values with the following meaning:
          - X axis remap (a value of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z.
                          which indicates that the physical X axis of the chip
                          is remapped to a different axis)
          - Y axis remap (see above)
          - Z axis remap (see above)
          - X axis sign (a value of AXIS_REMAP_POSITIVE or AXIS_REMAP_NEGATIVE
                         which indicates if the X axis values should be positive/
                         normal or negative/inverted.  The default is positive.)
          - Y axis sign (see above)
          - Z axis sign (see above)

        Note that by default the axis orientation of the BNO chip looks like
        the following (taken from section 3.4, page 24 of the datasheet).  Notice
        the dot in the corner that corresponds to the dot on the BNO chip:

                           | Z axis
                           |
                           |   / X axis
                       ____|__/____
          Y axis     / *   | /    /|
          _________ /______|/    //
                   /___________ //
                  |____________|/
        """
        # Get the axis remap register value.
        map_config = self.readBytes(BNO055.BNO055_AXIS_MAP_CONFIG_ADDR)[0]
        z = (map_config >> 4) & 0x03
        y = (map_config >> 2) & 0x03
        x = map_config & 0x03
        # Get the axis remap sign register value.
        sign_config = self.readBytes(BNO055.BNO055_AXIS_MAP_SIGN_ADDR)[0]
        x_sign = (sign_config >> 2) & 0x01
        y_sign = (sign_config >> 1) & 0x01
        z_sign = sign_config & 0x01
        # Return the results as a tuple of all 3 values.
        return (x, y, z, x_sign, y_sign, z_sign)

    def set_axis_remap(self, x, y, z, x_sign=None, y_sign=None, z_sign=None):
        if x is None: mode = BNO055.AXIS_REMAP_POSITIVE
        if y is None: mode = BNO055.AXIS_REMAP_POSITIVE
        if z is None: mode = BNO055.AXIS_REMAP_POSITIVE

        """Set axis remap for each axis.  The x, y, z parameter values should
        be set to one of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z and will
        change the BNO's axis to represent another axis.  Note that two axises
        cannot be mapped to the same axis, so the x, y, z params should be a
        unique combination of AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z values.

        The x_sign, y_sign, z_sign values represent if the axis should be positive
        or negative (inverted).

        See the get_axis_remap documentation for information on the orientation
        of the axises on the chip, and consult section 3.4 of the datasheet.
        """
        # Switch to configuration mode.
        prevMode = self._mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        # Set the axis remap register value.
        map_config = 0x00
        map_config |= (z & 0x03) << 4
        map_config |= (y & 0x03) << 2
        map_config |= x & 0x03
        self.writeBytes(BNO055.BNO055_AXIS_MAP_CONFIG_ADDR, [map_config])
        # Set the axis remap sign register value.
        sign_config = 0x00
        sign_config |= (x_sign & 0x01) << 2
        sign_config |= (y_sign & 0x01) << 1
        sign_config |= z_sign & 0x01
        self.writeBytes(BNO055.BNO055_AXIS_MAP_SIGN_ADDR, [sign_config])
        # Go back to normal operation mode.
        self.setMode(prevMode)



    def getTemp(self):
        return self.readBytes(BNO055.BNO055_TEMP_ADDR)[0]

    def getVector(self, vectorType):
        buf = self.readBytes(vectorType, 6)
        xyz = struct.unpack('hhh', struct.pack('BBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]))
        if vectorType == BNO055.VECTOR_MAGNETOMETER:    scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GYROSCOPE:    scalingFactor = 900.0
        elif vectorType == BNO055.VECTOR_EULER:         scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GRAVITY:    scalingFactor = 100.0
        else:                                            scalingFactor = 1.0
        return tuple([i/scalingFactor for i in xyz])

    def getQuat(self):
        buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
        wxyz = struct.unpack('hhhh', struct.pack('BBBBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]))
        return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

    def readBytes(self, register, numBytes=1):
        return self._bus.read_i2c_block_data(self._address, register, numBytes)

    def writeBytes(self, register, byteVals):
        return self._bus.write_i2c_block_data(self._address, register, byteVals)


if __name__ == '__main__':
    bno = BNO055()
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    bno.setExternalCrystalUse(True)
    while True:
        print(bno.getVector(BNO055.VECTOR_EULER))
        time.sleep(0.01)

