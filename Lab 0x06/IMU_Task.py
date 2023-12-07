class IMU_Task:
    def __init__(self,
                 IMU,
                 print_flag: bool):
        
        # Internal Variables
        self.state = 0
        self.EulerAngles = 3*[69]
        self.angularVelocities = 3*[69]

        # Attributes
        self.IMU = IMU
        self.print_flag = print_flag

    def run(self):
        while True:
            # Read the latest data from the IMU.
            self.EulerAngles = self.IMU.read_Euler_angles()
            self.angularVelocities = self.IMU.read_angular_velocities()     
                
            # Print the data from the IMU and the calibration status.    
            if self.print_flag == True:
                print(f"Heading: {self.EulerAngles[0]} deg, Roll: {self.EulerAngles[1]} deg, Yaw: {self.EulerAngles[2]} deg")
                print(f"W_x = {self.angularVelocities[0]} rad/s, W_y = {self.angularVelocities[1]} rad/s, W_z = {self.angularVelocities[2]} rad/s")
                print(self.IMU.retrieve_calibration_status())

            yield self.state

                