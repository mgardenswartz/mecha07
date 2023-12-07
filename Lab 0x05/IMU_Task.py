class IMU_Task:
    def __init__(self,IMU):
        
        # Internal Variables
        self.state = 0

        # Attributes
        self.IMU = IMU
        self.Euler_angles = 3*[69]
        self.print_flag = False

    def run(self):
        while True:
            if self.state == 0:
                # Initialization
                #self.IMU.begin()
                # cal_coeffs = [203, 255, 238, 255, 250, 255, 250, 246, 123, 252, 141, 5, 3, 0, 254, 255, 0, 0, 232, 3, 100, 32]
                # self.IMU.write_coefficients(cal_coeffs)
                print("IMU initialized")

                # State Transition
                self.state = 1
            elif self.state == 1:
                #print(f"IMU Task is in state {self.state}.")
                # Begin reading IMU data? Calibraiton? I'm not sure.
                self.Euler_angles = self.IMU.read_Euler_angles()
                self.omegas = self.IMU.read_angular_velocities()     
                
                if self.print_flag == True:
                    print(f"Heading: {self.Euler_angles[0]} deg, Roll: {self.Euler_angles[1]} deg, Yaw: {self.Euler_angles[2]} deg")
                    print(f"W_x = {self.omegas[0]} rad/s, W_y = {self.omegas[1]} rad/s, W_z = {self.omegas[2]} rad/s")
                    #print(self.IMU.retrieve_calibration_status())
                    #print(self.IMU.retrieve_coefficients())

            else:
                # Invalid state
                raise ValueError(f"Invalid State: {self.state}\r")
            
            yield self.state

                