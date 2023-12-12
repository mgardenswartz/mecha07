"""!
@file IMU_Task.py
This file reads pertinent data from a BNO055 IMU and prints it, if requested.

@author Max Gardenswartz
@date   2023-Nov-17 MLG Approximate date of creation of file
@date   2023-Dec-15 MLG Latest itteration.

It is intended for educational use only, but its use is not limited thereto.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


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

                