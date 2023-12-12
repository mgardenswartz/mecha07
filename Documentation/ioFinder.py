"""!
@file iolist.py
This file checks which Nucleo pin a given CPU pin is located at.

@author Max Gardenswartz
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

while True:    
    # For a given string representing a pin, return the index and which list above it's found in.
    try:
        myPin = input('Enter a pin to find here: ').upper()
    except KeyboardInterrupt:
        print("\r\nYou have exited the program.")
        break

    # Add P in front if the user did not.
    if not(myPin[0] in 'pP'):
        myPin = 'P'+myPin

    # IO
    CN7 = ['PC10', 'PC11', 'PC12', 'PD2', 'VDD', 'E5V', 'BOOT0', 'GND', '', '', '', 'IOREF', 'PA13', 'RESET', 'PA14', '+3.3V', 'PA15', '+5V', 'GND', 'GND', 'PB7', 'GND', 'PC13', 'VIN', 'PC14', '', 'PC15', 'PA0', 'PH0', 'PA1', 'PH1', 'PA4', 'VBAT', 'PB0', 'PC2', 'PC1', 'PC3', 'PC0']
    CN10 = ['PC9', 'PC8', 'PB8', 'PC6', 'PB9', 'PC5', 'AVDD', 'U5V', 'GND', '', 'PA5', 'PA12', 'PA6', 'PA11', 'PA7', 'PB12', 'PB6', 'PB11', 'PC7', 'GND', 'PA9', 'PB2', 'PA8', 'PB1', 'PB10', 'PB15', 'PB4', 'PB14', 'PB5', 'PB13', 'PB3', 'AGND', 'PA10', 'PC4', 'PA2', '', 'PA3', '']

    # Find the index of the pin in CN7
    index_found_in = None
    found_in_list = None
    for i, item in enumerate(CN7):
        if item == myPin:
            index_found_in = i+1
            found_in_list = 'CN7'
            break

    # Find the index of the pin in CN10
    for i, item in enumerate(CN10):
        if item == myPin:
            index_found_in = i+1
            found_in_list = 'CN10'
            break

    # Determine which list the pin is found in

    # Print the results
    if index_found_in is not None:
        print(f"{myPin} is found at Pin {index_found_in} in {found_in_list}\r\n")
    else:
        print(f"{myPin} not found in either list.\r\n")