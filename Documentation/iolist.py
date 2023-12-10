# IO
CN7 = [None,'PC10', 'PC11', 'PC12', 'PD2', 'VDD', 'E5V', 'BOOT0', 'GND', '', '', '', 'IOREF', 'PA13', 'RESET', 'PA14', '+3.3V', 'PA15', '+5V', 'GND', 'GND', 'PB7', 'GND', 'PC13', 'VIN', 'PC14', '', 'PC15', 'PA0', 'PH0', 'PA1', 'PH1', 'PA4', 'VBAT', 'PB0', 'PC2', 'PC1', 'PC3', 'PC0']
CN10 = [None, 'PC9', 'PC8', 'PB8', 'PC6', 'PB9', 'PC5', 'AVDD', 'U5V', 'GND', '', 'PA5', 'PA12', 'PA6', 'PA11', 'PA7', 'PB12', 'PB6', 'PB11', 'PC7', 'GND', 'PA9', 'PB2', 'PA8', 'PB1', 'PB10', 'PB15', 'PB4', 'PB14', 'PB5', 'PB13', 'PB3', 'AGND', 'PA10', 'PC4', 'PA2', '', 'PA3', '']

# For a given string representing a pin, return the index and which list above it's found in.
myPin = 'IOREF'

# Find the index of the pin in CN7
index_in_CN7 = CN7.index(myPin) if myPin in CN7 else None

# Find the index of the pin in CN10
index_in_CN10 = CN10.index(myPin) if myPin in CN10 else None

# Determine which list the pin is found in
pin_list = None
if index_in_CN7 is not None:
    pin_list = 'CN7'
elif index_in_CN10 is not None:
    pin_list = 'CN10'

# Print the results
print(f"{myPin} is found at Pin {index_in_CN7} in {pin_list if pin_list == 'CN7' else 'CN10'}")
