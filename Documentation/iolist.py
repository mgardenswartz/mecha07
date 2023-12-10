# For a given string representing a pin, return the index and which list above it's found in.
myPin = input('Enter a pin to find here: ').upper()

# IO
CN7 = [None, 'PC10', 'PC11', 'PC12', 'PD2', 'VDD', 'E5V', 'BOOT0', 'GND', '', '', '', 'IOREF', 'PA13', 'RESET', 'PA14', '+3.3V', 'PA15', '+5V', 'GND', 'GND', 'PB7', 'GND', 'PC13', 'VIN', 'PC14', '', 'PC15', 'PA0', 'PH0', 'PA1', 'PH1', 'PA4', 'VBAT', 'PB0', 'PC2', 'PC1', 'PC3', 'PC0']
CN10 = [None, 'PC9', 'PC8', 'PB8', 'PC6', 'PB9', 'PC5', 'AVDD', 'U5V', 'GND', '', 'PA5', 'PA12', 'PA6', 'PA11', 'PA7', 'PB12', 'PB6', 'PB11', 'PC7', 'GND', 'PA9', 'PB2', 'PA8', 'PB1', 'PB10', 'PB15', 'PB4', 'PB14', 'PB5', 'PB13', 'PB3', 'AGND', 'PA10', 'PC4', 'PA2', '', 'PA3', '']

# Find the index of the pin in CN7
index_found_in = None
for i, item in enumerate(CN7):
    if item == myPin:
        index_found_in = i
        found_in_list = 'CN7'
        break

# Find the index of the pin in CN10
for i, item in enumerate(CN10):
    if item == myPin:
        index_found_in = i
        found_in_list = 'CN10'
        break

# Determine which list the pin is found in

# Print the results
if index_found_in is not None:
    print(f"{myPin} is found at Pin {index_found_in} in {found_in_list}")
else:
    print(f"{myPin} not found in either list.")
