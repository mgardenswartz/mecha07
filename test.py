Euler_angles = [0x8000, 0xFFFF, 0x00A6]

# Convert each element to signed integer and print
signed_integers = [int(hex_val & 0x7FFF) - int(hex_val & 0x8000) for hex_val in Euler_angles]

print(signed_integers)