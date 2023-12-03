# Euler_angles = [0x8000, 0xFFFF, 0x00A6]

# # Convert each element to signed integer and print
# signed_integers = [int(hex_val & 0x7FFF) - int(hex_val & 0x8000) for hex_val in Euler_angles]

# print(signed_integers)


# def int_to_16bit_hex(num):
#    if num not in range(-32768,32768):
#       raise ValueError("Out of range for a 16-bit number.")

#    # Convert to 16-bit hex
#    hex_representation = hex(num & 0xFFFF)
#    return hex_representation

# # Example usage
# negative_integer = 32767
# result = int_to_16bit_hex(negative_integer)
# print(result)


# def separate_16bit_hex(hex_number):
#     # Ensure the input is a valid hexadecimal string
#     if not isinstance(hex_number, str) or not hex_number.startswith("0x"):
#         raise ValueError("Input must be a 16-bit hexadecimal string starting with '0x'")

#     # Remove '0x' prefix and convert to an integer
#     num = int(hex_number, 16)

#     # Separate into two 8-bit hex numbers
#     high_byte = (num >> 8) & 0xFF
#     low_byte = num & 0xFF

#     return f"High Byte: 0x{high_byte:02X}, Low Byte: 0x{low_byte:02X}"

# # Example usage
# hex_number_example = "0xABCD"
# result = separate_16bit_hex(hex_number_example)
# print(result)

# hex_num = 0x6A

# print(hex(hex_num - 2 * 10))