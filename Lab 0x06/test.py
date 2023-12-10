original_list = ["Black", "White", "Black", "White"]

# Replace "Black" with 1 and "White" with 0
modified_list = [1 if color == "Black" else 0 for color in original_list]

print(modified_list)
