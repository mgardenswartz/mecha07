firstColors =[ "Black", "White" ] 
secondColors =[ "White", "Black" ] 
debug = True

# Control Logic
# Create a dictionary to map colors to values
color_mapping = {"White": 0, "Black": 1}

# Replace colors with values and convert to tuple
firstColorsTuple = tuple(color_mapping[color] for color in firstColors)
secondColorsTuple= tuple(color_mapping[color] for color in secondColors)
colorsTuple = (firstColorsTuple, secondColorsTuple)

# Printing for debugging.
if debug:
    print(f"Tuple reads: {colorsTuple}")

manuevers = {
    ((0, 0), 
        (0, 0)): "straight",

    ((1, 1),
        (1, 1)): "straight",

    ((0, 0),
        (1, 1)): "straight",

    ((1, 1),
        (0, 0)): "straight",

    ((0, 1),
        (1, 0)): "slight_right",

    ((0, 1),
        (0, 0)): "slight_right",

    ((0, 1),
        (0, 1)): "slight_right",

    ((1, 1),
        (0, 1)): "slight_right",

    ((0, 1),
        (1, 0)): "sharp_right",

    ((0, 0),
        (0, 1)): "sharp_right",

    ((1, 0),
        (1, 0)): "slight_left",

    ((1, 0),
        (0, 0)): "slight_left",

    ((1, 0),
        (0, 1)): "slight_left",

    ((1, 1),
        (1, 0)): "slight_left",

    ((1, 0),
        (0, 1)): "sharp_left",

    ((0, 0),
        (1, 0)): "sharp_left"}

# What should the robot do?
whatToDo = manuevers[colorsTuple]

if whatToDo == "straight":
    print("Performing straight maneuver.")

elif whatToDo == "slight_right":
    print("Performing slight right turn.")

elif whatToDo == "sharp_right":
    print("Performing sharp right turn.")

elif whatToDo == "slight_left":
    print("Performing slight left turn.")

elif whatToDo == "sharp_left":
    print("Performing sharp left turn.")

else:
    print("Unknown action. Something went wrong.")
    raise Exception("Something went wrong.")