from matplotlib import pyplot
import serial
import time

def myPlot(filedata: str):
    # Create lists for position, velocity, delta, and time
    positions = []
    velocities = []
    deltas = []  # Collect delta values, but do not plot them
    times = []

    # Split into a list of lines.
    lines = filedata.split('\n')

    # Remove all whitespace and split into a list of lists.
    for line in lines:
        values = line.strip().split(',')
        # Filter out empty strings and ensure there are at least three values.
        values = [val for val in values if val]
        if len(values) >= 3:
            positions.append(float(values[2]))  # Swap position and time
            velocities.append(float(values[1]))
            deltas.append(float(values[3]))  # Collect delta values
            times.append(float(values[0]))  # Swap position and time

    # Check if any data was collected before attempting to plot.
    if velocities and times:
        # Plot velocity against time.
        times = [x for x in times]
        pyplot.plot(times, velocities, label='Velocity', color='orange')
        pyplot.xlabel("Time (ms)")
        pyplot.ylabel("Velocity (rpm)")
        pyplot.title("Step Response")
        pyplot.show()  # Display the first plot

    if positions and times:
        # Plot position against time.
        times = [x / 10 for x in times]
        pyplot.plot(times, positions, label='Position')
        pyplot.xlabel("Time")
        pyplot.ylabel("Position")
        pyplot.title("Position vs Time")
        pyplot.show()  # Display the second plot

data_string = ""
data_plotted = False  # Flag to check if data has been plotted

with serial.Serial("COM6", 115200) as ser:
    ser.flush()
    time.sleep(2)  # Add a delay to allow the serial buffer to fill
    while True:
        if ser.in_waiting:
            output = ser.readline().decode().strip()  # Remove newline character
            # Print only the relevant information (velocity and time)
            print(str(output))
            data_string += output + "\n"
            
            if "~~~" in output and not data_plotted:
                myPlot(data_string)
                data_string = ""
                data_plotted = True  # Set the flag to True after plotting
                print("Data plotted")  # Print a message to indicate data has been plotted
