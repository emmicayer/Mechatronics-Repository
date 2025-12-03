import time
from matplotlib import pyplot as plt
import subprocess
import math

filename = "PuTTYdata.csv"
# To change PuTTY settings:
    # session
    # logging
    # browse
    # PuTTYdata.csv (file in Lab0x02 folder)

# Create empty lists and strings for x- and y- values and headers
x_vals_time_L = []                                 # Empty list for x values
x_vals_time_R = []                                 # Empty list for x values
y_vals_pos_L = []                                 # Empty list for y values
y_vals_vel_L = []                                 # Empty list for y values
y_vals_pos_R = []                                 # Empty list for y values
y_vals_vel_R = []                                 # Empty list for y values
# x_label = ""                                # Empty string for x header label
# y_label = ""                                # Empty string for y header label

with open(filename, "r") as file:           # Open file as read only
    lines = file.readlines()                # Read line of file in string

# Process header (line 1)
# if len(lines) > 0:                          # Check that file isn't empty
#     header = lines[0].strip().split(",")    # Rid of whitespace, split line
#     if len(header) >= 2:                    # Check (at least) 2 entries
#         x_label = header[0].strip()         # Makes 1st entry x_label
#         y_label = header[1].strip()         # Makes 2nd entry y_label

# Process data rows
for line_num, line in enumerate(lines[1:], start=2):
                                            # Number lines start at 2
                                            # Loop through line 2 and on
    # Remove comments
    if "#" in line:                         # Check if comment in a line
        line = line.split("#", 1)[0]
                                            # Cut off comment and rewrite

    # Split into columns
    values = [value.strip() for value in line.split(",")]
                                            # Split each line at the comma
                                            # values is a list of strings

    if len(values) < 2:                     # Checks if less than 2 values
        print(f"Line {line_num} rejected: fewer than 2 columns")
        continue                            # Print a rejection, next row

    try:                                    # Attempt to ...
        x_time_L = 0.001*float(values[0])                # convert value to float, x, 0 column in CSV file
        y_pos_L = 2*math.pi*(1/(12*119.76))*float(values[1])                # convert value to float, y
        y_vel_L = 2*1000*math.pi*(1/(12*119.76))*float(values[2])
        x_time_R = 0.001*float(values[3])                # convert value to float, x, 0 column in CSV file
        y_pos_R = 2*math.pi*(1/(12*119.76))*float(values[4])                # convert value to float, y
        y_vel_R = 2*1000*math.pi*(1/(12*119.76))*float(values[5])
        x_vals_time_L.append(x_time_L)                    # Add "x" to end of x_vals list
        y_vals_pos_L.append(y_pos_L)                    # Add "y" to end of y_vals list
        y_vals_vel_L.append(y_vel_L)
        x_vals_time_R.append(x_time_R)                    # Add "x" to end of x_vals list
        y_vals_pos_R.append(y_pos_R)                    # Add "y" to end of y_vals list
        y_vals_vel_R.append(y_vel_R)

    except ValueError:                      # If something goes wrong
        print(f"Line {line_num} rejected: non-numeric data")
        continue                            # Print rejection, next row

# Plot 1
plt.plot(x_vals_time_L, y_vals_pos_L, marker="o")  # Plot x_vals vs y_vals
plt.xlabel("Time [ms]")                         # Use "x_label" as the x label
plt.ylabel("Position [rad]")                         # Use "y_label"  as the y label
plt.title("Left Motor Position vs. Time")                  # Title the plot "CSV Data Plot"
plt.grid(True)                              # Turn on the plot grid
plt.savefig("plot1_left_pos.png")
plt.clf()

# Plot 2
plt.plot(x_vals_time_L, y_vals_vel_L, marker="o")  # Plot x_vals vs y_vals
plt.xlabel("Time [ms]")                         # Use "x_label" as the x label
plt.ylabel("Velocity [rad/ms]")                         # Use "y_label"  as the y label
plt.title("Left Motor Velocity vs. Time")                  # Title the plot "CSV Data Plot"
plt.grid(True)                              # Turn on the plot grid
plt.savefig("plot2_left_vel.png")
plt.clf()

# Plot 3
plt.plot(x_vals_time_R, y_vals_pos_R, marker="o")  # Plot x_vals vs y_vals
plt.xlabel("Time [ms]")                         # Use "x_label" as the x label
plt.ylabel("Position [rad]")                         # Use "y_label"  as the y label
plt.title("Right Motor Position vs. Time")                  # Title the plot "CSV Data Plot"
plt.grid(True)                              # Turn on the plot grid
plt.savefig("plot3_right_pos.png")
plt.clf()

# Plot 4
plt.plot(x_vals_time_R, y_vals_vel_R, marker="o")   # Plot x_vals vs y_vals
plt.xlabel("Time [ms]")                             # Use "x_label" as the x label
plt.ylabel("Velocity [rad/ms]")                         # Use "y_label"  as the y label
plt.title("Right Motor Velocity vs. Time")                  # Title the plot "CSV Data Plot"
plt.grid(True)                              # Turn on the plot grid
plt.savefig("plot4_right_vel.png")
plt.clf()