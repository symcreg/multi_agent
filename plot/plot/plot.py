# format: t,y1,y2,y3,y4
file = open("../../cmake-build-debug/output.csv", "r")
lines = file.readlines()
file.close()
lines = lines[1:]  # Skip the first line
lines = [line.split(",") for line in lines]
lines = [[float(line[0]), float(line[1]), float(line[2]), float(line[3]), float(line[4])] for line in lines]
# Extract the t y, z, and u values
t = [line[0] for line in lines]
y1 = [line[1] for line in lines]
y2 = [line[2] for line in lines]
y3 = [line[3] for line in lines]
y4 = [line[4] for line in lines]
# Create a figure and axis
import matplotlib.pyplot as plt
import numpy as np
# Create a figure and axis
fig, ax = plt.subplots()
# Plot the y and z values
ax.plot(t, y1, label='y1', color='blue')
ax.plot(t, y2, label='y2', color='orange')
ax.plot(t, y3, label='y3', color='green')
ax.plot(t, y4, label='y4', color='red')
# Set the x and y axis labels
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (m)')
# Set the title of the plot
# ax.set_title('Position vs Time')
# Add a legend
ax.legend()
# Show the plot
plt.show()
