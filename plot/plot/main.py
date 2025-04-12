
# format time,y,z,u (from the second line)
file = open("../../cmake-build-debug/agent1_output.csv", "r")
lines = file.readlines()
file.close()
lines = lines[1:]  # Skip the first line
lines = [line.split(",") for line in lines]
lines = [[float(line[0]), float(line[1]), float(line[2]), float(line[3])] for line in lines]

# plot y z
import matplotlib.pyplot as plt
import numpy as np

# Extract the t y, z, and u values
t = [line[0] for line in lines]
y = [line[1] for line in lines]
z = [line[2] for line in lines]
u = [line[3] for line in lines]

# Create a figure and axis
fig, ax = plt.subplots()
# Plot the y and z values
ax.plot(t, y, label='y', color='blue')
ax.plot(t, z, label='z', color='orange')
# Set the x and y axis labels
ax.set_xlabel('Time (s)')
# ax.set_ylabel('Position (m)')
# Set the title of the plot
# ax.set_title('Position vs Time')
# Add a legend
ax.legend()
# Show the plot
plt.show()