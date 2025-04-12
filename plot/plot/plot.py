# format: t,y1,z1,,y2,z2,,y3,z3,,y4,z4
file = open("../../cmake-build-debug/output.csv", "r")
lines = file.readlines()
file.close()
lines = lines[1:]  # Skip the first line
lines = [line.split(",") for line in lines]
lines_z = [[float(line[0]), float(line[2]), float(line[4]), float(line[6]), float(line[8])] for line in lines]
lines = [[float(line[0]), float(line[1]), float(line[3]), float(line[5]), float(line[7])] for line in lines]
# Extract values
t = [line[0] for line in lines]
y1 = [line[1] for line in lines]
y2 = [line[2] for line in lines]
y3 = [line[3] for line in lines]
y4 = [line[4] for line in lines]

import matplotlib.pyplot as plt
import numpy as np
fig, ax = plt.subplots()
ax.plot(t, y1, label='y1', color='blue')
ax.plot(t, y2, label='y2', color='orange')
ax.plot(t, y3, label='y3', color='green')
ax.plot(t, y4, label='y4', color='red')
ax.set_title('y')
ax.set_xlabel('t')
ax.set_ylabel('yi')
ax.legend()
z1 = [line[1] for line in lines_z]
z2 = [line[2] for line in lines_z]
z3 = [line[3] for line in lines_z]
z4 = [line[4] for line in lines_z]

fig_z, ax_z = plt.subplots()

ax_z.plot(t, z1, label='z1', color='blue')
ax_z.plot(t, z2, label='z2', color='orange')
ax_z.plot(t, z3, label='z3', color='green')
ax_z.plot(t, z4, label='z4', color='red')
ax_z.set_title('z')
ax_z.set_xlabel('t')
ax_z.set_ylabel('zi')
ax_z.legend()
plt.show()

# save z plot
fig_z.savefig("z.png")