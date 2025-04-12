import numpy as np
from numpy.linalg import lstsq

# Define system
A = np.array([[0, 1, 0],
              [-1, 0, 1],
              [2, 0, 1]])
B = np.array([[0], [1], [1]])
C = np.array([[0, 1, 0]])

# Build equation AX + BU = 0 => [A B] * [X; U] = 0
M1 = np.hstack([A, B])  # (3x4)
b1 = np.zeros((3,))

# C X = 1
M2 = np.hstack([C, np.zeros((1,1))])  # (1x4)
b2 = np.array([1.0])

# Stack the full system
M = np.vstack([M1, M2])
b = np.hstack([b1, b2])

# Solve least-squares
sol, residuals, rank, s = lstsq(M, b, rcond=None)

print("Solution X:", sol[:3])
print("U:", sol[3])
print("Residuals:", residuals)

from scipy.linalg import block_diag

n = A.shape[0]
Ctrb = B
for i in range(1, n):
    Ctrb = np.hstack([Ctrb, np.linalg.matrix_power(A, i) @ B])
rank = np.linalg.matrix_rank(Ctrb)
print("Controllability rank:", rank)

