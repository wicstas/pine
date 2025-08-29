import numpy as np
import matplotlib.pyplot as plt

# Define parameters
s = 1.0  # Fixed s parameter
b_vals = np.linspace(0.1, 5, 50)
c_vals = np.linspace(0.1, 5, 50)
B, C = np.meshgrid(b_vals, c_vals)
Z = np.zeros_like(B)

t = np.linspace(1, 100, 1000)

# Compute the integral for each (b, c)
for i in range(B.shape[0]):
    for j in range(B.shape[1]):
        b = B[i, j]
        c = C[i, j]
        integrand = np.exp(-s * t) / (t**2 + b * t + c)
        Z[i, j] = np.trapezoid(integrand, t)

# Plot the surface
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(B, C, Z)
ax.set_xlabel('b')
ax.set_ylabel('c')
ax.set_zlabel('Integral value')
plt.show()