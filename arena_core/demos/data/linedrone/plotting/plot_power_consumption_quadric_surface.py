from sympy import symbols, Eq, solve, N
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

# Define the variables for the unknowns
A, B, C, D, E, F, G, H, J, K = symbols('A B C D E F G H J K')

# Define the six equations based on the user's points
drone_power_pitch = 3102
drone_power_roll = 3102
drone_power_ascent = 3475
drone_power_descent = 3080

eq1 = Eq(B*drone_power_pitch**2 + drone_power_pitch*H + 1, 0)
eq2 = Eq(A*drone_power_roll**2 + drone_power_roll*G + 1, 0)
eq3 = Eq(C*drone_power_ascent**2 + drone_power_ascent*J + 1, 0)
eq4 = Eq(C*(-drone_power_descent)**2 - drone_power_descent*J + 1, 0)
eq5 = Eq(A*(-drone_power_roll)**2 - drone_power_roll*G + 1, 0)
eq6 = Eq(B*(-drone_power_pitch)**2 - drone_power_pitch*H + 1, 0)

# Solve the system of equations
solution = solve([eq1, eq2, eq3, eq4, eq5, eq6], (A, B, C, G, H, J))

# Convert the symbolyc solution to numerical values
solution = {key: float(N(value)) for key, value in solution.items()}

# Print the solution
print(solution)

# 3D plot of the quadractic surface based on the solution's coefficients
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X = range(-3200, 3200, 1)
Y = range(-3200, 3200, 1)
X, Y = np.meshgrid(X, Y)

# quadratic formula
Z1 = (-solution[J] + np.sqrt(solution[J]**2.0 - 4.0 * solution[C]*(1 + solution[A] * X**2.0 + solution[B] * Y**2.0))) / (2.0 * solution[C])
Z2 = (-solution[J] - np.sqrt(solution[J]**2.0 - 4.0 * solution[C]*(1 + solution[A] * X**2.0 + solution[B] * Y**2.0))) / (2.0 * solution[C])

# Plot the surface
ax.plot_surface(X, Y, Z1, alpha=1.0, color='red')
ax.plot_surface(X, Y, Z2, alpha=1.0, color='blue')
ax.set_xlabel('Permanent power roll axis')
ax.set_ylabel('Permanent power pitch axis')
ax.set_zlabel('Permanent power Z axis')
plt.show()
