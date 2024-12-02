import numpy as np
from scipy.optimize import linprog

# Define problem parameters
theta_max = np.pi  # Maximum angle (radians) for each joint
theta_min = -np.pi  # Minimum angle (radians)
tau_max = 50  # Maximum torque (Nm) for each joint
tau_min = -50  # Minimum torque (Nm)
desired_accuracy = 0.02  # Desired maximum error threshold for joints (rad)

# Desired joint angles (target angles in radians)
theta_d1 = np.deg2rad(45)  # Target angle for Joint 1
theta_d2 = np.deg2rad(30)  # Target angle for Joint 2

# Current joint angles (example values in radians)
theta_1 = np.deg2rad(40)  # Current angle for Joint 1
theta_2 = np.deg2rad(25)  # Current angle for Joint 2

# Compute initial angle errors
delta_theta_1 = theta_d1 - theta_1
delta_theta_2 = theta_d2 - theta_2

# Bounds for epsilon (angle error threshold)
bounds = [(0, desired_accuracy),  # Epsilon for Angle 1
          (0, desired_accuracy)]  # Epsilon for Angle 2

# Coefficients for the objective function: minimize epsilon (both angle errors)
c = [1, 1]  # Minimize both errors equally

# Inequality constraints (Ax <= b)
A = [
    [1, 0],   # |Delta Theta1| <= Epsilon
    [0, 1],   # |Delta Theta2| <= Epsilon
    [-1, 0],  # Lower bound for Delta Theta1
    [0, -1]   # Lower bound for Delta Theta2
]

b = [
    abs(delta_theta_1),  # Upper bound for Delta Theta1
    abs(delta_theta_2),  # Upper bound for Delta Theta2
    -abs(delta_theta_1), # Lower bound for Delta Theta1
    -abs(delta_theta_2)  # Lower bound for Delta Theta2
]

# Solve the optimization problem
res = linprog(c, A_ub=A, b_ub=b, bounds=bounds, method='highs')

# Display results
if res.success:
    print("Optimal epsilon values found:")
    print(f"Epsilon1 (Angle1 Error Threshold): {res.x[0]:.4f} rad")
    print(f"Epsilon2 (Angle2 Error Threshold): {res.x[1]:.4f} rad")
else:
    print("Optimization failed. Check constraints or parameters.")
