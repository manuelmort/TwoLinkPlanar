import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Link lengths
l1 = 7  # Base to first joint
l2 = 7  # First joint to second joint
l3 = 7  # Second joint to end effector

# Reachable target coordinates
x = 4
y = 4
z = 10

# Calculate theta1 (base rotation)
theta1 = np.arctan2(y, x)

# Distance from the base to the target in the x-y plane
r = np.sqrt(x**2 + y**2)

# Distance from the first joint to the target
d = np.sqrt(r**2 + z**2)

# Check if the target is reachable
if d > l2 + l3 or d < abs(l2 - l3):
    print("Target is out of reach.")
else:
    # Calculate cos(theta3) using the law of cosines
    C3 = (d**2 - l2**2 - l3**2) / (2 * l2 * l3)

    # Ensure C3 is within valid range for acos
    if -1 <= C3 <= 1:
        # Calculate sin(theta3)
        S3 = np.sqrt(1 - C3**2)

        # Calculate theta3
        theta3 = np.arctan2(S3, C3)

        # Calculate theta2 using the law of cosines
        phi = np.arctan2(z, r)
        beta = np.arctan2(l3 * S3, l2 + l3 * C3)
        theta2 = phi - beta

        # Calculate joint positions
        # Joint 1 (Base)
        x1, y1, z1 = 0, 0, 0

        # Joint 2 (Shoulder)
        x2 = l2 * np.cos(theta1) * np.cos(theta2)
        y2 = l2 * np.sin(theta1) * np.cos(theta2)
        z2 = l2 * np.sin(theta2)

        # Joint 3 (Elbow)
        x3 = x2 + l3 * np.cos(theta1) * np.cos(theta2 + theta3)
        y3 = y2 + l3 * np.sin(theta1) * np.cos(theta2 + theta3)
        z3 = z2 + l3 * np.sin(theta2 + theta3)

        # Plotting the arm
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the links
        ax.plot([x1, x2, x3], [y1, y2, y3], [z1, z2, z3], '-o', markersize=8, label="Manipulator Links")

        # Set plot limits to positive values only
        ax.set_xlim(0, 15)  # X-axis
        ax.set_ylim(0, 15)  # Y-axis
        ax.set_zlim(0, 15)  # Z-axis

        # Add labels and grid
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        ax.set_title("3-DOF Robotic Arm Visualization")
        ax.legend()
        ax.grid()

        # Show the plot
        plt.show()
    else:
        print("Error: C3 is out of range. Check target coordinates.")
