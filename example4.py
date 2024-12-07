import numpy as np
from ThreeDOF.ThreeDOFJacobian import ThreeDOFJacobian

# Parameters for the circle
center = (10.5, 10.5, 9)  # Center of the circle (x, y, z)
radius = 2.0  # Radius of the circle
num_points = 35  # Number of points on the circle

# Generate circle points
theta = np.linspace(0, 2 * np.pi, num_points)
circle_targets = [(center[0] + radius * np.cos(t), center[1] + radius * np.sin(t), center[2]) for t in theta]

# Print generated circle targets (optional)
for target in circle_targets:
    print(target)

# Create an instance of the ThreeDOFJacobian class
manipulator = ThreeDOFJacobian(
    l1=10,  # Length of the first link
    l2=10,  # Length of the second link
    l3=10,  # Length of the third link
    theta1=45,  # Initial angle of the first joint in degrees
    theta2=30,  # Initial angle of the second joint in degrees (upward bend)
    theta3=-30,  # Initial angle of the third joint in degrees (upward bend)
    targets=circle_targets,  # List of circle targets
    t_per_target=0.05  # Time per target
)

# Run the animation
manipulator.animate()
