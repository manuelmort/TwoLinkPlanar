import numpy as np
from ThreeDOF.ThreeDOFJacobian import ThreeDOFJacobian

#Example Code


#Defining Targets for 3DOF Robot Arm (In This example, the arm will be drawing a cube)
targets = [
    (9, 9, 9),  # Starting point
    (9, 12, 9),  # Move to top-left front
    (12, 12, 9),  # Move to top-right front
    (12, 9, 9),  # Move to bottom-right front
    (9, 9, 9),  # Back to bottom-left front
    (9, 9, 12),  # Move to bottom-left back
    (9, 12, 12),  # Move to top-left back
    (12, 12, 12),  # Move to top-right back
    (12, 9, 12),  # Move to bottom-right back
    (9, 9, 12),  # Back to bottom-left back
    (9, 12, 12),  # Connect to top-left back
    (9, 12, 9),  # Connect to top-left front
    (12, 12, 9),  # Connect to top-right front
    (12, 12, 12),  # Connect to top-right back
    (12, 9, 12),  # Connect to bottom-right back
    (12, 9, 9),  # Connect to bottom-right front
    (9, 9, 9)   # Return to starting point
]
# Create an instance of the JacobianThreeDOFManipulator class
manipulator = ThreeDOFJacobian(
    l1=10,  # Length of the first link
    l2=10,  # Length of the second link
    l3=10,  # Length of the third link
    theta1=45,  # Initial angle of the first joint in degrees
    theta2=30,  # Initial angle of the second joint in degrees (upward bend)
    theta3=-30,  # Initial angle of the third joint in degrees (upward bend)
    targets=targets,  # List of targets
    t_per_target=100  # Time per target
)

# Run the animation
manipulator.animate()