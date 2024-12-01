import numpy as np
from TwoLinkAnimation import TwoLinkAnimation
from ThreeDOFJacobian import ThreeDOFJacobian
#Example Code

#Declare Link Lenghts
l1, l2 = 2.0, 1.0
x,y = 1.0, 1.0
xf, yf = 2.0, 2.0
m1 = 1.0
m2 = 1.0

#TwoLinkAnimation(l1,l2,x,y,xf,yf,m1,m2)
TwoLinkSimulation = TwoLinkAnimation(l1=l1,l2 =l2,x=x,y=y,xf=xf,yf=yf,m1=m1,m2=m2)
#TwoLinkSimulation.animate()

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