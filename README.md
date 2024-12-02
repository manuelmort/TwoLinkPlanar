# Robotic Arm Toolbox Python Library
<img src="ArmPyLogo.png" alt="Robotic Arm Diagram" title="2-Link Arm" width="200">


## Goal of the project

**Python Control and Mathematical Library for Robotic Arms**

This project aims to create a Python library that simplifies the process of modeling and controlling two-link robotic arms. It incorporates mathematical models and control theory techniques to ensure both precision and stability. This library's purpose is to help develop a foundational understanding and create a versatile tool for simulating robotic arms. Starting with a basic 2-link planar arm, to 3-axis.

***Refer to [Research Notes Directory](https://github.com/manuelmort/TwoLinkPlanar/tree/main/Research%20Notes) for current updates on my project***

***Control Theory Notes[2LinkPlanar Research Notes.pdf] for current updates on my project***

### Kinematics

#### Two Link Inverse Kinematic Equations
$$ \theta_{2} = -\cos^{-1} \left( \frac{x^{2} + y^{2} - a_{1}^{2} - a_{2}^{2}}{2a_{1}a_{2}} \right) $$
$$\theta_{1} = \tan^{-1}\left(\frac{y}{x}\right) - \tan^{-1}\left(\frac{a_{2} \sin(\theta_{2})}{a_{1} + a_{2} \cos(\theta_{2})}\right)$$
### Control
- Implementing Control System Theory from EC 501 Class

***Integration with Control Theory***: By deriving clear, mathematical models of motion, the Lagrangian framework integrates seamlessly with modern control theory techniques. This integration is essential for developing effective control strategies that enhance the robotic arm's precision and stability.

**Simplification of Equations**: The Lagrangian method reduces the complexity of the motion equations by focusing on energy differences rather than forces. This is especially useful in systems like a two-link robotic arm where multiple forces and torques interact.

### Example of Library Usage
```import numpy as np
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
#
#
#TwoLinkSimulation.animate()

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
```

## Resources

### Articles
[Geometric Approach Inverse Kinematics](https://medium.com/@manuelmort/inverse-kinematics-of-two-link-planar-arm-geometric-approach-5f3ffdfde16d "Geometric Approach Inverse Kinematics")


[A Two-Link Robot Manipulator: Simulation and Control Design](https://www.vibgyorpublishers.org/content/ijre/ijre-5-028.pdf)

## Textbooks


### Control
Friedland, Bernard - Control System Design - An Introduction to State-Space --- 1986 -- Dover Publications 

Roger W. Brockett - Finite Dimensional Linear Systems (Decision & Control) (1970)

Robot Modeling and Control 2nd Eidition by Mark W. Spong | Seth Hutchinson | M. Vidyasagar

### Linear Algebra
A first course in Linear Algebra | Daniel Zelinsky

## Future Development

 - Expansion to Complex Robotic Arms: Extend the library to handle more sophisticated systems, including robotic arms with more than two links.
 - Provide examples and tutorials to help others understand the mathematical models and control systems used in robotic arm development.