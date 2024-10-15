# Two Link Planar Arm Python Library
## Goal of the project

**Python Control and Mathematical Library for Robotic Arms**

This project aims to create a Python library that simplifies the process of modeling and controlling two-link robotic arms. It incorporates mathematical models and control theory techniques to ensure both precision and stability. The library will serve as both a tool for developing robotic systems and a learning resource for others.

***Refer to [2LinkPlanar Research Notes.pdf](https://github.com/manuelmort/TwoLinkPlanar/blob/main/2LinkPlanar%20Research%20Notes.pdf) for current updates on my project***

### Kinematics
$$ \theta_{2} = -\cos^{-1} \left( \frac{x^{2} + y^{2} - a_{1}^{2} - a_{2}^{2}}{2a_{1}a_{2}} \right) $$
$$\theta_{1} = \tan^{-1}\left(\frac{y}{x}\right) - \tan^{-1}\left(\frac{a_{2} \sin(\theta_{2})}{a_{1} + a_{2} \cos(\theta_{2})}\right)$$
### Control
- Implementing Control System Theory from EC 501 Class

***Integration with Control Theory***: By deriving clear, mathematical models of motion, the Lagrangian framework integrates seamlessly with modern control theory techniques. This integration is essential for developing effective control strategies that enhance the robotic arm's precision and stability.

**Simplification of Equations**: The Lagrangian method reduces the complexity of the motion equations by focusing on energy differences rather than forces. This is especially useful in systems like a two-link robotic arm where multiple forces and torques interact.

## Resources

### Articles
[Geometric Approach Inverse Kinematics](https://medium.com/@manuelmort/inverse-kinematics-of-two-link-planar-arm-geometric-approach-5f3ffdfde16d "Geometric Approach Inverse Kinematics")


[A Two-Link Robot Manipulator: Simulation and Control Design](https://www.vibgyorpublishers.org/content/ijre/ijre-5-028.pdf)

## Textbooks


### Control
Friedland, Bernard - Control System Design - An Introduction to State-Space --- 1986 -- Dover Publications 

Roger W. Brockett - Finite Dimensional Linear Systems (Decision & Control) (1970)

Robot Modeling and Control 2nd Eidition by Mark W. Spong | Seth Hutchinson | M. Vidyasagar


## Future Development

 - Expansion to Complex Robotic Arms: Extend the library to handle more sophisticated systems, including robotic arms with more than two links.
 - Provide examples and tutorials to help others understand the mathematical models and control systems used in robotic arm development.