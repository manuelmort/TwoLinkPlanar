import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class RoboticArm3DOF:
    def __init__(self, link_lengths):
        self.link_lengths = link_lengths
        self.joint_angles = [0, 0, 0]  # Initial joint angles

    def forward_kinematics(self):
        """Compute the forward kinematics and return the 3D positions of the joints."""
        l1, l2, l3 = self.link_lengths
        theta1, theta2, theta3 = self.joint_angles

        # Base joint
        x0, y0, z0 = 0, 0, 0

        # Joint 1 position
        x1 = l1 * np.cos(theta1)
        y1 = l1 * np.sin(theta1)
        z1 = 0

        # Joint 2 position
        x2 = x1 + l2 * np.cos(theta1) * np.cos(theta2)
        y2 = y1 + l2 * np.sin(theta1) * np.cos(theta2)
        z2 = z1 + l2 * np.sin(theta2)

        # End-effector position
        x3 = x2 + l3 * np.cos(theta1) * np.cos(theta2 + theta3)
        y3 = y2 + l3 * np.sin(theta1) * np.cos(theta2 + theta3)
        z3 = z2 + l3 * np.sin(theta2 + theta3)

        return np.array([[x0, y0, z0], [x1, y1, z1], [x2, y2, z2], [x3, y3, z3]])

    def update_angles(self, angles):
        """Update the joint angles."""
        self.joint_angles = angles

# Initialize the robotic arm with link lengths
link_lengths = [1, 1, 1]
arm = RoboticArm3DOF(link_lengths)

# Set up the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(-3, 3)
line, = ax.plot([], [], [], '-o', lw=2)

def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return line,

def update(frame):
    # Update joint angles (simple example: oscillating angles)
    theta1 = np.radians(frame % 360)
    theta2 = np.radians((frame * 2) % 360)
    theta3 = np.radians((frame * 3) % 360)
    arm.update_angles([theta1, theta2, theta3])

    # Get the positions of the arm
    positions = arm.forward_kinematics()
    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]

    # Update the plot
    line.set_data(x, y)
    line.set_3d_properties(z)
    return line,

# Animate
ani = FuncAnimation(fig, update, frames=360, init_func=init, blit=False, interval=30)
plt.title("3-DOF Robotic Arm Simulation")
plt.show()
