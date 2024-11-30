import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Example usage of TwoLink classes
from TwoLinkInverse import TwoLinkInverse
from TwoLinkJacobian import TwoLinkJacobian
from TwoLinkDynamics import TwoLinkDynamics
from TwoLinkPIDController import TwoLinkPIDController

# Parameters
l1 = 2.0
l2 = 1.0
robot_arm = TwoLinkInverse(a1=l1, a2=l2)

# Starting and desired end-effector positions
x, y = 1.0, 1.0  # Initial end-effector position
xf, yf = 2.0, 2.0  # Desired end-effector position

# Initial and desired joint angles
theta1_down, theta2_down = robot_arm.inverse_kinematics(x, y, elbow="down")
theta1d_down, theta2d_down = robot_arm.inverse_kinematics(xf, yf, elbow="down")

# Dynamics and PID controllers
dynamics = TwoLinkDynamics(m1=1.0, m2=1.0, l1=l1, l2=l2)
Kp = [15, 15]
Ki = [5, 5]
Kd = [20, 15]
controller = TwoLinkPIDController(Kp, Ki, Kd, integral_limit=2)

# Initial and desired states
theta = np.array([np.deg2rad(theta1_down), np.deg2rad(theta2_down)])
theta_dot = np.array([0.0, 0.0])
theta_d = np.array([np.deg2rad(theta1d_down), np.deg2rad(theta2d_down)])
theta_dot_d = np.array([0.0, 0.0])
theta_ddot_d = np.array([0.0, 0.0])

# Simulation parameters
dt = 0.05
time_steps = 500
epsilon = 0.015

# Data for plotting
times = []
angles1 = []
angles2 = []
torques1 = []
torques2 = []
path_x = [x]  # Path tracing
path_y = [y]

# Set up the figure
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
ax1, ax2, ax3, ax4 = axs.flat

# Arm motion plot
ax1.set_xlim(0, l1 + l2 + 0.5)
ax1.set_ylim(0, l1 + l2 + 0.5)
ax1.set_title("2-Link Arm Motion")
line_arm, = ax1.plot([], [], '-o', markersize=8, label="Arm")
path_line, = ax1.plot([], [], 'r-', lw=1, label="Path")
ax1.legend()

# Joint angles plot
ax2.set_xlim(0, 5)
ax2.set_ylim(-np.pi, np.pi)
ax2.set_title("Joint Angles")
line_theta1, = ax2.plot([], [], label="Theta1 (rad)")
line_theta2, = ax2.plot([], [], label="Theta2 (rad)")
ax2.legend()

# Control torques plot
ax3.set_xlim(0, 5)
ax3.set_ylim(-30, 50)
ax3.set_title("Control Torques")
line_tau1, = ax3.plot([], [], label="Torque1 (Nm)")
line_tau2, = ax3.plot([], [], label="Torque2 (Nm)")
ax3.legend()

# Error plot
ax4.set_xlim(0, 5)
ax4.set_ylim(-np.pi, np.pi)
ax4.set_title("Joint Errors")
line_error1, = ax4.plot([], [], label="Error1 (rad)")
line_error2, = ax4.plot([], [], label="Error2 (rad)")
ax4.legend()

def init():
    """Initialize all plot lines."""
    line_arm.set_data([], [])
    path_line.set_data([], [])
    line_theta1.set_data([], [])
    line_theta2.set_data([], [])
    line_tau1.set_data([], [])
    line_tau2.set_data([], [])
    line_error1.set_data([], [])
    line_error2.set_data([], [])
    return line_arm, path_line, line_theta1, line_theta2, line_tau1, line_tau2, line_error1, line_error2

def update(frame):
    """Update the simulation and plots at each frame."""
    global theta, theta_dot, path_x, path_y

    # Compute dynamics
    M = dynamics.inertia_matrix(theta)
    C = dynamics.coriolis_matrix(theta, theta_dot)
    G = dynamics.gravity_vector(theta)
    
    # Compute torques using PID controller
    tau = controller.compute_control_torque(theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, M, C, G)
    
    # Compute accelerations and update states
    theta_ddot = np.linalg.inv(M).dot(tau - np.dot(C, theta_dot) - G)
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    error1 = theta_d[0] - theta[0]
    error2 = theta_d[1] - theta[1]

    # Stop the animation if joint errors are below the threshold
    if abs(error1) < epsilon and abs(error2) < epsilon:
        print(f"Animation stopped at frame {frame} as joint errors are minimal.")
        ani.event_source.stop()
    # Forward kinematics for the arm
    x1 = l1 * np.cos(theta[0])
    y1 = l1 * np.sin(theta[0])
    x2 = x1 + l2 * np.cos(theta[0] + theta[1])
    y2 = y1 + l2 * np.sin(theta[0] + theta[1])

    # Append path data for visualization
    path_x.append(x2)
    path_y.append(y2)

    # Append data for plotting
    times.append(frame * dt)
    angles1.append(theta[0])
    angles2.append(theta[1])
    torques1.append(tau[0])
    torques2.append(tau[1])

    # Update arm motion plot
    line_arm.set_data([0, x1, x2], [0, y1, y2])
    path_line.set_data(path_x, path_y)

    # Update joint angles plot
    line_theta1.set_data(times, angles1)
    line_theta2.set_data(times, angles2)

    # Update control torques plot
    line_tau1.set_data(times, torques1)
    line_tau2.set_data(times, torques2)

    # Update errors plot
    line_error1.set_data(times, [theta_d[0] - a for a in angles1])
    line_error2.set_data(times, [theta_d[1] - a for a in angles2])

    return line_arm, path_line, line_theta1, line_theta2, line_tau1, line_tau2, line_error1, line_error2

# Create the animation
ani = FuncAnimation(fig, update, frames=range(time_steps), init_func=init, blit=False, interval=dt * 1000)

plt.tight_layout()
plt.show()
