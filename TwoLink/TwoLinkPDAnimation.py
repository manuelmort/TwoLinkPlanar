import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from TwoLink.TwoLinkInverse import TwoLinkInverse
from TwoLink.TwoLinkDynamics import TwoLinkDynamics
from Controllers.PDController import PDController


class TwoLinkPDAnimation:
    def __init__(self, l1, l2, x, y, xf, yf, m1, m2):
        self.l1 = l1
        self.l2 = l2
        self.x = x
        self.y = y
        self.xf = xf
        self.yf = yf
        self.m1 = m1
        self.m2 = m2
        self.sum = 0
        self.x_axis_limit = 7

        # Dynamics and PD controller
        self.robot_arm = TwoLinkInverse(a1=self.l1, a2=self.l2)
        self.dynamics = TwoLinkDynamics(m1=self.m1, m2=self.m2, l1=self.l1, l2=self.l2)

        Kp = [15, 15]
        Kd = [30, 15]
        self.controller = PDController(Kp, Kd)

        # Simulation parameters
        self.dt = 0.05
        self.time_steps = 500
        self.epsilon = 0.019

        # Compute initial and desired joint angles
        self.theta1_down, self.theta2_down = self.robot_arm.inverse_kinematics(self.x, self.y, elbow="down")
        self.theta1d_down, self.theta2d_down = self.robot_arm.inverse_kinematics(self.xf, self.yf, elbow="down")

        # Initial and desired states
        self.theta = np.array([np.deg2rad(self.theta1_down), np.deg2rad(self.theta2_down)])
        self.theta_dot = np.array([0.0, 0.0])
        self.theta_d = np.array([np.deg2rad(self.theta1d_down), np.deg2rad(self.theta2d_down)])
        self.theta_dot_d = np.array([0.0, 0.0])
        self.theta_ddot_d = np.array([0.0, 0.0])

        # Data for plotting
        self.times = []
        self.angles1 = []
        self.angles2 = []
        self.torques1 = []
        self.torques2 = []
        self.path_x = [self.x]  # Path tracing
        self.path_y = [self.y]

    def init(self):
        """Initialize all plot lines."""
        self.line_arm.set_data([], [])
        self.path_line.set_data([], [])
        self.line_theta1.set_data([], [])
        self.line_theta2.set_data([], [])
        self.line_tau1.set_data([], [])
        self.line_tau2.set_data([], [])
        self.gravity_line1.set_data([], [])
        self.gravity_line2.set_data([], [])
        self.line_error1.set_data([], [])
        self.line_error2.set_data([], [])
        return (
            self.line_arm,
            self.path_line,
            self.line_theta1,
            self.line_theta2,
            self.line_tau1,
            self.line_tau2,
            self.gravity_line1,
            self.gravity_line2,
            self.line_error1,
            self.line_error2,
        )

    def update(self, frame):
        """Update the simulation and plots at each frame."""
        # Compute dynamics
        M = self.dynamics.inertia_matrix(self.theta)
        C = self.dynamics.coriolis_matrix(self.theta, self.theta_dot)
        G = self.dynamics.gravity_vector(self.theta)

        # Compute torques using PD controller
        tau = self.controller.compute_control_torque(
            self.theta, self.theta_dot, self.theta_d, self.theta_dot_d, self.theta_ddot_d, M, C, G
        )

        # Compute accelerations and update states
        theta_ddot = np.linalg.inv(M).dot(tau - np.dot(C, self.theta_dot) - G)
        self.theta_dot += theta_ddot * self.dt
        self.theta += self.theta_dot * self.dt

        error1 = self.theta_d[0] - self.theta[0]
        error2 = self.theta_d[1] - self.theta[1]

        # Stop the animation if joint errors are below the threshold
        if (abs(error1) < self.epsilon and abs(error2) < self.epsilon) or (error1 < 0 and error2 > 0):
            print(f"Animation stopped at frame {frame} as joint errors are minimal.")
            self.ani.event_source.stop()

        # Forward kinematics for the arm
        x1 = self.l1 * np.cos(self.theta[0])
        y1 = self.l1 * np.sin(self.theta[0])
        x2 = x1 + self.l2 * np.cos(self.theta[0] + self.theta[1])
        y2 = y1 + self.l2 * np.sin(self.theta[0] + self.theta[1])

        # Append path data for visualization
        self.path_x.append(x2)
        self.path_y.append(y2)

        # Append data for plotting
        self.times.append(frame * self.dt)
        self.angles1.append(self.theta[0])
        self.angles2.append(self.theta[1])
        self.torques1.append(tau[0])
        self.torques2.append(tau[1])

        # Update arm motion plot
        self.line_arm.set_data([0, x1, x2], [0, y1, y2])
        self.path_line.set_data(self.path_x, self.path_y)

        # Update joint angles plot
        self.line_theta1.set_data(self.times, self.angles1)
        self.line_theta2.set_data(self.times, self.angles2)

        # Update control torques plot
        self.line_tau1.set_data(self.times, self.torques1)
        self.line_tau2.set_data(self.times, self.torques2)
        self.gravity_line1.set_data(self.times, [G[0]] * len(self.times))
        self.gravity_line2.set_data(self.times, [G[1]] * len(self.times))

        # Update errors plot
        self.line_error1.set_data(self.times, [self.theta_d[0] - a for a in self.angles1])
        self.line_error2.set_data(self.times, [self.theta_d[1] - a for a in self.angles2])

        return (
            self.line_arm,
            self.path_line,
            self.line_theta1,
            self.line_theta2,
            self.line_tau1,
            self.line_tau2,
            self.gravity_line1,
            self.gravity_line2,
            self.line_error1,
            self.line_error2,
        )

    def animate(self):
        # Set up the figure
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        ax1, ax2, ax3, ax4 = self.axs.flat

        # Arm motion plot
        ax1.set_xlim(0, self.l1 + self.l2 + 0.5)
        ax1.set_ylim(0, self.l1 + self.l2 + 0.5)
        ax1.set_title("2-Link Arm Motion")
        ax1.grid(True)
        self.line_arm, = ax1.plot([], [], "-o", color="black", markersize=8, label="Arm")
        self.path_line, = ax1.plot([], [], "r-", lw=1, label="Path")
        ax1.legend()

        # Joint angles plot
        ax2.set_xlim(0, 5)
        ax2.set_ylim(-np.pi, np.pi)
        ax2.set_title("Joint Angles")
        ax2.grid(True)
        self.line_theta1, = ax2.plot([], [], color="black", label="Theta1 (rad)")
        self.line_theta2, = ax2.plot([], [], color="deepskyblue", label="Theta2 (rad)")
        ax2.axhline(self.theta_d[0], linestyle="--", color="black", label="Desired Theta1 (rad)")
        ax2.axhline(self.theta_d[1], linestyle="--", color="skyblue", label="Desired Theta2 (rad)")
        ax2.legend()

        # Control torques plot
        ax3.set_xlim(0, 5)
        ax3.set_ylim(-50, 50)
        ax3.set_title(r"Control Torques ($\tau_1$ and $\tau_2$)")
        ax3.grid(True)
        self.line_tau1, = ax3.plot([], [], label=r"$\tau_1$ (Nm)")
        self.line_tau2, = ax3.plot([], [], label=r"$\tau_2$ (Nm)")
        self.gravity_line1, = ax3.plot([], [], '--', color='gray', label="Gravity Vector Joint 1")
        self.gravity_line2, = ax3.plot([], [], '--', color='lightgray', label="Gravity Vector Joint 2")
        ax3.legend()

        # Error plot
        ax4.set_xlim(0, 5)
        ax4.set_ylim(-np.pi, np.pi)
        ax4.set_title("Joint Errors")
        ax4.grid(True)
        self.line_error1, = ax4.plot([], [], label="Error1 (rad)")
        self.line_error2, = ax4.plot([], [], label="Error2 (rad)")
        ax4.axhline(0, color='gray', linestyle='--', linewidth=1, label="Ideal Error (0)")
        ax4.legend()

        # Create the animation
        self.ani = FuncAnimation(
            self.fig, self.update, frames=range(self.time_steps), init_func=self.init, blit=False, interval=self.dt * 1000
        )

        # Display the animation
        plt.tight_layout()
        plt.show()
