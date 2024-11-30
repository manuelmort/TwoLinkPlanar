import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import matplotlib.gridspec as gridspec

# PID Controller Class
class TwoLinkPIDController:
    def __init__(self, kp, ki, kd, dt=0.01, integral_limit=5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral_error = 0.0
        self.prev_error = 0.0

    def update(self, target, current):
        error = target - current
        derivative = (error - self.prev_error) / self.dt
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)
        output = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        self.prev_error = error
        return output

# Jacobian Class
class JacobianTwoLinkPlanar:
    def __init__(self, a1, a2, theta1, theta2, targets, dt, pid_theta1, pid_theta2):
        self.a1 = a1
        self.a2 = a2
        self.theta1 = theta1
        self.theta2 = theta2
        self.targets = targets
        self.dt = dt
        self.pid_theta1 = pid_theta1
        self.pid_theta2 = pid_theta2

        # Histories
        self.time_history = [0]
        self.theta1_history = [theta1]
        self.theta2_history = [theta2]
        self.path_x = []
        self.path_y = []

        # Initialize first target
        self.current_target_index = 0
        self.xf, self.yf = targets[0]

    def forward_kinematics(self):
        x1 = self.a1 * np.cos(self.theta1)
        y1 = self.a1 * np.sin(self.theta1)
        x2 = x1 + self.a2 * np.cos(self.theta1 + self.theta2)
        y2 = y1 + self.a2 * np.sin(self.theta1 + self.theta2)
        return [0, x1, x2], [0, y1, y2]

    def update(self):
        # Compute current end-effector position
        x1 = self.a1 * np.cos(self.theta1)
        y1 = self.a1 * np.sin(self.theta1)
        px = x1 + self.a2 * np.cos(self.theta1 + self.theta2)
        py = y1 + self.a2 * np.sin(self.theta1 + self.theta2)

        # Compute errors
        error_x = self.xf - px
        error_y = self.yf - py

        # Jacobian matrix
        J = np.array([
            [-self.a1 * np.sin(self.theta1) - self.a2 * np.sin(self.theta1 + self.theta2), -self.a2 * np.sin(self.theta1 + self.theta2)],
            [self.a1 * np.cos(self.theta1) + self.a2 * np.cos(self.theta1 + self.theta2), self.a2 * np.cos(self.theta1 + self.theta2)]
        ])

        # Compute inverse kinematics
        d_theta = np.linalg.pinv(J).dot(np.array([error_x, error_y]))

        # Update joint angles using PID controllers
        self.theta1 += self.pid_theta1.update(d_theta[0], 0) * self.dt
        self.theta2 += self.pid_theta2.update(d_theta[1], 0) * self.dt

        # Update histories
        self.theta1_history.append(self.theta1)
        self.theta2_history.append(self.theta2)
        self.time_history.append(self.time_history[-1] + self.dt)

        # Store current path
        self.path_x.append(px)
        self.path_y.append(py)

        # Check if target is reached
        if np.linalg.norm([error_x, error_y]) < 0.01:
            self.current_target_index += 1
            if self.current_target_index < len(self.targets):
                self.xf, self.yf = self.targets[self.current_target_index]

# Initialize Parameters
a1, a2 = 10, 7
theta1, theta2 = np.radians(45), np.radians(45)
targets = [(10, 8), (9, 6), (8, 12)]
dt = 0.05
pid_theta1 = TwoLinkPIDController(kp=1.5, ki=0.1, kd=0.05, dt=dt)
pid_theta2 = TwoLinkPIDController(kp=1.2, ki=0.1, kd=0.05, dt=dt)
manipulator = JacobianTwoLinkPlanar(a1, a2, theta1, theta2, targets, dt, pid_theta1, pid_theta2)

# Create a figure with 3 rows and 1 column
fig = plt.figure(figsize=(12, 12))
gs = gridspec.GridSpec(3, 1, height_ratios=[2, 1, 1])

# Arm motion plot (top row)
ax1 = fig.add_subplot(gs[0])
line1, = ax1.plot([], [], '-o', label="Arm Configuration", markersize=8)
path_line, = ax1.plot([], [], 'r-', lw=1.5, label="Path Trace")
ax1.set_xlim(-20, 20)
ax1.set_ylim(-20, 20)
ax1.set_title("Arm Motion")
ax1.set_xlabel("X-axis")
ax1.set_ylabel("Y-axis")
ax1.legend()

# Theta1 plot (middle row)
ax2 = fig.add_subplot(gs[1])
line2, = ax2.plot([], [], label="Theta1 (Actual)", color="blue")
line2_target, = ax2.plot([], [], '--', label="Theta1 (Target)", color="red")
ax2.set_xlim(0, 15)  # Adjusted based on simulation time
ax2.set_ylim(0, np.pi)  # For radians
ax2.set_title("Theta1 Control")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Theta1 (rad)")
ax2.legend()

# Theta2 plot (bottom row)
ax3 = fig.add_subplot(gs[2])
line3, = ax3.plot([], [], label="Theta2 (Actual)", color="orange")
line3_target, = ax3.plot([], [], '--', label="Theta2 (Target)", color="green")
ax3.set_xlim(0, 15)  # Adjusted based on simulation time
ax3.set_ylim(0, np.pi)  # For radians
ax3.set_title("Theta2 Control")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Theta2 (rad)")
ax3.legend()

# Initialization function for the animation
def init():
    line1.set_data([], [])
    path_line.set_data([], [])
    line2.set_data([], [])
    line2_target.set_data([], [])
    line3.set_data([], [])
    line3_target.set_data([], [])
    return line1, path_line, line2, line2_target, line3, line3_target

# Update function for the animation
def update(frame):
    manipulator.update()  # Update the manipulator's state
    x, y = manipulator.forward_kinematics()

    # Update arm motion plot
    line1.set_data(x, y)
    path_line.set_data(manipulator.path_x, manipulator.path_y)

    # Update Theta1 control plot
    time = manipulator.time_history
    line2.set_data(time, manipulator.theta1_history)
    line2_target.set_data(time, manipulator.theta1_target_history)

    # Update Theta2 control plot
    line3.set_data(time, manipulator.theta2_history)
    line3_target.set_data(time, manipulator.theta2_target_history)

    return line1, path_line, line2, line2_target, line3, line3_target

# Create the animation
ani = FuncAnimation(
    fig,
    update,
    init_func=init,
    frames=300,  # Number of frames for animation
    blit=False,  # Blit improves performance but requires compatible graphics
    interval=50  # Interval in milliseconds
)

plt.tight_layout()
plt.show()