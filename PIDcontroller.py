import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RobotArmPIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.01, integral_limit=5):
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)
    
    def calculate_pid_output(self, error, error_dot):
        # Update integral term with limit
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)
        
        P_term = self.Kp * error
        I_term = self.Ki * self.integral_error
        D_term = self.Kd * error_dot
        return P_term + I_term + D_term

    def compute_control_torque(self, theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, M, C, G):
        error = theta_d - theta
        error_dot = theta_dot_d - theta_dot
        pid_output = self.calculate_pid_output(error, error_dot)
        tau = np.dot(M, theta_ddot_d) + np.dot(C, theta_dot_d) + G + pid_output
        return tau

def inertia_matrix(theta):
    m1, m2, L1, L2 = 1.0, 1.0, 2.0, 1.0
    return np.array([ 
        [(m1+m2)*L1**2 + 2 * m2 * L1 * L2 * np.cos(theta[1]) + m2*L2**2, m2 * (L1 * L2 * np.cos(theta[1]) + L2**2)],
        [m2 * (L1 * L2 * np.cos(theta[1]) + L2**2), m2 * L2**2]
    ])

def coriolis_matrix(theta, theta_dot):
    m2, L1, L2 = 1.0, 2.0, 1.0
    return np.array([
        [-m2 * L1 * L2 * np.sin(theta[1]) * theta_dot[1], -m2 * L1 * L2 * np.sin(theta[1]) * theta[0]],
        [m2 * L1 * L2 * np.sin(theta[1]) * theta_dot[0], 0]
    ])

def gravity_vector(theta):
    m1, m2, L1, L2, g = 1.0, 1.0, 2.0, 1.0, 9.81
    return np.array([
        (m1 + m2) * L1 * g * np.cos(theta[0]) + m2 * L2 * g * np.cos(theta[0] + theta[1]),
        m2 * L2 * g * np.cos(theta[0] + theta[1])
    ])

# PID gains for each joint
Kp = [25, 25]
Ki = [20, 20]
Kd = [15, 10]

controller = RobotArmPIDController(Kp, Ki, Kd, integral_limit=2)

# Desired trajectory
theta_d = np.array([np.pi / 2, -np.pi])
theta_dot_d = np.array([0.0, 0.0])
theta_ddot_d = np.array([0.0, 0.0])

# Initial conditions
theta = np.array([0.0, 0.0])
theta_dot = np.array([0.0, 0.0])
# Simulation parameters
time_steps = 1000
dt = 0.01

epsilon = 0.001

# Set up the figure
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Configure the first plot for Joint 1
ax1.set_xlim(0, time_steps * dt)
ax1.set_ylim(-1, 5)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Joint 1 Angle (rad)")
line1, = ax1.plot([], [], lw=2, label="Joint 1 Angle")
target_line1 = ax1.axhline(theta_d[0], color='r', linestyle='--', label="Joint 1 Target")
ax1.legend()

# Configure the second plot for Joint 2
ax2.set_xlim(0, time_steps * dt)
ax2.set_ylim(-1, -5)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Joint 2 Angle (rad)")
line2, = ax2.plot([], [], lw=2, label="Joint 2 Angle")
target_line2 = ax2.axhline(theta_d[1], color='g', linestyle='--', label="Joint 2 Target")
ax2.legend()

# Data for real-time plotting
times = []
angles1 = []
angles2 = []

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1, line2

def update(frame):
    global theta, theta_dot
    
    # Compute the current dynamics
    M = inertia_matrix(theta)
    C = coriolis_matrix(theta, theta_dot)
    G = gravity_vector(theta)
    
    # Compute control torque using the PID controller
    tau = controller.compute_control_torque(theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, M, C, G)
    
    # Compute joint accelerations and update velocities and positions
    theta_ddot = np.linalg.inv(M).dot(tau - np.dot(C, theta_dot) - G)
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt
    
    # Check error for stopping condition
    error = np.abs(theta_d - theta)
    if np.all(error < epsilon):  # Stop if all errors are below epsilon
        print("Error below threshold. Stopping animation.")
        ani.event_source.stop()  # Stop the animation
        
    # Append data for plotting
    times.append(frame * dt)
    angles1.append(theta[0])
    angles2.append(theta[1])
    
    # Update the lines on the plot
    line1.set_data(times, angles1)
    line2.set_data(times, angles2)
    return line1, line2

ani = FuncAnimation(fig, update, frames=range(time_steps), init_func=init, blit=True, interval=dt*1000)

plt.suptitle("Real-Time PID Control of Joint Angles")
plt.tight_layout()
plt.show()
