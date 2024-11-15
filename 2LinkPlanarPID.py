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
    # Mass Matrix for 2 Link Manipulator
    m1, m2, L1, L2 = 1.0, 1.0, 1.0, 1.0
    return np.array([ 
        [(m1+m2)*L1**2 + 2 * m2 * L1 * L2 * np.cos(theta[1]) + m2*L2**2, m2 * (L1 * L2 * np.cos(theta[1]) + L2**2)],
        [m2 * (L1 * L2 * np.cos(theta[1]) + L2**2), m2 * L2**2]
    ])

def coriolis_matrix(theta, theta_dot):
    m2, L1, L2 = 1.0, 1.0, 1.0
    return np.array([
        [-m2 * L1 * L2 * np.sin(theta[1]) * theta_dot[1]**2 - 2 * m2 * L1 * L2 * np.sin(theta[1]) * theta_dot[0] * theta_dot[1]],
        [m2 * L1 * L2 * np.sin(theta[1]) * theta_dot[0]**2]
    ])

def gravity_vector(theta):
    #Gravity Matrix
    m1, m2, L1, L2, g = 1.0, 1.0, 1.0, 1.0, 9.81
    return np.array([
        (m1 + m2) * L1 * g * np.cos(theta[0]) + m2 * L2 * g * np.cos(theta[0] + theta[1]),
        m2 * L2 * g * np.cos(theta[0] + theta[1])
    ])

# PID gains for each joint to stabilize the response
Kp = [5, 5]
Ki = [0.1, 0.1]
Kd = [1, 1]

# Controller instance with integral limit to prevent windup
controller = RobotArmPIDController(Kp, Ki, Kd, integral_limit=2)

# Desired trajectory (target joint positions, velocities, accelerations)
theta_d = np.array([np.pi / 4, np.pi / 3])
theta_dot_d = np.array([0.0, 0.0])
theta_ddot_d = np.array([0.0, 0.0])

# Initial conditions
theta = np.array([0.0, 0.0])
theta_dot = np.array([0.0, 0.0])

# Simulation parameters
time_steps = 200
dt = 0.01

# Set up the plot for real-time simulation
fig, ax = plt.subplots()
ax.set_xlim(0, time_steps * dt)
ax.set_ylim(0, 1.5)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Joint 1 Angle (rad)")
line, = ax.plot([], [], lw=2, label="Joint 1 Angle")
target_line = ax.axhline(theta_d[0], color='r', linestyle='--', label="Joint 1 Target")
ax.legend()

# Data for real-time plot
times = []
angles = []

def init():
    line.set_data([], [])
    return line,

def update(frame):
    global theta, theta_dot
    
    # Compute current dynamics matrices
    M = inertia_matrix(theta)
    C = coriolis_matrix(theta, theta_dot)
    G = gravity_vector(theta)
    
    # Compute control torque
    tau = controller.compute_control_torque(theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, M, C, G)
    
    # Update joint accelerations based on torque
    theta_ddot = np.linalg.inv(M).dot(tau - np.dot(C, theta_dot) - G)
    
    # Integrate to get new joint velocities and positions
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt
    
    # Append data for real-time plotting
    times.append(frame * dt)
    angles.append(theta[0])  # Only tracking Joint 1 angle here
    
    # Update the line plot
    line.set_data(times, angles)
    return line,

# Create the animation
ani = FuncAnimation(fig, update, frames=range(time_steps), init_func=init, blit=True, interval=dt*1000)

plt.title("Real-Time PID Control of Joint 1 Angle with Adjusted Gains")
plt.show()
