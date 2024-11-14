import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.previous_time = time.time()

    def update(self, current_value):
        error = self.setpoint - current_value
        current_time = time.time()
        delta_time = current_time - self.previous_time
        self.previous_time = current_time

        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * delta_time
        I = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0
        D = self.Kd * derivative
        
        # Update previous error
        self.previous_error = error

        # Calculate final output
        output = P + I + D
        return output

# Initialize the PID controller with sample gains
pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=10)

# Simulation parameters
current_value = 0
time_steps = 800
response = []  # Stores current value for plotting
control_outputs = []  # Stores control output for plotting

# Set up the real-time plot
fig, ax = plt.subplots()
ax.set_xlim(0, time_steps)
ax.set_ylim(0, 12)  # Adjust based on expected range of response
line, = ax.plot([], [], label="Current Value", color="blue")
setpoint_line = ax.axhline(pid.setpoint, color='red', linestyle='--', label="Setpoint")
ax.legend()
ax.set_xlabel("Time Steps")
ax.set_ylabel("Current Value")

# Data containers
time_data = []
current_value_data = []

def init():
    line.set_data([], [])
    return line,

def update(frame):
    global current_value

    # Get the control output from the PID controller
    control_output = pid.update(current_value)
    control_outputs.append(control_output)
    
    # Simulate the system's response (this part would depend on your system)
    current_value += control_output * 0.1  # Example system response
    
    # Update data for plotting
    time_data.append(frame)
    current_value_data.append(current_value)
    
    # Update line plot
    line.set_data(time_data, current_value_data)
    
    return line,

# Create the animation
ani = FuncAnimation(fig, update, frames=range(time_steps), init_func=init, blit=True, interval=20)

plt.title("Real-Time PID Controller Response")
plt.show()
