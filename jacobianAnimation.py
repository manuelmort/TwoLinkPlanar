import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation

a1 = 10
a2 = 7
theta1 = 45
theta2 = 45
xf = 11
yf = 10
t = 10

theta1_rad = np.deg2rad(theta1)
theta2_rad = np.deg2rad(theta2)

px = a1* np.cos(theta1_rad) + a2 * np.cos(theta1_rad + theta2_rad)
py = a1* np.sin(theta1_rad) + a2 * np.sin(theta1_rad + theta2_rad)
velocity_x = (xf - px)/t
velocity_y = (yf - py)/t
velocity_matrix = np.array([[velocity_x],[velocity_y]])

def plot_robot_configuration(theta1_deg, theta2_deg, a1, a2):
    theta1_rad = math.radians(theta1_deg)
    theta2_rad = math.radians(theta2_deg)
    
    x1 = a1 * math.cos(theta1_rad)
    y1 = a1 * math.sin(theta1_rad)
    
    x2 = x1 + a2 * math.cos(theta1_rad + theta2_rad)
    y2 = y1 + a2 * math.sin(theta1_rad + theta2_rad)

    return [0, x1, x2], [0, y1, y2]

fig, ax = plt.subplots()
ax.axis('equal')

line, = ax.plot([], [], '-o', markersize=10)

def init():
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 15)
    return line,

def update(frame):
    global theta1, theta2, a1, a2, velocity_matrix
    epsilon = 0.01  # Threshold for proximity
    xcurr = a1 * np.cos(np.radians(theta1)) + a2 * np.cos(np.radians(theta1 + theta2))
    ycurr = a1 * np.sin(np.radians(theta1)) + a2 * np.sin(np.radians(theta1 + theta2))

    if abs(xcurr - xf) > epsilon and abs(ycurr - yf) > epsilon:
        matrix = np.array([[-a1 * np.sin(theta1_rad) - a2 * np.sin(theta1_rad + theta2_rad), -a2 * np.sin(theta1_rad + theta2_rad)],
                        [a1 * np.cos(theta1_rad) + a2 * np.cos(theta1_rad + theta2_rad), a2 * np.cos(theta1_rad + theta2_rad)]])

        inverse_matrix = np.linalg.inv(matrix)
        newAngle = np.dot(inverse_matrix,velocity_matrix)

        theta1 += newAngle[0]
        theta2 += newAngle[1]

    x_data, y_data = plot_robot_configuration(theta1, theta2, a1, a2)
    line.set_data(x_data, y_data)
    return line,

ani = FuncAnimation(fig, update, frames=np.arange(0, 100, 2), init_func=init, blit=True, interval=5)

plt.title("2-Link Manipulator - Elbow Down and Elbow Up Configurations")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid()
plt.show()
