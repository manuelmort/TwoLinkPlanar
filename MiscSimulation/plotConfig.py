import math
import matplotlib.pyplot as plt

def plot_robot_configuration(theta1_deg, theta2_deg, a1, a2):
    theta1_rad = math.radians(theta1_deg)
    theta2_rad = math.radians(theta2_deg)
    
    x1 = a1 * math.cos(theta1_rad)
    y1 = a1 * math.sin(theta1_rad)
    
    x2 = x1 + a2 * math.cos(theta1_rad + theta2_rad)
    y2 = y1 + a2 * math.sin(theta1_rad + theta2_rad)

    plt.plot([0, x1, x2], [0, y1, y2], '-o', markersize=10)

