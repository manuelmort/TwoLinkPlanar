import matplotlib.pyplot as plt
import math

def inverse_kinematics_2link_elbow_down(x, y, a1, a2):
    d = math.sqrt(x**2 + y**2)

    # Ensure the target is reachable
    if d > a1 + a2:
        print("Target is unreachable")
        return None

    # Calculate the angle between the first and second links
    cos_theta2 = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)
    theta2 = math.acos(cos_theta2)

    # Calculate the angle between the first link and the horizontal plane
    theta1 = math.atan2(y, x) - math.atan2(a2 * math.sin(theta2), a1 + a2 * math.cos(theta2))

    # Convert to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    return theta1_deg, theta2_deg

def inverse_kinematics_2link_elbow_up(x, y, a1, a2):
    d = math.sqrt(x**2 + y**2)

    # Ensure the target is reachable
    if d > a1 + a2:
        print("Target is unreachable")
        return None

    # Calculate the angle between the first and second links
    cos_theta2 = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)
    theta2 = -math.acos(cos_theta2)
    
    # Calculate the angle between the first link and the horizontal plane
    theta1 = math.atan2(y, x) - math.atan2(a2 * math.sin(theta2), a1 + a2 * math.cos(theta2))

    # Convert to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    return theta1_deg, theta2_deg

def plot_robot_configuration(theta1_deg, theta2_deg, a1, a2):
    theta1_rad = math.radians(theta1_deg)
    theta2_rad = math.radians(theta2_deg)
    
    x1 = a1 * math.cos(theta1_rad)
    y1 = a1 * math.sin(theta1_rad)
    
    x2 = x1 + a2 * math.cos(theta1_rad + theta2_rad)
    y2 = y1 + a2 * math.sin(theta1_rad + theta2_rad)

    plt.figure(figsize=(6, 6))
    plt.plot([0, x1, x2], [0, y1, y2], '-o', markersize=8)
    plt.xlim(0, 5)
    plt.ylim(0, 5)
    plt.grid(True)
    plt.title("Robot Arm Configuration")
    plt.show()

# Parameters
a1, a2 = 2.5, 2.0
x, y = 3, 3

# Calculate angles
theta1_down, theta2_down = inverse_kinematics_2link_elbow_down(x, y, a1, a2)
theta1_up, theta2_up = inverse_kinematics_2link_elbow_up(x, y, a1, a2)

# Plot results
print("Elbow Down Configuration: Theta1 = {:.2f}, Theta2 = {:.2f}".format(theta1_down, theta2_down))
plot_robot_configuration(theta1_down, theta2_down, a1, a2)

print("Elbow Up Configuration: Theta1 = {:.2f}, Theta2 = {:.2f}".format(theta1_up, theta2_up))
plot_robot_configuration(theta1_up, theta2_up, a1, a2)
