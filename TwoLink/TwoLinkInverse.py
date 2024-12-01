import math
import matplotlib.pyplot as plt

class TwoLinkInverse:
    def __init__(self, a1, a2):
        """
        Initialize the 2-link robot arm with given link lengths.
        :param a1: Length of the first link.
        :param a2: Length of the second link.
        """
        self.a1 = a1
        self.a2 = a2

    def inverse_kinematics(self, x, y, elbow="down"):
        """
        Calculate the inverse kinematics for a given end-effector position.
        :param x: X-coordinate of the end-effector.
        :param y: Y-coordinate of the end-effector.
        :param elbow: "down" for elbow-down configuration, "up" for elbow-up configuration.
        :return: Tuple (theta1, theta2) in degrees.
        """
        d = math.sqrt(x**2 + y**2)

        # Check if the target is reachable
        if d > self.a1 + self.a2:
            raise ValueError("Target is unreachable")

        cos_theta2 = (x**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        if elbow == "down":
            theta2 = math.acos(cos_theta2)
        elif elbow == "up":
            theta2 = -math.acos(cos_theta2)
        else:
            raise ValueError("Invalid elbow configuration. Choose 'down' or 'up'.")

        theta1 = math.atan2(y, x) - math.atan2(self.a2 * math.sin(theta2), self.a1 + self.a2 * math.cos(theta2))

        return math.degrees(theta1), math.degrees(theta2)

    def plot_configuration(self, theta1_deg, theta2_deg):
        """
        Plot the robot arm configuration for given joint angles.
        :param theta1_deg: Angle of the first joint in degrees.
        :param theta2_deg: Angle of the second joint in degrees.
        """
        theta1_rad = math.radians(theta1_deg)
        theta2_rad = math.radians(theta2_deg)

        x1 = self.a1 * math.cos(theta1_rad)
        y1 = self.a1 * math.sin(theta1_rad)

        x2 = x1 + self.a2 * math.cos(theta1_rad + theta2_rad)
        y2 = y1 + self.a2 * math.sin(theta1_rad + theta2_rad)

        plt.figure(figsize=(6, 6))
        plt.plot([0, x1, x2], [0, y1, y2], '-o', markersize=8, label="Arm Configuration")
        plt.xlim(0, self.a1 + self.a2 + 1)  # Restrict to positive X-axis
        plt.ylim(0, self.a1 + self.a2 + 1)  # Restrict to positive Y-axis
        plt.grid(True)
        plt.title("2-Link Robot Arm Configuration (Positive Quadrant Only)")
        plt.legend()
        plt.show()



