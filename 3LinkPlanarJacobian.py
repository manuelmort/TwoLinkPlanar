import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ThreeLinkJacobian:
    def __init__(self, l1, l2, l3, theta1, theta2, theta3, targets, t_per_target, elbow="down"):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.targets = targets  # List of (xf, yf) tuples
        self.t_per_target = t_per_target
        self.epsilon = 0.01  # Accuracy threshold
        self.elbow = elbow  # "up" or "down"

        # Compute initial angles using inverse kinematics for the specified configuration
        self.theta1, self.theta2, self.theta3 = self.inverse_kinematics(
            targets[0][0], targets[0][1]
        )

        # Current target index
        self.current_target_index = 0
        self.xf, self.yf = self.targets[self.current_target_index]

        # Initialize position
        self.update_current_position()

        # Velocity components
        self.update_velocity()

        # Path tracing
        self.path_x = [self.px]
        self.path_y = [self.py]

        # Setup plot
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        self.line, = self.ax.plot([], [], '-o', markersize=10)  # Robot arm
        self.path_line, = self.ax.plot([], [], 'r-', lw=1.5)  # Path trace

    def inverse_kinematics(self, xf, yf):
        """Compute the inverse kinematics for the given target position."""
        d = np.sqrt(xf**2 + yf**2)

        # Check if the target is reachable
        if d > self.l1 + self.l2 + self.l3:
            raise ValueError("Target is unreachable")

        # Solve for theta3
        cos_theta3 = (xf**2 + yf**2 - self.l1**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)  # Clamp to [-1, 1]
        sin_theta3 = np.sqrt(1 - cos_theta3**2) if self.elbow == "down" else -np.sqrt(1 - cos_theta3**2)
        theta3 = np.arctan2(sin_theta3, cos_theta3)

        # Solve for theta2
        k1 = self.l2 + self.l3 * cos_theta3
        k2 = self.l3 * sin_theta3
        theta2 = np.arctan2(yf, xf) - np.arctan2(k2, k1)

        # Solve for theta1
        theta1 = np.arctan2(yf, xf) - theta2 - theta3

        return theta1, theta2, theta3


    def forward_kinematics(self, theta1, theta2, theta3):
        """Compute forward kinematics to get end-effector position."""
        x1 = self.l1 * np.cos(theta1)
        y1 = self.l1 * np.sin(theta1)

        x2 = x1 + self.l2 * np.cos(theta1 + theta2)
        y2 = y1 + self.l2 * np.sin(theta1 + theta2)

        x3 = x2 + self.l3 * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.l3 * np.sin(theta1 + theta2 + theta3)

        return [0, x1, x2, x3], [0, y1, y2, y3]

    def update_current_position(self):
        """Update the current position of the end effector."""
        x_data, y_data = self.forward_kinematics(self.theta1, self.theta2, self.theta3)
        self.px = x_data[-1]  # The last x-coordinate (end-effector position)
        self.py = y_data[-1]  # The last y-coordinate (end-effector position)

    def update_velocity(self):
        """Update velocity for the current target."""
        self.velocity_x = (self.xf - self.px) / self.t_per_target
        self.velocity_y = (self.yf - self.py) / self.t_per_target
        self.velocity_vector = np.array([self.velocity_x, self.velocity_y])

    def jacobian_matrix(self):
        """Compute the Jacobian matrix."""
        s1 = np.sin(self.theta1)
        c1 = np.cos(self.theta1)
        s12 = np.sin(self.theta1 + self.theta2)
        c12 = np.cos(self.theta1 + self.theta2)
        s123 = np.sin(self.theta1 + self.theta2 + self.theta3)
        c123 = np.cos(self.theta1 + self.theta2 + self.theta3)

        J = np.array([
            [
                -self.l1 * s1 - self.l2 * s12 - self.l3 * s123,
                -self.l2 * s12 - self.l3 * s123,
                -self.l3 * s123
            ],
            [
                self.l1 * c1 + self.l2 * c12 + self.l3 * c123,
                self.l2 * c12 + self.l3 * c123,
                self.l3 * c123
            ]
        ])
        return J

    def init_plot(self):
        """Initialize the plot."""
        self.ax.set_xlim(-self.l1 - self.l2 - self.l3 - 1, self.l1 + self.l2 + self.l3 + 1)
        self.ax.set_ylim(-self.l1 - self.l2 - self.l3 - 1, self.l1 + self.l2 + self.l3 + 1)
        return self.line, self.path_line

    def update_plot(self, frame):
        """Update the manipulator configuration."""
        # Update the current position
        self.update_current_position()

        # Compute the error
        error_x = self.xf - self.px
        error_y = self.yf - self.py
        error_magnitude = np.linalg.norm([error_x, error_y])

        if error_magnitude > self.epsilon:
            # Compute Jacobian and update angles
            J = self.jacobian_matrix()
            d_theta = np.linalg.pinv(J).dot(self.velocity_vector)
            self.theta1 += d_theta[0]
            self.theta2 += d_theta[1]
            self.theta3 += d_theta[2]
        else:
            # Move to the next target if available
            if self.current_target_index + 1 < len(self.targets):
                time.sleep(0.2)  # Pause before moving to the next target
                self.current_target_index += 1
                self.xf, self.yf = self.targets[self.current_target_index]
                self.update_velocity()

        # Store the current position for path tracing
        self.path_x.append(self.px)
        self.path_y.append(self.py)

        # Update plot data
        x_data, y_data = self.forward_kinematics(self.theta1, self.theta2, self.theta3)
        self.line.set_data(x_data, y_data)
        self.path_line.set_data(self.path_x, self.path_y)

        return self.line, self.path_line

    def animate(self):
        """Run the animation."""
        ani = FuncAnimation(self.fig, self.update_plot, frames=np.arange(0, 1000), init_func=self.init_plot, blit=True, interval=10)
        plt.title(f"3-Link Manipulator - Path Tracing ({self.elbow.capitalize()} Elbow)")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid()
        plt.show()

# Example usage
targets = [(10, 10)]  # List of targets

manipulator = ThreeLinkJacobian(
    l1=8, l2=6, l3=4,
    theta1=30, theta2=45, theta3=60,
    targets=targets,
    t_per_target=100,
    elbow="up"  # Change to "down" for elbow-down configuration
)

manipulator.animate()
