import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class TwoLinkJacobian:
    def __init__(self, l1, l2, theta1, theta2, targets, t_per_target):
        self.l1 = l1
        self.l2 = l2
        self.theta1 = np.deg2rad(theta1)
        self.theta2 = np.deg2rad(theta2)
        self.targets = targets  # List of (xf, yf) tuples
        self.t_per_target = t_per_target
        self.epsilon = 0.006  # Smaller threshold for accuracy

        # Current target index
        self.current_target_index = 0
        self.xf, self.yf = self.targets[self.current_target_index]

        # Initialize position
        self.update_current_position()

        # Velocity components
        self.update_velocity()

        # For tracing the path
        self.path_x = [self.px]
        self.path_y = [self.py]

        # Setup plot
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        self.line, = self.ax.plot([], [], '-o', markersize=10)  # Robot arm
        self.path_line, = self.ax.plot([], [], 'r-', lw=1.5)  # Path trace

    def update_current_position(self):
        """Update the current position of the end effector."""
        self.px = self.l1 * np.cos(self.theta1) + self.l2 * np.cos(self.theta1 + self.theta2)
        self.py = self.l1 * np.sin(self.theta1) + self.l2 * np.sin(self.theta1 + self.theta2)

    def update_velocity(self):
        """Update velocity for the current target."""
        self.velocity_x = (self.xf - self.px) / self.t_per_target
        self.velocity_y = (self.yf - self.py) / self.t_per_target
        self.velocity_matrix = np.array([[self.velocity_x], [self.velocity_y]])

    def jacobian_matrix(self):
        """Compute the Jacobian matrix."""
        return np.array([
            [-self.l1 * np.sin(self.theta1) - self.l2 * np.sin(self.theta1 + self.theta2), -self.l2 * np.sin(self.theta1 + self.theta2)],
            [self.l1 * np.cos(self.theta1) + self.l2 * np.cos(self.theta1 + self.theta2), self.l2 * np.cos(self.theta1 + self.theta2)]
        ])

    def plot_robot_configuration(self, theta1, theta2):
        """Calculate and return the manipulator configuration."""
        x1 = self.l1 * np.cos(theta1)
        y1 = self.l1 * np.sin(theta1)
        x2 = x1 + self.l2 * np.cos(theta1 + theta2)
        y2 = y1 + self.l2 * np.sin(theta1 + theta2)
        return [0, x1, x2], [0, y1, y2]

    def init_plot(self):
        """Initialize the plot."""
        self.ax.set_xlim(0, 15)
        self.ax.set_ylim(0, 15)
        return self.line, self.path_line

    def update_plot(self, frame):
        """Update the manipulator configuration."""
        # Compute current position
        self.update_current_position()

        # Check both x and y proximity to target
        x_reached = abs(self.px - self.xf) < self.epsilon
        y_reached = abs(self.py - self.yf) < self.epsilon

        if not (x_reached and y_reached):
            # Compute Jacobian and update angles
            jacobian = self.jacobian_matrix()
            inverse_matrix = np.linalg.inv(jacobian)
            new_angle = np.dot(inverse_matrix, self.velocity_matrix)
            self.theta1 += new_angle[0, 0]
            self.theta2 += new_angle[1, 0]
        else:
            # If the target is reached, move to the next target after a delay
            if self.current_target_index + 1 < len(self.targets):
                time.sleep(0.1)  # Wait for a moment
                self.current_target_index += 1
                self.xf, self.yf = self.targets[self.current_target_index]
                self.update_velocity()

        # Store the current position for path tracing
        self.path_x.append(self.px)
        self.path_y.append(self.py)

        # Get updated configuration
        x_data, y_data = self.plot_robot_configuration(self.theta1, self.theta2)
        print(f"Current Position: ({self.px:.4f}, {self.py:.4f}), Target: ({self.xf:.4f}, {self.yf:.4f})")  # Debugging output
        self.line.set_data(x_data, y_data)
        self.path_line.set_data(self.path_x, self.path_y)
        return self.line, self.path_line

    def animate(self):
        """Run the animation."""
        ani = FuncAnimation(self.fig, self.update_plot, frames=np.arange(0, 1000, 2), init_func=self.init_plot, blit=True, interval=5)
        plt.title("2-Link Manipulator - Path Tracing")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid()
        plt.show()

