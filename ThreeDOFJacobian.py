import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import time

class ThreeDOFJacobian:
    def __init__(self, l1, l2, l3, theta1, theta2, theta3, targets, t_per_target):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.theta1 = np.radians(theta1)
        self.theta2 = np.radians(theta2)
        self.theta3 = np.radians(theta3)
        self.targets = targets  # List of (xf, yf, zf) tuples
        self.t_per_target = t_per_target
        self.epsilon = 0.01  # Accuracy threshold

        # Current target index
        self.current_target_index = 0
        self.xf, self.yf, self.zf = self.targets[self.current_target_index]

        # Initialize current position
        self.update_current_position()

        # Velocity components
        self.update_velocity()

        # For tracing the path
        self.path_x = [self.px]
        self.path_y = [self.py]
        self.path_z = [self.pz]

        # Setup plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line, = self.ax.plot([], [], [], '-o', markersize=8)  # Robot arm
        self.path_line, = self.ax.plot([], [], [], 'r-', lw=1.5)  # Path trace

    def update_current_position(self):
        """Update the current position of the end effector."""
        # Forward kinematics
        x1, y1, z1 = 0, 0, self.l1
        x2 = x1 + self.l2 * np.cos(self.theta1) * np.cos(self.theta2)
        y2 = y1 + self.l2 * np.sin(self.theta1) * np.cos(self.theta2)
        z2 = z1 + self.l2 * np.sin(self.theta2)

        x3 = x2 + self.l3 * np.cos(self.theta1) * np.cos(self.theta2 + self.theta3)
        y3 = y2 + self.l3 * np.sin(self.theta1) * np.cos(self.theta2 + self.theta3)
        z3 = z2 + self.l3 * np.sin(self.theta2 + self.theta3)

        self.px, self.py, self.pz = x3, y3, z3
        self.positions = np.array([[x1, x2, x3], [y1, y2, y3], [z1, z2, z3]])

    def update_velocity(self):
        """Update velocity for the current target."""
        self.velocity_x = (self.xf - self.px) / self.t_per_target
        self.velocity_y = (self.yf - self.py) / self.t_per_target
        self.velocity_z = (self.zf - self.pz) / self.t_per_target
        self.velocity_vector = np.array([self.velocity_x, self.velocity_y, self.velocity_z])

    def calculate_jacobian(self):
        """Calculate the Jacobian matrix."""
        c1 = np.cos(self.theta1)
        s1 = np.sin(self.theta1)
        c2 = np.cos(self.theta2)
        s2 = np.sin(self.theta2)
        c23 = np.cos(self.theta2 + self.theta3)
        s23 = np.sin(self.theta2 + self.theta3)

        J = np.zeros((3, 3))
        J[0, 0] = -self.l2 * s1 * c2 - self.l3 * s1 * c23
        J[0, 1] = -self.l2 * c1 * s2 - self.l3 * c1 * s23
        J[0, 2] = -self.l3 * c1 * s23
        J[1, 0] = self.l2 * c1 * c2 + self.l3 * c1 * c23
        J[1, 1] = -self.l2 * s1 * s2 - self.l3 * s1 * s23
        J[1, 2] = -self.l3 * s1 * s23
        J[2, 0] = 0
        J[2, 1] = self.l2 * c2 + self.l3 * c23
        J[2, 2] = self.l3 * c23
        return J

    def init_plot(self):
        """Initialize the plot."""
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        self.ax.set_zlim(0, 20)
        self.ax.set_xlabel("X-axis")
        self.ax.set_ylabel("Y-axis")
        self.ax.set_zlabel("Z-axis")
        self.ax.set_title("3-DOF Manipulator - Path Tracing")
        return self.line, self.path_line

    def update_plot(self, frame):
        """Update the manipulator configuration."""
        # Update the current position
        self.update_current_position()

        # Compute the error
        error_vector = np.array([self.xf - self.px, self.yf - self.py, self.zf - self.pz])
        error_magnitude = np.linalg.norm(error_vector)

        if error_magnitude > self.epsilon:
            # Compute Jacobian and update angles
            J = self.calculate_jacobian()
            d_theta = np.linalg.pinv(J).dot(error_vector * 0.05)  # Scaled down further for slower movement
            self.theta1 += d_theta[0]
            self.theta2 += d_theta[1]
            self.theta3 += d_theta[2]
        else:
            # Move to the next target if available
            if self.current_target_index + 1 < len(self.targets):
                print(f"Target {self.current_target_index} reached.")
                time.sleep(0.2)  # Small delay before moving to the next target
                self.current_target_index += 1
                self.xf, self.yf, self.zf = self.targets[self.current_target_index]
                self.update_velocity()

        # Store the current position for path tracing
        self.path_x.append(self.px)
        self.path_y.append(self.py)
        self.path_z.append(self.pz)

        # Update plot data
        self.line.set_data(self.positions[0], self.positions[1])
        self.line.set_3d_properties(self.positions[2])
        self.path_line.set_data(self.path_x, self.path_y)
        self.path_line.set_3d_properties(self.path_z)

        # Log the error and end effector position every 5 iterations
        if frame % 5 == 0:
            print(f"Iteration {frame}: End Effector Position: ({self.px:.2f}, {self.py:.2f}, {self.pz:.2f}), Error: {error_magnitude:.2f}")

        return self.line, self.path_line

    def animate(self):
        """Run the animation."""
        ani = FuncAnimation(self.fig, self.update_plot, frames=np.arange(0, 1000), init_func=self.init_plot, blit=True, interval=20)  # Slower animation interval
        plt.show()

# Example usage

