import numpy as np

class TwoLinkDynamics:
    def __init__(self, m1, m2, l1, l2, g=9.81):
        """
        Initialize the TwoLinkDynamics class with the system parameters.
        :param m1: Mass of the first link.
        :param m2: Mass of the second link.
        :param l1: Length of the first link.
        :param l2: Length of the second link.
        :param g: Gravitational acceleration (default: 9.81).
        """
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.l2 = l2
        self.g = g

    def inertia_matrix(self, theta):
        """
        Compute the inertia matrix for the system.
        :param theta: Array of joint angles [theta1, theta2].
        :return: 2x2 inertia matrix.
        """
        cos_theta2 = np.cos(theta[1])
        return np.array([
            [
                (self.m1 + self.m2) * self.l1**2 + 2 * self.m2 * self.l1 * self.l2 * cos_theta2 + self.m2 * self.l2**2,
                self.m2 * (self.l1 * self.l2 * cos_theta2 + self.l2**2)
            ],
            [
                self.m2 * (self.l1 * self.l2 * cos_theta2 + self.l2**2),
                self.m2 * self.l2**2
            ]
        ])

    def coriolis_matrix(self, theta, theta_dot):
        """
        Compute the Coriolis matrix for the system.
        :param theta: Array of joint angles [theta1, theta2].
        :param theta_dot: Array of joint angular velocities [theta1_dot, theta2_dot].
        :return: 2x2 Coriolis matrix.
        """
        sin_theta2 = np.sin(theta[1])
        return np.array([
            [-self.m2 * self.l1 * self.l2 * sin_theta2 * theta_dot[1], -self.m2 * self.l1 * self.l2 * sin_theta2 * theta_dot[0]],
            [self.m2 * self.l1 * self.l2 * sin_theta2 * theta_dot[0], 0]
        ])

    def gravity_vector(self, theta):
        """
        Compute the gravity vector for the system.
        :param theta: Array of joint angles [theta1, theta2].
        :return: 2x1 gravity vector.
        """
        cos_theta1 = np.cos(theta[0])
        cos_theta1_theta2 = np.cos(theta[0] + theta[1])
        return np.array([
            (self.m1 + self.m2) * self.l1 * self.g * cos_theta1 + self.m2 * self.l2 * self.g * cos_theta1_theta2,
            self.m2 * self.l2 * self.g * cos_theta1_theta2
        ])

# Example Usage:
'''
if __name__ == "__main__":
    # Initialize the system
    dynamics = TwoLinkDynamics(m1=1.0, m2=1.0, l1=2.0, l2=1.0)

    # Define joint states
    theta = np.array([np.pi / 4, np.pi / 6])  # Joint angles
    theta_dot = np.array([0.5, 0.2])  # Joint angular velocities

    # Compute dynamics components
    M = dynamics.inertia_matrix(theta)
    C = dynamics.coriolis_matrix(theta, theta_dot)
    G = dynamics.gravity_vector(theta)

    print("Inertia Matrix (M):")
    print(M)
    print("\nCoriolis Matrix (C):")
    print(C)
    print("\nGravity Vector (G):")
    print(G)


'''
