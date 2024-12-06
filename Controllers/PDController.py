import numpy as np

class PDController:
    def __init__(self, Kp, Kd, dt=0.01):
        """
        Initialize the PD controller.

        Parameters:
        - Kp: Proportional gains (array of size 2 for a 2-link system)
        - Kd: Derivative gains (array of size 2 for a 2-link system)
        - dt: Time step for derivative computation
        """
        self.Kp = np.array(Kp)
        self.Kd = np.array(Kd)
        self.dt = dt
        self.prev_error = np.zeros(2)  # Previous error for derivative calculation

    def calculate_pd_output(self, error, error_dot):
        """
        Calculate the PD output based on the error and its derivative.

        Parameters:
        - error: Current position error (desired position - current position)
        - error_dot: Current velocity error (desired velocity - current velocity)
        
        Returns:
        - pd_output: Control signal computed from the PD controller
        """
        # Proportional term
        P_term = self.Kp * error

        # Derivative term
        D_term = self.Kd * error_dot

        # Return combined PD control signal
        return P_term + D_term

    def compute_control_torque(self, theta, theta_dot, theta_d, theta_dot_d, M, C, G):
        """
        Compute the control torque for the robotic system.

        Parameters:
        - theta: Current joint angles
        - theta_dot: Current joint velocities
        - theta_d: Desired joint angles
        - theta_dot_d: Desired joint velocities
        - M: Inertia matrix
        - C: Coriolis matrix
        - G: Gravity vector

        Returns:
        - tau: Control torque for the robotic arm
        """
        # Calculate errors
        error = theta_d - theta
        error_dot = theta_dot_d - theta_dot

        # Get PD output
        pd_output = self.calculate_pd_output(error, error_dot)

        # Compute control torque
        tau = np.dot(M, theta_dot_d) + np.dot(C, theta_dot) + G + pd_output
        return tau
