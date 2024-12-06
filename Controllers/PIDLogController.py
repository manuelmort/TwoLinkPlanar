import numpy as np

class PIDLogController:
    def __init__(self, Kp, Ki, Kd, dt=0.01, integral_limit=5, log_base=10):
        """
        Initialize the log-scale PID controller.
        
        Parameters:
        - Kp, Ki, Kd: PID gains (arrays of size 2 for a 2-link system)
        - dt: Time step for integration
        - integral_limit: Maximum allowed value for integral term
        - log_base: Base of the logarithm used in scaling (default: 10)
        """
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        self.dt = dt
        self.integral_limit = integral_limit
        self.log_base = log_base
        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)
    
    def log_transform(self, error):
        """
        Apply logarithmic scaling to the error, preserving the sign.
        """
        return np.sign(error) * np.log1p(np.abs(error)) / np.log(self.log_base)
    
    def calculate_log_pid_output(self, error, error_dot):
        """
        Calculate the PID output using log-transformed error terms.
        """
        # Log-transform the error and error_dot
        log_error = self.log_transform(error)
        log_error_dot = self.log_transform(error_dot)
        
        # Update integral term with limit
        self.integral_error += log_error * self.dt
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)
        
        # Calculate PID terms
        P_term = self.Kp * log_error
        I_term = self.Ki * self.integral_error
        D_term = self.Kd * log_error_dot
        
        return P_term + I_term + D_term

    def compute_control_torque(self, theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, M, C, G):
        """
        Compute the control torque for the system.
        
        Parameters:
        - theta: Current joint angles
        - theta_dot: Current joint velocities
        - theta_d: Desired joint angles
        - theta_dot_d: Desired joint velocities
        - theta_ddot_d: Desired joint accelerations
        - M: Inertia matrix
        - C: Coriolis matrix
        - G: Gravity vector
        """
        # Calculate errors
        error = theta_d - theta
        error_dot = theta_dot_d - theta_dot
        
        # Get log-scale PID output
        pid_output = self.calculate_log_pid_output(error, error_dot)
        
        # Compute control torque
        tau = np.dot(M, theta_ddot_d) + np.dot(C, theta_dot_d) + G + pid_output
        return tau
