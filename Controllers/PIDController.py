import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.01, integral_limit=5):
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)
    
    def calculate_pid_output(self, error, error_dot):
        # Update integral term with limit
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)
        
        P_term = self.Kp * error
        I_term = self.Ki * self.integral_error
        D_term = self.Kd * error_dot
        return P_term + I_term + D_term

    def compute_control_torque(self, theta, theta_dot, theta_d, theta_dot_d, theta_ddot_d, M, C, G):
        error = theta_d - theta
        error_dot = theta_dot_d - theta_dot
        pid_output = self.calculate_pid_output(error, error_dot)
        tau = np.dot(M, theta_ddot_d) + np.dot(C, theta_dot_d) + G + pid_output
        return tau




