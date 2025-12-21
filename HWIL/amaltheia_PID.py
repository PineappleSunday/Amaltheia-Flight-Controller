
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        
    def update(self, setpoint, measured_value, dt, error_override=None):
        error = error_override if error_override is not None else setpoint - measured_value
        p_term = self.kp * error
        self.integral += error * dt
        
        if self.output_limit:
             self.integral = np.clip(self.integral, -self.output_limit, self.output_limit)
             
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        self.prev_error = error
        output = p_term + self.ki * self.integral + d_term
        
        if self.output_limit:
            output = np.clip(output, -self.output_limit, self.output_limit)
        return output