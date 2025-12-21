
import numpy as np
from amaltheia_PID import PID

class QuadcopterController:
    def __init__(self, params):
        self.params = params
        
        # --- LAYER 1: Position (Outer Loop) ---
        self.pid_pos_x = PID(kp=1.0, ki=0.0, kd=0.0, output_limit=10.0)
        self.pid_pos_y = PID(kp=1.0, ki=0.0, kd=0.0, output_limit=10.0)
        self.pid_pos_z = PID(kp=2.0, ki=0.0, kd=0.0, output_limit=3.0)

        # --- LAYER 2: Velocity (Middle Loop) ---
        self.pid_vel_x = PID(kp=2.0, ki=0.5, kd=0.1, output_limit=5.0) 
        self.pid_vel_y = PID(kp=2.0, ki=0.5, kd=0.1, output_limit=5.0)
        self.pid_vel_z = PID(kp=5.0, ki=2.0, kd=0.5, output_limit=15.0)

        # --- LAYER 3: Attitude (Inner Loop) ---
        self.pid_roll  = PID(kp=20.0, ki=0.0, kd=5.0)
        self.pid_pitch = PID(kp=20.0, ki=0.0, kd=5.0)
        self.pid_yaw   = PID(kp=20.0, ki=0.0, kd=5.0)
        
    def compute_inputs(self, target_state, feedforward_vel, current_state, dt):
        # Unpack State
        curr_x, curr_y, curr_z = current_state[0:3]
        curr_vx, curr_vy, curr_vz = current_state[3:6]
        curr_phi, curr_theta, curr_psi = current_state[6:9]
        
        # Unpack Target
        target_x, target_y, target_z, target_yaw = target_state
        
        # --- LAYER 1: POSITION CONTROL ---
        corr_vel_x = self.pid_pos_x.update(target_x, curr_x, dt)
        corr_vel_y = self.pid_pos_y.update(target_y, curr_y, dt)
        corr_vel_z = self.pid_pos_z.update(target_z, curr_z, dt)
        
        # Add Feedforward
        t_vel_x = corr_vel_x + feedforward_vel[0]
        t_vel_y = corr_vel_y + feedforward_vel[1]
        t_vel_z = corr_vel_z + feedforward_vel[2]
        
        # --- LAYER 2: VELOCITY CONTROL ---
        # Generates desired accelerations in Inertial Frame
        t_accel_x = self.pid_vel_x.update(t_vel_x, curr_vx, dt)
        t_accel_y = self.pid_vel_y.update(t_vel_y, curr_vy, dt)
        t_accel_z = self.pid_vel_z.update(t_vel_z, curr_vz, dt)
        
        # Z-Axis Logic
        total_thrust = self.params.m * (self.params.g + t_accel_z)
        
        # --- ROTATION & MAPPING ---
        # Rotate Inertial Accelerations to Body Frame
        c_psi, s_psi = np.cos(curr_psi), np.sin(curr_psi)
        
        accel_forward = c_psi * t_accel_x + s_psi * t_accel_y
        accel_right   = -s_psi * t_accel_x + c_psi * t_accel_y
        
        # --- FIX: SIGNS FLIPPED TO MATCH PHYSICS ENGINE ---
        # Based on your Rotation Matrix: 
        # +Theta creates +X Force. We want +X Accel -> +Theta.
        target_theta = accel_forward / self.params.g
        
        # +Phi creates -Y Force. We want +Y Accel -> -Phi.
        target_phi   = -accel_right / self.params.g
        
        # Clamp angles
        target_theta = np.clip(target_theta, -0.5, 0.5)
        target_phi   = np.clip(target_phi, -0.5, 0.5)

        # --- LAYER 3: ATTITUDE CONTROL ---
        tau_phi = self.pid_roll.update(target_phi, curr_phi, dt)
        tau_theta = self.pid_pitch.update(target_theta, curr_theta, dt)
        
        yaw_error = target_yaw - curr_psi
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
        tau_psi = self.pid_yaw.update(0, 0, dt, error_override=yaw_error)
        
        # --- MIXING ---
        k, L, b = self.params.k, self.params.L, self.params.b
        t_term = total_thrust / (4 * k)
        r_term = tau_phi / (2 * k * L)
        p_term = tau_theta / (2 * k * L)
        y_term = tau_psi / (4 * b)
        
        return np.array([
            t_term + r_term + y_term,
            t_term + p_term - y_term,
            t_term - r_term + y_term,
            t_term - p_term - y_term
        ])