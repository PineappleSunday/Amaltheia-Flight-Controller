

import numpy as np

# ==========================================
# 3. Helper Functions
# ==========================================
def rotation_matrix(phi, theta, psi):
    c_p, s_p = np.cos(phi), np.sin(phi)
    c_t, s_t = np.cos(theta), np.sin(theta)
    c_ps, s_ps = np.cos(psi), np.sin(psi)
    return np.array([
        [c_ps*c_t,  c_ps*s_t*s_p - s_ps*c_p,  c_ps*s_t*c_p + s_ps*s_p],
        [s_ps*c_t,  s_ps*s_t*s_p + c_ps*c_p,  s_ps*s_t*c_p - c_ps*s_p],
        [-s_t,      c_t*s_p,                  c_t*c_p]
    ])

def omega_to_thetadot(phi, theta, omega):
    c_p, s_p = np.cos(phi), np.sin(phi)
    c_t, s_t = np.cos(theta), np.sin(theta)
    if np.abs(c_t) < 1e-6: c_t = 1e-6
    T_inv = np.array([[1, s_p*s_t/c_t, c_p*s_t/c_t],[0, c_p, -s_p],[0, s_p/c_t, c_p/c_t]])
    return T_inv @ omega

def motor_model(desired_w_sq, current_w_sq, dt, motor_time_constant=0.04):
    alpha = dt / (motor_time_constant + dt)
    return current_w_sq * (1 - alpha) + desired_w_sq * alpha

def motor_mixing(inputs, params):
    max_w_sq = (params.max_rpm)**2
    inputs = np.clip(inputs, 0.0, max_w_sq)
    w1, w2, w3, w4 = inputs
    T = params.k * np.sum(inputs)
    tau_phi = params.L * params.k * (w1 - w3)
    tau_theta = params.L * params.k * (w2 - w4)
    tau_psi = params.b * (w1 - w2 + w3 - w4)
    return np.array([0, 0, T]), np.array([tau_phi, tau_theta, tau_psi])

def quad_dynamics_derivative(state, motor_inputs, params):
    x, y, z = state[0:3]
    v_inertial = state[3:6]
    phi, theta, psi = state[6:9]
    omega_body = state[9:12]
    thrust_body, torques_body = motor_mixing(motor_inputs, params)
    R = rotation_matrix(phi, theta, psi)
    thrust_inertial = R @ thrust_body
    gravity = np.array([0, 0, -params.m * params.g])
    drag = -params.kd * v_inertial
    accel = (thrust_inertial + gravity + drag) / params.m
    gyro = np.cross(omega_body, params.I @ omega_body)
    omega_dot = params.I_inv @ (torques_body - gyro)
    euler_rates = omega_to_thetadot(phi, theta, omega_body)
    return np.concatenate([v_inertial, accel, euler_rates, omega_dot])