# Author: C.RUNNION
# Date: 2023-11-15
# Description: Main flight control loop for Amaltheia drone (HWIL Version)

import amaltheia_controller as Amalcontroller
import amaltheia_mission_manager as mission_manager
import amaltheia_mission_components as mission_components
from amaltheia_helper import quad_dynamics_derivative, motor_model
import numpy as np
import time
import serial
import threading 
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==========================================
# 1. Constants & State
# ==========================================
class PhysicalConstants:
    def __init__(self):
        self.m = 5.0          
        self.L = 0.30         
        self.g = 9.81         
        self.k = 1e-6         
        self.b = 1e-7         
        self.kd = 0.25        
        ixx = 2 * self.m * self.L**2 
        iyy = 2 * self.m * self.L**2 
        izz = 4 * self.m * self.L**2 
        self.I = np.diag([ixx, iyy, izz]) 
        self.I_inv = np.linalg.inv(self.I) 
        self.max_rpm = 12000.0 

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        
        # Real-time Snapshots
        self.gyro = np.zeros(3)
        self.lidar_dist = 0.0
        self.dt_mcu = 0.0
        self.mission = None
        
        # History Buffers (For Plotting)
        self.history_len = 200 
        self.t_hist = deque(maxlen=self.history_len)
        
        # Position 
        self.x_hist = deque(maxlen=self.history_len)
        self.y_hist = deque(maxlen=self.history_len)
        self.z_hist = deque(maxlen=self.history_len)
        self.tx_hist = deque(maxlen=self.history_len)
        self.ty_hist = deque(maxlen=self.history_len)
        self.tz_hist = deque(maxlen=self.history_len)
        
        # Attitude 
        self.roll_hist = deque(maxlen=self.history_len)
        self.pitch_hist = deque(maxlen=self.history_len)
        self.yaw_hist = deque(maxlen=self.history_len)

# ==========================================
# 2. The Engine (Physics/Control Loop)
# ==========================================
def hwil_engine_thread(shared_state, ser_port):
    print("Physics/Control Logic: Thread Started.")
    
    # Init Components
    params = PhysicalConstants()
    ctrl = Amalcontroller.QuadcopterController(params)
    manager = mission_manager.MissionManager(shared_state.mission)
    
    # Init Vectors
    state = np.zeros(12) 
    motor_speeds = np.zeros(4)
    start_time = time.time()
    
    # Open Serial
    try:
        ser = serial.Serial(ser_port, 115200, timeout=1)
        ser.reset_input_buffer()
        print(f"Engine: Connected to {ser_port}")
    except Exception as e:
        print(f"Engine: Serial Error - {e}")
        shared_state.running = False
        return

    while shared_state.running:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # PARSE DATA from STM32
                # Expected Format: DATA,gx,gy,gz,ax,ay,az,mx,my,mz,dist,roll,pitch,yaw,dt
                if line.startswith("DATA,"):
                    parts = line.replace("DATA,", "").split(',')
                    
                    # We need at least 14 items (indices 0 to 13)
                    if len(parts) >= 14:
                        # 1. Parse Real Data
                        g_x, g_y, g_z = float(parts[0]), float(parts[1]), float(parts[2])
                        dist_cm = float(parts[9])
                        
                        # Attitude Estimates from STM32 (Kalman)
                        stm_roll  = float(parts[10])
                        stm_pitch = float(parts[11])
                        stm_yaw   = float(parts[12])
                        
                        dt = float(parts[13]) # <--- Index 13 is Time

                        # Sanity Check DT
                        if dt <= 0.0001 or dt > 0.5: dt = 0.01

                        # Update Sim Time
                        curr_t = time.time() - start_time

                        # --- HWIL INJECTION (THE CRITICAL PART) ---
                        # Overwrite the Physics State with Real Sensor Data
                        
                        # Z-Axis: Use Lidar
                        state[2] = dist_cm / 100.0 
                        
                        # Attitude: Use STM32 Kalman Filter Output
                        # Convert Degrees to Radians for the Controller
                        state[6] = np.radians(stm_roll)
                        state[7] = np.radians(stm_pitch)
                        state[8] = np.radians(stm_yaw)

                        # --- GUIDANCE ---
                        target, ff_vel, dist_err, idx = manager.get_target(curr_t, state)

                        # --- CONTROL ---
                        # Compute motor commands based on REAL attitude and REAL altitude
                        inputs = ctrl.compute_inputs(target, ff_vel, state, dt)

                        # --- PHYSICS (Simulation Step) ---
                        # Calculate dynamics for X/Y (Virtual) based on real Z/Attitude
                        # We still integrate to see where the drone *would* go laterally
                        motor_speeds = motor_model(inputs, motor_speeds, dt)
                        d_state = quad_dynamics_derivative(state, motor_speeds, params)
                        
                        # Integrate State
                        state += d_state * dt
                        
                        # Re-Enforce Floor Constraint
                        if state[2] < 0: state[2] = 0; state[5] = 0

                        # --- UPDATE SHARED STATE (For Plotting) ---
                        with shared_state.lock:
                            shared_state.gyro = np.array([g_x, g_y, g_z])
                            shared_state.lidar_dist = dist_cm
                            shared_state.dt_mcu = dt
                            
                            shared_state.t_hist.append(curr_t)
                            
                            # Plot Virtual Position vs Target
                            shared_state.x_hist.append(state[0])
                            shared_state.y_hist.append(state[1])
                            shared_state.z_hist.append(state[2]) # Real Z
                            
                            shared_state.tx_hist.append(target[0])
                            shared_state.ty_hist.append(target[1])
                            shared_state.tz_hist.append(target[2])
                            
                            # Plot Real Attitude
                            shared_state.roll_hist.append(stm_roll)
                            shared_state.pitch_hist.append(stm_pitch)
                            shared_state.yaw_hist.append(stm_yaw)

            except ValueError:
                continue
    
    ser.close()
    print("Engine: Thread Stopped.")

# ==========================================
# 3. Main (Visualization)
# ==========================================
def main():
    # Setup
    state = SharedState()
    
    # Load Mission 
    # Make sure PIL_Hover_1.json exists!
    try:
        loaded_plan = mission_components.MissionLoader.load_from_json("PIL_Hover_1.json")
        state.mission = loaded_plan
    except Exception as e:
        print(f"Error loading mission: {e}")
        return

    # Start Engine
    # CHANGE 'COM3' to your actual port if needed
    engine = threading.Thread(target=hwil_engine_thread, args=(state, 'COM3'), daemon=True)
    engine.start()

    # --- SAFETY CHECK ---
    # Wait to see if Serial connects. If not, exit before Plotting.
    print("Waiting for Serial Connection...")
    time.sleep(1.5)
    if not state.running:
        print("Main: Engine failed to start (Check COM Port / Permissions). Exiting.")
        return
    # --------------------

    # Setup Plot
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("HWIL Flight Dashboard")
    
    # Ax0: Position
    ax_pos = axs[0, 0]
    l_pos, = ax_pos.plot([], [], 'b-', label='Est. Pos')
    l_tpos, = ax_pos.plot([], [], 'r--', label='Target')
    ax_pos.set_title("Lateral Position (Virtual X/Y)")
    ax_pos.legend(loc='upper right')
    ax_pos.grid(True); ax_pos.axis('equal')

    # Ax1: Altitude
    ax_alt = axs[0, 1]
    l_z, = ax_alt.plot([], [], 'b-', label='Lidar Z')
    l_tz, = ax_alt.plot([], [], 'r--', label='Target Z')
    ax_alt.set_title("Altitude (Real Z)")
    ax_alt.legend(loc='upper right'); ax_alt.grid(True)

    # Ax2: Attitude
    ax_att = axs[1, 0]
    l_roll, = ax_att.plot([], [], 'r-', label='Roll')
    l_pitch, = ax_att.plot([], [], 'g-', label='Pitch')
    ax_att.set_title("Real Attitude (STM32)")
    ax_att.set_ylim(-30, 30)
    ax_att.legend(loc='upper right'); ax_att.grid(True)

    # Ax3: Yaw
    ax_yaw = axs[1, 1]
    l_yaw, = ax_yaw.plot([], [], 'k-', label='Yaw')
    ax_yaw.set_title("Real Heading (Yaw)")
    ax_yaw.legend(loc='upper right'); ax_yaw.grid(True)

    def update_plot(frame):
        if not state.running:
            plt.close(fig)
            return
        
        with state.lock:
            t = list(state.t_hist)
            x = list(state.x_hist); tx = list(state.tx_hist)
            y = list(state.y_hist); ty = list(state.ty_hist)
            z = list(state.z_hist); tz = list(state.tz_hist)
            r = list(state.roll_hist)
            p = list(state.pitch_hist)
            ya = list(state.yaw_hist)
        
        if len(t) > 0:
            l_pos.set_data(x, y)
            l_tpos.set_data(tx, ty)
            ax_pos.relim(); ax_pos.autoscale_view()

            l_z.set_data(t, z)
            l_tz.set_data(t, tz)
            ax_alt.set_xlim(t[-1] - 5, t[-1] + 0.5)
            ax_alt.set_ylim(0, max(max(z), max(tz)) + 0.5)

            l_roll.set_data(t, r)
            l_pitch.set_data(t, p)
            ax_att.set_xlim(t[-1] - 5, t[-1] + 0.5)
            
            l_yaw.set_data(t, ya)
            ax_yaw.set_xlim(t[-1] - 5, t[-1] + 0.5)
            ax_yaw.set_ylim(min(ya)-10, max(ya)+10)

        return l_pos, l_z, l_roll, l_pitch, l_yaw

    ani = FuncAnimation(fig, update_plot, interval=50) 
    
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        engine.join()
        print("Main: System Exit.")

if __name__ == "__main__":
    main()