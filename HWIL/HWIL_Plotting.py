# HWIL/HWIL_Plotting.py

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from collections import deque
import argparse
import time

# --- Configuration ---
MAX_POINTS = 500  # Keep ~5 seconds of data at 100Hz
MAX_TERMINAL_LINES = 10

# --- Data Buffers ---
time_data = deque(maxlen=MAX_POINTS)

# Raw Sensor Data
gyro_x = deque(maxlen=MAX_POINTS)
gyro_y = deque(maxlen=MAX_POINTS)
gyro_z = deque(maxlen=MAX_POINTS)

# Estimated State (From STM32 Kalman)
est_roll = deque(maxlen=MAX_POINTS)
est_pitch = deque(maxlen=MAX_POINTS)
est_yaw = deque(maxlen=MAX_POINTS)

# Altitude
range_data = deque(maxlen=MAX_POINTS)

# Terminal
raw_lines = deque(maxlen=MAX_TERMINAL_LINES)

# Global State
start_time = time.time()
ser = None

def update_plot(frame):
    """Reads serial buffer and updates all graphs."""
    if ser.in_waiting:
        try:
            # Read everything in buffer to stay real-time
            block = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            lines = block.splitlines()
            
            for line in lines:
                if not line.startswith("DATA,"):
                    continue
                
                # Update Terminal
                raw_lines.append(line)
                
                # Parse CSV: DATA,gx,gy,gz,ax,ay,az,mx,my,mz,dist,roll,pitch,yaw,dt
                parts = line.replace("DATA,", "").strip().split(',')
                print(parts)
                # We need at least 14 values (sensors + attitude + dt)
                if len(parts) >= 13: 
                    try:
                        # 1. Parse Gyro (0-2)
                        gx, gy, gz = float(parts[0]), float(parts[1]), float(parts[2])
                        
                        # 2. Parse Distance (Index 9)
                        dist = float(parts[9])
                        
                        # 3. Parse Attitude (Indices 10-12)
                        # Ensure your C code prints these!
                        roll, pitch, yaw = float(parts[10]), float(parts[11]), float(parts[12])

                        # DEBUG PRINT
                        print(f"Gyro: gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}")
                        print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
                        print(f"Distance: {dist:.2f} cm")
                        
                        # Store
                        t = time.time() - start_time
                        time_data.append(t)
                        
                        gyro_x.append(gx); gyro_y.append(gy); gyro_z.append(gz)
                        range_data.append(dist)
                        est_roll.append(roll); est_pitch.append(pitch); est_yaw.append(yaw)
                        
                    except ValueError:
                        continue # Skip bad lines

        except Exception as e:
            print(f"Serial Read Error: {e}")

    # --- Update Graphics ---
    if len(time_data) > 0:
        # 1. Gyro
        l_gx.set_data(time_data, gyro_x)
        l_gy.set_data(time_data, gyro_y)
        l_gz.set_data(time_data, gyro_z)
        ax_gyro.set_xlim(time_data[0], time_data[-1] + 0.2)
        ax_gyro.relim(); ax_gyro.autoscale_view()

        # 2. Attitude (Kalman)
        l_roll.set_data(time_data, est_roll)
        l_pitch.set_data(time_data, est_pitch)
        l_yaw.set_data(time_data, est_yaw)
        ax_att.set_xlim(time_data[0], time_data[-1] + 0.2)
        # Fix Y-axis for Attitude to see stability (-180 to 180 is too zoomed out usually)
        ax_att.set_ylim(-90, 90) 

        # 3. Altitude
        l_alt.set_data(time_data, range_data)
        ax_alt.set_xlim(time_data[0], time_data[-1] + 0.2)
        ax_alt.relim(); ax_alt.autoscale_view()

        # 4. Terminal
        ax_term.clear()
        ax_term.axis('off')
        ax_term.text(0, 1, '\n'.join(raw_lines), va='top', family='monospace', fontsize=8)

    return l_gx, l_gy, l_gz, l_roll, l_pitch, l_yaw, l_alt

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('port', help='COM port (e.g., COM4)')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"Connected to {args.port}")
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Error: {e}")
        exit(1)

    # --- Layout ---
    # 3 Rows, 2 Columns (Right column is terminal)
    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 2, width_ratios=[3, 1])

    # Plot 1: Gyro
    ax_gyro = fig.add_subplot(gs[0, 0])
    l_gx, = ax_gyro.plot([], [], 'r-', label='Roll Rate')
    l_gy, = ax_gyro.plot([], [], 'g-', label='Pitch Rate')
    l_gz, = ax_gyro.plot([], [], 'b-', label='Yaw Rate')
    ax_gyro.set_title("Gyroscope (Raw Rates)")
    ax_gyro.legend(loc='upper right')
    ax_gyro.grid(True)

    # Plot 2: Attitude (Kalman)
    ax_att = fig.add_subplot(gs[1, 0], sharex=ax_gyro)
    l_roll, = ax_att.plot([], [], 'r-', lw=2, label='Roll (Est)')
    l_pitch, = ax_att.plot([], [], 'g-', lw=2, label='Pitch (Est)')
    l_yaw, = ax_att.plot([], [], 'b-', lw=1, label='Yaw (Est)', alpha=0.5)
    ax_att.set_title("Attitude (STM32 Kalman)")
    ax_att.set_ylabel("Degrees")
    ax_att.legend(loc='upper right')
    ax_att.grid(True)

    # Plot 3: Altitude
    ax_alt = fig.add_subplot(gs[2, 0], sharex=ax_gyro)
    l_alt, = ax_alt.plot([], [], 'k-', lw=2, label='Lidar')
    ax_alt.set_title("Altitude (cm)")
    ax_alt.set_xlabel("Time (s)")
    ax_alt.grid(True)

    # Terminal
    ax_term = fig.add_subplot(gs[:, 1])
    ax_term.axis('off')

    ani = animation.FuncAnimation(fig, update_plot, interval=30, blit=False, cache_frame_data=False)
    
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt: pass
    finally: ser.close()