import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from collections import deque
import argparse
import time

# --- Configuration ---
# Formula for this data is MAX_POINTS * refesh_interval (ms) / 1000 = total time window in Seconds
MAX_POINTS = 1000  # Window size for plots
MAX_TERMINAL_LINES = 15 # Raw data lines to show

# --- Data Storage ---
time_data = deque(maxlen=MAX_POINTS)

# IMU Data
gyro_x = deque(maxlen=MAX_POINTS)
gyro_y = deque(maxlen=MAX_POINTS)
gyro_z = deque(maxlen=MAX_POINTS)

accel_x = deque(maxlen=MAX_POINTS)
accel_y = deque(maxlen=MAX_POINTS)
accel_z = deque(maxlen=MAX_POINTS)

mag_x = deque(maxlen=MAX_POINTS)
mag_y = deque(maxlen=MAX_POINTS)
mag_z = deque(maxlen=MAX_POINTS)

# Range Data
range_data = deque(maxlen=MAX_POINTS)

# Terminal Buffer
raw_lines = deque(maxlen=MAX_TERMINAL_LINES)

# --- Serial Port ---
ser = None
start_time = 0

def update_plot(frame):
    """Called periodically by FuncAnimation to read serial and update graphs."""
    try:
        # 1. Read all waiting data to prevent buffer lag
        if ser.in_waiting:
            # Read block, decode, split into lines
            raw_block = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            lines = raw_block.splitlines()
            
            for line in lines:
                # Store raw line for the "Terminal" view
                if line.strip():
                    raw_lines.append(line)

                # Parse: "DATA,gx,gy,gz,ax,ay,az,mx,my,mz,dist,dt"
                if line.startswith("DATA,"):
                    try:
                        # Strip "DATA," and split by comma
                        parts = line.replace("DATA,", "").strip().split(',')
                        
                        # We expect 11 values based on your C code
                        if len(parts) >= 11:
                            # Parse Gyro (Indices 0, 1, 2)
                            g_x = float(parts[0])
                            g_y = float(parts[1])
                            g_z = float(parts[2])
                            
                            ax_g = float(parts[3])  # Not used currently
                            ay_g = float(parts[4])  # Not used currently
                            az_g = float(parts[5])  # Not used currently
                            
                            mx_g = float(parts[6])  # Not used currently
                            my_g = float(parts[7])  # Not used currently
                            mz_g = float(parts[8])  # Not used currently
                            
                            # Parse Distance (Index 9)
                            dist = float(parts[9])
                            
                            dt_mcu = float(parts[10]) 

                            # Append to Buffers
                            current_time = time.time() - start_time
                            time_data.append(current_time)
                            
                            gyro_x.append(g_x)
                            gyro_y.append(g_y)
                            gyro_z.append(g_z)
                            
                            accel_x.append(ax_g)
                            accel_y.append(ay_g)
                            accel_z.append(az_g)
                            
                            mag_x.append(mx_g)
                            mag_y.append(my_g)
                            mag_z.append(mz_g)
                            
                            range_data.append(dist)
                            
                    except ValueError:
                        continue # Skip corrupted lines

        # 2. Update Plot Data
        if len(time_data) > 0:
            # Gyro Lines
            line_gx.set_data(time_data, gyro_x)
            line_gy.set_data(time_data, gyro_y)
            line_gz.set_data(time_data, gyro_z)
            
            # Range Line
            line_range.set_data(time_data, range_data)

            # Auto-scale Axes
            ax1.relim()
            ax1.autoscale_view()
            
            ax2.relim()
            ax2.autoscale_view()
            
            #ax3.relim()
            #ax3.autoscale_view()
            
            # Keep X-axis scrolling
            ax1.set_xlim(max(0, time_data[-1] - 10), time_data[-1] + 0.5)
            ax2.set_xlim(max(0, time_data[-1] - 10), time_data[-1] + 0.5)

        # 3. Update Terminal View
        ax_term.clear()
        ax_term.set_title('Raw Serial Monitor')
        ax_term.set_xticks([])
        ax_term.set_yticks([])
        ax_term.axis('off')
        
        # Draw text block
        text_content = '\n'.join(raw_lines)
        ax_term.text(0.01, 0.99, text_content, transform=ax_term.transAxes, 
                     va='top', ha='left', family='monospace', fontsize=8)

    except Exception as e:
        print(f"Update Error: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='STM32 Flight Controller Dashboard')
    parser.add_argument('port', help='COM port (e.g., COM4)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.01)
        print(f"Connected to {args.port} at {args.baud} baud.")
        start_time = time.time()
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        exit(1)


    # --- 3D Plotting Setup ---
    
    # fig3d = plt.figure(figsize=(8, 6))
    # ax3 = fig3d.add_subplot(111, projection='3d')
    # ax3.set_title('3D Attitude Visualization')
    # ax3.set_xlabel('X Axis Degrees')
    # ax3.set_ylabel('Y Axis Degrees')
    # ax3.set_zlabel('Z Axis Degrees')
    # ax3.set_xlim([-180, 180])
    # ax3.set_ylim([-180, 180])
    # ax3.set_zlim([-180, 180])
    # attitude_line, = ax3.plot([], [], [], 'b-', lw=2)
    
    
    # --- Plot Layout ---
    
    
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle(f'Flight Controller Telemetry ({args.port})', fontsize=16)

    # Grid: 2 Rows, 2 Columns (Left col for plots, Right col for text)
    gs = gridspec.GridSpec(2, 2, width_ratios=[3, 1])

    # Plot 1: Gyroscope (Top Left)
    ax1 = fig.add_subplot(gs[0, 0]) 
    line_gx, = ax1.plot([], [], 'r-', label='Roll Rate (X)', alpha=0.8)
    line_gy, = ax1.plot([], [], 'g-', label='Pitch Rate (Y)', alpha=0.8)
    line_gz, = ax1.plot([], [], 'b-', label='Yaw Rate (Z)', alpha=0.8)
    ax1.set_ylabel('Degrees / sec')
    ax1.set_title('Gyroscope Activity')
    ax1.legend(loc='upper right')
    ax1.grid(True)

    # Plot 2: Rangefinder (Bottom Left)
    ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)
    line_range, = ax2.plot([], [], 'm-', lw=2, label='Lidar Dist')
    ax2.set_ylabel('Distance (cm)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Altitude (TFmini)')
    ax2.legend(loc='upper right')
    ax2.grid(True)

    # Terminal: Right Side (Spanning both rows)
    ax_term = fig.add_subplot(gs[:, 1])

    # Animation Loop
    ani = animation.FuncAnimation(fig, update_plot, interval=30, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if ser: ser.close()
        print("Connection Closed.")