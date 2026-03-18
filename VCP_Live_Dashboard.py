import argparse
import csv
import os
import queue
import struct
import threading
import time
from collections import deque
from datetime import datetime

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import serial
import serial.tools.list_ports


HEADER_VALUE = 0x31504356  # "VCP1"
FOOTER_VALUE = 0xAB
FRAME_FORMAT = "<IIHBBBB" + ("f" * 52) + "B"
FRAME_STRUCT = struct.Struct(FRAME_FORMAT)
FRAME_SIZE = FRAME_STRUCT.size
HEADER_BYTES = struct.pack("<I", HEADER_VALUE)

FRAME_FIELDS = [
    "header",
    "tick_ms",
    "sequence",
    "armed",
    "drone_mode",
    "flight_mode",
    "sat_flags",
    "dt_sec",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "roll",
    "pitch",
    "yaw",
    "roll_rate",
    "pitch_rate",
    "yaw_rate",
    "accel_x",
    "accel_y",
    "accel_z",
    "mag_x",
    "mag_y",
    "mag_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "target_x",
    "target_y",
    "target_z",
    "target_roll",
    "target_pitch",
    "target_yaw",
    "target_rate_roll",
    "target_rate_pitch",
    "target_rate_yaw",
    "target_ff_vz",
    "pid_roll_p",
    "pid_roll_i",
    "pid_roll_d",
    "pid_roll_out",
    "pid_pitch_p",
    "pid_pitch_i",
    "pid_pitch_d",
    "pid_pitch_out",
    "pid_yaw_p",
    "pid_yaw_i",
    "pid_yaw_d",
    "pid_yaw_out",
    "pid_velz_p",
    "pid_velz_i",
    "pid_velz_d",
    "pid_velz_out",
    "motor1_pct",
    "motor2_pct",
    "motor3_pct",
    "motor4_pct",
    "magic_footer",
]

assert FRAME_SIZE == 223, f"Unexpected frame size: {FRAME_SIZE}"


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []

        while True:
            idx = self.buffer.find(HEADER_BYTES)
            if idx < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                break

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < FRAME_SIZE:
                break

            candidate = bytes(self.buffer[:FRAME_SIZE])
            if candidate[-1] != FOOTER_VALUE:
                self.bad_frames += 1
                del self.buffer[0]
                continue

            values = FRAME_STRUCT.unpack(candidate)
            frame = dict(zip(FRAME_FIELDS, values))
            if frame["header"] != HEADER_VALUE or frame["magic_footer"] != FOOTER_VALUE:
                self.bad_frames += 1
                del self.buffer[0]
                continue

            frames.append(frame)
            del self.buffer[:FRAME_SIZE]

        return frames


def auto_pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    scored = []
    for p in ports:
        text = f"{p.device} {p.description} {p.hwid}".lower()
        score = 0
        if "stlink" in text or "st-link" in text:
            score += 5
        if "vcp" in text:
            score += 4
        if "usb serial" in text:
            score += 3
        if "stm32" in text:
            score += 2
        scored.append((score, p.device))

    scored.sort(key=lambda x: (-x[0], x[1]))
    return scored[0][1]


def serial_reader(ser, out_q, stop_event, stats):
    parser = FrameParser()
    while not stop_event.is_set():
        try:
            chunk = ser.read(4096)
        except serial.SerialException:
            break

        if not chunk:
            continue

        stats["bytes_in"] += len(chunk)
        frames = parser.feed(chunk)
        stats["bad_frames"] = parser.bad_frames

        for frame in frames:
            stats["frames_parsed"] += 1
            try:
                out_q.put_nowait(frame)
            except queue.Full:
                stats["queue_drops"] += 1


def build_status_text(state):
    latest = state["latest"]
    port_line = state["port"] if state["port"] else "(not selected)"
    conn_state = "Connected" if state["connected"] else "Disconnected"
    ports_preview = ", ".join(state["ports"][:6]) if state["ports"] else "none"

    if latest is None:
        return (
            f"Connection: {conn_state}\n"
            f"Port: {port_line} @ {state['baud']}\n"
            f"Detected Ports: {ports_preview}\n"
            f"{state['conn_msg']}\n"
            "Waiting for frames...\n"
            "\n"
            f"Frames parsed: {state['frames_seen']}\n"
            f"Queue drops: {state['queue_drops']}\n"
            f"Bad frames: {state['bad_frames']}\n"
        )

    return (
        f"Connection: {conn_state}\n"
        f"Port: {port_line} @ {state['baud']}\n"
        f"Detected Ports: {ports_preview}\n"
        f"{state['conn_msg']}\n"
        f"FPS: {state['fps']:.1f}\n"
        f"Frames parsed: {state['frames_seen']}\n"
        f"Seq drops: {state['seq_drops']}\n"
        f"Queue drops: {state['queue_drops']}\n"
        f"Bad frames: {state['bad_frames']}\n"
        "\n"
        f"Armed: {latest['armed']}  Mode: {latest['drone_mode']}  FM: {latest['flight_mode']}\n"
        f"Sat: 0b{latest['sat_flags']:08b}\n"
        f"dt: {latest['dt_sec']:.4f}s\n"
        "\n"
        f"AHRS  R/P/Y: {latest['roll']:+7.2f}, {latest['pitch']:+7.2f}, {latest['yaw']:+7.2f}\n"
        f"Rate  p/q/r: {latest['roll_rate']:+7.2f}, {latest['pitch_rate']:+7.2f}, {latest['yaw_rate']:+7.2f}\n"
        f"Cmd r/p/y: {latest['target_rate_roll']:+7.2f}, {latest['target_rate_pitch']:+7.2f}, {latest['target_rate_yaw']:+7.2f}\n"
        "\n"
        f"PID Roll  P/I/D/O: {latest['pid_roll_p']:+6.3f} {latest['pid_roll_i']:+6.3f} {latest['pid_roll_d']:+6.3f} {latest['pid_roll_out']:+6.3f}\n"
        f"PID Pitch P/I/D/O: {latest['pid_pitch_p']:+6.3f} {latest['pid_pitch_i']:+6.3f} {latest['pid_pitch_d']:+6.3f} {latest['pid_pitch_out']:+6.3f}\n"
        f"PID Yaw   P/I/D/O: {latest['pid_yaw_p']:+6.3f} {latest['pid_yaw_i']:+6.3f} {latest['pid_yaw_d']:+6.3f} {latest['pid_yaw_out']:+6.3f}\n"
        "\n"
        f"Motors %: {latest['motor1_pct']:.1f}, {latest['motor2_pct']:.1f}, {latest['motor3_pct']:.1f}, {latest['motor4_pct']:.1f}\n"
        f"Z/VZ: {latest['z']:+.3f} m / {latest['vz']:+.3f} m/s\n"
        f"Target Z/FFvz: {latest['target_z']:+.3f} m / {latest['target_ff_vz']:+.3f}\n"
    )


def main():
    # 1. Get the current date and time object
    now = datetime.now()

    # 2. Format as a string (YYYY-MM-DD HH:MM:SS)
    date_string = now.strftime("%Y-%m-%d %H:%M:%S")
    parser = argparse.ArgumentParser(description="Amaltheia VCP live dashboard + logger")
    parser.add_argument("port", nargs="?", help="Initial COM port (optional)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--window-sec", type=float, default=15.0, help="Plot history window in seconds")
    parser.add_argument("--update-ms", type=int, default=40, help="Plot refresh interval")
    parser.add_argument("--queue-size", type=int, default=4096, help="Frame queue size")
    parser.add_argument("--no-log", action="store_true", help="Disable CSV logging")
    parser.add_argument("--log-dir", default=f"flight_log_VCP", help="CSV output directory")
    args = parser.parse_args()

    initial_port = args.port or auto_pick_port() or ""
    print("Dashboard started in offline mode. Use the GUI controls to connect.")
    print(f"Frame size: {FRAME_SIZE} bytes")

    max_points = max(200, int(args.window_sec * 200))
    t = deque(maxlen=max_points)
    roll = deque(maxlen=max_points)
    pitch = deque(maxlen=max_points)
    yaw = deque(maxlen=max_points)
    roll_rate = deque(maxlen=max_points)
    pitch_rate = deque(maxlen=max_points)
    yaw_rate = deque(maxlen=max_points)
    target_roll_rate = deque(maxlen=max_points)
    target_pitch_rate = deque(maxlen=max_points)
    target_yaw_rate = deque(maxlen=max_points)
    pid_roll_out = deque(maxlen=max_points)
    pid_pitch_out = deque(maxlen=max_points)
    pid_yaw_out = deque(maxlen=max_points)
    motor1 = deque(maxlen=max_points)
    motor2 = deque(maxlen=max_points)
    motor3 = deque(maxlen=max_points)
    motor4 = deque(maxlen=max_points)
    z = deque(maxlen=max_points)
    target_z = deque(maxlen=max_points)
    vz = deque(maxlen=max_points)
    ff_vz = deque(maxlen=max_points)

    csv_file = None
    csv_writer = None
    csv_flush_counter = 0
    if not args.no_log:
        os.makedirs(args.log_dir, exist_ok=True)
        log_name = datetime.now().strftime("vcp_dump_%Y%m%d_%H%M%S.csv")
        log_path = os.path.join(args.log_dir, log_name)
        csv_file = open(log_path, "w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(csv_file, fieldnames=["host_time_s"] + FRAME_FIELDS)
        csv_writer.writeheader()
        print(f"Logging CSV: {log_path}")

    frame_q = queue.Queue(maxsize=args.queue_size)
    reader_stats = {"bytes_in": 0, "frames_parsed": 0, "queue_drops": 0, "bad_frames": 0}
    ser = None
    reader_thread = None
    reader_stop_event = None

    start = time.perf_counter()
    last_seq = None
    fps_last_t = start
    fps_last_n = 0

    state = {
        "port": initial_port,
        "baud": args.baud,
        "frames_seen": 0,
        "seq_drops": 0,
        "queue_drops": 0,
        "bad_frames": 0,
        "fps": 0.0,
        "latest": None,
        "connected": False,
        "ports": [],
        "conn_msg": "Select a COM port and click Connect.",
    }

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Amaltheia VCP Live Telemetry", fontsize=14)
    fig.subplots_adjust(bottom=0.14)
    gs = gridspec.GridSpec(5, 2, width_ratios=[4.0, 1.8])

    ax_att = fig.add_subplot(gs[0, 0])
    l_roll, = ax_att.plot([], [], "r-", label="Roll")
    l_pitch, = ax_att.plot([], [], "g-", label="Pitch")
    l_yaw, = ax_att.plot([], [], "b-", alpha=0.7, label="Yaw")
    ax_att.set_ylabel("deg")
    ax_att.set_title("AHRS Attitude")
    ax_att.set_ylim(-180.0, 180.0)
    ax_att.grid(True)
    ax_att.legend(loc="upper right")

    ax_rate = fig.add_subplot(gs[1, 0], sharex=ax_att)
    l_rr, = ax_rate.plot([], [], "r-", label="Roll Rate")
    l_pr, = ax_rate.plot([], [], "g-", label="Pitch Rate")
    l_yr, = ax_rate.plot([], [], "b-", label="Yaw Rate")
    l_rr_cmd, = ax_rate.plot([], [], "r--", alpha=0.5, label="Roll Cmd")
    l_pr_cmd, = ax_rate.plot([], [], "g--", alpha=0.5, label="Pitch Cmd")
    l_yr_cmd, = ax_rate.plot([], [], "b--", alpha=0.5, label="Yaw Cmd")
    ax_rate.set_ylabel("deg/s")
    ax_rate.set_title("Rate Loop Tracking")
    ax_rate.grid(True)
    ax_rate.legend(loc="upper right", ncol=2, fontsize=8)

    ax_pid = fig.add_subplot(gs[2, 0], sharex=ax_att)
    l_pid_r, = ax_pid.plot([], [], "r-", label="PID Roll Out")
    l_pid_p, = ax_pid.plot([], [], "g-", label="PID Pitch Out")
    l_pid_y, = ax_pid.plot([], [], "b-", label="PID Yaw Out")
    ax_pid.set_ylabel("PID out")
    ax_pid.set_title("Inner Loop PID Outputs")
    ax_pid.grid(True)
    ax_pid.legend(loc="upper right")

    ax_mot = fig.add_subplot(gs[3, 0], sharex=ax_att)
    l_m1, = ax_mot.plot([], [], "r-", label="M1")
    l_m2, = ax_mot.plot([], [], "g-", label="M2")
    l_m3, = ax_mot.plot([], [], "b-", label="M3")
    l_m4, = ax_mot.plot([], [], "k-", label="M4")
    ax_mot.set_ylabel("Motor %")
    ax_mot.set_ylim(-5.0, 105.0)
    ax_mot.set_title("Motor Outputs")
    ax_mot.grid(True)
    ax_mot.legend(loc="upper right", ncol=4, fontsize=8)

    ax_alt = fig.add_subplot(gs[4, 0], sharex=ax_att)
    l_z, = ax_alt.plot([], [], "m-", label="Z")
    l_tz, = ax_alt.plot([], [], "m--", alpha=0.7, label="Target Z")
    l_vz, = ax_alt.plot([], [], "c-", label="VZ")
    l_ffvz, = ax_alt.plot([], [], "c--", alpha=0.7, label="FF VZ")
    ax_alt.set_ylabel("m / m/s")
    ax_alt.set_xlabel("Host Time (s)")
    ax_alt.set_title("Altitude / Vertical Dynamics")
    ax_alt.grid(True)
    ax_alt.legend(loc="upper right", ncol=2, fontsize=8)

    ax_text = fig.add_subplot(gs[:, 1])
    ax_text.axis("off")
    status_text = ax_text.text(
        0.01,
        0.99,
        "",
        va="top",
        ha="left",
        family="monospace",
        fontsize=9,
    )

    # Bottom control strip: port selection and connection controls.
    ax_port_box = fig.add_axes([0.72, 0.02, 0.14, 0.04])
    port_box = TextBox(ax_port_box, "Port", initial=initial_port)
    ax_btn_connect = fig.add_axes([0.87, 0.02, 0.06, 0.04])
    ax_btn_disconnect = fig.add_axes([0.94, 0.02, 0.05, 0.04])
    ax_btn_refresh = fig.add_axes([0.72, 0.07, 0.14, 0.04])
    btn_connect = Button(ax_btn_connect, "Connect")
    btn_disconnect = Button(ax_btn_disconnect, "Disconnect")
    btn_refresh = Button(ax_btn_refresh, "Refresh Ports")

    def refresh_ports(_event=None):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        state["ports"] = ports
        if ports:
            state["conn_msg"] = f"Ports: {', '.join(ports[:6])}"
            if not port_box.text.strip():
                port_box.set_val(ports[0])
        else:
            state["conn_msg"] = "No COM ports detected."

    def disconnect_serial(_event=None):
        nonlocal ser, reader_thread, reader_stop_event
        if reader_stop_event is not None:
            reader_stop_event.set()
            reader_stop_event = None
        if reader_thread is not None:
            try:
                reader_thread.join(timeout=1.0)
            except RuntimeError:
                pass
            reader_thread = None
        if ser is not None:
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                pass
            ser = None
        state["connected"] = False
        state["conn_msg"] = "Disconnected."

    def connect_serial(_event=None):
        nonlocal ser, reader_thread, reader_stop_event, last_seq, fps_last_t, fps_last_n
        selected_port = port_box.text.strip()
        if not selected_port:
            state["conn_msg"] = "Enter/select a COM port first."
            return

        disconnect_serial()

        try:
            ser = serial.Serial(selected_port, args.baud, timeout=0.02)
        except serial.SerialException as exc:
            state["conn_msg"] = f"Open failed: {exc}"
            state["port"] = selected_port
            return

        while True:
            try:
                frame_q.get_nowait()
            except queue.Empty:
                break

        reader_stats["bytes_in"] = 0
        reader_stats["frames_parsed"] = 0
        reader_stats["queue_drops"] = 0
        reader_stats["bad_frames"] = 0

        reader_stop_event = threading.Event()
        reader_thread = threading.Thread(
            target=serial_reader,
            args=(ser, frame_q, reader_stop_event, reader_stats),
            daemon=True,
        )
        reader_thread.start()

        state["port"] = selected_port
        state["connected"] = True
        state["conn_msg"] = f"Connected to {selected_port}"
        last_seq = None
        fps_last_t = time.perf_counter()
        fps_last_n = state["frames_seen"]

    btn_connect.on_clicked(connect_serial)
    btn_disconnect.on_clicked(disconnect_serial)
    btn_refresh.on_clicked(refresh_ports)
    refresh_ports()

    def update(_):
        nonlocal last_seq, csv_flush_counter, fps_last_t, fps_last_n

        drained = 0
        while drained < 1500:
            try:
                frame = frame_q.get_nowait()
            except queue.Empty:
                break

            now_s = time.perf_counter() - start
            seq = frame["sequence"]
            if last_seq is not None:
                expected = (last_seq + 1) & 0xFFFF
                if seq != expected:
                    state["seq_drops"] += (seq - expected) & 0xFFFF
            last_seq = seq

            state["frames_seen"] += 1
            state["latest"] = frame

            t.append(now_s)
            roll.append(frame["roll"])
            pitch.append(frame["pitch"])
            yaw.append(frame["yaw"])
            roll_rate.append(frame["roll_rate"])
            pitch_rate.append(frame["pitch_rate"])
            yaw_rate.append(frame["yaw_rate"])
            target_roll_rate.append(frame["target_rate_roll"])
            target_pitch_rate.append(frame["target_rate_pitch"])
            target_yaw_rate.append(frame["target_rate_yaw"])
            pid_roll_out.append(frame["pid_roll_out"])
            pid_pitch_out.append(frame["pid_pitch_out"])
            pid_yaw_out.append(frame["pid_yaw_out"])
            motor1.append(frame["motor1_pct"])
            motor2.append(frame["motor2_pct"])
            motor3.append(frame["motor3_pct"])
            motor4.append(frame["motor4_pct"])
            z.append(frame["z"])
            target_z.append(frame["target_z"])
            vz.append(frame["vz"])
            ff_vz.append(frame["target_ff_vz"])

            if csv_writer is not None:
                row = {"host_time_s": now_s}
                row.update(frame)
                csv_writer.writerow(row)
                csv_flush_counter += 1
                if csv_flush_counter >= 100:
                    csv_file.flush()
                    csv_flush_counter = 0

            drained += 1

        now_perf = time.perf_counter()
        elapsed = now_perf - fps_last_t
        if elapsed >= 0.5:
            state["fps"] = (state["frames_seen"] - fps_last_n) / elapsed
            fps_last_n = state["frames_seen"]
            fps_last_t = now_perf

        state["queue_drops"] = reader_stats["queue_drops"]
        state["bad_frames"] = reader_stats["bad_frames"]

        if len(t) > 0:
            l_roll.set_data(t, roll)
            l_pitch.set_data(t, pitch)
            l_yaw.set_data(t, yaw)

            l_rr.set_data(t, roll_rate)
            l_pr.set_data(t, pitch_rate)
            l_yr.set_data(t, yaw_rate)
            l_rr_cmd.set_data(t, target_roll_rate)
            l_pr_cmd.set_data(t, target_pitch_rate)
            l_yr_cmd.set_data(t, target_yaw_rate)

            l_pid_r.set_data(t, pid_roll_out)
            l_pid_p.set_data(t, pid_pitch_out)
            l_pid_y.set_data(t, pid_yaw_out)

            l_m1.set_data(t, motor1)
            l_m2.set_data(t, motor2)
            l_m3.set_data(t, motor3)
            l_m4.set_data(t, motor4)

            l_z.set_data(t, z)
            l_tz.set_data(t, target_z)
            l_vz.set_data(t, vz)
            l_ffvz.set_data(t, ff_vz)

            tmax = t[-1]
            tmin = max(0.0, tmax - args.window_sec)
            for ax in (ax_att, ax_rate, ax_pid, ax_mot, ax_alt):
                ax.set_xlim(tmin, tmax + 0.05)

            for ax in (ax_rate, ax_pid, ax_alt):
                ax.relim()
                ax.autoscale_view()

        state["port"] = port_box.text.strip()
        status_text.set_text(build_status_text(state))

        return (
            l_roll,
            l_pitch,
            l_yaw,
            l_rr,
            l_pr,
            l_yr,
            l_rr_cmd,
            l_pr_cmd,
            l_yr_cmd,
            l_pid_r,
            l_pid_p,
            l_pid_y,
            l_m1,
            l_m2,
            l_m3,
            l_m4,
            l_z,
            l_tz,
            l_vz,
            l_ffvz,
            status_text,
        )

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=max(10, args.update_ms),
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        disconnect_serial()
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        del ani
        print("Closed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
