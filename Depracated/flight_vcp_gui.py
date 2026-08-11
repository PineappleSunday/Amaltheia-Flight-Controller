#!/usr/bin/env python3
"""
Tkinter GUI for Amaltheia VCP telemetry stream.

This tool reads the binary VCP packet from firmware:
  header = 0x31504356 ("VCP1")
  payload = fixed 223-byte packed struct
  footer = 0xAB

It provides:
  - Live AHRS / rate / PID / motor plots
  - Live key-value telemetry panel (roll/pitch/yaw, PID, tracking errors, status)
  - In-memory logging of every decoded field
  - CSV export for post-analysis
"""

from __future__ import annotations

import csv
import datetime as dt
import math
import struct
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk
import matplotlib.pyplot as plt
try:
    import serial
    from serial.tools import list_ports
except Exception as exc:  # pragma: no cover
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = str(exc)
else:
    SERIAL_IMPORT_ERROR = ""

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


HEADER_VALUE = 0x31504356
FOOTER_VALUE = 0xAB
FRAME_FORMAT = "<IIHBBBB" + ("f" * 65) + "B"
BME_FRAME_FORMAT = "<IIHBBBB" + ("f" * 57) + "B"
LEGACY_FRAME_FORMAT = "<IIHBBBB" + ("f" * 52) + "B"
FRAME_STRUCT = struct.Struct(FRAME_FORMAT)
BME_FRAME_STRUCT = struct.Struct(BME_FRAME_FORMAT)
LEGACY_FRAME_STRUCT = struct.Struct(LEGACY_FRAME_FORMAT)
FRAME_SIZE = FRAME_STRUCT.size
BME_FRAME_SIZE = BME_FRAME_STRUCT.size
LEGACY_FRAME_SIZE = LEGACY_FRAME_STRUCT.size
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
    "bme280_valid",
    "bme280_temp_c",
    "bme280_pressure_pa",
    "bme280_humidity_rh",
    "bme280_altitude_m",
    "gps_valid",
    "gps_sats",
    "gps_lat_deg",
    "gps_lon_deg",
    "gps_alt_m",
    "gps_speed_mps",
    "gps_course_deg",
    "gps_hdop",
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

BME_FRAME_FIELDS = [field for field in FRAME_FIELDS if not field.startswith("gps_")]
LEGACY_FRAME_FIELDS = [field for field in BME_FRAME_FIELDS if not field.startswith("bme280_")]

assert FRAME_SIZE == 275
assert BME_FRAME_SIZE == 243
assert LEGACY_FRAME_SIZE == 223


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, chunk: bytes) -> list[dict[str, float | int]]:
        self.buffer.extend(chunk)
        out: list[dict[str, float | int]] = []

        while True:
            idx = self.buffer.find(HEADER_BYTES)
            if idx < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                break

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < LEGACY_FRAME_SIZE:
                break

            frame_size = 0
            frame_struct = None
            frame_fields = None
            if len(self.buffer) >= FRAME_SIZE and self.buffer[FRAME_SIZE - 1] == FOOTER_VALUE:
                frame_size = FRAME_SIZE
                frame_struct = FRAME_STRUCT
                frame_fields = FRAME_FIELDS
            elif len(self.buffer) >= BME_FRAME_SIZE and self.buffer[BME_FRAME_SIZE - 1] == FOOTER_VALUE:
                frame_size = BME_FRAME_SIZE
                frame_struct = BME_FRAME_STRUCT
                frame_fields = BME_FRAME_FIELDS
            elif self.buffer[LEGACY_FRAME_SIZE - 1] == FOOTER_VALUE:
                frame_size = LEGACY_FRAME_SIZE
                frame_struct = LEGACY_FRAME_STRUCT
                frame_fields = LEGACY_FRAME_FIELDS
            else:
                self.bad_frames += 1
                del self.buffer[0]
                continue

            frame_bytes = bytes(self.buffer[:frame_size])
            values = frame_struct.unpack(frame_bytes)
            frame = dict(zip(frame_fields, values))
            if frame["header"] != HEADER_VALUE or frame["magic_footer"] != FOOTER_VALUE:
                self.bad_frames += 1
                del self.buffer[0]
                continue

            if frame_size == LEGACY_FRAME_SIZE:
                frame["bme280_valid"] = 0.0
                frame["bme280_temp_c"] = 0.0
                frame["bme280_pressure_pa"] = 0.0
                frame["bme280_humidity_rh"] = 0.0
                frame["bme280_altitude_m"] = 0.0
            if frame_size != FRAME_SIZE:
                frame["gps_valid"] = 0.0
                frame["gps_sats"] = 0.0
                frame["gps_lat_deg"] = 0.0
                frame["gps_lon_deg"] = 0.0
                frame["gps_alt_m"] = 0.0
                frame["gps_speed_mps"] = 0.0
                frame["gps_course_deg"] = 0.0
                frame["gps_hdop"] = 0.0

            out.append(frame)
            del self.buffer[:frame_size]

        return out


class FlightVcpGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Amaltheia Flight VCP Dashboard")
        self.root.geometry("1500x980")

        self.ser: serial.Serial | None = None
        self.parser = FrameParser()
        self.logging_enabled = False
        self.log_rows: list[list[str]] = []
        self.last_seq: int | None = None
        self.seq_drops = 0
        self.frames_seen = 0
        self.fps = 0.0
        self._fps_last_count = 0
        self._fps_last_time = time.perf_counter()
        self.start_t = time.perf_counter()

        self.connection_var = tk.StringVar(value="Disconnected")
        self.status_var = tk.StringVar(value="Idle")
        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value="921600")
        self.window_sec_var = tk.StringVar(value="15")
        self.command_var = tk.StringVar(value="")
        self.pause_plots_var = tk.BooleanVar(value=False)
        self.log_btn_text = tk.StringVar(value="Start Log")

        self.armed_var = tk.StringVar(value="0")
        self.mode_var = tk.StringVar(value="0")
        self.flight_mode_var = tk.StringVar(value="0")
        self.seq_var = tk.StringVar(value="0")
        self.sat_var = tk.StringVar(value="0b00000000")
        self.dt_var = tk.StringVar(value="0.0000")
        self.fps_var = tk.StringVar(value="0.0")
        self.drops_var = tk.StringVar(value="0")
        self.bad_frames_var = tk.StringVar(value="0")

        self.rpy_var = tk.StringVar(value="0, 0, 0")
        self.rate_var = tk.StringVar(value="0, 0, 0")
        self.cmd_rate_var = tk.StringVar(value="0, 0, 0")
        self.rate_err_var = tk.StringVar(value="0, 0, 0")
        self.pid_roll_var = tk.StringVar(value="0, 0, 0, 0")
        self.pid_pitch_var = tk.StringVar(value="0, 0, 0, 0")
        self.pid_yaw_var = tk.StringVar(value="0, 0, 0, 0")
        self.motors_var = tk.StringVar(value="0, 0, 0, 0")
        self.z_var = tk.StringVar(value="0")
        self.vz_var = tk.StringVar(value="0")
        self.target_z_var = tk.StringVar(value="0")
        self.ff_vz_var = tk.StringVar(value="0")
        self.bme280_valid_var = tk.StringVar(value="0")
        self.bme280_temp_var = tk.StringVar(value="0")
        self.bme280_pressure_var = tk.StringVar(value="0")
        self.bme280_humidity_var = tk.StringVar(value="0")
        self.bme280_altitude_var = tk.StringVar(value="0")
        self.gps_status_var = tk.StringVar(value="0")
        self.gps_position_var = tk.StringVar(value="0, 0")
        self.gps_motion_var = tk.StringVar(value="0")
        self.accel_norm_var = tk.StringVar(value="0")
        self.gyro_norm_var = tk.StringVar(value="0")

        self.plot_max_points = 3000
        self.plot_time = deque(maxlen=self.plot_max_points)
        self.roll = deque(maxlen=self.plot_max_points)
        self.pitch = deque(maxlen=self.plot_max_points)
        self.yaw = deque(maxlen=self.plot_max_points)
        self.roll_rate = deque(maxlen=self.plot_max_points)
        self.pitch_rate = deque(maxlen=self.plot_max_points)
        self.yaw_rate = deque(maxlen=self.plot_max_points)
        self.roll_rate_cmd = deque(maxlen=self.plot_max_points)
        self.pitch_rate_cmd = deque(maxlen=self.plot_max_points)
        self.yaw_rate_cmd = deque(maxlen=self.plot_max_points)
        self.pid_roll_out = deque(maxlen=self.plot_max_points)
        self.pid_pitch_out = deque(maxlen=self.plot_max_points)
        self.pid_yaw_out = deque(maxlen=self.plot_max_points)
        self.m1 = deque(maxlen=self.plot_max_points)
        self.m2 = deque(maxlen=self.plot_max_points)
        self.m3 = deque(maxlen=self.plot_max_points)
        self.m4 = deque(maxlen=self.plot_max_points)

        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(15, self._poll_serial)
        self.root.after(60, self._refresh_plots)

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=10)
        container.pack(fill=tk.BOTH, expand=True)

        conn = ttk.LabelFrame(container, text="Connection", padding=8)
        conn.pack(fill=tk.X)

        ttk.Label(conn, text="Port").grid(row=0, column=0, sticky=tk.W, padx=(0, 6))
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=24, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky=tk.W)
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(conn, text="Baud").grid(row=0, column=3, sticky=tk.W, padx=(10, 6))
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=4, sticky=tk.W)

        self.connect_btn = ttk.Button(conn, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=(10, 0))

        ttk.Label(conn, text="Window s").grid(row=0, column=6, sticky=tk.W, padx=(18, 6))
        ttk.Entry(conn, textvariable=self.window_sec_var, width=8).grid(row=0, column=7, sticky=tk.W)
        ttk.Checkbutton(conn, text="Pause Plots", variable=self.pause_plots_var).grid(row=0, column=8, padx=(16, 4))
        ttk.Button(conn, text="Clear Plots", command=self._clear_plots).grid(row=0, column=9, padx=6)

        ttk.Label(conn, text="State:").grid(row=0, column=10, sticky=tk.E, padx=(16, 4))
        ttk.Label(conn, textvariable=self.connection_var).grid(row=0, column=11, sticky=tk.W)

        ttk.Label(conn, text="Command").grid(row=1, column=0, sticky=tk.W, padx=(0, 6), pady=(8, 0))
        cmd_entry = ttk.Entry(conn, textvariable=self.command_var, width=48)
        cmd_entry.grid(row=1, column=1, columnspan=4, sticky=tk.EW, pady=(8, 0))
        cmd_entry.bind("<Return>", lambda _event: self._send_command())
        ttk.Button(conn, text="Send", command=self._send_command).grid(row=1, column=5, padx=(10, 0), pady=(8, 0))

        telem = ttk.LabelFrame(container, text="Live Telemetry", padding=8)
        telem.pack(fill=tk.X, pady=(10, 0))

        self._kv_label(telem, 0, "Status", self.status_var)
        self._kv_label(telem, 1, "Armed", self.armed_var)
        self._kv_label(telem, 2, "Mode", self.mode_var)
        self._kv_label(telem, 3, "Flight Mode", self.flight_mode_var)
        self._kv_label(telem, 4, "Sequence", self.seq_var)
        self._kv_label(telem, 5, "Sat Flags", self.sat_var)
        self._kv_label(telem, 6, "dt (s)", self.dt_var)
        self._kv_label(telem, 7, "FPS", self.fps_var)
        self._kv_label(telem, 8, "Seq Drops", self.drops_var)
        self._kv_label(telem, 9, "Bad Frames", self.bad_frames_var)
        self._kv_label(telem, 10, "Roll/Pitch/Yaw", self.rpy_var)
        self._kv_label(telem, 11, "Rate p/q/r", self.rate_var)
        self._kv_label(telem, 12, "Cmd p/q/r", self.cmd_rate_var)
        self._kv_label(telem, 13, "Rate Err", self.rate_err_var)
        self._kv_label(telem, 14, "PID Roll P/I/D/O", self.pid_roll_var)
        self._kv_label(telem, 15, "PID Pitch P/I/D/O", self.pid_pitch_var)
        self._kv_label(telem, 16, "PID Yaw P/I/D/O", self.pid_yaw_var)
        self._kv_label(telem, 17, "Motors %", self.motors_var)
        self._kv_label(telem, 18, "Z", self.z_var)
        self._kv_label(telem, 19, "VZ", self.vz_var)
        self._kv_label(telem, 20, "Target Z", self.target_z_var)
        self._kv_label(telem, 21, "FF VZ", self.ff_vz_var)
        self._kv_label(telem, 22, "BME Status", self.bme280_valid_var)
        self._kv_label(telem, 23, "BME Temp C", self.bme280_temp_var)
        self._kv_label(telem, 24, "BME Press Pa", self.bme280_pressure_var)
        self._kv_label(telem, 25, "BME Hum %", self.bme280_humidity_var)
        self._kv_label(telem, 26, "BME Alt m", self.bme280_altitude_var)
        self._kv_label(telem, 27, "GPS Status", self.gps_status_var)
        self._kv_label(telem, 28, "GPS Lat/Lon", self.gps_position_var)
        self._kv_label(telem, 29, "GPS Alt/Spd", self.gps_motion_var)
        self._kv_label(telem, 30, "|Accel|", self.accel_norm_var)
        self._kv_label(telem, 31, "|Gyro|", self.gyro_norm_var)

        body = ttk.Frame(container)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        plot_frame = ttk.LabelFrame(body, text="Live Plots", padding=6)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(10.5, 7.2), dpi=100)
        self.ax_att = self.fig.add_subplot(411)
        self.ax_rate = self.fig.add_subplot(412, sharex=self.ax_att)
        self.ax_pid = self.fig.add_subplot(413, sharex=self.ax_att)
        self.ax_mot = self.fig.add_subplot(414, sharex=self.ax_att)

        self.l_roll, = self.ax_att.plot([], [], "r-", label="Roll")
        self.l_pitch, = self.ax_att.plot([], [], "g-", label="Pitch")
        self.l_yaw, = self.ax_att.plot([], [], "b-", label="Yaw")
        self.ax_att.set_ylabel("deg")
        self.ax_att.set_title("AHRS")
        self.ax_att.set_ylim(-180.0, 180.0)
        self.ax_att.grid(True)
        self.ax_att.legend(loc="upper right")

        self.l_rr, = self.ax_rate.plot([], [], "r-", label="Roll rate")
        self.l_pr, = self.ax_rate.plot([], [], "g-", label="Pitch rate")
        self.l_yr, = self.ax_rate.plot([], [], "b-", label="Yaw rate")
        self.l_rr_cmd, = self.ax_rate.plot([], [], "r--", alpha=0.5, label="Roll cmd")
        self.l_pr_cmd, = self.ax_rate.plot([], [], "g--", alpha=0.5, label="Pitch cmd")
        self.l_yr_cmd, = self.ax_rate.plot([], [], "b--", alpha=0.5, label="Yaw cmd")
        self.ax_rate.set_ylabel("deg/s")
        self.ax_rate.set_title("Rate Tracking")
        self.ax_rate.grid(True)
        self.ax_rate.legend(loc="upper right", ncol=2, fontsize=8)

        self.l_pid_r, = self.ax_pid.plot([], [], "r-", label="Roll PID out")
        self.l_pid_p, = self.ax_pid.plot([], [], "g-", label="Pitch PID out")
        self.l_pid_y, = self.ax_pid.plot([], [], "b-", label="Yaw PID out")
        self.ax_pid.set_ylabel("PID out")
        self.ax_pid.set_title("PID Outputs")
        self.ax_pid.grid(True)
        self.ax_pid.legend(loc="upper right")

        self.l_m1, = self.ax_mot.plot([], [], "r-", label="M1")
        self.l_m2, = self.ax_mot.plot([], [], "g-", label="M2")
        self.l_m3, = self.ax_mot.plot([], [], "b-", label="M3")
        self.l_m4, = self.ax_mot.plot([], [], "k-", label="M4")
        self.ax_mot.set_ylabel("%")
        self.ax_mot.set_xlabel("Host Time (s)")
        self.ax_mot.set_title("Motor Outputs")
        self.ax_mot.set_ylim(-5.0, 105.0)
        self.ax_mot.grid(True)
        self.ax_mot.legend(loc="upper right", ncol=4, fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        log_frame = ttk.LabelFrame(body, text="Log", padding=8)
        log_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)

        btn_row = ttk.Frame(log_frame)
        btn_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(btn_row, textvariable=self.log_btn_text, command=self._toggle_logging).pack(side=tk.LEFT)
        ttk.Button(btn_row, text="Save CSV", command=self._save_csv).pack(side=tk.LEFT, padx=6)
        ttk.Button(btn_row, text="Clear Text", command=self._clear_log_text).pack(side=tk.LEFT)

        self.log_text = tk.Text(log_frame, width=52, height=34, wrap=tk.NONE)
        self.log_text.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        self.log_text.configure(state=tk.DISABLED)
        yscroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=yscroll.set)

    @staticmethod
    def _kv_label(parent: ttk.LabelFrame, row: int, key: str, value_var: tk.StringVar) -> None:
        r = row // 4
        c = (row % 4) * 2
        ttk.Label(parent, text=f"{key}:").grid(row=r, column=c, sticky=tk.W, padx=(0, 4), pady=2)
        ttk.Label(parent, textvariable=value_var, width=24).grid(row=r, column=c + 1, sticky=tk.W, padx=(0, 14), pady=2)

    def _refresh_ports(self) -> None:
        ports: list[str] = []
        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def _toggle_connection(self) -> None:
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        if serial is None:
            messagebox.showerror("Missing Dependency", f"pyserial import failed:\n{SERIAL_IMPORT_ERROR}")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No Port", "Select a COM port first.")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showwarning("Bad Baud", "Baud must be an integer.")
            return

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0, write_timeout=0.2)
        except Exception as exc:
            messagebox.showerror("Connect Failed", str(exc))
            return

        self.connection_var.set(f"Connected: {port} @ {baud}")
        self.connect_btn.configure(text="Disconnect")
        self.status_var.set("Connected")
        self.parser = FrameParser()
        self.last_seq = None
        self.seq_drops = 0
        self.frames_seen = 0
        self._fps_last_count = 0
        self._fps_last_time = time.perf_counter()
        self.start_t = time.perf_counter()
        self._append_log(f"# Connected to {port} @ {baud}")

    def _disconnect(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connection_var.set("Disconnected")
        self.connect_btn.configure(text="Connect")
        self.status_var.set("Disconnected")
        self._append_log("# Disconnected")

    def _send_command(self) -> None:
        command = self.command_var.get().strip()
        if not command:
            self.status_var.set("Enter a command first")
            return
        if self.ser is None or not self.ser.is_open:
            self.status_var.set("Connect before sending commands")
            return
        try:
            self.ser.write((command + "\n").encode("ascii", errors="ignore"))
        except Exception as exc:
            self._append_log(f"! TX error: {exc}")
            self._disconnect()
            return

        self._append_log(f"> {command}")
        self.status_var.set("Command sent")
        self.command_var.set("")

    def _clear_plots(self) -> None:
        for d in (
            self.plot_time,
            self.roll,
            self.pitch,
            self.yaw,
            self.roll_rate,
            self.pitch_rate,
            self.yaw_rate,
            self.roll_rate_cmd,
            self.pitch_rate_cmd,
            self.yaw_rate_cmd,
            self.pid_roll_out,
            self.pid_pitch_out,
            self.pid_yaw_out,
            self.m1,
            self.m2,
            self.m3,
            self.m4,
        ):
            d.clear()

    def _poll_serial(self) -> None:
        if self.ser is not None and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(waiting)
                    for frame in self.parser.feed(chunk):
                        self._handle_frame(frame)
                self.bad_frames_var.set(str(self.parser.bad_frames))
            except Exception as exc:
                self._append_log(f"! RX error: {exc}")
                self._disconnect()

        self.root.after(15, self._poll_serial)

    def _handle_frame(self, frame: dict[str, float | int]) -> None:
        self.frames_seen += 1
        seq = int(frame["sequence"])
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0xFFFF
            if seq != expected:
                self.seq_drops += (seq - expected) & 0xFFFF
        self.last_seq = seq

        now = time.perf_counter()
        elapsed = now - self._fps_last_time
        if elapsed >= 0.5:
            self.fps = (self.frames_seen - self._fps_last_count) / elapsed
            self._fps_last_count = self.frames_seen
            self._fps_last_time = now
            self.fps_var.set(f"{self.fps:.1f}")

        host_t = now - self.start_t
        self.plot_time.append(host_t)
        self.roll.append(float(frame["roll"]))
        self.pitch.append(float(frame["pitch"]))
        self.yaw.append(float(frame["yaw"]))
        self.roll_rate.append(float(frame["roll_rate"]))
        self.pitch_rate.append(float(frame["pitch_rate"]))
        self.yaw_rate.append(float(frame["yaw_rate"]))
        self.roll_rate_cmd.append(float(frame["target_rate_roll"]))
        self.pitch_rate_cmd.append(float(frame["target_rate_pitch"]))
        self.yaw_rate_cmd.append(float(frame["target_rate_yaw"]))
        self.pid_roll_out.append(float(frame["pid_roll_out"]))
        self.pid_pitch_out.append(float(frame["pid_pitch_out"]))
        self.pid_yaw_out.append(float(frame["pid_yaw_out"]))
        self.m1.append(float(frame["motor1_pct"]))
        self.m2.append(float(frame["motor2_pct"]))
        self.m3.append(float(frame["motor3_pct"]))
        self.m4.append(float(frame["motor4_pct"]))

        roll_rate = float(frame["roll_rate"])
        pitch_rate = float(frame["pitch_rate"])
        yaw_rate = float(frame["yaw_rate"])
        cmd_r = float(frame["target_rate_roll"])
        cmd_p = float(frame["target_rate_pitch"])
        cmd_y = float(frame["target_rate_yaw"])

        accel_norm = math.sqrt(
            float(frame["accel_x"]) ** 2 + float(frame["accel_y"]) ** 2 + float(frame["accel_z"]) ** 2
        )
        gyro_norm = math.sqrt(
            float(frame["gyro_x"]) ** 2 + float(frame["gyro_y"]) ** 2 + float(frame["gyro_z"]) ** 2
        )

        self.status_var.set("Streaming")
        self.armed_var.set(str(int(frame["armed"])))
        self.mode_var.set(str(int(frame["drone_mode"])))
        self.flight_mode_var.set(str(int(frame["flight_mode"])))
        self.seq_var.set(str(seq))
        self.sat_var.set(f"0b{int(frame['sat_flags']):08b}")
        self.dt_var.set(f"{float(frame['dt_sec']):.4f}")
        self.drops_var.set(str(self.seq_drops))
        self.rpy_var.set(f"{float(frame['roll']):+.2f}, {float(frame['pitch']):+.2f}, {float(frame['yaw']):+.2f}")
        self.rate_var.set(f"{roll_rate:+.2f}, {pitch_rate:+.2f}, {yaw_rate:+.2f}")
        self.cmd_rate_var.set(f"{cmd_r:+.2f}, {cmd_p:+.2f}, {cmd_y:+.2f}")
        self.rate_err_var.set(f"{(cmd_r-roll_rate):+.2f}, {(cmd_p-pitch_rate):+.2f}, {(cmd_y-yaw_rate):+.2f}")
        self.pid_roll_var.set(
            f"{float(frame['pid_roll_p']):+.3f}, {float(frame['pid_roll_i']):+.3f}, {float(frame['pid_roll_d']):+.3f}, {float(frame['pid_roll_out']):+.3f}"
        )
        self.pid_pitch_var.set(
            f"{float(frame['pid_pitch_p']):+.3f}, {float(frame['pid_pitch_i']):+.3f}, {float(frame['pid_pitch_d']):+.3f}, {float(frame['pid_pitch_out']):+.3f}"
        )
        self.pid_yaw_var.set(
            f"{float(frame['pid_yaw_p']):+.3f}, {float(frame['pid_yaw_i']):+.3f}, {float(frame['pid_yaw_d']):+.3f}, {float(frame['pid_yaw_out']):+.3f}"
        )
        self.motors_var.set(
            f"{float(frame['motor1_pct']):.1f}, {float(frame['motor2_pct']):.1f}, {float(frame['motor3_pct']):.1f}, {float(frame['motor4_pct']):.1f}"
        )
        self.z_var.set(f"{float(frame['z']):+.3f}")
        self.vz_var.set(f"{float(frame['vz']):+.3f}")
        self.target_z_var.set(f"{float(frame['target_z']):+.3f}")
        self.ff_vz_var.set(f"{float(frame['target_ff_vz']):+.3f}")
        bme_status = int(float(frame["bme280_valid"]))
        if bme_status == 1:
            self.bme280_valid_var.set("1")
        elif bme_status == -5:
            self.bme280_valid_var.set("-5 no I2C3 ACK")
        elif bme_status == -6:
            bme_addr = int(float(frame["bme280_pressure_pa"]))
            bme_count = int(float(frame["bme280_temp_c"]))
            self.bme280_valid_var.set(f"-6 first=0x{bme_addr:02X} count={bme_count}")
        else:
            bme_addr = int(float(frame["bme280_pressure_pa"]))
            bme_chip = int(float(frame["bme280_temp_c"]))
            self.bme280_valid_var.set(f"{bme_status} addr=0x{bme_addr:02X} chip=0x{bme_chip:02X}")
        self.bme280_temp_var.set(f"{float(frame['bme280_temp_c']):+.2f}")
        self.bme280_pressure_var.set(f"{float(frame['bme280_pressure_pa']):.1f}")
        self.bme280_humidity_var.set(f"{float(frame['bme280_humidity_rh']):.2f}")
        self.bme280_altitude_var.set(f"{float(frame['bme280_altitude_m']):+.3f}")
        self.gps_status_var.set(
            f"valid={int(float(frame['gps_valid']))} sats={int(float(frame['gps_sats']))} hdop={float(frame['gps_hdop']):.2f}"
        )
        self.gps_position_var.set(f"{float(frame['gps_lat_deg']):+.6f}, {float(frame['gps_lon_deg']):+.6f}")
        self.gps_motion_var.set(
            f"alt={float(frame['gps_alt_m']):+.1f}m spd={float(frame['gps_speed_mps']):.2f}m/s crs={float(frame['gps_course_deg']):.1f}"
        )
        self.accel_norm_var.set(f"{accel_norm:.3f}")
        self.gyro_norm_var.set(f"{gyro_norm:.3f}")

        if self.frames_seen % 100 == 0:
            self._append_log(
                f"[RX] seq={seq} rpy=({float(frame['roll']):+.1f}, {float(frame['pitch']):+.1f}, {float(frame['yaw']):+.1f}) "
                f"pid_out=({float(frame['pid_roll_out']):+.2f}, {float(frame['pid_pitch_out']):+.2f}, {float(frame['pid_yaw_out']):+.2f})"
            )

        self._append_csv_row(host_t, frame)

    def _refresh_plots(self) -> None:
        if not self.pause_plots_var.get() and len(self.plot_time) > 2:
            x = list(self.plot_time)
            self.l_roll.set_data(x, self.roll)
            self.l_pitch.set_data(x, self.pitch)
            self.l_yaw.set_data(x, self.yaw)

            self.l_rr.set_data(x, self.roll_rate)
            self.l_pr.set_data(x, self.pitch_rate)
            self.l_yr.set_data(x, self.yaw_rate)
            self.l_rr_cmd.set_data(x, self.roll_rate_cmd)
            self.l_pr_cmd.set_data(x, self.pitch_rate_cmd)
            self.l_yr_cmd.set_data(x, self.yaw_rate_cmd)

            self.l_pid_r.set_data(x, self.pid_roll_out)
            self.l_pid_p.set_data(x, self.pid_pitch_out)
            self.l_pid_y.set_data(x, self.pid_yaw_out)

            self.l_m1.set_data(x, self.m1)
            self.l_m2.set_data(x, self.m2)
            self.l_m3.set_data(x, self.m3)
            self.l_m4.set_data(x, self.m4)

            window = self._safe_window_seconds()
            xmax = x[-1]
            xmin = max(0.0, xmax - window)
            for ax in (self.ax_att, self.ax_rate, self.ax_pid, self.ax_mot):
                ax.set_xlim(xmin, xmax + 0.05)

            for ax in (self.ax_rate, self.ax_pid):
                ax.relim()
                ax.autoscale_view()

            self.canvas.draw_idle()

        self.root.after(60, self._refresh_plots)

    def _safe_window_seconds(self) -> float:
        try:
            window = float(self.window_sec_var.get().strip())
        except ValueError:
            return 15.0
        if window < 2.0:
            return 2.0
        if window > 120.0:
            return 120.0
        return window

    def _append_csv_row(self, host_time_s: float, frame: dict[str, float | int]) -> None:
        if not self.logging_enabled:
            return
        row = [dt.datetime.now().isoformat(timespec="milliseconds"), f"{host_time_s:.6f}"]
        for key in FRAME_FIELDS:
            row.append(str(frame[key]))
        self.log_rows.append(row)

    def _append_log(self, text: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _toggle_logging(self) -> None:
        self.logging_enabled = not self.logging_enabled
        if self.logging_enabled:
            self.log_rows.clear()
            self.log_btn_text.set("Stop Log")
            self._append_log("# Logging enabled")
        else:
            self.log_btn_text.set("Start Log")
            self._append_log(f"# Logging disabled ({len(self.log_rows)} rows)")

    def _save_csv(self) -> None:
        if not self.log_rows:
            messagebox.showinfo("No Data", "No logged rows to save.")
            return

        default_name = f"flight_vcp_log_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
        out_path = filedialog.asksaveasfilename(
            title="Save telemetry log",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not out_path:
            return

        try:
            with open(out_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["pc_time_iso", "host_time_s"] + FRAME_FIELDS)
                writer.writerows(self.log_rows)
        except Exception as exc:
            messagebox.showerror("Save Failed", str(exc))
            return

        self._append_log(f"# Saved CSV: {out_path} ({len(self.log_rows)} rows)")

    def _clear_log_text(self) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _on_close(self) -> None:
        self._disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ttk.Style().theme_use("clam")
    FlightVcpGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
