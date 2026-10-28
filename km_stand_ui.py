#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KM Stand UI — однофайловое десктопное ПО для стенда коленного модуля.

Зависимости:
    pip install pyserial matplotlib

Запуск:
    python km_stand_ui.py
    python km_stand_ui.py --sim

Использует tkinter + matplotlib + pyserial.
"""

import argparse
import csv
import os
import queue
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from tkinter import (
    Tk, ttk, StringVar, BooleanVar, DoubleVar, IntVar, messagebox,
    filedialog, scrolledtext, Frame, Label, Entry, Button, Checkbutton,
    OptionMenu
)

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator

import serial
import serial.tools.list_ports


# ---------------------------------------------------------------------------
# Протокол обмена (соответствует Core/Src/parser.cpp и Core/Src/main.c)
# ---------------------------------------------------------------------------
SYNC1 = 0xFF
SYNC2 = 0xFA
END   = 0x0A

MT_MODE1_CONFIG = 0x10
MT_MODE1_START  = 0x11
MT_MODE1_STOP   = 0x12
MT_CALIBRATION  = 0x01
MT_MODE2_CONFIG = 0x20
MT_MODE2_START  = 0x21
MT_MODE2_STOP   = 0x22
MT_TORQUE       = 0x32
MT_PID_CONFIG   = 0x06
MT_LOAD_ZERO    = 0x07
MT_STREAM_DATA  = 0x40

STM_STATES = {
    0x00: "Инициализация",
    0x01: "Калибровка",
    0x10: "Конфиг режима 1",
    0x11: "Режим 1 - работа",
    0x12: "Режим 1 - стоп",
    0x20: "Конфиг режима 2",
    0x21: "Режим 2 - работа",
    0x22: "Режим 2 - удержание",
    0x23: "Режим 2 - возврат",
    0x24: "Режим 2 - стоп",
    0x30: "Конфиг режима 3",
    0x31: "Режим 3 - работа",
    0x32: "Режим 3 - стоп",
    0x33: "ACK",
    0x40: "Поток данных",
}

CALIB_STATES = {
    0: "RIGHT_CALIB (подъезд к правому концевику)",
    1: "RIGHT_RUN_SLOW (не используется)",
    2: "LEFT_CALIB (подъезд к левому концевику)",
    3: "LEFT_RUN_SLOW (не используется)",
    4: "RIGHT_LOGIC (не используется)",
    5: "LEFT_LOGIC (не используется)",
    6: "END",
    7: "RIGHT_DELAY (не используется)",
    8: "LEFT_DELAY (не используется)",
}

# Структуры прошивки (порядок полей важен!):
# mode1_config_t { float min_angle; float max_angle; float freq; int count; float moment; }
# mode2_config_t { float speed; float start_angle; float end_angle; uint32_t hold_ms; }
# torque_t       { float torque; uint32_t duration; uint8_t direction; }
# stream_data_t  { int ts_ms; float load; float pos; float angle1; float angle2; float temp; uint16_t flags; uint32_t cycles; }

MODE1_CONFIG_FMT = "<fffIf"  # 20 байт: min_angle, max_angle, freq, count, moment
MODE2_CONFIG_FMT = "<fffI"   # 16 байт
TORQUE_FMT       = "<fIB"    # 9 байт
STREAM_FMT       = "<IfffffHI"  # 30 байт
STREAM_DIAG_FMT = "<ffiH"  # target deg/s, motor RPM, signed command, control flags
STREAM_FMT_LEGACY = "<IfffffH"  # 26 байт, старые прошивки без cycles

CSV_FIELDS = [
    "pc_time", "ts_ms", "angle_deg", "omega_degs", "load_kg",
    "pos_mm", "temp_c", "flags", "cycles_done",
    "target_omega_degs", "motor_rpm", "motor_command", "control_flags",
]

SAFETY_TEMP_MAX_C = 80.0
SAFETY_TEMP_RESUME_C = 70.0
SAFETY_LOAD_MAX_KG = 5000.0
SAFETY_ANGLE_TOL_DEG = 5.0
ENCODER_RPM_TO_DEG_S = 6.0
# Must match MODE1_RAMP_FRACTION in Core/Src/main.c.
MODE1_RAMP_FRACTION = 0.20
CYCLE_DIRECTION_EPS_DEG = 0.25
CYCLE_TURN_MIN_DELTA_DEG = 5.0
CYCLE_TURN_DEBOUNCE_S = 0.25


def mode1_motion_profile(t: float, a_min: float, a_max: float, f_hz: float):
    """Ideal Mode1 angle and angular speed (deg/s), including both ramps."""
    if f_hz <= 0.0 or a_max <= a_min:
        return a_min, 0.0
    half_period = 0.5 / f_hz
    phase = t % (2.0 * half_period)
    forward = phase < half_period
    leg_time = phase if forward else phase - half_period
    ramp_time = half_period * (2.0 * MODE1_RAMP_FRACTION) / (1.0 + 2.0 * MODE1_RAMP_FRACTION)
    distance = a_max - a_min
    peak_speed = distance / (half_period - ramp_time)
    acceleration = peak_speed / ramp_time
    if leg_time < ramp_time:
        travelled = 0.5 * acceleration * leg_time * leg_time
        speed = acceleration * leg_time
    elif leg_time < half_period - ramp_time:
        travelled = 0.5 * peak_speed * ramp_time + peak_speed * (leg_time - ramp_time)
        speed = peak_speed
    else:
        time_left = half_period - leg_time
        travelled = distance - 0.5 * acceleration * time_left * time_left
        speed = acceleration * time_left
    if forward:
        return a_min + travelled, speed
    return a_max - travelled, -speed


def decode_stream_payload(payload: bytes):
    legacy_size = struct.calcsize(STREAM_FMT_LEGACY)
    stream_size = struct.calcsize(STREAM_FMT)
    if len(payload) < legacy_size:
        raise ValueError("short telemetry payload")
    ts_ms, load, pos, angle, rpm, temp, flags = struct.unpack_from(STREAM_FMT_LEGACY, payload)
    sample = {
        "ts_ms": int(ts_ms), "load": float(load), "pos": float(pos),
        "angle": float(angle), "omega": float(rpm) * ENCODER_RPM_TO_DEG_S,
        "temp": float(temp), "flags": int(flags), "cycles": None,
        "target_omega": None, "motor_rpm": None, "motor_command": None,
        "control_flags": None,
    }
    if len(payload) >= stream_size:
        sample["cycles"] = struct.unpack_from("<I", payload, legacy_size)[0]
    if len(payload) >= stream_size + struct.calcsize(STREAM_DIAG_FMT):
        target, motor_rpm, command, control_flags = struct.unpack_from(STREAM_DIAG_FMT, payload, stream_size)
        sample.update(
            target_omega=float(target) if control_flags & 1 else None,
            motor_rpm=float(motor_rpm), motor_command=int(command),
            control_flags=int(control_flags),
        )
    return sample


def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def build_packet(msg_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too long")
    crc_input = bytes([msg_type & 0xFF, len(payload) & 0xFF]) + payload
    crc = crc16_ccitt(crc_input)
    return bytes([SYNC1, SYNC2, msg_type & 0xFF, len(payload) & 0xFF]) + payload + struct.pack("<H", crc) + bytes([END])


class PacketParser:
    """
    Универсальный парсер, поддерживающий два варианта кадра:

    A. "Правильный" формат (используем при отправке ПК -> СТМ):
       [0xFF, 0xFA, msg_type, len_payload, payload, CRC(msg_type,len,payload), 0x0A]

    B. "Багgy" формат прошивки (то, что СТМ посылает сама):
       [0xFF, 0xFA, msg_type, len_payload+6, payload, CRC(payload_only), 0x0A]
       В прошивке make_buff записывает в 4-й байт len_packet=l+6,
       а crc16_ccitt считается только по payload.
    """

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        packets = []
        while True:
            if len(self.buf) < 7:
                break
            idx = self.buf.find(bytes([SYNC1, SYNC2]))
            if idx == -1:
                # Preserve a possible first sync byte at a read boundary.
                self.buf[:] = self.buf[-1:] if self.buf[-1:] == bytes([SYNC1]) else b""
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < 7:
                break

            msg_type = self.buf[2]
            b3 = self.buf[3]

            # --- интерпретация A: правильный формат ---
            payload_len_a = b3
            total_a = payload_len_a + 7  # sync(2)+type(1)+len(1)+payload+crc(2)+end(1)
            if len(self.buf) >= total_a:
                pkt_a = bytes(self.buf[:total_a])
                if pkt_a[-1] == END:
                    payload_a = pkt_a[4:4 + payload_len_a]
                    crc_recv_a = struct.unpack("<H", pkt_a[4 + payload_len_a:4 + payload_len_a + 2])[0]
                    crc_calc_a = crc16_ccitt(bytes([msg_type, b3]) + payload_a)
                    if crc_recv_a == crc_calc_a:
                        packets.append((msg_type, payload_a))
                        del self.buf[:total_a]
                        continue

            # --- интерпретация B: формат прошивки ---
            if b3 >= 6:
                payload_len_b = b3 - 6
                total_b = payload_len_b + 7  # = b3 + 1
                if len(self.buf) >= total_b:
                    pkt_b = bytes(self.buf[:total_b])
                    if pkt_b[-1] == END:
                        payload_b = pkt_b[4:4 + payload_len_b]
                        crc_recv_b = struct.unpack("<H", pkt_b[4 + payload_len_b:4 + payload_len_b + 2])[0]
                        crc_calc_b = crc16_ccitt(payload_b)  # прошивка: CRC только по payload
                        if crc_recv_b == crc_calc_b:
                            packets.append((msg_type, payload_b))
                            del self.buf[:total_b]
                            continue

            # An incomplete frame must survive until the next serial read.
            if len(self.buf) < total_a:
                break
            # Both complete frame interpretations failed: resynchronize.
            del self.buf[0]
        return packets


def build_packet_like_firmware(msg_type: int, payload: bytes = b"") -> bytes:
    """Имитирует make_buff() из Core/Src/parser.cpp (для симуляции)."""
    if len(payload) > 255:
        raise ValueError("payload too long")
    crc = crc16_ccitt(payload)
    return (bytes([SYNC1, SYNC2, msg_type & 0xFF, (len(payload) + 6) & 0xFF]) +
            payload + struct.pack("<H", crc) + bytes([END]))


# ---------------------------------------------------------------------------
# Поток COM-порта
# ---------------------------------------------------------------------------
class SerialThread(threading.Thread):
    def __init__(self, port: str, baud: int, rx_queue: queue.Queue, on_event, simulation: bool = False):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.rx_queue = rx_queue
        self.on_event = on_event
        self.simulation = simulation
        self.running = True
        self.ser = None
        self.parser = PacketParser()
        self._sim_start = time.time()
        self._sim_cfg = {"mode": None, "config": None}
        self._sim_lock = threading.Lock()
        self._send_queue = queue.Queue()

    def stop(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def send(self, pkt: bytes):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(pkt)
            except Exception as e:
                self.on_event("alarm", f"Ошибка отправки: {e}")

    def set_sim_profile(self, mode: str, config: dict):
        with self._sim_lock:
            self._sim_cfg = {"mode": mode, "config": config}
            self._sim_start = time.time()

    def _make_sim_stream(self) -> bytes:
        import random
        t = time.time() - self._sim_start
        with self._sim_lock:
            cfg = self._sim_cfg.get("config")
            mode = self._sim_cfg.get("mode")

        if mode == "mode1" and cfg:
            a_min = float(cfg.get("min_angle", 50.0))
            a_max = float(cfg.get("max_angle", 180.0))
            f_hz = float(cfg.get("freq", 0.5))
            angle, omega = mode1_motion_profile(t, a_min, a_max, f_hz)
        else:
            angle = 90.0 + 45.0 * (1 if int(t) % 4 < 2 else -1)  # упрощённая ручная пила

        if mode != "mode1" or not cfg:
            omega = 0.0

        load = 500.0 + 200.0 * (angle - 90.0) / 90.0 + random.gauss(0, 10)
        pos = 100.0 + 0.5 * angle + random.gauss(0, 0.5)
        temp = 35.0 + random.gauss(0, 0.1)
        flags = 0x11 if mode == "mode1" else 0x21 if mode == "mode2" else 0x00
        ts_ms = int((time.time() % 1000000) * 1000) & 0xFFFFFFFF
        # Firmware sends angle2 as encoder_rpm. The UI converts it to deg/s on receive.
        omega_rpm = omega / ENCODER_RPM_TO_DEG_S
        cycles = int(t * f_hz) if mode == "mode1" and cfg and f_hz > 0.0 else 0
        payload = struct.pack(STREAM_FMT, ts_ms, float(load), float(pos), float(angle), float(omega_rpm), float(temp), int(flags), cycles)
        control_flags = 1 if mode == "mode1" and cfg else 0
        payload += struct.pack(STREAM_DIAG_FMT, omega, 0.0, 0, control_flags)
        # Keep the legacy framing variant for parser compatibility testing.
        return build_packet_like_firmware(MT_STREAM_DATA, payload)

    def run(self):
        if not self.simulation:
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05
                )
                self.on_event("info", f"Порт {self.port} открыт на {self.baud}")
            except Exception as e:
                self.on_event("alarm", f"Не удалось открыть порт {self.port}: {e}")
                return
        else:
            self.on_event("info", "Симуляция: порт не используется")

        last_sim = time.time()
        sim_interval = 0.1  # 10 Гц в симуляции

        while self.running:
            try:
                if self.simulation:
                    now = time.time()
                    if now - last_sim >= sim_interval:
                        last_sim = now
                        for mt, pl in self.parser.feed(self._make_sim_stream()):
                            self.rx_queue.put((mt, pl))
                    time.sleep(0.01)
                else:
                    data = self.ser.read(512)
                    if data:
                        for mt, pl in self.parser.feed(data):
                            self.rx_queue.put((mt, pl))
            except serial.SerialException as e:
                self.on_event("alarm", f"Ошибка порта: {e}")
                break
            except Exception as e:
                self.on_event("alarm", f"Ошибка чтения: {e}")
                time.sleep(0.1)


# ---------------------------------------------------------------------------
# Главное окно
# ---------------------------------------------------------------------------
class KMStandApp:
    def __init__(self, root: Tk, simulation_default: bool = False):
        self.root = root
        self.root.title("KM Stand — ПО стенда коленного модуля")
        self.root.geometry("1366x680")
        self.root.minsize(1280, 640)

        # Очереди и потоки
        self.rx_queue = queue.Queue()
        self.serial_thread = None
        self.connected = False

        # Данные
        self.samples = deque(maxlen=2000)  # {pc_ts, ts_ms, angle, omega, load, pos, temp, flags}
        self.test_active = False
        self.test_mode = None
        self.test_start_time = None
        self.mode1_seen_running = False
        self.cycles_done = 0
        self.cycle_state = self._new_cycle_state()
        self.paused_by_overheat = False
        self.overheat = False
        self.log_file = None
        self.csv_writer = None
        self.csv_file_obj = None

        # UI переменные
        self.var_port = StringVar(value="")
        self.var_baud = StringVar(value="115200")
        self.var_sim = BooleanVar(value=simulation_default)

        # Режим 1
        self.var_m1_min = DoubleVar(value=50.0)
        self.var_m1_max = DoubleVar(value=180.0)
        self.var_m1_freq = DoubleVar(value=0.5)
        self.var_m1_cycles = IntVar(value=630000)
        self.var_m1_moment = DoubleVar(value=89.0)

        # Режим 2
        self.var_m2_start_angle = DoubleVar(value=90.0)
        self.var_m2_end_angle = DoubleVar(value=150.0)
        self.var_m2_speed = DoubleVar(value=50.0)

        # Безопасность
        self.var_safe_temp_max = DoubleVar(value=SAFETY_TEMP_MAX_C)
        self.var_safe_temp_resume = DoubleVar(value=SAFETY_TEMP_RESUME_C)
        self.var_safe_load_max = DoubleVar(value=SAFETY_LOAD_MAX_KG)
        self.var_safe_angle_tol = DoubleVar(value=SAFETY_ANGLE_TOL_DEG)
        self.var_safe_auto_resume = BooleanVar(value=False)

        self.var_log_path = StringVar(value=os.path.join(os.getcwd(), "km_stand_control_log.csv"))
        self.var_status = StringVar(value="Не подключено")

        self._build_ui()
        self._update_ui()

    # -----------------------------------------------------------------------
    # Интерфейс
    # -----------------------------------------------------------------------
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=2)
        main.pack(fill="both", expand=True)
        main.rowconfigure(1, weight=1)
        main.columnconfigure(0, weight=1)

        # Верхняя панель: слева — вкладки + журнал, справа — текущие значения
        top_frame = ttk.Frame(main)
        top_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 2))
        top_frame.columnconfigure(0, weight=3)
        top_frame.columnconfigure(1, weight=1)
        top_frame.rowconfigure(0, weight=0)
        top_frame.rowconfigure(1, weight=1)

        notebook = ttk.Notebook(top_frame)
        notebook.grid(row=0, column=0, sticky="nsew")

        tab_conn = ttk.Frame(notebook, padding=2)
        tab_mode1 = ttk.Frame(notebook, padding=2)
        tab_mode2 = ttk.Frame(notebook, padding=2)
        tab_safety = ttk.Frame(notebook, padding=2)
        tab_log = ttk.Frame(notebook, padding=2)

        notebook.add(tab_conn, text="Подключение")
        notebook.add(tab_mode1, text="Режим 1")
        notebook.add(tab_mode2, text="Режим 2")
        notebook.add(tab_safety, text="Безопасность")
        notebook.add(tab_log, text="Логирование")

        self._build_connection_frame(tab_conn)
        self._build_mode1_frame(tab_mode1)
        self._build_mode2_frame(tab_mode2)
        self._build_safety_frame(tab_safety)
        self._build_log_frame(tab_log)

        # Журнал — под вкладками, слева от текущих значений
        self._build_event_log_frame(top_frame, row=1, column=0)

        # Текущие значения — справа, на всю высоту верхней панели
        self._build_live_frame(top_frame, row=0, column=1, rowspan=2)

        # Основная область: только графики
        content = ttk.Frame(main)
        content.grid(row=1, column=0, sticky="nsew")
        content.rowconfigure(0, weight=1)
        content.columnconfigure(0, weight=1)

        self._build_plots_frame(content)

    def _build_connection_frame(self, parent):
        frm = ttk.Frame(parent, padding=2)
        frm.pack(fill="both", expand=True)

        row1 = ttk.Frame(frm)
        row1.pack(fill="x")
        ttk.Label(row1, text="Порт:").pack(side="left")
        ports = [p.device for p in serial.tools.list_ports.comports()] + [""]
        self.combo_port = ttk.Combobox(row1, textvariable=self.var_port, values=ports, width=16)
        self.combo_port.pack(side="left", padx=(4, 0))
        ttk.Button(row1, text="⟳", width=3, command=self._refresh_ports).pack(side="left", padx=(4, 0))

        ttk.Label(row1, text="  Скорость:").pack(side="left", padx=(12, 0))
        ttk.Combobox(row1, textvariable=self.var_baud, values=["9600", "19200", "38400", "57600", "115200"], width=10).pack(side="left", padx=(4, 0))
        ttk.Checkbutton(row1, text="Симуляция", variable=self.var_sim).pack(side="left", padx=(12, 0))

        row2 = ttk.Frame(frm)
        row2.pack(fill="x", pady=(4, 0))
        self.btn_connect = ttk.Button(row2, text="Подключить", command=self._connect)
        self.btn_connect.pack(side="left", fill="x", expand=True, padx=(0, 2))
        self.btn_disconnect = ttk.Button(row2, text="Отключить", command=self._disconnect, state="disabled")
        self.btn_disconnect.pack(side="left", fill="x", expand=True, padx=(2, 0))

        ttk.Label(frm, textvariable=self.var_status, foreground="gray").pack(anchor="w", pady=(2, 0))

    def _build_mode1_frame(self, parent):
        frm = ttk.Frame(parent, padding=2)
        frm.pack(fill="both", expand=True)

        def make_field(parent_row, label, var, width=8):
            ttk.Label(parent_row, text=label).pack(side="left", padx=(4, 0))
            ttk.Entry(parent_row, textvariable=var, width=width).pack(side="left", padx=(2, 0))

        row1 = ttk.Frame(frm)
        row1.pack(fill="x")
        make_field(row1, "Нач. угол, °:", self.var_m1_min)
        make_field(row1, "Кон. угол, °:", self.var_m1_max)
        make_field(row1, "Частота, Гц:", self.var_m1_freq)

        row2 = ttk.Frame(frm)
        row2.pack(fill="x", pady=(4, 0))
        make_field(row2, "Циклы:", self.var_m1_cycles)
        make_field(row2, "Момент, Н·м:", self.var_m1_moment)

        row3 = ttk.Frame(frm)
        row3.pack(fill="x", pady=(4, 0))
        ttk.Button(row3, text="Калибровка", command=self._start_calibration).pack(side="left", fill="x", expand=True, padx=(4, 2))
        ttk.Button(row3, text="Конфиг", command=self._send_mode1_config).pack(side="left", fill="x", expand=True, padx=(2, 2))
        ttk.Button(row3, text="Старт", command=self._start_mode1).pack(side="left", fill="x", expand=True, padx=(2, 2))
        ttk.Button(row3, text="Стоп", command=self._stop_mode1).pack(side="left", fill="x", expand=True, padx=(2, 0))

    def _build_mode2_frame(self, parent):
        frm = ttk.Frame(parent, padding=2)
        frm.pack(fill="both", expand=True)

        def make_field(parent_row, label, var, width=10):
            ttk.Label(parent_row, text=label).pack(side="left", padx=(4, 0))
            ttk.Entry(parent_row, textvariable=var, width=width).pack(side="left", padx=(2, 0))

        row1 = ttk.Frame(frm)
        row1.pack(fill="x")
        make_field(row1, "Нач. угол, °:", self.var_m2_start_angle)
        make_field(row1, "Кон. угол, °:", self.var_m2_end_angle)
        make_field(row1, "Скорость:", self.var_m2_speed)

        row2 = ttk.Frame(frm)
        row2.pack(fill="x", pady=(4, 0))
        ttk.Button(row2, text="Конфиг", command=self._send_mode2_config).pack(side="left", fill="x", expand=True, padx=(4, 2))
        ttk.Button(row2, text="Старт", command=self._start_mode2).pack(side="left", fill="x", expand=True, padx=(2, 2))
        ttk.Button(row2, text="Стоп", command=self._stop_mode2).pack(side="left", fill="x", expand=True, padx=(2, 0))

    def _build_safety_frame(self, parent):
        frm = ttk.Frame(parent, padding=2)
        frm.pack(fill="both", expand=True)

        def make_field(parent_row, label, var, width=8):
            ttk.Label(parent_row, text=label).pack(side="left", padx=(4, 0))
            ttk.Entry(parent_row, textvariable=var, width=width).pack(side="left", padx=(2, 0))

        row1 = ttk.Frame(frm)
        row1.pack(fill="x")
        make_field(row1, "Tmax, °C:", self.var_safe_temp_max)
        make_field(row1, "Tresume, °C:", self.var_safe_temp_resume)
        make_field(row1, "Fmax, кг:", self.var_safe_load_max)
        make_field(row1, "Допуск, °:", self.var_safe_angle_tol)

        ttk.Checkbutton(frm, text="Автовозобновление после перегрева", variable=self.var_safe_auto_resume).pack(anchor="w", pady=(2, 0))

    def _build_log_frame(self, parent):
        frm = ttk.Frame(parent, padding=2)
        frm.pack(fill="both", expand=True)

        row1 = ttk.Frame(frm)
        row1.pack(fill="x")
        ttk.Entry(row1, textvariable=self.var_log_path).pack(side="left", fill="x", expand=True)
        ttk.Button(row1, text="…", width=3, command=self._choose_log_path).pack(side="left", padx=(4, 0))

        row2 = ttk.Frame(frm)
        row2.pack(fill="x", pady=(4, 0))
        self.btn_log_start = ttk.Button(row2, text="Начать запись", command=self._start_logging)
        self.btn_log_start.pack(side="left", fill="x", expand=True, padx=(0, 2))
        self.btn_log_stop = ttk.Button(row2, text="Остановить запись", command=self._stop_logging, state="disabled")
        self.btn_log_stop.pack(side="left", fill="x", expand=True, padx=(2, 0))

    def _build_live_frame(self, parent, row=0, column=0, rowspan=1):
        frm = ttk.LabelFrame(parent, text="Текущие значения", padding=2)
        frm.grid(row=row, column=column, rowspan=rowspan, sticky="nsew", padx=(2, 0))
        frm.columnconfigure((0, 1), weight=1)

        labels = [
            ("Угол", "°", "angle"),
            ("Угл. скорость", "°/с", "omega"),
            ("Нагрузка", "кг", "load"),
            ("Положение", "мм", "pos"),
            ("Температура", "°C", "temp"),
            ("Циклы", "", "cycles"),
        ]
        self.live_labels = {}
        for i, (name, unit, key) in enumerate(labels):
            r, c = divmod(i, 2)
            cell = ttk.Frame(frm)
            cell.grid(row=r, column=c, padx=4, pady=1, sticky="w")
            ttk.Label(cell, text=name, font=("", 9)).pack(anchor="w")
            lbl = ttk.Label(cell, text="—", font=("", 14, "bold"))
            lbl.pack(anchor="w")
            ttk.Label(cell, text=unit, font=("", 8), foreground="gray").pack(anchor="w")
            self.live_labels[key] = lbl

        # Кнопка очистки графиков — в свободном пространстве снизу блока текущих значений
        btn_frm = ttk.Frame(frm)
        btn_frm.grid(row=3, column=0, columnspan=2, sticky="se", padx=2, pady=(2, 0))
        ttk.Button(btn_frm, text="Сброс нагрузки", command=self._zero_load).pack(side="right", padx=(2, 0))
        ttk.Button(btn_frm, text="Очистить графики", command=self._clear_plots).pack(side="right")

    def _build_plots_frame(self, parent):
        frm = ttk.LabelFrame(parent, text="Графики телеметрии", padding=6)
        frm.grid(row=0, column=0, sticky="nsew", pady=(0, 2))
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)

        self.fig = Figure(figsize=(12.5, 6.2), dpi=105, facecolor="#f8fafc", constrained_layout=True)
        gs = self.fig.add_gridspec(
            2, 2,
            height_ratios=(1.18, 1.0),
            width_ratios=(1.0, 1.0),
        )
        self.ax_angle = self.fig.add_subplot(gs[0, :])
        self.ax_omega = self.fig.add_subplot(gs[1, 0], sharex=self.ax_angle)
        self.ax_load = self.fig.add_subplot(gs[1, 1], sharex=self.ax_angle)

        self.plot_lines = {
            "angle": self.ax_angle.plot([], [], color="#2563eb", linewidth=1.8, antialiased=True)[0],
            "omega": self.ax_omega.plot([], [], color="#059669", linewidth=1.6, antialiased=True)[0],
            "load": self.ax_load.plot([], [], color="#dc2626", linewidth=1.6, antialiased=True)[0],
        }
        self.plot_axes = (self.ax_angle, self.ax_omega, self.ax_load)

        self._style_plot_axis(self.ax_angle, "Угол сгиба колена", "Угол, °")
        self._style_plot_axis(self.ax_omega, "Угловая скорость", "°/с")
        self._style_plot_axis(self.ax_load, "Нагрузка", "кг")
        self.ax_omega.set_xlabel("Время, с", fontsize=8, labelpad=5, color="#475569")
        self.ax_load.set_xlabel("Время, с", fontsize=8, labelpad=5, color="#475569")
        self.ax_angle.tick_params(labelbottom=False)
        self._reset_plot_limits()

        self.canvas = FigureCanvasTkAgg(self.fig, master=frm)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Адаптивный размер графиков под размер окна
        self._resize_after_id = None
        frm.bind("<Configure>", self._on_plot_resize)

    def _build_event_log_frame(self, parent, row=0, column=0):
        frm = ttk.LabelFrame(parent, text="Журнал событий", padding=1)
        frm.grid(row=row, column=column, sticky="nsew", pady=(1, 0))
        frm.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)

        self.event_log = scrolledtext.ScrolledText(frm, height=2, state="disabled", wrap="word")
        self.event_log.grid(row=0, column=0, sticky="nsew")

        btn_row = ttk.Frame(frm)
        btn_row.grid(row=1, column=0, sticky="e", pady=(1, 0))
        ttk.Button(btn_row, text="Копировать", command=self._copy_log).pack(side="right", padx=(2, 0))
        ttk.Button(btn_row, text="Очистить", command=self._clear_log).pack(side="right")

    def _on_plot_resize(self, event):
        # Откладываем перерисовку, чтобы не гонять canvas на каждый пиксель
        if self._resize_after_id:
            self.root.after_cancel(self._resize_after_id)
        self._resize_after_id = self.root.after(200, self._do_plot_resize, event.width, event.height)

    def _do_plot_resize(self, width: int, height: int):
        try:
            # Вычитаем отступы рамки
            new_w = max(6.0, (width - 18) / self.fig.dpi)
            new_h = max(4.0, (height - 34) / self.fig.dpi)
            self.fig.set_size_inches(new_w, new_h)
            self.canvas.draw_idle()
        except Exception:
            pass

    def _style_plot_axis(self, ax, title: str, ylabel: str):
        ax.set_facecolor("#ffffff")
        ax.set_title(title, fontsize=10, fontweight="bold", color="#0f172a", pad=8, loc="left")
        ax.set_ylabel(ylabel, fontsize=10, labelpad=8, color="#475569")
        ax.tick_params(axis="both", labelsize=8, colors="#475569", pad=3)
        ax.grid(True, color="#cbd5e1", alpha=0.55, linewidth=0.7)
        ax.set_axisbelow(True)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=7))
        ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
        for spine in ax.spines.values():
            spine.set_color("#cbd5e1")
            spine.set_linewidth(0.8)

    @staticmethod
    def _axis_bounds(values, pad_fraction: float = 0.10, min_pad: float = 1.0):
        if not values:
            return -1.0, 1.0
        v_min = min(values)
        v_max = max(values)
        if v_min == v_max:
            pad = max(abs(v_min) * pad_fraction, min_pad)
        else:
            pad = max((v_max - v_min) * pad_fraction, min_pad)
        return v_min - pad, v_max + pad

    def _reset_plot_limits(self):
        for ax in self.plot_axes:
            ax.set_xlim(0.0, 1.0)
            ax.set_ylim(-1.0, 1.0)

    def _update_plot_axis(self, ax, line, t, values):
        line.set_data(t, values)
        if t:
            x_min = t[0]
            x_max = t[-1]
            if x_min == x_max:
                x_max = x_min + 1.0
            ax.set_xlim(x_min, x_max)
        y_min, y_max = self._axis_bounds(values)
        ax.set_ylim(y_min, y_max)

    # -----------------------------------------------------------------------
    # События и лог
    # -----------------------------------------------------------------------
    def _log_event(self, level: str, message: str):
        tag = level
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] [{level.upper()}] {message}\n"
        self.event_log.configure(state="normal")
        self.event_log.insert("end", line, tag)
        self.event_log.configure(state="disabled")
        self.event_log.see("end")
        self.event_log.tag_config("alarm", foreground="red")
        self.event_log.tag_config("warn", foreground="orange")
        self.event_log.tag_config("info", foreground="black")

    def _copy_log(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.event_log.get("1.0", "end"))

    def _clear_log(self):
        self.event_log.configure(state="normal")
        self.event_log.delete("1.0", "end")
        self.event_log.configure(state="disabled")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_port["values"] = ports + [""]
        if ports and not self.var_port.get():
            self.var_port.set(ports[0])

    def _choose_log_path(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if path:
            self.var_log_path.set(path)

    # -----------------------------------------------------------------------
    # Подключение
    # -----------------------------------------------------------------------
    def _connect(self):
        if self.connected:
            return
        port = self.var_port.get().strip()
        baud = int(self.var_baud.get())
        sim = self.var_sim.get()
        if not sim and not port:
            messagebox.showerror("Ошибка", "Укажите COM-порт или включите симуляцию")
            return

        self.rx_queue = queue.Queue()
        self.serial_thread = SerialThread(
            port=port, baud=baud, rx_queue=self.rx_queue,
            on_event=lambda lvl, msg: self.root.after(0, lambda: self._log_event(lvl, msg)),
            simulation=sim
        )
        self.serial_thread.start()
        self.connected = True
        self.var_status.set(f"Подключено{' (симуляция)' if sim else ''}")
        self.btn_connect.configure(state="disabled")
        self.btn_disconnect.configure(state="normal")
        self._log_event("info", f"Подключение {'(симуляция)' if sim else port + ' @ ' + str(baud)}")

    def _disconnect(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
        self.connected = False
        self.var_status.set("Отключено")
        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        self._log_event("info", "Отключение")

    def _send(self, pkt: bytes, desc: str = ""):
        if not self.connected or not self.serial_thread:
            messagebox.showerror("Ошибка", "Нет подключения")
            return False
        self.serial_thread.send(pkt)
        if desc:
            self._log_event("info", f"Отправлено: {desc}")
        return True

    # -----------------------------------------------------------------------
    # Команды режимов
    # -----------------------------------------------------------------------
    def _begin_mode1_tracking(self):
        self.test_active = True
        self.test_mode = 1
        self.test_start_time = time.monotonic()
        self.mode1_seen_running = False
        self.cycles_done = 0
        self.cycle_state = self._new_cycle_state()
        self.live_labels["cycles"].configure(text="0")
        if self.serial_thread and self.var_sim.get():
            self.serial_thread.set_sim_profile("mode1", {
                "min_angle": self.var_m1_min.get(),
                "max_angle": self.var_m1_max.get(),
                "freq": self.var_m1_freq.get(),
            })

    def _start_calibration(self):
        if self._send(build_packet(MT_CALIBRATION, b"\x00"), "CALIBRATION"):
            self.test_active = False
            self.test_mode = None
            self.mode1_seen_running = False
            self._log_event("info", "Запущена калибровка")

    def _send_mode1_config(self):
        try:
            payload = struct.pack(
                MODE1_CONFIG_FMT,
                float(self.var_m1_min.get()),
                float(self.var_m1_max.get()),
                float(self.var_m1_freq.get()),
                int(self.var_m1_cycles.get()),
                float(self.var_m1_moment.get()),
            )
        except Exception as e:
            messagebox.showerror("Ошибка", f"Неверные параметры режима 1: {e}")
            return False
        return self._send(build_packet(MT_MODE1_CONFIG, payload), "MODE1_CONFIG")

    def _start_mode1(self):
        if self._send(build_packet(MT_MODE1_START, b"\x00"), "MODE1_START"):
            self._begin_mode1_tracking()
            self._log_event("info", "Режим 1: отправлен старт")

    def _stop_mode1(self):
        self._send(build_packet(MT_MODE1_STOP, b"\x00"), "MODE1_STOP")
        self.test_active = False
        self.test_mode = None
        self.mode1_seen_running = False
        self._log_event("info", "Режим 1 остановлен")
        if self.serial_thread:
            self.serial_thread.set_sim_profile(None, None)

    def _send_mode2_config(self):
        try:
            # mode2_config_t: speed, start_angle, end_angle, hold_ms.
            # hold_ms is kept as zero for protocol compatibility.
            payload = struct.pack(
                MODE2_CONFIG_FMT,
                float(self.var_m2_speed.get()),
                float(self.var_m2_start_angle.get()),
                float(self.var_m2_end_angle.get()),
                0,
            )
        except Exception as e:
            messagebox.showerror("Ошибка", f"Неверные параметры режима 2: {e}")
            return False
        return self._send(build_packet(MT_MODE2_CONFIG, payload), "MODE2_CONFIG")

    def _start_mode2(self):
        if self._send(build_packet(MT_MODE2_START, b"\x00"), "MODE2_START"):
            self.test_active = True
            self.test_mode = 2
            self.test_start_time = time.monotonic()
            self._log_event("info", "Режим 2: отправлен старт движения")

    def _stop_mode2(self):
        self._send(build_packet(MT_MODE2_STOP, b"\x00"), "MODE2_STOP")
        self.test_active = False
        self.test_mode = None
        self._log_event("info", "Режим 2 остановлен")

    def _zero_load(self):
        if self._send(build_packet(MT_LOAD_ZERO, b"\x00"), "LOAD_ZERO"):
            self._log_event("info", "Запрошен сброс нуля нагрузки")

    # -----------------------------------------------------------------------
    # Логирование в CSV
    # -----------------------------------------------------------------------
    def _start_logging(self):
        path = self.var_log_path.get()
        try:
            if os.path.exists(path) and os.path.getsize(path) > 0:
                with open(path, newline="", encoding="utf-8") as existing:
                    if next(csv.reader(existing), None) != CSV_FIELDS:
                        raise ValueError("В этом CSV другой набор колонок. Выберите новый файл для записи диагностики.")
            self.csv_file_obj = open(path, "a", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file_obj)
            if os.path.getsize(path) == 0:
                self.csv_writer.writerow(CSV_FIELDS)
            self.btn_log_start.configure(state="disabled")
            self.btn_log_stop.configure(state="normal")
            self._log_event("info", f"Запись лога начата: {path}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось открыть файл лога: {e}")

    def _stop_logging(self):
        if self.csv_file_obj:
            self.csv_file_obj.close()
            self.csv_file_obj = None
            self.csv_writer = None
        self.btn_log_start.configure(state="normal")
        self.btn_log_stop.configure(state="disabled")
        self._log_event("info", "Запись лога остановлена")

    # -----------------------------------------------------------------------
    # Безопасность
    # -----------------------------------------------------------------------
    def _check_safety(self, sample: dict):
        events = []
        temp = sample["temp"]
        load = sample["load"]
        angle = sample["angle"]

        temp_max = self.var_safe_temp_max.get()
        temp_resume = self.var_safe_temp_resume.get()
        load_max = self.var_safe_load_max.get()
        angle_tol = self.var_safe_angle_tol.get()

        if not self.overheat and temp > temp_max:
            self.overheat = True
            events.append({
                "level": "alarm", "type": "OVERHEAT",
                "message": f"Температура {temp:.2f}°C превысила {temp_max:.2f}°C"
            })
        elif self.overheat and temp < temp_resume:
            self.overheat = False
            events.append({
                "level": "info", "type": "TEMP_RECOVERED",
                "message": f"Температура {temp:.2f}°C ниже порога {temp_resume:.2f}°C"
            })

        if load > load_max:
            events.append({
                "level": "alarm", "type": "LOAD_MAX",
                "message": f"Нагрузка {load:.2f} кг превысила {load_max:.2f} кг"
            })

        # Проверка соответствия угла расчётному (только режим 1)
        if self.test_active and self.test_mode == 1 and self.test_start_time is not None:
            expected = self._expected_mode1_angle(time.monotonic())
            if expected is not None and abs(angle - expected) > angle_tol:
                events.append({
                    "level": "alarm", "type": "ANGLE_MISMATCH",
                    "message": f"Угол {angle:.2f}° не совпадает с расчётным {expected:.2f}°"
                })

        return events

    def _expected_mode1_angle(self, now: float) -> float:
        cfg = {
            "min": self.var_m1_min.get(),
            "max": self.var_m1_max.get(),
            "freq": self.var_m1_freq.get(),
        }
        expected_angle, _ = mode1_motion_profile(
            now - self.test_start_time, cfg["min"], cfg["max"], cfg["freq"]
        )
        return expected_angle

    def _update_cycles(self, angle: float):
        if not (self.test_active and self.test_mode == 1):
            return
        a_min = self.var_m1_min.get()
        a_max = self.var_m1_max.get()
        tol = self.var_safe_angle_tol.get()
        near_max = angle >= (a_max - tol)
        near_min = angle <= (a_min + tol)
        st = self.cycle_state
        if near_max and st["half"] == 0:
            st["half"] = 1
        if near_min and st["half"] == 1:
            st["half"] = 0
            self.cycles_done += 1
            self._log_event("info", f"Цикл {self.cycles_done}")

    # -----------------------------------------------------------------------
    # Обновление UI
    # -----------------------------------------------------------------------
    @staticmethod
    def _new_cycle_state():
        return {
            "last_angle": None,
            "direction": 0,
            "last_turn_angle": None,
            "last_turn_ts": 0.0,
            "seen_peak": False,
        }

    def _update_cycles_from_telemetry(self, sample: dict):
        if not (self.test_active and self.test_mode == 1):
            return

        stm_state = int(sample.get("flags", 0)) & 0xFF
        if stm_state != MT_MODE1_START:
            self.cycle_state = self._new_cycle_state()
            return

        angle = float(sample["angle"])
        now = float(sample["pc_ts"])
        st = self.cycle_state

        last_angle = st["last_angle"]
        if last_angle is None:
            st["last_angle"] = angle
            st["last_turn_angle"] = angle
            st["last_turn_ts"] = now
            return

        delta = angle - last_angle
        st["last_angle"] = angle
        if abs(delta) < CYCLE_DIRECTION_EPS_DEG:
            return

        new_direction = 1 if delta > 0.0 else -1
        if st["direction"] == 0:
            st["direction"] = new_direction
            return
        if new_direction == st["direction"]:
            return

        last_turn_angle = st["last_turn_angle"]
        if last_turn_angle is not None and abs(angle - last_turn_angle) < CYCLE_TURN_MIN_DELTA_DEG:
            return
        if (now - st["last_turn_ts"]) < CYCLE_TURN_DEBOUNCE_S:
            return

        previous_direction = st["direction"]
        st["direction"] = new_direction
        st["last_turn_angle"] = angle
        st["last_turn_ts"] = now

        if previous_direction > 0 and new_direction < 0:
            st["seen_peak"] = True
        elif previous_direction < 0 and new_direction > 0 and st["seen_peak"]:
            st["seen_peak"] = False
            self.cycles_done += 1
            self._log_event("info", f"Cycle {self.cycles_done}")

    def _update_ui(self):
        # Забираем пакеты из очереди
        while True:
            try:
                mt, pl = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_packet(mt, pl)

        self._redraw_plots()
        self.root.after(100, self._update_ui)

    def _handle_packet(self, msg_type: int, payload: bytes):
        if msg_type == MT_STREAM_DATA:
            try:
                sample = decode_stream_payload(payload)
            except (ValueError, struct.error) as e:
                self._log_event("warn", f"Ошибка распаковки телеметрии: {e}")
                return
            sample["pc_ts"] = time.time()
            cycles_from_firmware = sample["cycles"]
            flags = sample["flags"]
            self.samples.append(sample)
            if cycles_from_firmware is None:
                self._update_cycles_from_telemetry(sample)
            else:
                if cycles_from_firmware != self.cycles_done:
                    self.cycles_done = cycles_from_firmware
                    if self.test_active and self.test_mode == 1:
                        self._log_event("info", f"Cycle {self.cycles_done}")

            # Безопасность
            events = self._check_safety(sample)
            for ev in events:
                self._log_event(ev["level"], f"[{ev['type']}] {ev['message']}")
                if ev["type"] == "OVERHEAT":
                    self.paused_by_overheat = True
                    self._send(build_packet(MT_MODE1_STOP, b"\x00"), "STOP из-за перегрева")
                elif ev["type"] == "TEMP_RECOVERED" and self.paused_by_overheat and self.var_safe_auto_resume.get():
                    self.paused_by_overheat = False
                    self._send(build_packet(MT_MODE1_START, b"\x00"), "Автозапуск после охлаждения")
                elif ev["level"] == "alarm":
                    self.test_active = False
                    self.test_mode = None
                    if self.serial_thread:
                        self.serial_thread.set_sim_profile(None, None)

            # Статус СТМ из флагов
            stm_state = int(flags) & 0xFF
            calib_substate = (int(flags) >> 8) & 0xFF
            state_name = STM_STATES.get(stm_state, f"Состояние 0x{stm_state:02X}")
            if stm_state == 0x01:
                calib_name = CALIB_STATES.get(calib_substate, f"подсостояние {calib_substate}")
                state_name += f" | {calib_name}"
            base_status = self.var_status.get().split(" | ")[0]
            self.var_status.set(f"{base_status} | {state_name}")
            if self.test_active and self.test_mode == 1 and stm_state == MT_MODE1_START:
                self.mode1_seen_running = True
            if stm_state == 0x33 and self.test_active:
                if self.test_mode == 1:
                    if self.mode1_seen_running:
                        self._log_event("warn", "STM state = ACK")
                    else:
                        self._log_event("info", "Mode1 state = ACK (not running; check calibration/config)")
                    self.mode1_seen_running = False
                elif self.test_mode == 2:
                    self._log_event("info", "Mode2 state = ACK (idle/waiting for START)")
                else:
                    self._log_event("warn", "STM state = ACK")
                self.test_active = False
                self.test_mode = None

            # Запись в CSV
            if self.csv_writer:
                self.csv_writer.writerow([
                    datetime.now().isoformat(), sample["ts_ms"], sample["angle"],
                    sample["omega"], sample["load"], sample["pos"],
                    sample["temp"], sample["flags"], self.cycles_done,
                    sample["target_omega"], sample["motor_rpm"], sample["motor_command"], sample["control_flags"]
                ])

        elif msg_type == MT_MODE1_START:
            self._log_event("info", "Получен MODE1_START")
        elif msg_type == MT_MODE1_STOP:
            self._log_event("info", "Получен MODE1_STOP")
        elif msg_type == MT_MODE2_START:
            self._log_event("info", "Получен MODE2_START")
        elif msg_type == MT_MODE2_STOP:
            self._log_event("info", "Получен MODE2_STOP")
        else:
            self._log_event("info", f"Пакет типа 0x{msg_type:02X}, {len(payload)} байт")

    def _redraw_plots(self):
        if not self.samples:
            return

        N = min(len(self.samples), 500)
        data = list(self.samples)[-N:]
        t0 = data[0]["pc_ts"]
        t = [s["pc_ts"] - t0 for s in data]
        angles = [s["angle"] for s in data]
        omegas = [s["omega"] for s in data]
        loads = [s["load"] for s in data]

        self._update_plot_axis(self.ax_angle, self.plot_lines["angle"], t, angles)
        self._update_plot_axis(self.ax_omega, self.plot_lines["omega"], t, omegas)
        self._update_plot_axis(self.ax_load, self.plot_lines["load"], t, loads)
        self.canvas.draw_idle()

        # Обновление цифр
        last = data[-1]
        self.live_labels["angle"].configure(text=f"{last['angle']:.2f}")
        self.live_labels["omega"].configure(text=f"{last['omega']:.2f}")
        self.live_labels["load"].configure(text=f"{last['load']:.1f}")
        self.live_labels["pos"].configure(text=f"{last['pos']:.2f}")
        self.live_labels["temp"].configure(text=f"{last['temp']:.2f}")
        self.live_labels["cycles"].configure(text=str(self.cycles_done))

    def _clear_plots(self):
        self.samples.clear()
        for line in self.plot_lines.values():
            line.set_data([], [])
        self._reset_plot_limits()
        self.canvas.draw_idle()

    def on_close(self):
        self._disconnect()
        self._stop_logging()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="KM Stand UI")
    parser.add_argument("--sim", action="store_true", help="Запустить в режиме симуляции")
    args = parser.parse_args()

    root = Tk()
    app = KMStandApp(root, simulation_default=args.sim)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
