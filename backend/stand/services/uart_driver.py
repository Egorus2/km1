import time
import threading
import collections
import struct
import numpy as np

try:
    import serial
except Exception:
    serial = None

from .protocol import PacketParser, build_packet, MT_STREAM_DATA


class UARTDriver(threading.Thread):

    def __init__(self, port: str | None, baud: int, on_packet, simulation: bool = False, sim_rate_hz: int = 50):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.on_packet = on_packet
        self.simulation = bool(simulation)
        self.port_open = False
        self.last_error = None
        self.ready = threading.Event()

        self.alive = False
        self.ser = None
        self.parser = PacketParser()

        self.sim_rate_hz = sim_rate_hz
        self.send_queue = collections.deque()
        self.send_lock = threading.Lock()

        self.sim_profile = {"mode": None, "config": None}
        self.sim_faults = {
            "angle_offset_deg": 0.0,
            "angle_spike_deg": 0.0,
            "angle_noise_sigma": 0.8,
            "temp_offset_c": 0.0,
            "load_offset_n": 0.0,
            "drop_rate": 0.0,
        }
        self._sim_start_wall = time.time()
        self._sim_packet_counter = 0

    def run(self):
        self.alive = True

        if not self.simulation:
            if serial is None:
                self.last_error = "pyserial not installed"
                self.ready.set()
                return
            if self.port is None:
                self.last_error = "port is None"
                self.ready.set()
                return

            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.2
                )
                self.port_open = True
                self._flush_queue()
                self.ready.set()
            except Exception as e:
                self.last_error = f"Failed to open serial: {e}"
                self.ser = None
                self.port_open = False
                self.ready.set()
                return
        else:
            self.ready.set()

        last_sim = time.time()
        sim_interval = 1.0 / max(1, self.sim_rate_hz)

        while self.alive:
            try:
                if self.simulation:
                    now = time.time()
                    if now - last_sim >= sim_interval:
                        last_sim = now
                        pkt = self._build_sim_stream_packet()
                        if not pkt:
                            continue
                        for (mt, pl) in self.parser.feed(pkt):
                            self.on_packet(mt, pl)
                    time.sleep(min(0.01, sim_interval / 2))
                else:
                    data = self.ser.read(512)
                    if data:
                        for (mt, pl) in self.parser.feed(data):
                            self.on_packet(mt, pl)
            except Exception:
                self.alive = False

        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

    def stop(self):
        self.alive = False

    def send(self, pkt: bytes):
        # если порта нет — в очередь
        with self.send_lock:
            if self.ser is None:
                self.send_queue.append(pkt)
                return

        try:
            for b in pkt:
                self.ser.write(bytes([b]))
                time.sleep(0.05)
        except Exception:
            with self.send_lock:
                self.send_queue.append(pkt)

    def _flush_queue(self):
        with self.send_lock:
            while self.send_queue and self.ser is not None:
                pkt = self.send_queue.popleft()
                try:
                    self.ser.write(pkt)
                except Exception:
                    self.send_queue.appendleft(pkt)
                    break

    def _build_sim_stream_packet(self):
        self._sim_packet_counter += 1

        ts_ms = int((time.time() % 1000000) * 1000) & 0xFFFFFFFF

        t = time.time() - self._sim_start_wall

        if self.sim_profile["mode"] == "mode1" and self.sim_profile["config"]:
            cfg = self.sim_profile["config"]
            a_min = float(cfg.get("min_angle_deg", 50.0))
            a_max = float(cfg.get("max_angle_deg", 180.0))
            f_hz = float(cfg.get("freq_hz", 0.5))
            angle1 = self._triangle_angle(t, a_min, a_max, f_hz)
        else:
            angle1 = 90.0 + 10.0 * np.sin(2 * np.pi * 0.2 * t)

        load = 500.0 + 50.0 * np.sin(2 * np.pi * 0.2 * t)
        pos = 100.0 + 10.0 * np.sin(2 * np.pi * 0.1 * t + 0.3)
        temp = 35.0 + 2.0 * np.sin(2 * np.pi * 0.02 * t)

        f = self.sim_faults

        angle1 += np.random.randn() * float(f.get("angle_noise_sigma", 0.8))
        load += np.random.randn() * 5.0
        pos += np.random.randn() * 0.2
        temp += np.random.randn() * 0.05

        angle1 += float(f.get("angle_offset_deg", 0.0))

        spike = float(f.get("angle_spike_deg", 0.0))
        if spike != 0.0:
            angle1 += spike
            self.sim_faults["angle_spike_deg"] = 0.0

        temp += float(f.get("temp_offset_c", 0.0))
        load += float(f.get("load_offset_n", 0.0))

        drop_rate = float(f.get("drop_rate", 0.0))
        if drop_rate > 0.0 and np.random.rand() < drop_rate:
            return b""

        a2 = 0.0
        flags = 0
        payload = struct.pack("<IfffffH", ts_ms, float(load), float(pos), float(angle1), float(a2), float(temp),
                              int(flags))
        return build_packet(MT_STREAM_DATA, payload)

    def set_sim_profile(self, mode: str | None, config: dict | None):
        self.sim_profile["mode"] = mode
        self.sim_profile["config"] = config

    def set_sim_faults(self, **kwargs):
        for k, v in kwargs.items():
            if k in self.sim_faults and v is not None:
                self.sim_faults[k] = float(v)

    @staticmethod
    def _triangle_angle(t: float, a_min: float, a_max: float, f_hz: float) -> float:
        if f_hz <= 0:
            return a_min
        T = 1.0 / f_hz
        x = (t % T) / T
        tri = 1.0 - abs(2.0 * x - 1.0)
        return a_min + (a_max - a_min) * tri
