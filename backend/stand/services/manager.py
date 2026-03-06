import os
import time
import math
import struct
import threading
from dataclasses import asdict

from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from django.utils import timezone

from stand.models import TestRun, TelemetrySample, EventLog
from .protocol import (
    MT_STREAM_DATA, MT_ACK,
    MT_MODE1_CONFIG, MT_MODE1_START, MT_MODE1_STOP,
    build_packet
)
from .uart_driver import UARTDriver
from .safety import SafetyConfig, SafetyMonitor

_channel = get_channel_layer()


def ws_broadcast(message: dict):
    async_to_sync(_channel.group_send)("telemetry", {"type": "telemetry.message", "data": message})


class StandManager:
    _lock = threading.Lock()
    _instance = None

    def __init__(self):
        self.mode1_started_monotonic: float | None = None
        self.mode1_phase_offset_s: float = 0.0
        self.paused_by_overheat: bool = False
        self._cycle_state = {"at_min": False, "at_max": False, "half": 0}

        self.driver: UARTDriver | None = None
        self.connected = False
        self.simulation = True
        self.port = None
        self.baud = 115200

        self.current_test_id: int | None = None
        self.safety: SafetyMonitor | None = None

        self.mode1_cfg = {
            "min_angle_deg": 50.0,
            "max_angle_deg": 180.0,
            "freq_hz": 0.5,
            "cycles": 630000,
            "moment_nm": 89.0,
        }

    @classmethod
    def get(cls) -> "StandManager":
        with cls._lock:
            if cls._instance is None:
                cls._instance = StandManager()
            return cls._instance

    def connect(self, port: str | None, baud: int, simulation: bool):
        if not simulation and not port:
            raise RuntimeError("Port is required when simulation=false")

        if self.connected:
            return

        self.port = port
        self.baud = baud
        self.driver = UARTDriver(port=port, baud=baud, on_packet=self.on_packet, simulation=simulation, sim_rate_hz=50)
        self.driver.start()

        self.driver.ready.wait(timeout=2.0)

        if not simulation:
            if not self.driver.port_open:
                err = self.driver.last_error or "unknown connect error"

                self.driver.stop()
                self.driver = None
                self.connected = False
                raise RuntimeError(f"UART connect failed: {err}")

        self.connected = True
        self.simulation = bool(simulation)

        self._log(None, "info", "CONNECTED", f"Подключено (simulation={simulation}, port={port}, baud={baud})", {})

    def disconnect(self):
        if self.driver:
            self.driver.stop()
        self.driver = None
        self.connected = False
        self._log(None, "info", "DISCONNECTED", "Отключено", {})

    def status(self) -> dict:
        test = TestRun.objects.filter(id=self.current_test_id).first() if self.current_test_id else None
        return {
            "connected": self.connected,
            "simulation": self.simulation,
            "port_open": bool(self.driver and self.driver.port_open),
            "driver_error": (self.driver.last_error if self.driver else None),
            "port": self.port,
            "baud": self.baud,
            "current_test": self._test_to_dict(test) if test else None,
        }

    def start_mode1(self, config: dict, safety: dict):
        if not self.connected:
            raise RuntimeError("Not connected")

        # create TestRun
        tr = TestRun.objects.create(
            mode=TestRun.Mode.MODE1,
            status=TestRun.Status.RUNNING,
            started_at=timezone.now(),
            config=config,
            safety=safety,
            cycles_target=int(config.get("cycles", 0)),
        )
        self.current_test_id = tr.id

        self.mode1_cfg = config
        self.mode1_started_monotonic = time.monotonic()
        self.mode1_phase_offset_s = 0.0
        self.paused_by_overheat = False
        self.driver.set_sim_faults(angle_offset_deg=10.0)

        cfg = SafetyConfig(
            temp_max_c=float(safety.get("temp_max_c", 80.0)),
            temp_resume_c=float(safety.get("temp_resume_c", 70.0)),
            allow_auto_resume=bool(safety.get("allow_auto_resume", False)),
            load_max_n=float(safety.get("load_max_n", 5000.0)),
            angle_tolerance_deg=float(safety.get("angle_tolerance_deg", 5.0)),
        )
        self.safety = SafetyMonitor(cfg)

        # отправка config на плату (пример: freq(float), cycles(u16?) moment(float))
        # ВНИМАНИЕ: uint16, но нужно 630k => это не влезает.
        # Посмотреть порядок и данные которые отправляются !!!!
        payload = struct.pack("<ffIf", float(config["freq_hz"]), float(config["min_angle_deg"]), int(config["cycles"]),
                              float(config["moment_nm"]))
        pkt_cfg = build_packet(MT_MODE1_CONFIG, payload)
        self.driver.send(pkt_cfg)

        pkt_start = build_packet(MT_MODE1_START, b"\x00")
        self.driver.send(pkt_start)

        self._log(tr.id, "info", "MODE1_START", "Запуск режима 1", {"config": config, "safety": safety})
        ws_broadcast({"type": "test", "event": "started", "test": self._test_to_dict(tr)})

        return tr

    def stop_test(self, reason: str = "manual"):
        tr = TestRun.objects.filter(id=self.current_test_id).first() if self.current_test_id else None
        if not tr:
            return None

        if self.driver and tr.mode == TestRun.Mode.MODE1:
            self.driver.send(build_packet(MT_MODE1_STOP, b"\x00"))

        tr.status = TestRun.Status.STOPPED if reason == "manual" else TestRun.Status.FAILED
        tr.ended_at = timezone.now()
        tr.save(update_fields=["status", "ended_at"])

        self._log(tr.id, "info" if reason == "manual" else "alarm", "TEST_STOP", f"Останов испытания ({reason})", {})
        ws_broadcast({"type": "test", "event": "stopped", "reason": reason, "test": self._test_to_dict(tr)})

        if self.driver and self.simulation:
            self.driver.set_sim_profile(None, None)
            self.driver.set_sim_faults(
                angle_offset_deg=0.0,
                angle_spike_deg=0.0,
                temp_offset_c=0.0,
                load_offset_n=0.0,
                drop_rate=0.0,
            )

        self.current_test_id = None
        self.safety = None
        return tr

    def on_packet(self, msg_type: int, payload: bytes):
        if msg_type == MT_STREAM_DATA:
            # формат: <I f f f f f H
            if len(payload) < 26:
                return
            ts_ms, load_n, pos_mm, angle_deg, _a2, temp_c, flags = struct.unpack("<IfffffH", payload[:26])

            sample = {
                "ts_ms": int(ts_ms),
                "load_n": float(load_n),
                "pos_mm": float(pos_mm),
                "angle_deg": float(angle_deg),
                "temp_c": float(temp_c),
                "flags": int(flags),
            }

            tr = TestRun.objects.filter(id=self.current_test_id).first() if self.current_test_id else None

            TelemetrySample.objects.create(
                test_run=tr,
                ts_ms=sample["ts_ms"],
                angle_deg=sample["angle_deg"],
                pos_mm=sample["pos_mm"],
                load_n=sample["load_n"],
                temp_c=sample["temp_c"],
                flags=sample["flags"],
            )

            if tr:
                tr.last_angle_deg = sample["angle_deg"]
                tr.last_pos_mm = sample["pos_mm"]
                tr.last_load_n = sample["load_n"]
                tr.last_temp_c = sample["temp_c"]
                tr.save(update_fields=["last_angle_deg", "last_pos_mm", "last_load_n", "last_temp_c"])

            if tr and tr.mode == TestRun.Mode.MODE1 and tr.status == TestRun.Status.RUNNING:
                if self._mode1_update_cycles(sample["angle_deg"]):
                    tr.cycles_done += 1
                    tr.save(update_fields=["cycles_done"])
                    ws_broadcast({"type": "cycles", "cycles_done": tr.cycles_done, "cycles_target": tr.cycles_target})
                    if tr.cycles_target and tr.cycles_done >= tr.cycles_target:
                        self._log(tr.id, "info", "COMPLETED", "Испытание завершено по количеству циклов", {})
                        self.stop_test(reason="completed")
                        return

            if tr and self.safety:
                expected = None
                if tr.mode == TestRun.Mode.MODE1 and self.mode1_started_monotonic is not None:
                    expected = self._mode1_expected_angle(time.monotonic())

                events = self.safety.check(sample, expected_angle_deg=expected)

                for ev in events:
                    self._log(tr.id, ev["level"], ev["type"], ev["message"], ev.get("details", {}))
                    ws_broadcast(
                        {"type": "event", "level": ev["level"], "event_type": ev["type"], "message": ev["message"]})

                    if ev["type"] == "OVERHEAT":
                        # pause
                        if tr.status == TestRun.Status.RUNNING:
                            tr.status = TestRun.Status.PAUSED
                            tr.save(update_fields=["status"])
                            self.paused_by_overheat = True
                            if self.driver:
                                self.driver.send(build_packet(MT_MODE1_STOP, b"\x00"))
                            ws_broadcast({"type": "test", "event": "paused", "reason": "overheat"})
                        break

                    if ev["type"] == "TEMP_RECOVERED":
                        if self.paused_by_overheat and self.safety.cfg.allow_auto_resume:
                            tr.status = TestRun.Status.RUNNING
                            tr.save(update_fields=["status"])
                            self.paused_by_overheat = False
                            if self.driver:
                                self.driver.send(build_packet(MT_MODE1_START, b"\x00"))
                            ws_broadcast({"type": "test", "event": "resumed", "reason": "temp_recovered"})
                        continue

                    if ev["level"] == "alarm":
                        self.stop_test(reason=ev["type"])
                        break

            ws_broadcast({"type": "telemetry", "sample": sample})

        elif msg_type == MT_ACK:
            self._log(self.current_test_id, "info", "ACK", "ACK получен", {"raw": payload.hex()})
            ws_broadcast({"type": "ack", "payload_hex": payload.hex()})
        else:
            ws_broadcast({"type": "packet", "msg_type": msg_type, "payload_hex": payload.hex()})

    def _log(self, test_id, level: str, type_: str, message: str, details: dict):
        tr = TestRun.objects.filter(id=test_id).first() if test_id else None
        EventLog.objects.create(test_run=tr, level=level, type=type_, message=message, details=details)

    def _test_to_dict(self, tr: TestRun) -> dict:
        return {
            "id": tr.id,
            "mode": tr.mode,
            "status": tr.status,
            "started_at": tr.started_at.isoformat() if tr.started_at else None,
            "ended_at": tr.ended_at.isoformat() if tr.ended_at else None,
            "config": tr.config,
            "safety": tr.safety,
            "cycles_target": tr.cycles_target,
            "cycles_done": tr.cycles_done,
            "last": {
                "angle_deg": tr.last_angle_deg,
                "pos_mm": tr.last_pos_mm,
                "load_n": tr.last_load_n,
                "temp_c": tr.last_temp_c,
            }
        }

    def _mode1_expected_angle(self, now_monotonic: float) -> float:
        cfg = self.mode1_cfg
        a_min = float(cfg["min_angle_deg"])
        a_max = float(cfg["max_angle_deg"])
        f = float(cfg["freq_hz"])
        if f <= 0:
            return a_min

        T = 1.0 / f
        t = (now_monotonic - (self.mode1_started_monotonic or now_monotonic) + self.mode1_phase_offset_s) % T
        x = t / T
        tri = 1.0 - abs(2.0 * x - 1.0)
        return a_min + (a_max - a_min) * tri

    def _mode1_update_cycles(self, angle_deg: float) -> bool:
        cfg = self.mode1_cfg
        a_min = float(cfg["min_angle_deg"])
        a_max = float(cfg["max_angle_deg"])
        tol = float((self.safety.cfg.angle_tolerance_deg if self.safety else 5.0))

        near_min = angle_deg <= (a_min + tol)
        near_max = angle_deg >= (a_max - tol)

        st = self._cycle_state

        if near_max and st["half"] == 0:
            st["half"] = 1

        if near_min and st["half"] == 1:
            st["half"] = 0
            return True

        return False
