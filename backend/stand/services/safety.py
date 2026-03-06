import math
from dataclasses import dataclass
from datetime import datetime, timezone


@dataclass
class SafetyConfig:
    temp_max_c: float = 80.0
    temp_resume_c: float = 70.0
    allow_auto_resume: bool = False
    load_max_n: float = 5000.0  # 5kN
    angle_tolerance_deg: float = 5.0


class SafetyMonitor:
    def __init__(self, cfg: SafetyConfig):
        self.cfg = cfg
        self.overheat = False

    def check(self, sample: dict, expected_angle_deg: float | None = None) -> list[dict]:
        events = []

        temp = float(sample["temp_c"])
        load = float(sample["load_n"])
        angle = float(sample["angle_deg"])

        if (not self.overheat) and (temp > self.cfg.temp_max_c):
            self.overheat = True
            events.append({
                "level": "alarm",
                "type": "OVERHEAT",
                "message": f"Температура {temp:.2f}°C выше предела {self.cfg.temp_max_c:.2f}°C",
                "details": {"temp_c": temp, "temp_max_c": self.cfg.temp_max_c},
            })

        elif self.overheat and (temp < self.cfg.temp_resume_c):
            self.overheat = False
            events.append({
                "level": "info",
                "type": "TEMP_RECOVERED",
                "message": f"Температура {temp:.2f}°C ниже порога возобновления {self.cfg.temp_resume_c:.2f}°C",
                "details": {"temp_c": temp, "temp_resume_c": self.cfg.temp_resume_c},
            })

        if load > self.cfg.load_max_n:
            events.append({
                "level": "alarm",
                "type": "LOAD_MAX",
                "message": f"Нагрузка {load:.2f}N выше предела {self.cfg.load_max_n:.2f}N",
                "details": {"load_n": load, "load_max_n": self.cfg.load_max_n},
            })

        if expected_angle_deg is not None:
            if abs(angle - expected_angle_deg) > self.cfg.angle_tolerance_deg:
                events.append({
                    "level": "alarm",
                    "type": "ANGLE_MISMATCH",
                    "message": (
                        f"Угол {angle:.2f}° не совпадает с расчетным {expected_angle_deg:.2f}° "
                        f"(tolerance {self.cfg.angle_tolerance_deg:.2f}°)"
                    ),
                    "details": {
                        "angle_deg": angle,
                        "expected_angle_deg": expected_angle_deg,
                        "tol": self.cfg.angle_tolerance_deg
                    },
                })

        return events
