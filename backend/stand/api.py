from ninja import NinjaAPI, Schema
from typing import Optional, List
from django.http import HttpResponse
import csv

from stand.models import TestRun, EventLog, TelemetrySample
from stand.services.manager import StandManager

api = NinjaAPI(title="KM Stand API")


class ConnectIn(Schema):
    port: Optional[str] = None
    baud: int = 115200
    simulation: bool = True


class Mode1Config(Schema):
    min_angle_deg: float = 50.0
    max_angle_deg: float = 180.0
    freq_hz: float = 0.5
    cycles: int = 630000
    moment_nm: float = 89.0


class SafetyCfg(Schema):
    temp_max_c: float = 80.0
    temp_resume_c: float = 70.0
    allow_auto_resume: bool = False
    load_max_n: float = 5000.0
    angle_tolerance_deg: float = 5.0


class StartMode1In(Schema):
    config: Mode1Config
    safety: SafetyCfg


@api.post("connect")
def connect(request, payload: ConnectIn):
    mgr = StandManager.get()
    try:
        mgr.connect(payload.port, payload.baud, payload.simulation)
        return mgr.status()
    except Exception as e:
        return {"ok": False, "error": str(e)}


@api.post("disconnect")
def disconnect(request):
    mgr = StandManager.get()
    mgr.disconnect()
    return {"ok": True}


@api.get("status")
def status(request):
    return StandManager.get().status()


@api.post("test/mode1/start")
def start_mode1(request, payload: StartMode1In):
    mgr = StandManager.get()
    tr = mgr.start_mode1(config=payload.config.dict(), safety=payload.safety.dict())
    return {"ok": True, "test": {"id": tr.id}}


@api.post("test/stop")
def stop_test(request):
    mgr = StandManager.get()
    tr = mgr.stop_test(reason="manual")
    return {"ok": True, "test": {"id": tr.id} if tr else None}


@api.get("test/{test_id}/events")
def test_events(request, test_id: int):
    qs = EventLog.objects.filter(test_run_id=test_id).order_by("-ts")[:500]
    return [
        {"ts": e.ts.isoformat(), "level": e.level, "type": e.type, "message": e.message, "details": e.details}
        for e in qs
    ]


@api.get("test/{test_id}/telemetry/latest")
def latest(request, test_id: int):
    s = TelemetrySample.objects.filter(test_run_id=test_id).order_by("-id").first()
    if not s:
        return None
    return {
        "ts_ms": s.ts_ms,
        "angle_deg": s.angle_deg,
        "pos_mm": s.pos_mm,
        "load_n": s.load_n,
        "temp_c": s.temp_c,
        "flags": s.flags,
        "created_at": s.created_at.isoformat(),
    }


@api.get("test/{test_id}/export.csv")
def export_csv(request, test_id: int):
    resp = HttpResponse(content_type="text/csv")
    resp["Content-Disposition"] = f'attachment; filename="test_{test_id}.csv"'
    w = csv.writer(resp)
    w.writerow(["ts_ms", "angle_deg", "pos_mm", "load_n", "temp_c", "flags", "created_at"])
    for s in TelemetrySample.objects.filter(test_run_id=test_id).order_by("id").iterator(chunk_size=2000):
        w.writerow([s.ts_ms, s.angle_deg, s.pos_mm, s.load_n, s.temp_c, s.flags, s.created_at.isoformat()])
    return resp
