from django.contrib import admin
from .models import TestRun, TelemetrySample, EventLog

@admin.register(TestRun)
class TestRunAdmin(admin.ModelAdmin):
    list_display = ("id", "mode", "status", "started_at", "ended_at", "cycles_target", "cycles_done")

@admin.register(TelemetrySample)
class TelemetryAdmin(admin.ModelAdmin):
    list_display = ("id", "test_run", "ts_ms", "angle_deg", "pos_mm", "load_n", "temp_c", "created_at")

@admin.register(EventLog)
class EventLogAdmin(admin.ModelAdmin):
    list_display = ("id", "test_run", "level", "type", "ts")