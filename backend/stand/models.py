from django.db import models


class TestRun(models.Model):
    class Status(models.TextChoices):
        IDLE = "idle"
        RUNNING = "running"
        STOPPED = "stopped"
        FAILED = "failed"
        COMPLETED = "completed"
        PAUSED = "paused"

    class Mode(models.IntegerChoices):
        MODE1 = 1
        MODE2 = 2
        MODE3 = 3

    mode = models.IntegerField(choices=Mode.choices)
    status = models.CharField(max_length=16, choices=Status.choices, default=Status.IDLE)

    started_at = models.DateTimeField(null=True, blank=True)
    ended_at = models.DateTimeField(null=True, blank=True)

    config = models.JSONField(default=dict, blank=True)
    safety = models.JSONField(default=dict, blank=True)

    cycles_target = models.BigIntegerField(default=0)
    cycles_done = models.BigIntegerField(default=0)

    last_angle_deg = models.FloatField(null=True, blank=True)
    last_pos_mm = models.FloatField(null=True, blank=True)
    last_load_n = models.FloatField(null=True, blank=True)
    last_temp_c = models.FloatField(null=True, blank=True)

    created_at = models.DateTimeField(auto_now_add=True)


class TelemetrySample(models.Model):
    test_run = models.ForeignKey(TestRun, on_delete=models.CASCADE, related_name="samples", null=True, blank=True)

    ts_ms = models.BigIntegerField()  # из устройства
    angle_deg = models.FloatField()
    pos_mm = models.FloatField()
    load_n = models.FloatField()
    temp_c = models.FloatField()
    flags = models.IntegerField(default=0)

    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        indexes = [
            models.Index(fields=["created_at"]),
            models.Index(fields=["ts_ms"]),
        ]


class EventLog(models.Model):
    class Level(models.TextChoices):
        INFO = "info"
        WARN = "warn"
        ALARM = "alarm"

    ts = models.DateTimeField(auto_now_add=True)
    test_run = models.ForeignKey(TestRun, on_delete=models.CASCADE, related_name="events", null=True, blank=True)
    level = models.CharField(max_length=8, choices=Level.choices, default=Level.INFO)
    type = models.CharField(max_length=64)
    message = models.TextField()
    details = models.JSONField(default=dict, blank=True)
