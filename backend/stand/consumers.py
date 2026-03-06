import json
from channels.generic.websocket import AsyncWebsocketConsumer


class TelemetryConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        from stand.services.manager import StandManager

        mgr = StandManager.get()
        st = mgr.status()

        if not mgr.connected:
            await self.close()
            return

        await self.channel_layer.group_add("telemetry", self.channel_name)
        await self.accept()

        if st.get("current_test"):
            ct = st["current_test"]
            await self.send(text_data=json.dumps({
                "type": "cycles",
                "cycles_done": ct.get("cycles_done", 0),
                "cycles_target": ct.get("cycles_target"),
            }))

        await self.send(text_data=json.dumps({
            "type": "status",
            "data": st,
        }))

    async def disconnect(self, close_code):
        await self.channel_layer.group_discard("telemetry", self.channel_name)

    async def telemetry_message(self, event):
        await self.send(text_data=json.dumps(event["data"]))