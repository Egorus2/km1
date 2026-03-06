import struct

SYNC1 = 0xFF
SYNC2 = 0xFA
END = 0x0A

MT_MODE1_CONFIG = 0x10
MT_MODE1_START = 0x11
MT_MODE1_STOP = 0x12
MT_MODE2_CONFIG = 0x20
MT_MODE2_START = 0x21
MT_MODE2_STOP = 0x22
MT_MODE3_KM_REQ = 0x30
MT_MODE3_KM_RESP = 0x31
MT_MODE3_TORQUE = 0x32
MT_ACK = 0x33
MT_STREAM_DATA = 0x40


def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000):
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def build_packet(msg_type: int, payload: bytes = b"") -> bytes:
    if not isinstance(payload, (bytes, bytearray)):
        raise TypeError("payload must be bytes")
    if len(payload) > 255:
        raise ValueError("payload too long (max 255)")

    header = bytes([SYNC1, SYNC2, msg_type & 0xFF, len(payload) & 0xFF])
    crc_input = bytes([msg_type & 0xFF, len(payload) & 0xFF]) + payload
    crc = crc16_ccitt(crc_input)
    crc_bytes = struct.pack("<H", crc)
    return header + payload + crc_bytes + bytes([END])


class PacketParser:
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
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < 7:
                break

            msg_type = self.buf[2]
            length = self.buf[3]
            total_len = 2 + 1 + 1 + length + 2 + 1
            if len(self.buf) < total_len:
                break

            pkt = bytes(self.buf[:total_len])
            if pkt[-1] != END:
                del self.buf[0]
                continue

            payload = pkt[4:4 + length]
            crc_recv = struct.unpack("<H", pkt[4 + length:4 + length + 2])[0]
            crc_calc = crc16_ccitt(bytes([msg_type, length]) + payload)
            if crc_recv != crc_calc:
                del self.buf[0]
                continue

            packets.append((msg_type, payload))
            del self.buf[:total_len]
        return packets
