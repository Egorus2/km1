import struct
import unittest

from km_stand_ui import (
    PacketParser, STREAM_FMT, STREAM_FMT_LEGACY, STREAM_DIAG_FMT,
    build_packet, build_packet_like_firmware, decode_stream_payload,
)


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.base = (1000, 2.0, 5.0, 90.0, -3.0, 25.0, 0x11)
        self.payload = struct.pack(STREAM_FMT, *self.base, 7)
        self.extended = self.payload + struct.pack(STREAM_DIAG_FMT, -126.0, -600.0, -1500, 3)

    def test_legacy_payload(self):
        sample = decode_stream_payload(struct.pack(STREAM_FMT_LEGACY, *self.base))
        self.assertEqual(sample['omega'], -18.0)
        self.assertIsNone(sample['cycles'])
        self.assertIsNone(sample['target_omega'])

    def test_cycles_payload(self):
        sample = decode_stream_payload(self.payload)
        self.assertEqual(sample['cycles'], 7)
        self.assertIsNone(sample['motor_command'])

    def test_extended_payload(self):
        self.assertEqual(len(self.extended), 44)
        sample = decode_stream_payload(self.extended)
        self.assertEqual(sample['omega'], -18.0)
        self.assertEqual(sample['target_omega'], -126.0)
        self.assertEqual(sample['motor_rpm'], -600.0)
        self.assertEqual(sample['motor_command'], -1500)
        self.assertEqual(sample['control_flags'], 3)

    def test_inactive_target_is_not_plotted(self):
        sample = decode_stream_payload(self.payload + struct.pack(STREAM_DIAG_FMT, 123.0, 0.0, 0, 0))
        self.assertIsNone(sample['target_omega'])

    def test_short_payload_is_rejected(self):
        with self.assertRaises(ValueError):
            decode_stream_payload(b'\x00' * 25)

    def test_every_split_point_for_both_frame_versions(self):
        for builder in (build_packet, build_packet_like_firmware):
            packet = builder(0x40, self.extended)
            for split in range(1, len(packet)):
                with self.subTest(builder=builder.__name__, split=split):
                    parser = PacketParser()
                    self.assertEqual(parser.feed(packet[:split]) + parser.feed(packet[split:]), [(0x40, self.extended)])

    def test_bytewise_stream(self):
        for builder in (build_packet, build_packet_like_firmware):
            parser = PacketParser()
            result = []
            packet = builder(0x40, self.extended)
            for byte in packet * 3:
                result.extend(parser.feed(bytes([byte])))
            self.assertEqual(result, [(0x40, self.extended)] * 3)

    def test_noise_and_sync_split(self):
        parser = PacketParser()
        packet = build_packet(0x40, self.extended)
        self.assertEqual(parser.feed(b'noise-noise' + packet[:1]), [])
        self.assertEqual(parser.feed(packet[1:]), [(0x40, self.extended)])

    def test_bad_crc_is_not_accepted(self):
        parser = PacketParser()
        packet = bytearray(build_packet(0x40, self.extended))
        packet[-2] ^= 1
        self.assertEqual(parser.feed(packet), [])
        self.assertEqual(parser.feed(build_packet(0x40, self.extended)), [(0x40, self.extended)])


if __name__ == '__main__':
    unittest.main()
