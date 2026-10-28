"""Checks the motion profile used by the desktop simulator and angle monitor."""
import math
from pathlib import Path
import re
import unittest

from km_stand_ui import MODE1_RAMP_FRACTION, mode1_motion_profile


class Mode1ProfileTests(unittest.TestCase):
    def test_firmware_and_desktop_ramp_settings_match(self):
        source = (Path(__file__).resolve().parents[1] / "Core/Src/main.c").read_text()
        match = re.search(r"#define MODE1_RAMP_FRACTION ([0-9.]+)f", source)
        self.assertIsNotNone(match)
        self.assertEqual(float(match.group(1)), MODE1_RAMP_FRACTION)

    def test_endpoints_and_period(self):
        for frequency in (0.1, 0.5, 1.0):
            period = 1.0 / frequency
            for t, angle in ((0, 50), (period / 2, 180), (period, 50)):
                actual_angle, speed = mode1_motion_profile(t, 50, 180, frequency)
                self.assertAlmostEqual(actual_angle, angle)
                self.assertAlmostEqual(speed, 0)

    def test_speed_is_derivative_of_angle(self):
        for frequency in (0.1, 0.5, 1.0):
            period = 1.0 / frequency
            delta = period * 1e-6
            for i in range(1, 100):
                t = period * i / 100
                before, _ = mode1_motion_profile(t - delta, 50, 180, frequency)
                after, _ = mode1_motion_profile(t + delta, 50, 180, frequency)
                _, speed = mode1_motion_profile(t, 50, 180, frequency)
                self.assertAlmostEqual((after - before) / (2 * delta), speed, places=4)

    def test_acceleration_cruise_and_braking(self):
        half_period = 1.0
        ramp_time = half_period * 2 * MODE1_RAMP_FRACTION / (1 + 2 * MODE1_RAMP_FRACTION)
        samples = [mode1_motion_profile(t, 50, 180, 0.5) for t in
                   (ramp_time / 4, ramp_time / 2, ramp_time, 0.5,
                    1 - ramp_time, 1 - ramp_time / 2, 1 - ramp_time / 4)]
        speeds = [sample[1] for sample in samples]
        self.assertLess(speeds[0], speeds[1])
        self.assertLess(speeds[1], speeds[2])
        self.assertAlmostEqual(speeds[2], speeds[3])
        self.assertAlmostEqual(speeds[3], speeds[4])
        self.assertGreater(speeds[4], speeds[5])
        self.assertGreater(speeds[5], speeds[6])
        self.assertAlmostEqual(samples[2][0], 50 + 130 * MODE1_RAMP_FRACTION)
        self.assertAlmostEqual(samples[4][0], 180 - 130 * MODE1_RAMP_FRACTION)

    def test_continuity_at_ramps_and_reversals(self):
        ramp_time = 2 * MODE1_RAMP_FRACTION / (1 + 2 * MODE1_RAMP_FRACTION)
        for boundary in (ramp_time, 1 - ramp_time, 1, 1 + ramp_time, 2 - ramp_time, 2):
            before = mode1_motion_profile(boundary - 1e-8, 50, 180, 0.5)
            after = mode1_motion_profile(boundary + 1e-8, 50, 180, 0.5)
            for left, right in zip(before, after):
                self.assertAlmostEqual(left, right, places=4)

    def test_bounds_and_symmetry(self):
        for i in range(1000):
            t = 2 * i / 1000
            angle, speed = mode1_motion_profile(t, 50, 180, 0.5)
            mirrored_angle, mirrored_speed = mode1_motion_profile(t + 1, 50, 180, 0.5)
            self.assertTrue(50 <= angle <= 180)
            self.assertTrue(math.isfinite(speed))
            self.assertAlmostEqual(angle + mirrored_angle, 230)
            self.assertAlmostEqual(speed, -mirrored_speed)

    def test_invalid_motion_is_stationary(self):
        for minimum, maximum, frequency in ((50, 180, 0), (50, 180, -1), (50, 50, 1), (180, 50, 1)):
            self.assertEqual(mode1_motion_profile(5, minimum, maximum, frequency), (minimum, 0))


if __name__ == "__main__":
    unittest.main()
