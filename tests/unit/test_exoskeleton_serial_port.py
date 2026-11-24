from __future__ import annotations

import unittest

from teleoperator.serial_port import select_serial_port


class ExoskeletonSerialPortTest(unittest.TestCase):
    def test_explicit_port_is_preserved(self) -> None:
        self.assertEqual(select_serial_port("/dev/ttyACM7"), "/dev/ttyACM7")

    def test_auto_prefers_stable_qnbot_path(self) -> None:
        selected = select_serial_port(
            "auto",
            stable_candidates=["/dev/serial/by-id/usb-Qnbot-if00"],
            fallback_candidates=["/dev/ttyACM0"],
        )
        self.assertEqual(selected, "/dev/serial/by-id/usb-Qnbot-if00")

    def test_auto_rejects_ambiguous_fallback(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "多个候选串口"):
            select_serial_port(
                "auto",
                stable_candidates=[],
                fallback_candidates=["/dev/ttyACM0", "/dev/ttyACM1"],
            )


if __name__ == "__main__":
    unittest.main()
