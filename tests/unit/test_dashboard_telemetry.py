from __future__ import annotations

import json
import unittest

from teleoperator.dashboard.telemetry import TelemetryPublisher


class DashboardTelemetryTest(unittest.TestCase):
    def test_encode_serializes_numpy_like_values_and_adds_timestamp(self) -> None:
        class _Value:
            def tolist(self):
                return [1, 2, 3]

        result = json.loads(TelemetryPublisher._encode({"joint": _Value()}).decode("utf-8"))
        self.assertEqual(result["joint"], [1, 2, 3])
        self.assertIn("published_at_unix_s", result)


if __name__ == "__main__":
    unittest.main()
