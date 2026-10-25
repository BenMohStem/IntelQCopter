"""Synthetic serial-protocol tests only; no ESP32 emulation or real sensor evidence."""
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import record_poses as recorder


class FakeSerial:
    def __init__(self, text):
        self.input = io.BytesIO(text.encode("ascii"))
        self.commands = []

    def readline(self):
        return self.input.readline()

    def write(self, data):
        self.commands.append(data)


class RecorderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = recorder.Samples()
        rows = []
        for index in range(60000):
            ax = -100 if index % 2 == 0 else 100
            line = f"S,{index},{ax},-20,4096,0,1,2,-3"
            rows.append(line)
            cls.samples.add(line)
        cls.means = dict(enumerate(cls.samples.mean))
        cls.end = f"END,60000,60000000,28,{cls.samples.hash:08X}"
        cls.header = "QUERY_REPLY\nHELLO,POSE1000_V1,test-sha\n" + recorder.CONFIG + "\nBOARD,synthetic,0000\n"
        cls.header += "".join(f"OFFSET,{address},0\n" for address in ("06", "08", "0A", "13", "15", "17"))
        cls.header += "READY,send_s_to_record_60000_samples\n"
        cls.transcript = cls.header + "BEGIN,60000\n" + "\n".join(rows) + "\n"
        cls.transcript += "".join(f"MEAN_COUNTS,{index},{mean:.9f}\n" for index, mean in cls.means.items())
        cls.transcript += cls.end + "\n"

    def test_known_statistics(self):
        result = self.samples.finish(self.end, self.means)
        self.assertAlmostEqual(result["mean_physical"]["az"], 9.80665)
        self.assertAlmostEqual(result["mean_counts"]["ax"], 0)
        self.assertAlmostEqual(result["sample_std_counts"]["ax"], 100 * math.sqrt(60000 / 59999))
        self.assertEqual(result["observed_average_hz"], 1000)

    def test_missing_duplicate_and_out_of_range(self):
        for line in ("S,1,0,0,0,0,0,0,0", "S,0,99999,0,0,0,0,0,0", "S,0,0"):
            with self.subTest(line=line), self.assertRaises((ValueError, recorder.struct.error)):
                recorder.Samples().add(line)
        samples = recorder.Samples()
        samples.add("S,0,0,0,0,0,0,0,0")
        with self.assertRaises(ValueError):
            samples.add("S,0,0,0,0,0,0,0,0")

    def test_invalid_completion(self):
        for ending in (self.end.replace("60000,", "59999,", 1),
                       self.end.replace("60000000", "120000000"),
                       self.end.replace(",28,", ",1024,"),
                       self.end[:-8] + "00000000"):
            with self.subTest(ending=ending), self.assertRaises(ValueError):
                self.samples.finish(ending, self.means)
        with self.assertRaises(ValueError):
            self.samples.finish(self.end, {})

    def test_complete_capture_preserves_original(self):
        device = FakeSerial(self.transcript)
        with tempfile.TemporaryDirectory() as temporary, patch.object(recorder.time, "sleep"), patch("builtins.print"):
            folder = Path(temporary)
            result = recorder.capture(device, folder, "test-sha")
            self.assertEqual(result["status"], "CAPTURE_COMPLETE_REVIEW_REQUIRED")
            self.assertEqual((folder / "serial.log").read_bytes(), self.transcript.encode("ascii"))
            self.assertEqual(device.commands, [b"?", b"s"])

    def test_overflow_preserves_failed_recording(self):
        device = FakeSerial(self.header + "BEGIN,60000\nERROR,FIFO_OVERFLOW_OR_BACKLOG\n")
        with tempfile.TemporaryDirectory() as temporary, patch.object(recorder.time, "sleep"), patch("builtins.print"):
            folder = Path(temporary)
            with self.assertRaises(ValueError):
                recorder.capture(device, folder, "test-sha")
            result = json.loads((folder / "summary.json").read_text())
            self.assertEqual(result["status"], "FAILED")
            self.assertIn("OVERFLOW", result["error"])
            self.assertEqual(device.commands[-1], b"x")
            self.assertTrue((folder / "serial.log").exists())

    def test_wrong_source_and_baseline_refused_before_start(self):
        for sha, baseline in (("wrong-sha", None), ("test-sha", ["changed baseline"])):
            device = FakeSerial(self.header)
            with tempfile.TemporaryDirectory() as temporary, patch("builtins.print"):
                with self.assertRaises(ValueError):
                    recorder.capture(device, Path(temporary), sha, baseline)
                self.assertNotIn(b"s", device.commands)


if __name__ == "__main__":
    unittest.main(verbosity=2)
