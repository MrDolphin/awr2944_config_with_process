import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest


HOST = "host_packet_parse_monotonic"
COUNTER = "frame_counter_x_cfg_period_estimate_only"
CFG = ("dfeDataOutputMode 1\nadcCfg 2 0\n"
       "profileCfg 0 77 186 7 57.14 0 0 10 1 32 1000 0 0 158\n"
       "chirpCfg 0 0 0 0 0 0 0 1\nframeCfg 0 0 16 0 32 10 1 0")


class FieldPpiTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)

    def make_capture(self, name, clock, *, roll=90, echo=True, rpm=30, event_delay=0):
        path = self.root / name
        path.mkdir()
        frames, events, turns, raw = [], [], [], bytearray()
        registration = dict(enabled=True, motor_active=True, encoder_direction_verified=True,
                            motor_direction="ccw", direction=1, zero_deg=0,
                            mounting_yaw_deg=0, mounting_roll_deg=roll, time_basis=clock,
                            experimental_frame_period_ms=None if clock == HOST else 10)
        samples = [dict(enabled=True, speed_rpm=rpm)] * 4
        for i in range(12):
            number, stamp = i + 100, 1 + i * .01
            bins = [100] * 16
            if echo and i % 4 == (1 if clock == HOST else 2):
                bins[8] = 800
            payload = struct.pack("<16H", *bins)
            packet = (struct.pack("<8sIIIIIIII", bytes.fromhex("0201040306050807"),
                                  0, 48 + len(payload), 0, number, 0, 0, 1, 0)
                      + struct.pack("<II", 2, len(payload)) + payload)
            frames.append(dict(frame_num=number, host_monotonic_s=stamp, tlv_types=[2],
                               raw_offset=len(raw), raw_length=len(packet)))
            raw.extend(packet)
            events.append(dict(frame_num=number, host_monotonic_s=stamp + event_delay, status="registered",
                               time_basis=clock, yaw_deg=[350, 359, 4, 15][i % 4],
                               registration_direction=1, registration_zero_deg=0,
                               registration_mounting_yaw_deg=0, registration_mounting_roll_deg=roll,
                               max_gap_s=.1, max_query_duration_s=.05, max_span_deg=10,
                               max_possible_rpm=130, stale_after_s=1))
        turns.append(dict(time_basis=clock, first_raw_frame_num=96, last_raw_frame_num=99))
        for start in (100, 104, 108):
            turns.append(dict(time_basis=clock, first_raw_frame_num=start, last_raw_frame_num=start + 3,
                              first_frame_num=start, last_frame_num=start + 3,
                              max_gap_deg=335 if clock == HOST else 330, valid_tlv2_spokes=4,
                              raw_frame_missing=0, tlv2_missing=0, registration_rejected={}))
        records = dict(frames=frames, motor_samples=samples, turn_diagnostics=turns,
                       live_registration=events)
        meta = dict(recording_status="completed", writer_error=None, registration=registration,
                    active_config=dict(content=CFG, sha256=hashlib.sha256(CFG.encode()).hexdigest()))
        for label, rows in records.items():
            (path / f"{label}.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            key = "registration_events" if label == "live_registration" else label
            meta[key] = len(rows)
            meta[f"dropped_{key}"] = 0
        (path / "metadata.json").write_text(json.dumps(meta), encoding="utf-8")
        (path / "frames.tlv").write_bytes(raw)
        return path

    def run_report(self, baseline, experiment):
        return subprocess.run(
            [sys.executable, "-m", "tools.compare_field_ppi", str(baseline), str(experiment),
             "--range-m", "3.75", "--azimuth-deg", "0", "--range-gate-m", ".6",
             "--min-contrast", "100"],
            cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True,
        )

    def test_reports_target_shift_across_zero_from_binary_and_recorded_yaw(self):
        a = self.make_capture("base", HOST)
        b = self.make_capture("trial", COUNTER)
        run = self.run_report(a, b)
        self.assertEqual(run.returncode, 0, run.stderr)
        report = json.loads(run.stdout)
        self.assertEqual(report["baseline"]["complete_turns"], 3)
        self.assertEqual(report["baseline"]["ignored_edge_turns"], 1)
        self.assertEqual(report["max_gap_change_deg"], -5)
        self.assertEqual(report["baseline"]["target_bias_deg"], -1)
        self.assertEqual(report["experiment"]["target_bias_deg"], 4)
        self.assertEqual(report["target_bias_change_deg"], 5)
        self.assertFalse(report["rf_time_calibrated"])

    def test_weak_return_remains_undetermined(self):
        a = self.make_capture("base", HOST)
        b = self.make_capture("trial", COUNTER, echo=False)
        run = self.run_report(a, b)
        self.assertEqual(run.returncode, 0, run.stderr)
        report = json.loads(run.stdout)
        self.assertIsNone(report["experiment"]["target_bias_deg"])
        self.assertIsNone(report["target_bias_change_deg"])

    def test_rejects_an_installation_change(self):
        a = self.make_capture("base", HOST)
        b = self.make_capture("trial", COUNTER, roll=0)
        run = self.run_report(a, b)
        self.assertEqual(run.returncode, 2, run.stderr)
        self.assertIn("mounting_roll_deg", run.stderr)

    def test_incompatible_measured_speed_suppresses_comparison(self):
        a = self.make_capture("base", HOST)
        b = self.make_capture("trial", COUNTER, rpm=36)
        run = self.run_report(a, b)
        self.assertEqual(run.returncode, 0, run.stderr)
        report = json.loads(run.stdout)
        self.assertFalse(report["comparable_median_speed"])
        self.assertIsNone(report["max_gap_change_deg"])
        self.assertIsNone(report["target_bias_change_deg"])

    def test_two_host_clock_captures_are_not_a_field_ab(self):
        a = self.make_capture("base", HOST)
        b = self.make_capture("trial", HOST)
        run = self.run_report(a, b)
        self.assertEqual(run.returncode, 2, run.stderr)
        self.assertIn(COUNTER, run.stderr)

    def test_same_frame_number_without_matching_original_timestamp_is_not_evidence(self):
        a = self.make_capture("base", HOST)
        b = self.make_capture("trial", COUNTER, event_delay=.5)
        run = self.run_report(a, b)
        self.assertEqual(run.returncode, 0, run.stderr)
        report = json.loads(run.stdout)
        self.assertEqual(report["experiment"]["target_candidate_turns"], 0)
        self.assertIsNone(report["target_bias_change_deg"])
