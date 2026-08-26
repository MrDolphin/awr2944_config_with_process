"""Hardware-independent tests for the JY931 acquisition package."""

import csv
import struct
import tempfile
import threading
import unittest
from datetime import datetime
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tools.jy931.main import (
    DEFAULT_OUTPUT_DIR,
    build_parser,
    resolve_output_path,
    run,
)
from tools.jy931.reader import FrameParser, SerialFrameReader
from tools.jy931.protocol import parse_frame
from tools.jy931.recorder import CsvRecorder


def make_frame(sample_type: int, values) -> bytes:
    if sample_type == 0x59:
        payload = struct.pack("<4h", *values)
    else:
        payload = struct.pack("<3hh", *values)
    frame = bytes((0x55, sample_type)) + payload
    return frame + bytes((sum(frame) & 0xFF,))


class FakeSerial:
    def __init__(self, chunks, stop_event):
        self.stop_event = stop_event
        self.chunks = list(chunks)
        self.closed = False
        self.name = "fake"
        self.baudrate = 921600

    def read(self, size):
        if self.chunks:
            return self.chunks.pop(0)
        self.stop_event.set()
        return b""

    def close(self):
        self.closed = True


class TestProtocol(unittest.TestCase):
    def test_acc_scaling_and_temperature(self):
        frame = make_frame(0x51, (16384, -16384, 8192, 2500))
        sample = parse_frame(frame, timestamp=12.5)

        self.assertIsNotNone(sample)
        self.assertEqual(sample.type, "acc")
        self.assertAlmostEqual(sample.values[0], 8.0)
        self.assertAlmostEqual(sample.values[1], -8.0)
        self.assertAlmostEqual(sample.values[2], 4.0)
        self.assertAlmostEqual(sample.aux, 25.0)
        self.assertEqual(sample.timestamp, 12.5)

    def test_mag_is_0x54_and_quat_is_0x59(self):
        mag = parse_frame(make_frame(0x54, (100, -200, 300, 1500)), 1.0)
        quat = parse_frame(make_frame(0x59, (16384, 16384, 16384, 16384)), 2.0)

        self.assertEqual(mag.type, "mag")
        self.assertEqual(mag.values, (100.0, -200.0, 300.0))
        self.assertEqual(quat.type, "quat")
        self.assertAlmostEqual(quat.values[0], 0.5)
        self.assertAlmostEqual(quat.norm, 1.0)

    def test_gyro_scaling_and_voltage(self):
        sample = parse_frame(make_frame(0x52, (16384, 0, 0, 987)), 4.0)

        self.assertEqual(sample.type, "gyro")
        self.assertAlmostEqual(sample.values[0], 1000.0)
        self.assertAlmostEqual(sample.aux, 9.87)

    def test_rejects_bad_checksum_and_unknown_type(self):
        frame = bytearray(make_frame(0x53, (0, 0, 0, 0)))
        frame[-1] ^= 0x01
        self.assertIsNone(parse_frame(frame))
        self.assertIsNone(parse_frame(bytes((0x55, 0x70)) + bytes(9)))


class TestFrameParser(unittest.TestCase):
    def test_frames_split_across_reads(self):
        parser = FrameParser(timestamp_fn=lambda: 1.0)
        stream = make_frame(0x51, (0, 0, 0, 0)) + make_frame(
            0x52, (-32768, 0, 0, 0)
        )
        samples = []
        for index in range(len(stream)):
            samples.extend(parser.feed(stream[index:index + 1]))

        self.assertEqual([sample.type for sample in samples], ["acc", "gyro"])
        self.assertEqual(parser.valid_frames, 2)
        self.assertEqual(parser.invalid_frames, 0)

    def test_false_header_does_not_discard_next_frame(self):
        parser = FrameParser(timestamp_fn=lambda: 1.0)
        stream = bytes((0x55,)) + make_frame(0x53, (16384, 0, 0, 0))
        samples = parser.feed(stream)

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].type, "angle")
        self.assertEqual(parser.invalid_frames, 1)

    def test_garbage_without_header_is_cleared(self):
        parser = FrameParser(timestamp_fn=lambda: 1.0)
        samples = parser.feed(bytes((1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11)))

        self.assertEqual(samples, [])
        self.assertEqual(parser.buffer, bytearray())


class TestCsvRecorder(unittest.TestCase):
    def test_groups_one_frame_set_into_one_row(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "imu.csv"
            with CsvRecorder(path, flush_interval=0.0) as recorder:
                samples = [
                    parse_frame(make_frame(0x51, (16384, 0, 16384, 2500)), 3.25),
                    parse_frame(make_frame(0x52, (16384, 0, 0, 987)), 3.2501),
                    parse_frame(make_frame(0x53, (16384, 0, 0, 0)), 3.2502),
                    parse_frame(make_frame(0x54, (1, -2, 3, 1234)), 3.2503),
                    parse_frame(make_frame(0x59, (16384, 16384, 16384, 16384)), 3.2504),
                ]
                for sample in samples:
                    recorder.write(sample)

                # The next acc frame closes the previous sampling cycle.
                recorder.write(
                    parse_frame(make_frame(0x51, (0, 0, 16384, 2500)), 3.260)
                )


            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))

        self.assertEqual(rows[0], CsvRecorder.COLUMNS)
        self.assertEqual(rows[1][1], "1")
        self.assertEqual(rows[1][2:5], ["8.000000", "0.000000", "8.000000"])
        self.assertEqual(rows[1][5:8], ["1000.000000", "0.000000", "0.000000"])
        self.assertEqual(rows[1][8:11], ["90.000000", "0.000000", "0.000000"])
        self.assertEqual(rows[1][11:14], ["1.000000", "-2.000000", "3.000000"])
        self.assertEqual(rows[1][14:18], ["0.500000"] * 4)
        self.assertEqual(recorder.rows_written, 2)
        self.assertEqual(rows[2][2:5], ["0.000000", "0.000000", "8.000000"])

    def test_close_flushes_partial_frame_set(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "partial.csv"
            with CsvRecorder(path, flush_interval=0.0) as recorder:
                recorder.write(
                    parse_frame(make_frame(0x54, (1, -2, 3, 1234)), 3.25)
                )

            # Exiting the context manager closes the recorder and writes the
            # incomplete final cycle.
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))

        self.assertEqual(rows[0], CsvRecorder.COLUMNS)
        self.assertEqual(rows[1][1], "1")
        self.assertEqual(rows[1][11:14], ["1.000000", "-2.000000", "3.000000"])
        self.assertEqual(rows[1][14:], [""] * 4)
        self.assertEqual(recorder.rows_written, 1)


class TestCommandLine(unittest.TestCase):
    def test_help_defaults_are_valid(self):
        parser = build_parser()
        args = parser.parse_args([])
        self.assertEqual(args.port, "/dev/serial0")
        self.assertEqual(args.baud, 921600)
        self.assertEqual(args.rate, 10.0)
        self.assertIn("acc", args.types)

    def test_default_output_path_is_timestamped(self):
        args = build_parser().parse_args([])
        moment = datetime(2026, 8, 26, 12, 34, 56)
        path = resolve_output_path(args, moment)

        self.assertEqual(
            path,
            DEFAULT_OUTPUT_DIR / "imu_20260826_123456.csv",
        )

        explicit = build_parser().parse_args(["-o", "/tmp/imu.csv"])
        self.assertEqual(resolve_output_path(explicit, moment), Path("/tmp/imu.csv"))

        disabled = build_parser().parse_args(["--no-output"])
        self.assertIsNone(resolve_output_path(disabled, moment))

    def test_run_records_fake_serial_stream(self):
        frame = make_frame(0x59, (16384, 16384, 16384, 16384))
        stop_event = threading.Event()
        fake = FakeSerial([frame], stop_event)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "imu.csv"
            args = build_parser().parse_args(
                ["--no-print", "--rate", "0", "-o", str(output)]
            )
            stats = run(
                args,
                stop_event=stop_event,
                serial_factory=lambda **kwargs: fake,
            )

            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))

        self.assertEqual(stats.frames, 1)
        self.assertEqual(stats.type_counts["quat"], 1)
        self.assertTrue(fake.closed)
        self.assertEqual(rows[0], CsvRecorder.COLUMNS)
        self.assertEqual(rows[1][1], "1")
        self.assertEqual(rows[1][14:18], ["0.500000"] * 4)

    def test_no_output_does_not_create_csv(self):
        frame = make_frame(0x59, (16384, 16384, 16384, 16384))
        stop_event = threading.Event()
        fake = FakeSerial([frame], stop_event)
        args = build_parser().parse_args(
            ["--no-print", "--rate", "0", "--no-output"]
        )
        stats = run(args, stop_event=stop_event, serial_factory=lambda **kwargs: fake)

        self.assertEqual(stats.frames, 1)
        self.assertEqual(stats.type_counts["quat"], 1)
        self.assertTrue(fake.closed)

    def test_run_counts_invalid_only_stream(self):
        invalid_frame = bytearray(make_frame(0x51, (0, 0, 0, 0)))
        invalid_frame[-1] ^= 0x01
        stop_event = threading.Event()
        fake = FakeSerial([bytes(invalid_frame)], stop_event)
        args = build_parser().parse_args(["--no-print", "--no-output"])
        stats = run(args, stop_event=stop_event, serial_factory=lambda **kwargs: fake)

        self.assertEqual(stats.frames, 0)
        self.assertEqual(stats.invalid_frames, 1)

    def test_reconnect_closes_and_reopens(self):
        frame = make_frame(0x51, (0, 0, 0, 0))
        stop_event = threading.Event()
        first = FakeSerial([], stop_event)
        second = FakeSerial([frame], stop_event)
        instances = [first, second]

        def factory(**kwargs):
            return instances.pop(0)

        reader = SerialFrameReader("/dev/fake", serial_factory=factory)
        samples = []
        reader.open()
        reader.close()
        self.assertTrue(reader.reconnect(delay=0.0, stop_event=stop_event))
        samples.extend(reader.parser.feed(second.read(256)))
        reader.close()

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].type, "acc")


if __name__ == "__main__":
    unittest.main()
