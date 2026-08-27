"""CSV persistence for decoded JY931 samples."""

import csv
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Union

from .protocol import Sample


class CsvRecorder:
    COLUMNS = [
        "time",
        "index",
        "acc_1",
        "acc_2",
        "acc_3",
        "gyro_1",
        "gyro_2",
        "gyro_3",
        "angle_1",
        "angle_2",
        "angle_3",
        "mag_1",
        "mag_2",
        "mag_3",
        "quat_1",
        "quat_2",
        "quat_3",
        "quat_4",
    ]
    SAMPLE_ORDER = ("acc", "gyro", "angle", "mag", "quat")

    def __init__(
        self,
        path: Union[str, Path],
        flush_interval: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.path = Path(path).expanduser()
        self.flush_interval = flush_interval
        self.clock = clock
        self.rows_written = 0
        self.pending: Dict[str, Sample] = {}
        self._last_flush = self.clock()

        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.COLUMNS)

    def write(self, sample: Sample):
        if sample.type not in self.SAMPLE_ORDER:
            return

        # The module emits one short frame per quantity. Treat a repeated
        # quantity as the start of the next sampling cycle.
        if sample.type in self.pending:
            self.write_pending()
        self.pending[sample.type] = sample
        self.flush_if_due()

    def write_pending(self):
        if not self.pending:
            return

        timestamp = min(sample.timestamp for sample in self.pending.values())
        row = [
            datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S.%f"),
            str(self.rows_written + 1),
        ]
        for sample_type in self.SAMPLE_ORDER:
            sample = self.pending.get(sample_type)
            width = 4 if sample_type == "quat" else 3
            if sample is None:
                row.extend([""] * width)
            else:
                row.extend("{:.6f}".format(value) for value in sample.values[:width])
                row.extend([""] * (width - len(sample.values)))

        self.writer.writerow(row)
        self.rows_written += 1
        self.pending.clear()

    def flush_if_due(self, force: bool = False):
        now = self.clock()
        if (
            force
            or self.flush_interval == 0
            or (
                self.flush_interval > 0
                and now - self._last_flush >= self.flush_interval
            )
        ):
            self.file.flush()
            self._last_flush = now

    def close(self):
        if self.file.closed:
            return
        self.write_pending()
        self.flush_if_due(force=True)
        self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
