import unittest

import numpy as np

from simulation.dca1000_iq import decode_interleaved_iq


class Dca1000IqTests(unittest.TestCase):
    def test_decode_interleaved_iq_shape_and_values(self):
        values = np.arange(2 * 3 * 4 * 2, dtype="<i2")
        iq = decode_interleaved_iq(values.tobytes(), chirps=2, samples_per_chirp=3, rx_count=4)
        self.assertEqual(iq.shape, (2, 3, 4))
        self.assertEqual(iq[1, 2, 3], 46 + 47j)

    def test_decode_rejects_wrong_capture_length(self):
        with self.assertRaisesRegex(ValueError, "expected"):
            decode_interleaved_iq(b"\x00\x00", chirps=1, samples_per_chirp=1, rx_count=4)
