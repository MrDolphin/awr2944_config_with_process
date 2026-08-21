import unittest

import numpy as np

from simulation.dca1000_iq import (
    decode_interleaved_iq, encode_interleaved_iq, flatten_tdm_virtual_channels,
    reshape_tdm_virtual_channels,
)


class Dca1000IqTests(unittest.TestCase):
    def test_decode_interleaved_iq_shape_and_values(self):
        values = np.arange(2 * 3 * 4 * 2, dtype="<i2")
        iq = decode_interleaved_iq(values.tobytes(), chirps=2, samples_per_chirp=3, rx_count=4)
        self.assertEqual(iq.shape, (2, 3, 4))
        self.assertEqual(iq[1, 2, 3], 46 + 47j)

    def test_decode_rejects_wrong_capture_length(self):
        with self.assertRaisesRegex(ValueError, "expected"):
            decode_interleaved_iq(b"\x00\x00", chirps=1, samples_per_chirp=1, rx_count=4)

    def test_tdm_chirps_are_grouped_by_tx_sequence(self):
        iq = np.zeros((8, 2, 4), dtype=complex)
        for chirp in range(8):
            iq[chirp, :, :] = chirp
        virtual = reshape_tdm_virtual_channels(iq, tx_sequence=(0, 1, 2, 3))
        self.assertEqual(virtual.shape, (2, 2, 4, 4))
        self.assertTrue(np.all(virtual[1, :, :, 0] == 4))
        self.assertTrue(np.all(virtual[0, :, :, 3] == 3))

    def test_synthetic_tdm_roundtrip_preserves_virtual_channels(self):
        virtual = np.arange(2 * 2 * 4 * 4, dtype=float).reshape(2, 2, 4, 4).astype(complex)
        raw_chirps = flatten_tdm_virtual_channels(virtual)
        decoded = decode_interleaved_iq(
            encode_interleaved_iq(raw_chirps), chirps=8, samples_per_chirp=2, rx_count=4
        )
        self.assertTrue(np.array_equal(reshape_tdm_virtual_channels(decoded), virtual))
