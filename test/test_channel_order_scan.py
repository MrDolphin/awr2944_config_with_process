import unittest
from pathlib import Path

from simulation.run_v04_52_channel_order_scan import permute_channel
import numpy as np


class ChannelOrderScanTests(unittest.TestCase):
    def test_identity_permutation_preserves_channel(self):
        channel = np.arange(16).reshape(4, 4)
        np.testing.assert_array_equal(permute_channel(channel, (0, 1, 2, 3), (0, 1, 2, 3)), channel)

    def test_non_identity_permutation_changes_channel(self):
        channel = np.arange(16).reshape(4, 4)
        self.assertFalse(np.array_equal(permute_channel(channel, (1, 0, 2, 3), (0, 1, 2, 3)), channel))


if __name__ == "__main__":
    unittest.main()
