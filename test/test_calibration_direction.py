import unittest

import numpy as np

from simulation.calibration import complex_matrix


class CalibrationDirectionTests(unittest.TestCase):
    def test_inverse_is_the_only_candidate_that_cancels_injected_factor(self):
        factor = complex_matrix(np.full((4, 4), 1.1), np.full((4, 4), 15.0))
        np.testing.assert_allclose(factor * (1.0 / factor), np.ones((4, 4)))
        self.assertFalse(np.allclose(factor * factor, np.ones((4, 4))))
        self.assertFalse(np.allclose(factor / np.conj(factor), np.ones((4, 4))))


if __name__ == "__main__":
    unittest.main()
