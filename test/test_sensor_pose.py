import unittest

from sensor_pose import SensorPose, SensorPoseHistory, pose_metadata


class SensorPoseHistoryTests(unittest.TestCase):
    def pose(self, timestamp_ns, yaw=0.0):
        return SensorPose(timestamp_ns, yaw, -1.0, 0.0, "encoder")

    def test_retains_a_bounded_monotonic_history(self):
        history = SensorPoseHistory(capacity=2)
        history.append(self.pose(100))
        history.append(self.pose(200, 2.0))
        history.append(self.pose(300, 3.0))
        pose, age_ms = history.nearest(210)
        self.assertEqual(pose.yaw_deg, 2.0)
        self.assertAlmostEqual(age_ms, 0.00001)

    def test_rejects_out_of_order_samples(self):
        history = SensorPoseHistory()
        history.append(self.pose(200))
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            history.append(self.pose(200))

    def test_nearest_returns_signed_age_independent_absolute_age(self):
        history = SensorPoseHistory()
        history.append(self.pose(1_000_000_000, 10.0))
        history.append(self.pose(1_080_000_000, 20.0))
        pose, age_ms = history.nearest(1_030_000_000)
        self.assertEqual(pose.yaw_deg, 10.0)
        self.assertEqual(age_ms, 30.0)

    def test_metadata_marks_stale_and_unavailable(self):
        history = SensorPoseHistory()
        self.assertEqual(pose_metadata(history, 1_000_000_000)["status"], "unavailable")
        history.append(self.pose(1_000_000_000, 12.3))
        fresh = pose_metadata(history, 1_040_000_000)
        self.assertEqual(fresh["status"], "fresh")
        self.assertEqual(fresh["yaw_deg"], 12.3)
        self.assertEqual(pose_metadata(history, 1_051_000_000)["status"], "stale")


if __name__ == "__main__":
    unittest.main()
