import unittest

from radar_camera_sync import match_camera_frame
from tools.camera.camera_capture import CameraFrame, CameraFrameBuffer


def frame(frame_id: int, timestamp_ns: int) -> CameraFrame:
    return CameraFrame(frame_id, timestamp_ns, timestamp_ns + 1, 1280, 720, b"jpeg")


class RadarCameraSyncTests(unittest.TestCase):
    def test_exact_match_has_zero_offset(self):
        buffer = CameraFrameBuffer()
        buffer.append(frame(7, 1_000_000_000))
        result = match_camera_frame(1_000_000_000, buffer, "http://pi:8081")
        self.assertEqual(result.status, "matched")
        self.assertEqual(result.frame_id, 7)
        self.assertEqual(result.time_offset_ms, 0.0)
        self.assertEqual(result.frame_url, "http://pi:8081/camera/frame/7.jpg")

    def test_nearest_match_preserves_camera_minus_radar_sign(self):
        buffer = CameraFrameBuffer()
        buffer.append(frame(1, 900_000_000))
        buffer.append(frame(2, 1_030_000_000))
        result = match_camera_frame(1_000_000_000, buffer, "http://pi:8081")
        self.assertEqual(result.frame_id, 2)
        self.assertEqual(result.time_offset_ms, 30.0)

    def test_empty_buffer_is_unavailable(self):
        result = match_camera_frame(1_000_000_000, CameraFrameBuffer(), "http://pi:8081")
        self.assertEqual(result.status, "unavailable")
        self.assertIsNone(result.frame_id)

    def test_quality_bands_classify_degraded_and_stale(self):
        buffer = CameraFrameBuffer()
        buffer.append(frame(1, 930_000_000))
        self.assertEqual(match_camera_frame(1_000_000_000, buffer, "http://pi").status, "degraded")
        buffer = CameraFrameBuffer()
        buffer.append(frame(2, 850_000_000))
        self.assertEqual(match_camera_frame(1_000_000_000, buffer, "http://pi").status, "stale")

    def test_equal_distance_tie_is_deterministically_earlier_frame(self):
        buffer = CameraFrameBuffer()
        buffer.append(frame(1, 950_000_000))
        buffer.append(frame(2, 1_050_000_000))
        result = match_camera_frame(1_000_000_000, buffer, "http://pi")
        self.assertEqual(result.frame_id, 1)
        self.assertEqual(result.time_offset_ms, -50.0)


if __name__ == "__main__":
    unittest.main()
