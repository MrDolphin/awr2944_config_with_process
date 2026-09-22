"""Pure receive-time matching between radar messages and camera frames."""

from dataclasses import dataclass

from tools.camera.camera_capture import CameraFrameBuffer


@dataclass(frozen=True)
class CameraSyncResult:
    status: str
    frame_id: int | None
    capture_monotonic_ns: int | None
    time_offset_ms: float | None
    frame_url: str | None


def match_camera_frame(
    radar_monotonic_ns: int,
    frame_buffer: CameraFrameBuffer,
    base_url: str,
    matched_limit_ms: float = 50.0,
    stale_limit_ms: float = 100.0,
) -> CameraSyncResult:
    """Return the nearest frame; offset sign is ``camera_time - radar_time``."""
    if matched_limit_ms < 0 or stale_limit_ms < matched_limit_ms:
        raise ValueError("sync limits must satisfy 0 <= matched <= stale")
    frame = frame_buffer.nearest(radar_monotonic_ns)
    if frame is None:
        return CameraSyncResult("unavailable", None, None, None, None)

    offset_ms = (frame.host_monotonic_ns - radar_monotonic_ns) / 1_000_000
    magnitude_ms = abs(offset_ms)
    if magnitude_ms <= matched_limit_ms:
        status = "matched"
    elif magnitude_ms <= stale_limit_ms:
        status = "degraded"
    else:
        status = "stale"
    return CameraSyncResult(
        status=status,
        frame_id=frame.frame_id,
        capture_monotonic_ns=frame.host_monotonic_ns,
        time_offset_ms=offset_ms,
        frame_url=f"{base_url.rstrip('/')}/camera/frame/{frame.frame_id}.jpg",
    )
