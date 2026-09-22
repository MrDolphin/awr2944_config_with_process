"""Pure pinhole projection; hardware motion is intentionally absent."""
import math
from .calibration import RadarCameraCalibration
def project_radar_point(point_radar_m, calibration: RadarCameraCalibration, pose=None, *, clip_to_image=False):
    x, y, z = point_radar_m; r = calibration.rotation_radar_to_camera; t = calibration.translation_radar_to_camera_m
    if calibration.mount_mode == "fixed_camera" and pose is not None:
        yaw = math.radians(float(pose.yaw_deg))
        x, y = math.cos(yaw) * x - math.sin(yaw) * y, math.sin(yaw) * x + math.cos(yaw) * y
    xc, yc, zc = (sum(r[i][j] * (x, y, z)[j] for j in range(3)) + t[i] for i in range(3))
    if zc <= 0: return None
    xn, yn = xc / zc, yc / zc; k1, k2, p1, p2, k3 = calibration.distortion; radius = xn*xn + yn*yn; radial = 1 + k1*radius + k2*radius*radius + k3*radius*radius*radius
    xd = xn*radial + 2*p1*xn*yn + p2*(radius + 2*xn*xn); yd = yn*radial + p1*(radius + 2*yn*yn) + 2*p2*xn*yn
    u, v = calibration.fx*xd + calibration.cx, calibration.fy*yd + calibration.cy
    if clip_to_image and not (0 <= u < calibration.image_width and 0 <= v < calibration.image_height): return None
    return (u, v, zc)
