# 雷达—相机静态标定

本流程只在 PC 上运行 `tools/fusion/calibrate_radar_camera.py`；树莓派在线运行不导入 OpenCV。示例 JSON 的焦距为零，加载器会拒绝它，不能作为部署标定。

把普通 USB 相机与 AWR2944P 刚性固定在同一支架上。用雷达反射器和可见 AprilTag 或高对比棋盘格组成同一标靶，在 `-30、-15、0、15、30°` 方位和 `5、10、20 m` 距离采集静止样本。每个样本记录雷达三维点和对应像素；用卷尺记录相机—雷达平移，作为求解结果的独立合理性检查。

CSV 需要列 `radar_x,radar_y,radar_z,u,v`。先用独立的相机内参标定得到有效焦距，再运行：

```powershell
python tools/fusion/calibrate_radar_camera.py pairs.csv intrinsics.json Config/radar_camera_calibration.json
```

拟合集之外保留验证位置。验收门限为：相机内参 RMS 不大于 1.5 px；联合雷达—相机中位误差不大于 8 px，p95 不大于 20 px。未通过时先复核靶标关联、坐标轴方向和刚性安装，不要通过放宽投影阈值掩盖问题。
