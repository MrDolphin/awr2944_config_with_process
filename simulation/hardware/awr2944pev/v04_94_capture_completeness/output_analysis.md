# V0.4.94 已知角采集完整性检查

计划工况：30；基础文件齐全：0；覆盖率：0.0%。

## 完整条件

每个工况至少必须同时存在 `manifest.json`、`capture.bin/HDF5` 和 `profile.cfg`。这只是文件完整性，不等于通道顺序、校准矩阵或机械姿态已经验证。

## 下一步

collect missing cases before running V0.4.92 batch

图中绿色表示基础文件齐全，红色表示至少缺少一个文件。
