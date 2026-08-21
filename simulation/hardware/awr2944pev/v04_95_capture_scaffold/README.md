# V0.4.95 采集目录骨架

每个子目录已生成 `manifest.json`。当前所有 manifest 都是 `planned_awaiting_capture`，不代表已经采集。把真实 `capture.bin`/HDF5 和 `profile.cfg` 放入对应目录，再填写文件哈希、采集时间、姿态和通道/校准状态。

完成后使用 V0.4.94 检查完整性，再使用 V0.4.92 批处理。
