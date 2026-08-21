# V0.4.21 DCA1000 抓包接入

## 输入与输出

输入：真实 DCA1000 `.bin`、对应 CFG、chirp 数、每 chirp sample 数、RX 数和 TDM TX 顺序。

输出 HDF5：

```text
/decoded/iq             (chirp, sample, rx)
/recovered/virtual_iq   (frame, sample, rx, tx)
/calibrated/virtual_iq  可选，应用 4×4 复数校准后
```

结果目录：`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_21_capture_ingest/`。

## 验收边界

- 保留输入文件 SHA-256 和 CFG SHA-256；
- 明确记录 LVDS 交织假设和 TDM TX 顺序；
- `channel_order_verified=false`，直到角反射器/已知信号验证；
- 本阶段不自动选择距离门、不自动做 CFAR、不直接输出真实 AoA；
- 真实抓包到来后，可在此规范张量上继续做校准方向、距离-多普勒和 AoA 分析。
