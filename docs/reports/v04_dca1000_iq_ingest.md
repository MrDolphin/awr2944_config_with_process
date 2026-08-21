# V0.4.7 DCA1000 原始 IQ 接入记录

当前项目目录和资料目录未发现真实 DCA1000 `.bin/.raw` 捕获文件，因此本阶段没有声称完成实测通道验证。

已新增 `simulation/dca1000_iq.py`，支持明确声明的最小格式：

```text
little-endian int16
每个采样点：RX0-I, RX0-Q, RX1-I, RX1-Q, ...
输出维度：(chirp, sample, rx)
```

运行器会保存 HDF5 `/radar/iq` 和 JSON 元数据，并将 `channel_order_verified=false` 写入输出。这个标志在没有通过 DCA1000 捕获设置、LVDS lane 配置和已知测试信号验证前不能改为 true。

使用示例：

```powershell
python -m simulation.dca1000_iq `
  --input path\capture.bin `
  --output simulation\stages\v04_aoa_cfar_point_cloud\results\python\dca_capture\data\capture.h5 `
  --chirps 64 --samples-per-chirp 256 --rx-count 4
```

下一步：提供一份真实 `.bin` 后，用已知角反射器/静态目标检查通道顺序、I/Q 符号、LVDS lane 拼接、通道幅相和 `antGeometryCfg` 重排，再接入 AoA。
