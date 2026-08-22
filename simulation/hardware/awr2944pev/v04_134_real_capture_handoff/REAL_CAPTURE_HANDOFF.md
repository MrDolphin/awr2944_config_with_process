# V0.4.134 首批真实 DCA1000 采集交接包

## 首批工况

- `ka001_az-30_el-10_r10`：方位 -30°，俯仰 -10°，距离 10 m
- `ka002_az-15_el-10_r10`：方位 -15°，俯仰 -10°，距离 10 m
- `ka003_az+00_el-10_r10`：方位 0°，俯仰 -10°，距离 10 m
- `ka004_az+15_el-10_r10`：方位 15°，俯仰 -10°，距离 10 m
- `ka005_az+30_el-10_r10`：方位 30°，俯仰 -10°，距离 10 m

## 采集检查清单

1. Stop motion and set the radar/target geometry before capture.
2. Record the same CFG used by the radar and compute its SHA-256.
3. Save the DCA1000 capture configuration and raw capture.bin without CFAR.
4. Record target azimuth, elevation and range from survey/total station.
5. Record installation height, boresight, ship heading and IMU pose reference.
6. Run TI calibration and save the resulting RX channel phase/gain data.
7. Do not change channel order or reshape dimensions by hand after capture.
8. Run V0.4.133 readiness gate before V0.4.112 decode.

## 目录约定

每个工况目录至少包含：`capture.bin`、`manifest.json`、CFG、DCA1000 配置、TI 校准文件和姿态引用。文件名和目录名应与 `first_five_capture_plan.csv` 一致。

## 验收命令

```powershell
python -m simulation.run_v04_133_capture_readiness_gate `
  --root <真实采集根目录> `
  --output simulation/hardware/awr2944pev/v04_133_capture_readiness_gate_real

python -m simulation.run_v04_112_manifest_iq_decode `
  --manifest <工况目录>\manifest.json `
  --output <工况目录>\decoded
```

准入门输出 `ready_count` 必须大于 0，随后才进入 IQ 解码。当前模板不包含真实 capture.bin，也不代表硬件已验证。
