# V0.4.56 实测 AoA 验收包模板

## 本阶段目的

把真实 DCA1000 多场景 AoA 验收所需的数据组织固定下来，避免采集完成后缺少 CFG、角度真值或校准来源，导致结果无法复现。

新增模板：[measured_aoa_manifest_template.json](../../simulation/hardware/awr2944pev/v04_56_measured_aoa_package/measured_aoa_manifest_template.json)

每个场景必须包含：

- 原始 DCA1000 `.bin` 或解码 HDF5；
- 同次采集的 CFG；
- 已知目标方位、俯仰和距离真值 JSON；
- 实测校准矩阵 JSON；
- 操作者、采集时间、海况、安装高度和安装角度等 provenance。

## 当前模板检查结果

当前模板只有一个示例场景，路径均为待填占位符，因此检查结果为：

- 场景数至少 4：未通过；
- 原始 capture 文件：未通过；
- CFG 文件：未通过；
- truth 文件：未通过；
- 实测 calibration：未通过；
- 角度/距离覆盖声明：通过；
- `measured_aoa_ready`：`false`。

这不是代码错误，而是对当前证据状态的如实记录。

## 采集后如何使用

1. 将真实文件放入 manifest 中记录的目录；
2. 每个 truth JSON 至少包含：

```json
{
  "azimuth_deg": 15,
  "elevation_deg": 5,
  "range_m": 20,
  "target_rcs_m2": 1.0
}
```

3. calibration JSON 的 `calibration_status` 必须是 `measured`、`hardware_measured` 或 `ti_measured`；
4. 运行：

```powershell
python -m simulation.run_v04_56_measured_aoa_package `
  --manifest simulation\hardware\awr2944pev\v04_56_measured_aoa_package\measured_aoa_manifest.json `
  --output simulation\hardware\awr2944pev\v04_56_measured_aoa_package\measured_check
```

5. 只有 `measured_aoa_ready=true` 后，才进入 V0.55 的真实硬件 AoA 综合门禁。

## 输出文件

- [package_check.json](../../simulation/hardware/awr2944pev/v04_56_measured_aoa_package/template_check/package_check.json)
- [output_analysis.md](../../simulation/hardware/awr2944pev/v04_56_measured_aoa_package/template_check/output_analysis.md)

