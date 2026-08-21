# V0.4.58 实测 AoA 资料包流水线

## 本阶段目的

把 V0.4.56 的实测 AoA 资料清单，与 V0.4.53 的 HDF5 通道顺序验证器连接起来。这样每个场景都能同时记录：捕获文件是否存在、CFG 和真值是否匹配、是否存在实测校准文件、HDF5 的 AoA 回归结果，以及最终能否进入真实硬件验收。

## 已完成验证

本次使用 `v04_58_measured_package_pipeline/synthetic_manifest.json` 运行 3 个已知角度 HDF5 场景：

| 场景 | 真值方位角 | 真值俯仰角 | 真值距离 | HDF5 处理 | 最佳方位误差 | 最佳俯仰误差 |
|---|---:|---:|---:|---|---:|---:|
| `scene_az005_el002_r020` | 5° | 2° | 20 m | 是 | 0° | 0° |
| `scene_az015_el008_r030` | 15° | 8° | 30 m | 是 | 0° | 0° |
| `scene_az030_el005_r040` | 30° | 5° | 40 m | 是 | 0° | 0° |

## 结果解释

这 3 个场景证明“manifest → HDF5 → AoA 回归 → 输出汇总”的软件链路是通的，且当前合成已知角度回归没有误差。但是它们的 HDF5 provenance 仍是 synthetic，不是 DCA1000/LVDS 实测数据；manifest 也没有实测通道幅相校准文件。因此 `hardware_ready=false` 是预期结果，不应向领导表述为“真实板卡 AoA 已验收”。

详细机器可读结果保存在：

- `simulation/hardware/awr2944pev/v04_58_measured_package_pipeline/synthetic_run/pipeline_summary.json`
- `simulation/hardware/awr2944pev/v04_58_measured_package_pipeline/synthetic_run/scene_status.csv`
- `simulation/hardware/awr2944pev/v04_58_measured_package_pipeline/synthetic_run/output_analysis.md`

## 进入真实硬件验收的条件

至少需要 4 个不同已知角度/距离的实测场景，且每个场景同时具备 DCA1000 原始数据或可靠导出的 HDF5、实际使用的 CFG、角度/距离真值、TI 校准后的 RX 通道幅相文件，并在 manifest 中标记 `calibration_measured=true`。在这些条件满足前，本阶段只作为软件闭环和合成数据回归证据。

## 与 PCB/CAD 资料的关系

用户提供的 `sprr440a (1)` 和 `sprr441a` 已完成来源审计。ASCII `PROC113D_ASCII.PcbDoc` 可以提取 TX1–TX4、RX1–RX4 的铜区候选坐标，用于构造候选虚拟阵列；但它们仍不是电气相位中心。真实硬件验收仍需把 PCB/CAD 坐标、CFG 的通道顺序和实测校准矩阵同时锁定。
