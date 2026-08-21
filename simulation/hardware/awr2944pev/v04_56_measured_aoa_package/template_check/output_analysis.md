# V0.4.56 实测 AoA 验收包检查

真实 AoA 就绪：否

| 检查项 | 结果 |
|---|---|
| scene_count_at_least_4 | 未通过 |
| all_scene_keys_present | 通过 |
| all_capture_files_present | 未通过 |
| all_cfg_files_present | 未通过 |
| all_truth_files_present | 未通过 |
| all_measured_calibration_present | 未通过 |
| angle_distance_coverage_declared | 通过 |

## 场景逐项状态

| 场景 | capture | CFG | truth | calibration |
|---|---|---|---|---|
| corner_reflector_az000_el000_r020 | 缺失 | 缺失 | 缺失/字段不全 | 缺失/非实测 |

## 解释

该检查器只判断证据包是否完整，不会把合成 fixture 或 identity calibration 当作实测通过。所有场景必须同时具备原始 IQ、CFG、角度/距离真值和实测校准 provenance，才允许进入 V0.55 综合 AoA 门禁。
