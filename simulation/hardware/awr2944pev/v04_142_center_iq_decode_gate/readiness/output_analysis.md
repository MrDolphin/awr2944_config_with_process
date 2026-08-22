# V0.4.141 中心角短时采集准入检查

案例目录：`D:\hp-laptop\USV\awr2944_sea_clutter_v02\simulation\hardware\awr2944pev\v04_140_lvds_candidate_manifest_batch\lv003_az+00_el-10_r10`

检查状态：`awaiting_center_capture_inputs`

缺项或不一致：`capture_missing、capture_timestamp_missing、imu_or_platform_pose_reference_missing`

## 判定含义

本检查只验证 manifest、CFG 副本、CFG SHA-256、capture.bin 非空以及中心角记录是否齐全；它不解码 IQ，也不证明 LVDS wire order、TI 校准或硬件 AoA 精度。即使 `ready_for_decode=true`，仍需运行 V0.4.112 并完成已知角度通道顺序和校准验证。

下一步：complete the listed capture inputs, then rerun this check。
