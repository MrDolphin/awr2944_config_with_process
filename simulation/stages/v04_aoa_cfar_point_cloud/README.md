# V0.4 AoA、CFAR与三维点云

状态：V0.4.11 DCA1000/TDM/AoA 合成端到端闭环、通道排列故障指纹和 PCB/CAD 资料审计已完成；输入必须引用一个通过验收的 V0.3 复数回波运行目录。本阶段
加入二维 AoA、CFAR 和三维杂波点云，并输出检测概率、虚警率、角度误差、
异常跳变率及点云稳定性指标。

结果保存到 `results/python/<run_id>/` 或 `results/matlab/<run_id>/`，同时记录
所引用的 V0.3 `run_id` 和所有检测阈值。

## V0.4.0 当前边界

当前运行器 `simulation/run_v04_aoa.py` 使用软件定义的 4TX×4RX 虚拟阵列相位斜坡，
验证已知方位/俯仰角能否恢复。阵元坐标不是 AWR2944P EVM 实测布局，估计器也不是
TI SDK 的生产 AoA/MUSIC/ESPRIT 实现；当前不声称真实 AoA 精度、FOV、旁瓣、CFAR
或三维点云能力。下一步才加入噪声、真实阵列坐标和 AoA 扫描/CFAR。

V0.4.10 的故障指纹表见 `docs/reports/v04_channel_diagnostics.md`；真实 EVM 数据到来前，通道顺序仍标记为 `synthetic_only`。

V0.4.11 的 PCB/CAD 文件格式、SHA-256 和可提取字段见 `simulation/stages/v04_aoa_cfar_point_cloud/output/pcb_package_inventory.md`，分析报告见 `docs/reports/v04_pcb_package_audit.md`。

V0.4.12 将 PCB 虚拟通道与 `antGeometryCfg` 逐通道对齐，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_12_coordinate_mapping/` 和 `docs/reports/v04_coordinate_mapping.md`。

V0.4.13 对 PCB 坐标的原始、X 镜像、Y 镜像和 180° 旋转进行 AoA 筛查，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_13_coordinate_transform_scan/` 和 `docs/reports/v04_coordinate_transform_scan.md`。

V0.4.14 从 ASCII PCB 的 TX/RX 网络提取 Pad 和 Track 端点候选，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_14_rf_port_extraction/` 和 `docs/reports/v04_rf_port_extraction.md`。

V0.4.15 将 RF 网络的 Pad、Track、Arc、Region 建成几何连通图，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_15_rf_connectivity/` 和 `docs/reports/v04_rf_connectivity.md`。

V0.4.16 提取 PCB 层叠参数并统计 RF 对象层，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_16_pcb_layer_stack/` 和 `docs/reports/v04_pcb_layer_stack.md`。

V0.4.17 以 RF 几何路径和候选有效介电常数进行相位敏感性扫描，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_17_phase_sensitivity/` 和 `docs/reports/v04_phase_sensitivity.md`。

V0.4.18 建立 4×4 复数通道校准接口并验证补偿方向，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_18_calibration_scan/` 和 `docs/reports/v04_calibration_scan.md`。

V0.4.19 解析 CFG 的 `compRangeBiasAndRxChanPhase` 为 4×4 复数矩阵，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_19_cfg_calibration/` 和 `docs/reports/v04_cfg_calibration_ingest.md`。

V0.4.20 对 CFG 校准因子做直接、复数逆、共轭逆方向 A/B，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_20_calibration_direction/` 和 `docs/reports/v04_calibration_direction.md`。

V0.4.21 增加 DCA1000 `.bin` 抓包接入、TDM 重排和可选校准 HDF5 输出，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_21_capture_ingest/` 和 `docs/reports/v04_capture_ingest.md`。

V0.4.22 在规范 HDF5 上执行距离 FFT、Doppler FFT 和峰值单元 AoA，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_22_range_doppler_aoa/` 和 `docs/reports/v04_range_doppler_aoa.md`。

V0.4.23 在距离-多普勒功率图上执行 2D CA-CFAR、多峰筛选和三维点云输出，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_23_cfar_point_cloud/` 和 `docs/reports/v04_cfar_point_cloud.md`。

V0.4.24 对多个海况点云运行做统一统计，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_24_point_cloud_stats/` 和 `docs/reports/v04_point_cloud_stats.md`。

V0.4.25 自动生成各海况距离-速度、方位-俯仰、三维点云图和 Markdown 汇报，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_25_sea_state_visual_report/` 和 `docs/reports/v04_sea_state_visual_report.md`。

V0.4.26 对点云统计做相对基准的描述性判读，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_26_sea_state_interpretation/` 和 `docs/reports/v04_sea_state_interpretation.md`。

V0.4.27 将统计、图像和描述性判读串成一键海况报告包，结果见 `simulation/stages/v04_aoa_cfar_point_cloud/output/v04_27_sea_state_bundle/` 和 `docs/reports/v04_sea_state_bundle.md`。

V0.4.28 对用户新增的 AWR2944P EVM PCB/CAD 资料进行来源清单、ASCII PCB 记录、层叠、元件、网络和焊盘审计；结果见 `simulation/hardware/awr2944pev/v04_28_pcb_audit/` 和 `docs/reports/v04_28_pcb_audit.md`。TX/RX 网络与焊盘仍是候选，不代表已确认的天线相位中心。

V0.4.29 将 8 个已提取的 TX/RX 铜区与当前 `antGeometryCfg` 展开的 16 个虚拟通道做候选映射，并扫描平移归一化后的镜像/旋转解释；结果见 `simulation/hardware/awr2944pev/v04_29_candidate_mapping/` 和 `docs/reports/v04_29_candidate_mapping.md`。最小误差变换只代表相对几何筛查，不代表真实阵面方向或相位中心。

V0.4.30 对新增资料中的 3 个 PDF 和 25 个 SchDoc 做文档证据审计；原理图 PDF 第 2 页出现“50 ohm GCPW traces to antenna”及 TX1~TX4/RX1~RX4，可支持 RF 通道候选，但未确认相位中心。结果见 `simulation/hardware/awr2944pev/v04_30_document_evidence/` 和 `docs/reports/v04_30_document_evidence.md`。

V0.4.31 将 PCB RF 区域、`antGeometryCfg` 和原理图 GCPW 文字证据汇总到每个虚拟通道，并生成校准输入契约和缺口登记；结果见 `simulation/hardware/awr2944pev/v04_31_evidence_grade/` 和 `docs/reports/v04_31_evidence_grade.md`。16 个通道仍为 `candidate_only`。

V0.4.32 增加 DCA1000/HDF5、4×4 校准矩阵、CFG 和通道顺序的真实 AoA 就绪性闸门；当前合成校准文件仅通过形状检查，未通过实测校准和通道顺序检查。结果见 `simulation/hardware/awr2944pev/v04_32_calibration_readiness/` 和 `docs/reports/v04_32_calibration_readiness.md`。

V0.4.33 增加带真值元数据的已知角度 HDF5 回归夹具，并用同一 `antGeometryCfg` 几何运行 V0.4.22 AoA；低角度回归通过，高角度暴露空间间距/相位展开混叠风险。结果见 `simulation/hardware/awr2944pev/v04_33_known_angle_fixture/` 和 `docs/reports/v04_33_known_angle_fixture.md`。

V0.4.34 对 `cfg_raw`、row/column 交换、规则 TX/RX 阵列及镜像模型做成对角度网格扫描；结果见 `simulation/hardware/awr2944pev/v04_34_geometry_semantics_scan/` 和 `docs/reports/v04_34_geometry_semantics_scan.md`。扫描用于筛查几何语义，不等同于硬件真实阵列验证。

V0.4.35 从 ASCII PCB 提取板框原点、约 85 mm × 125 mm 板框、RF 区域板框相对坐标和机械参考候选；装配图未提供可直接提取的相位中心/安装姿态尺寸。结果见 `simulation/hardware/awr2944pev/v04_35_mechanical_datum/` 和 `docs/reports/v04_35_mechanical_datum.md`。

V0.4.36 增加 PCB 板框到雷达/船体坐标的旋转和平移候选接口，并输出板面法向与竖直方向夹角；当前姿态仍为 `pose_candidate`，未确认实际安装角。结果见 `simulation/hardware/awr2944pev/v04_36_pose_transform/` 和 `docs/reports/v04_36_pose_transform.md`。
