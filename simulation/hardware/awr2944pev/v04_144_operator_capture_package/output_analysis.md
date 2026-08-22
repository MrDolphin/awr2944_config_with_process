# V0.4.144 操作员采集包（只读准备）

本目录只保存候选 CFG、DCA1000 历史参考配置和人工操作清单；脚本没有执行任何雷达、DCA1000 或网络命令。

## 文件证据

- 候选 CFG：`D:\hp-laptop\USV\awr2944_sea_clutter_v02\simulation\hardware\awr2944pev\v04_139_lvds_candidate_cfg\profile_3d_3Azim_1ElevTx_awr2944P.cfg`
- 候选 CFG SHA-256：`d410727aea1f03c6211dcb58244386f7234bd17a5bdc23917848c5dee17135ac`
- DCA 参考文件：`D:\hp-laptop\USV\awr2944_config_and_process_with_trace\mathlab\PostProc\cf.json`
- DCA 参考 SHA-256：`bae8194e907550ace621428bdcaa4a99f6e3f54c95c5bd25f0ac0e74652d3019`

## 从历史参考文件读取的参数（不是当前硬件确认值）

- DCA IP：`<operator_verify>`；配置端口：`4096`；数据端口：`4098`
- 历史采集模式：`LVDSCapture`；LVDS mode：`2`
- 历史文件前缀：`adc_data`；历史帧数：`40`

## 人工执行顺序

1. 确认 AWR2944P SDK/固件接受 V0.4.139 候选 CFG；确认不使用正式配置覆盖。
2. 确认 PC 网卡、DCA1000 IP、MAC、配置端口和数据端口与现场实际值一致；历史 JSON 中的地址不能直接照搬。
3. 在 DCA1000 CLI 安装目录运行 `DCA1000EVM_CLI_Control.exe --help` 和 `DCA1000EVM_CLI_Record.exe --help`，按本机 CLI 版本确认参数顺序。
4. 先布置中心角 10 m 角反射器，记录安装高度、方位/俯仰和 IMU 姿态。
5. 手工执行 FPGA/configure、record/start、短时停止；不要在本项目脚本中自动下发。
6. 把原始文件复制到 V0.4.140 的 `lv003_az+00_el-10_r10`，并记录文件 SHA-256。
7. 运行 V0.4.141，再运行 V0.4.142；未通过准入时不得解码。

## 证据边界

本包不证明 DCA1000 已连接，也不证明历史网络参数仍适用。只有现场执行日志、capture.bin、CFG 快照、姿态记录和校准记录齐全后，才能进入真实 IQ/AoA 验证。
