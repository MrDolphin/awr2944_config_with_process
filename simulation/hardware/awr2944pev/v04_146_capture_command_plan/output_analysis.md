# V0.4.146 DCA1000 中心角采集 dry-run 命令计划

本阶段只生成命令文本，没有执行任何命令。

## 命令顺序

- `inspect`：`DCA1000EVM_CLI_Control.exe -h`
- `inspect`：`DCA1000EVM_CLI_Record.exe -h`
- `configure_fpga`：`DCA1000EVM_CLI_Control.exe fpga <verified_cf.json>`（需现场审核）
- `configure_record`：`DCA1000EVM_CLI_Control.exe record <verified_cf.json>`（需现场审核）
- `start_record`：`DCA1000EVM_CLI_Control.exe start_record <verified_cf.json>`（需现场审核）
- `capture_listener`：`python tools/dca1000_capture.py --cfg "D:\hp-laptop\USV\awr2944_sea_clutter_v02\simulation\hardware\awr2944pev\v04_139_lvds_candidate_cfg\profile_3d_3Azim_1ElevTx_awr2944P.cfg" --duration 3 --no-control`（需现场审核）
- `stop_record`：`DCA1000EVM_CLI_Control.exe stop_record <verified_cf.json>`（需现场审核）

## 安全边界

- 本文件只生成命令文本，不调用 subprocess，不连接网卡，不启动雷达或 DCA1000。
- 尖括号中的 JSON 必须由操作员现场确认，不能直接使用历史网络参数。
- 先确认 SDK/固件与候选 CFG 兼容，再执行 configure_fpga。
- 采集完成后必须保存 capture.bin、CFG、姿态、日志并运行 V0.4.141/V0.4.142。
