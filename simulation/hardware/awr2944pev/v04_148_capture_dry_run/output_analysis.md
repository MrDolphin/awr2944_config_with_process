# V0.4.148 DCA1000 采集工具 dry-run

本阶段实际调用 `tools/dca1000_capture.py --dry-run`，只解析候选 CFG 和脱敏 DCA JSON，不打开 UDP socket、不发送 DCA 命令、不写入 capture.bin。

结果字段应为：

- `status: dry_run_only`
- `hardware_commands_executed: false`
- `udp_socket_opened: false`

只有现场审核 dry-run 输出、确认 SDK/固件和网络参数后，才允许去掉 `--dry-run` 进入真实中心角采集。
