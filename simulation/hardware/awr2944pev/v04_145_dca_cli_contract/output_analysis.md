# V0.4.145 DCA1000 CLI 语法只读核对

## 结果

本机已安装的 `DCA1000EVM_CLI_Control.exe` 和 `DCA1000EVM_CLI_Record.exe` 均可执行。无参数运行输出了命令列表，证明当前 CLI 的帮助参数是 `-h`；使用 `--help` 会被识别为无效命令，不能写入现场操作手册。

## 已确认的 Control 命令

`fpga`、`reset_fpga`、`reset_ar_device`、`start_record`、`stop_record`、`record`、`dll_version`、`cli_version`、`fpga_version`、`query_status`、`query_sys_status`。

## 已确认的 Record 命令

`start_record`、`dll_version`、`cli_version`。

## 边界

本阶段只读 CLI 帮助，没有执行 FPGA 配置、网络连接、雷达配置或采集。JSON 参数、IP/MAC、网卡和 SDK/固件兼容性仍必须由操作员在现场确认。历史 `cf.json` 的内部网络标识不进入本阶段输出。

下一步仍是中心角短时采集，然后运行 V0.4.141 和 V0.4.142。
