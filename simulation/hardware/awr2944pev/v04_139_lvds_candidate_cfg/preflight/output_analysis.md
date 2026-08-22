# V0.4.137 DCA1000 采集环境预检

本阶段只检查本地文件、Python 依赖和 CFG，不打开串口、不连接 DCA1000、不启动雷达。

## 检查结果

- `cfg_exists`：`PASS`
- `lvds_stream_config_present`：`PASS`
- `dca1000_cli_control_present`：`PASS`
- `dca1000_cli_record_present`：`PASS`
- `python_serial_present`：`PASS`
- `python_numpy_present`：`PASS`
- `python_h5py_present`：`PASS`
- `output_writable`：`PASS`

## 解释

`lvdsStreamCfg` 缺失时，雷达 CFG 可能只输出 UART 点云，DCA1000 不一定收到 ADC 数据。CLI 工具存在只说明软件文件在本机，不代表网络、DCA1000 或雷达已经连通。

本次 `ready_for_operator_capture`：`True`。该状态不是硬件验证，只是采集前本地环境检查。
