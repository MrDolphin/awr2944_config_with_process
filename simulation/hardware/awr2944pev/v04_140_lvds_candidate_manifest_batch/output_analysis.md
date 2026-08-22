# V0.4.140 LVDS 候选配置五角度采集模板

## 本阶段做了什么

在 V0.4.135 正式 CFG 模板之外，使用 V0.4.139 生成的非权威 LVDS 候选 CFG，建立五个独立的已知角度目录。每个目录包含 manifest.json、CFG 副本和 README，且记录候选 CFG SHA-256。

## 结果如何解读

- 模板数量：5；方位为 -30°、-15°、0°、+15°、+30°，俯仰 -10°，距离 10 m。
- 候选 CFG SHA-256：`d410727aea1f03c6211dcb58244386f7234bd17a5bdc23917848c5dee17135ac`。
- `authoritative_hardware_cfg=false`：没有修改正式 CFG，也没有自动下发雷达。
- `real_capture_present=false`、`hardware_aoa_validated=false`：目前只能说明采集资料已准备，不能作为硬件 AoA 精度结论。

## 操作员准入

1. 确认当前 AWR2944P SDK/固件支持该 LVDS 命令，并确认 DCA1000 CLI 与采集主机连接。
2. 先只使用中心角 `lv003_az+00_el-10_r10` 做短时采集。
3. 保存 capture.bin、DCA 配置、TI 校准输出、线缆/通道顺序记录和 IMU/安装姿态。
4. 通过 V0.4.133 准入门后，才进入原始 IQ 解码和实测 AoA。

若 SDK/固件不兼容，应停止使用本批候选模板，回退到已验证的数据输出方案；不要把候选 CFG 当作 TI 官方配置。
