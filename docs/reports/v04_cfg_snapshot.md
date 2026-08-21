# V0.4.2 AWR2944P CFG 配置快照记录

已将项目基准配置 `Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg` 解析为：

```text
simulation/stages/v04_aoa_cfar_point_cloud/config/awr2944p_cfg_snapshot.json
```

解析结果确认：

- 平台：AWR2944P；
- TX mask：15（4 个 TX 开启）；
- RX mask：15（4 个 RX 开启）；
- `antGeometryCfg` 已保存原始 token 序列；
- 当前仓库 CFG 的 `aoaFovCfg` 是方位 -90°～90°、俯仰 -90°～90°；
- `measureRangeBiasAndRxChanPhase` 为关闭；
- `compRangeBiasAndRxChanPhase` 为单位幅度/零相位默认补偿；
- 校准状态标记为 `not_measured`；
- 阵列坐标状态标记为 `diagram_derived_or_pending_measurement`。

用户当前实际使用的配置如果是 `aoaFovCfg -1 -60 60 -20 20`，应作为单独的现场 CFG 文件保存并重新生成快照，不能把仓库里的 ±90° 配置误当成现场配置。`aoaFovCfg` 只定义处理搜索范围，不等于整段范围内的实测 AoA 精度。

本阶段没有把 TI 用户指南中的图片尺寸直接伪装成精确 CAD 坐标；下一步需从 EVM 原理图/PCB/装配图提取阵元坐标，并通过角反射器校准获得通道幅相补偿。
