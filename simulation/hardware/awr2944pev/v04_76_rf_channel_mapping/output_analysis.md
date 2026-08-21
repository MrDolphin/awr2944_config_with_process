# V0.4.76 PCB RF 网络到 CFG 虚拟通道映射

本阶段把 PCB ASCII 中的 TX1–TX4/RX1–RX4 网络名，与 CFG 的 chirpCfg 发射顺序和 antGeometryCfg 虚拟输入索引对齐。当前配置的 chirp 顺序是 TX1、TX2、TX3、TX4，每个 TX 与 RX1–RX4 组合形成 4 个虚拟输入。

共生成 16 个映射候选，虚拟输入唯一数为 16。

## 重要边界

该表是基于网络名和 CFG 顺序的工程候选，不等价于 TI 固件内部通道重排，也没有证明 PCB Pad 是天线电气相位中心。必须用 DCA1000 原始数据、已知角度目标和 TI 校准幅相矩阵验证通道顺序。
