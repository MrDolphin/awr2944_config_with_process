# V0.4.45 CA-CFAR 参数扫描

本阶段固定 V0.4.43 合成距离-多普勒功率谱，只改变 Pfa、训练单元和保护单元，区分 CFAR 门限效应与海况/散射模型效应。

| 海况 | CFAR 场景 | Pfa | 训练单元 | 检测点数 | 点/帧 | 平均功率 |
|---|---|---:|---|---:|---:|---:|
| ss0_flat | pfa1e2_train2 | 1e-02 | 2×2 | 0 | 0.000 | nan |
| ss0_flat | pfa1e3_train2 | 1e-03 | 2×2 | 0 | 0.000 | nan |
| ss0_flat | pfa1e4_train2 | 1e-04 | 2×2 | 0 | 0.000 | nan |
| ss0_flat | pfa1e2_train4 | 1e-02 | 4×4 | 0 | 0.000 | nan |
| ss0_flat | pfa1e3_train4 | 1e-03 | 4×4 | 0 | 0.000 | nan |
| ss0_flat | pfa1e4_train4 | 1e-04 | 4×4 | 0 | 0.000 | nan |
| ss0_flat | pfa1e2_train8 | 1e-02 | 8×8 | 0 | 0.000 | nan |
| ss0_flat | pfa1e3_train8 | 1e-03 | 8×8 | 0 | 0.000 | nan |
| ss0_flat | pfa1e4_train8 | 1e-04 | 8×8 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e2_train2 | 1e-02 | 2×2 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e3_train2 | 1e-03 | 2×2 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e4_train2 | 1e-04 | 2×2 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e2_train4 | 1e-02 | 4×4 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e3_train4 | 1e-03 | 4×4 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e4_train4 | 1e-04 | 4×4 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e2_train8 | 1e-02 | 8×8 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e3_train8 | 1e-03 | 8×8 | 0 | 0.000 | nan |
| ss1_rippled | pfa1e4_train8 | 1e-04 | 8×8 | 0 | 0.000 | nan |
| ss2_normal | pfa1e2_train2 | 1e-02 | 2×2 | 0 | 0.000 | nan |
| ss2_normal | pfa1e3_train2 | 1e-03 | 2×2 | 0 | 0.000 | nan |
| ss2_normal | pfa1e4_train2 | 1e-04 | 2×2 | 0 | 0.000 | nan |
| ss2_normal | pfa1e2_train4 | 1e-02 | 4×4 | 0 | 0.000 | nan |
| ss2_normal | pfa1e3_train4 | 1e-03 | 4×4 | 0 | 0.000 | nan |
| ss2_normal | pfa1e4_train4 | 1e-04 | 4×4 | 0 | 0.000 | nan |
| ss2_normal | pfa1e2_train8 | 1e-02 | 8×8 | 0 | 0.000 | nan |
| ss2_normal | pfa1e3_train8 | 1e-03 | 8×8 | 0 | 0.000 | nan |
| ss2_normal | pfa1e4_train8 | 1e-04 | 8×8 | 0 | 0.000 | nan |
| ss3_nominal | pfa1e2_train2 | 1e-02 | 2×2 | 5 | 0.122 | 7.990e+04 |
| ss3_nominal | pfa1e3_train2 | 1e-03 | 2×2 | 4 | 0.098 | 8.110e+04 |
| ss3_nominal | pfa1e4_train2 | 1e-04 | 2×2 | 3 | 0.073 | 8.919e+04 |
| ss3_nominal | pfa1e2_train4 | 1e-02 | 4×4 | 2 | 0.049 | 6.486e+04 |
| ss3_nominal | pfa1e3_train4 | 1e-03 | 4×4 | 2 | 0.049 | 6.486e+04 |
| ss3_nominal | pfa1e4_train4 | 1e-04 | 4×4 | 0 | 0.000 | nan |
| ss3_nominal | pfa1e2_train8 | 1e-02 | 8×8 | 0 | 0.000 | nan |
| ss3_nominal | pfa1e3_train8 | 1e-03 | 8×8 | 0 | 0.000 | nan |
| ss3_nominal | pfa1e4_train8 | 1e-04 | 8×8 | 0 | 0.000 | nan |
| ss3_upper | pfa1e2_train2 | 1e-02 | 2×2 | 11 | 0.268 | 9.632e+04 |
| ss3_upper | pfa1e3_train2 | 1e-03 | 2×2 | 6 | 0.146 | 1.309e+05 |
| ss3_upper | pfa1e4_train2 | 1e-04 | 2×2 | 6 | 0.146 | 1.309e+05 |
| ss3_upper | pfa1e2_train4 | 1e-02 | 4×4 | 3 | 0.073 | 9.814e+04 |
| ss3_upper | pfa1e3_train4 | 1e-03 | 4×4 | 2 | 0.049 | 1.309e+05 |
| ss3_upper | pfa1e4_train4 | 1e-04 | 4×4 | 2 | 0.049 | 1.309e+05 |
| ss3_upper | pfa1e2_train8 | 1e-02 | 8×8 | 0 | 0.000 | nan |
| ss3_upper | pfa1e3_train8 | 1e-03 | 8×8 | 0 | 0.000 | nan |
| ss3_upper | pfa1e4_train8 | 1e-04 | 8×8 | 0 | 0.000 | nan |

## 分析方法

先固定同一海况的所有输入，只横向比较 CFAR 场景；再固定 CFAR 场景，纵向比较 ss0～ss3。若同一海况的检测点数随 Pfa/训练窗显著变化，说明 V0.4.44 的零点现象部分来自门限配置，不能直接归因于海况。

## 边界

输入仍是 V0.4.43 合成海杂波功率谱，不是实测 IQ；本阶段只统计 CFAR，不重新估计 AoA；结果不能作为真实虚警率或探测距离。
