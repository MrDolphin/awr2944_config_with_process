# V0.4.107 理想阵列与 PCB 几何候选比较

本阶段将 V0.4.106 临时 TX/RX 端点候选接入稀疏阵列导向矢量求解器。扫描输入是无噪声合成 IQ；结果用于几何敏感性分析，不是硬件 AoA 验证。

| 实际模型 | 估计模型 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) | 最大绝对误差(°) |
|---|---|---:|---:|---:|---:
| ideal_half_lambda | ideal_half_lambda | 0.000 | 0.000 | 0.000 | 0.0 |
| ideal_half_lambda | provisional_pcb_endpoint | 43.028 | 10.954 | 31.396 | 86.0 |
| provisional_pcb_endpoint | ideal_half_lambda | 62.645 | 20.368 | 46.579 | 100.0 |
| provisional_pcb_endpoint | provisional_pcb_endpoint | 0.000 | 0.000 | 0.000 | 0.0 |

## 如何解释

同模型实际/估计时的误差代表当前网格搜索和角度量化误差；交叉模型误差表示若阵列几何假设不一致，AoA 可能产生偏差或空间混叠。PCB 候选含多波长间距，旁瓣和角度歧义是预期现象。

## 证据边界

PCB 模型来自 RF 走线端点的几何候选，状态为 `geometric_endpoint_candidate_not_phase_center`。未使用实测 DCA1000 IQ、方向图、互耦或校准矩阵，不能推导真实 AWR2944P AoA 精度。
