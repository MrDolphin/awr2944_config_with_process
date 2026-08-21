# V0.4.39 稀疏阵列导向矢量 AoA 扫描

本阶段使用逐角度导向矢量匹配替代固定轴相位 unwrap，用于检查 PCB 多波长间距候选阵列的相位折叠风险。

| 实际几何 | 估计几何 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) |
|---|---|---:|---:|---:
| ideal_half_lambda | ideal_half_lambda | 0.0000 | 0.0000 | 0.0000 |
| ideal_half_lambda | cfg_candidate | 43.1032 | 24.5787 | 35.0856 |
| ideal_half_lambda | pcb_centroid_candidate | 43.4202 | 15.5343 | 32.6085 |
| cfg_candidate | ideal_half_lambda | 63.0492 | 20.7020 | 46.9243 |
| cfg_candidate | cfg_candidate | 0.0000 | 0.0000 | 0.0000 |
| cfg_candidate | pcb_centroid_candidate | 63.7683 | 18.2585 | 46.9029 |
| pcb_centroid_candidate | ideal_half_lambda | 42.5320 | 19.9227 | 33.2106 |
| pcb_centroid_candidate | cfg_candidate | 55.5533 | 26.2385 | 43.4432 |
| pcb_centroid_candidate | pcb_centroid_candidate | 0.0000 | 0.0000 | 0.0000 |

## 解释

导向矢量网格搜索对每个候选角度计算复数通道相关性，并消除未知整体复幅度；它不依赖沿阵列轴线的相位 unwrap。若同一模型仍出现多个近似峰，说明阵列存在真实的角度歧义，而不是简单的 unwrap 失败。

## 边界

PCB 仍是 RF 铜区质心候选，不是电气相位中心；输入为无噪声合成 IQ，未包含实测校准矩阵、互耦、方向图和海杂波。结果不能作为真实 AWR2944P AoA 精度。
