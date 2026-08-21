# V0.4.78 通道映射对海杂波 AoA 点云的影响

本阶段复用 V0.4.72 已筛选的海杂波复数谱点，对 8 种 TX/RX 通道排列候选重新估计 AoA。检测器和检测单元不变，因此这里比较的是通道映射对角度点云的影响。

`nonfinite_aoa_count` 表示当前映射下 AoA 反演得到非有限角度的点数；均值和标准差只对有限角度统计。结果仍是合成数据和候选通道映射，不能替代 DCA1000/TI 固件验证。

## 全部 6040 个保存点的汇总观察

| 映射 | 非有限 AoA 点数 |
|---|---:|
| identity | 31 |
| tx_reverse | 42 |
| rx_reverse | 39 |
| tx_rx_reverse | 55 |
| transpose | 171 |
| transpose_tx_reverse | 211 |
| transpose_rx_reverse | 45 |
| transpose_both_reverse | 134 |

转置类映射明显增加非有限 AoA 点，说明把 TX/RX 维度直接交换会显著改变当前相位平面模型的可解释性。`identity` 的非有限点最少，但这只能说明它在当前合成数据上较稳定，不能证明它就是 EVM 的真实通道顺序。
