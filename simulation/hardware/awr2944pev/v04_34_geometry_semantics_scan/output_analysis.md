# V0.4.34 阵列几何语义成对扫描

扫描方位：-60°～60°；俯仰：-20°～20°。

本阶段不假设任何一个模型是真实 EVM 阵列，而是对候选‘实际几何—估计几何’组合逐对计算 AoA 误差。

| 假设实际模型 | 最优估计模型 | 综合 RMSE(°) |
|---|---|---:|
| cfg_raw | tx_rx_swapped | 17.9308 |
| cfg_swap_row_column | cfg_y_mirror | 32.1610 |
| tx_rx_regular | tx_rx_regular | 0.0000 |
| tx_rx_swapped | tx_rx_swapped | 0.0000 |
| cfg_y_mirror | tx_rx_swapped | 17.8553 |
| regular_y_mirror | regular_y_mirror | 0.0000 |

## 结论边界

对角度网格上的大误差说明对应坐标语义会发生空间混叠或相位展开失败；它不能单独证明哪一个模型是硬件真实模型。必须使用装配基准、Altium 网表/封装和已知角 DCA1000 数据选择模型。
