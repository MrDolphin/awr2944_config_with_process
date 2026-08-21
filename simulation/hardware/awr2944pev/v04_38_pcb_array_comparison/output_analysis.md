# V0.4.38 PCB 候选阵列 AoA 对比

本阶段把 PCB TX/RX 铜区几何质心求和模型接入 AoA 敏感性扫描，并与理想半波长阵列和 CFG 几何候选比较。

扫描方位：-60°～60°；俯仰：-20°～20°。

| 实际几何 | 估计几何 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) |
|---|---|---:|---:|---:
| ideal_half_lambda | ideal_half_lambda | 0.0000 | 0.0000 | 0.0000 |
| ideal_half_lambda | cfg_candidate | 29.0665 | 16.4955 | 23.6322 |
| ideal_half_lambda | pcb_centroid_candidate | 39.9891 | 14.1399 | 29.9922 |
| cfg_candidate | ideal_half_lambda | 64.3076 | 40.8539 | 53.8726 |
| cfg_candidate | cfg_candidate | 38.3208 | 22.6125 | 31.4628 |
| cfg_candidate | pcb_centroid_candidate | 39.9981 | 14.1416 | 29.9986 |
| pcb_centroid_candidate | ideal_half_lambda | 43.5454 | 30.4512 | 37.5731 |
| pcb_centroid_candidate | cfg_candidate | 40.2060 | 22.5825 | 32.6074 |
| pcb_centroid_candidate | pcb_centroid_candidate | 39.9998 | 14.1399 | 29.9994 |

## 如何读结果

对角度网格逐点比较真值和估计值；RMSE 越小表示在当前无噪声、无校准误差的几何模型下越一致。大误差只说明两套坐标定义不一致或出现空间混叠，不能单独证明 PCB 候选就是真实阵列。

## 结果解释和限制

PCB 候选的虚拟间距明显大于半波长，广角扫描会出现相位折叠/空间混叠。当前 V0.4 phase-plane estimator 依赖固定轴向 unwrap，因此即使 actual/assumed 使用同一 PCB 候选，广角自匹配也可能出现非零 RMSE；这说明需要下一阶段的稀疏阵列/整数相位歧义求解器，而不是说明 PCB 文件本身错误。

## 证据边界

PCB 模型来自 RF 铜区几何质心求和，状态为 `cad_copper_centroid_candidate_not_phase_center`。它用于布局敏感性分析，不是 AWR2944P 的实测相位中心、方向图或 TI SDK AoA 验证。
