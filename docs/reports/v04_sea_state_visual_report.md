# V0.4.25 海况点云图文报告

## 输出

对每个海况输出：

- 距离-速度检测图；
- 方位-俯仰散点图；
- 三维点云图。

同时输出跨海况的点数和速度展宽对比图，并生成可直接用于汇报的 `sea_state_comparison.md`。

输出目录：`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_25_sea_state_visual_report/`。

## 解释方法

先看点数变化，再看速度标准差和角度散布；点数增加可能来自海杂波增强，也可能来自 CFAR 阈值或噪声估计变化，不能单独作为海况识别结论。
