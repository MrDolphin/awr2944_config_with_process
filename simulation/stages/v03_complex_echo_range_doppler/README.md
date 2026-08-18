# V0.3 复数回波与Range-Doppler

状态：尚未开始。输入必须引用一个通过验收且不可覆盖的 V0.2 动态海面运行
目录。本阶段加入 AWR2944P 4TX/4RX FMCW 复数回波、距离处理和多普勒处理，
但暂不输出 AoA、CFAR 或三维检测点云。

结果保存到 `results/python/<run_id>/` 或 `results/matlab/<run_id>/`，并记录所
引用的 V0.2 `run_id`、CFG、虚拟阵列顺序、窗函数和 FFT 参数。
