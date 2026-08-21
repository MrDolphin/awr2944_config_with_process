# V0.4 AoA、CFAR与三维点云

状态：V0.4.8 DCA1000 IQ 到 4TX TDM 虚拟通道重排合同已完成；输入必须引用一个通过验收的 V0.3 复数回波运行目录。本阶段
加入二维 AoA、CFAR 和三维杂波点云，并输出检测概率、虚警率、角度误差、
异常跳变率及点云稳定性指标。

结果保存到 `results/python/<run_id>/` 或 `results/matlab/<run_id>/`，同时记录
所引用的 V0.3 `run_id` 和所有检测阈值。

## V0.4.0 当前边界

当前运行器 `simulation/run_v04_aoa.py` 使用软件定义的 4TX×4RX 虚拟阵列相位斜坡，
验证已知方位/俯仰角能否恢复。阵元坐标不是 AWR2944P EVM 实测布局，估计器也不是
TI SDK 的生产 AoA/MUSIC/ESPRIT 实现；当前不声称真实 AoA 精度、FOV、旁瓣、CFAR
或三维点云能力。下一步才加入噪声、真实阵列坐标和 AoA 扫描/CFAR。
