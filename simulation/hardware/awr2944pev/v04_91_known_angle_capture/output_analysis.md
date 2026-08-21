# V0.4.91 已知角 DCA1000 IQ 处理阶段

本阶段已经实现并测试了真实已知角采集的处理入口，但当前结果目录尚未放入用户的 DCA1000 BIN/HDF5，因此状态为“等待真实采集”，不是硬件 AoA 结果。

## 已完成的处理链

1. 读取 V0.4.90 `manifest.json`；
2. 检查目标方位、俯仰、距离、速度和 CFG 文件；
3. 解码 DCA1000 `int16` BIN 或读取 HDF5 IQ；
4. 按 TDM TX 顺序重排为 `(frame, sample, RX, TX)`；
5. 计算 Range-Doppler 频谱；
6. 根据目标距离和速度选择目标单元；
7. 对 `identity`、`mirror_x`、`mirror_y`、`rotate_180` 分别进行 AoA 估计；
8. 输出方位误差、俯仰误差和综合误差。

## 当前证据状态

| 项目 | 状态 |
|---|---|
| 处理代码 | 已完成 |
| 合成 BIN 回归测试 | 已通过 |
| 真实 DCA1000 文件 | 尚未提供 |
| 通道顺序 | 未由实测确认 |
| TI 校准矩阵 | 未提供 |
| 电气相位中心 | 未确认 |
| 真实硬件 AoA | 未验证 |

## 真实采集后运行

将 `known_angle_capture_manifest.template.json` 复制为采集目录下的 `manifest.json`，填写目标和采集参数，再执行：

```powershell
python -m simulation.run_v04_91_known_angle_capture `
  --manifest D:\your_capture\manifest.json `
  --candidate-csv simulation\hardware\awr2944pev\v04_85_pcb_array_candidate\virtual_array_candidates.csv `
  --output simulation\hardware\awr2944pev\v04_91_known_angle_capture\results\capture_id
```

只有在真实目标真值、DCA1000 原始 IQ、CFG、TDM 顺序和校准证据均可追溯时，输出才能进入真实 AoA 验证报告。
