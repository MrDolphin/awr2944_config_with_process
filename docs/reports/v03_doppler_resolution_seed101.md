# V0.3.5 Doppler 分辨率对比记录

固定 `ss3_upper`（Hs=1.00 m）、seed=101、散射微元、距离权重和 77 GHz/1 GHz FMCW，
只比较每帧慢时间 Chirp 数：

| Chirp 数 | 速度分辨率 | 最强峰距离 | 最强峰速度 |
|---:|---:|---:|---:|
| 64 | 0.304173 m/s | 10.540 m | 0 m/s |
| 256 | 0.076043 m/s | 10.540 m | -0.076043 m/s |

256 Chirp 后速度网格变细，最强峰从 0 m/s 移到 -0.076 m/s，说明之前的 0 m/s
结果部分受 Doppler 分辨率限制。但该峰仍只是固定网格海面微元的 Eulerian 斜距变化率，
不是水质点三维轨道速度，也不是实测海杂波谱。256 Chirp 的 CPI 为 25.6 ms，真实船体
运动时还必须考虑姿态变化和相位稳定性。

实际结果目录：`simulation/stages/v03_complex_echo_range_doppler/results/comparisons/v03c_doppler_seed101_ss3upper/`。
