# AWR2944P 岸基海面 400 m 原始 ADC 基线配置（V0）

## 结论先行

[`Config/shore_sea_400m_v0_raw_adc.cfg`](../../Config/shore_sea_400m_v0_raw_adc.cfg) 是给 Raspberry Pi + DCA1000 使用的**首轮海面基线采集配置**。它的目的很明确：

> 在固定岸基安装、明确记录海况的条件下，采到能够检查 `300–450 m` 范围内海面回波、噪声背景与时间稳定性的原始 ADC。

它并不宣称 “AWR2944P 已实现 400 m 油膜检测”。更不能由此推导船只、金属角反射器或油膜在 400 m 的 `P_D`、`P_FA`。

## 参数表

| 参数 | V0 值 | 原因 |
|---|---:|---|
| TX / RX | TX0 only / 4 RX | 首轮不使用 TDM MIMO。它避免 4 TX 慢时间交织、TX/RX 相位标定和虚拟阵列重排把海面统计混在一起。4 RX 仍保留接收通道一致性检查与未来角度处理的原始数据。 |
| 起始频率 | 77 GHz | 与当前设备及 77 GHz 海面试验链路一致。 |
| slope | 4 MHz/us | 在 25 MSps 实 ADC 下提供约 468 m 的理论非模糊量程；400 m 的 beat frequency 约 10.67 MHz，小于 12.5 MHz Nyquist。 |
| ADC samples | 3072 | ADC 有效采样窗口为 `3072 / 25 MHz = 122.88 us`。加 6 us ADC start 后仍在 132 us ramp 内。 |
| ADC sampling rate | 25 MSps | AWR2944P 数据表给出的最大 ADC sampling rate 是 45 MSps；25 MSps 不是靠近极限的设置，同时让 400 m 的 beat 留出 Nyquist 余量。 |
| ramp end | 132 us | 支撑 122.88 us ADC 窗口与 6 us ADC start，并约形成 504 MHz 有效 FMCW 带宽。 |
| idle time | 50 us | chirp 周期约 `50 + 132 = 182 us`，可支持数 m/s 量级海面散射的诊断性 Doppler 无模糊区间。 |
| loops/frame | 128 | 单 TX 下每帧 128 chirp；用于每帧的慢时间 FFT。 |
| frame period | 500 ms | 2 Hz Range-Time 更新；减少持续原始 ADC 的存储与网络压力，同时一帧内仍有 23.3 ms coherent processing interval。 |
| ADC 输出 | 16-bit real-only | 与当前 Pi 的已验证采集/解码合同一致。不得把它自动解释成 complex I/Q。 |
| LVDS | `lvdsStreamCfg -1 0 1 0` | 持续输出 raw ADC 到 DCA1000；DCA 物理 lane 拨码不是此 CLI 行能配置的内容。 |
| UART GUI 输出 | disabled | DCA1000 raw ADC 是主证据，关闭额外点云/UART 输出，避免串口数据干扰采集与日志。 |
| clutter removal | disabled | 海面基线正是要保留稳定和缓慢变化的海面回波，不能在采集端先扣除。 |

## 从参数得到的工程量

采用光速 `c = 299,792,458 m/s`、调频斜率 `S = 4×10^12 Hz/s`、采样率 `F_s = 25×10^6 s^-1`：

| 派生量 | 估算 | 含义 |
|---|---:|---|
| 400 m beat frequency | `2SR/c ≈ 10.67 MHz` | 低于 12.5 MHz Nyquist，400 m 在本 profile 的频率窗口内。 |
| 理论非模糊量程 | `cF_s/(4S) ≈ 468.4 m` | 只说明采样频带覆盖，不代表可检测距离。建议只把 `≤ 400 m` 作为本轮明确检查区域。 |
| 近似 range-bin spacing | `c/(2S×122.88 us) ≈ 0.305 m` | FFT bin 间隔，不是最终绝对距离精度。 |
| chirp period | `182 us` | 由 idle + ramp 粗略得出。 |
| 诊断性 Doppler bin | `λ/(2×128×182 us) ≈ 0.084 m/s` | 仅适用于确认 TX0 单发、raw 排列和慢时间相位连续性后。 |
| 诊断性无模糊速度 | `λ/(4×182 us) ≈ ±5.35 m/s` | 不是波高、流速或油膜速度的直接测量。 |
| 每帧 raw payload | `3072×4×128×2 = 3,145,728 bytes` | 按当前 16-bit real-only 解码合同估算。 |
| 平均 raw rate | 约 `6.3 MB/s` | 2 Hz frame rate 下；60 秒约 377 MB。实际应以 DCA metadata 的 saved bytes、packet count、drops 为准。 |

## 为什么不在 V0 就用 4 TX

4 TX 是后续估计方向或形成虚拟阵列的基础，但不是“先证明海面 400 m 有稳定回波”的前置条件。现在立即使用 4-TX TDM 会同时引入：

1. 每个 TX 的慢时间采样率降低为四分之一；
2. RX/TX 相位、TX 顺序和虚拟阵列几何未验证；
3. 原始数据率与解码复杂度提高；
4. 任何目标差异都可能来自 TDM 重排或标定误差，而不是海面/油膜。

V0 先用 TX0 做可复核的 range / range-time / 单 TX range-Doppler 基线；确认海面回波门通过后，再复制 profile 并加入 4 TX TDM，作为独立的 V1 AoA 试验。

## Raspberry Pi 的运行方法

先确认 DCA 专用接口没有被 Wi-Fi 切换破坏：

```bash
ip -4 addr show dev eth0
ip route get 192.168.33.180
ip neigh show dev eth0
```

应看到 `eth0` 有 `192.168.33.30/24`，并且到 `192.168.33.180` 的路由从 `eth0` 出去。然后先做 DCA 控制应答检查：

```bash
cd ~/awr2944_config_with_process_github
source .venv/bin/activate

python tools/dca1000_configure.py \
  --dca-ip 192.168.33.180 \
  --system-ip 192.168.33.30 \
  --mac 12.34.56.78.90.12 \
  --packet-delay-us 25 \
  --timeout 5 \
  --apply
```

确认每条回应为 `status_hex: "00 00"` 后，先做 20 秒短采集。由于默认分析图范围通常只有 15 m，必须明确给出 `450 m`：

```bash
python tools/awr2944_capture_once.py \
  --cfg Config/shore_sea_400m_v0_raw_adc.cfg \
  --duration 20 \
  --analysis-max-range-m 450 \
  --post-analyze
```

这会使用 Pi 本机的 defaults 文件中保存的 CLI、DCA IP、端口、MAC 与输出目录。若 `--show-effective-config` 显示 `cfg` 不是本 V0 文件，必须显式传入上面的 `--cfg`，不要启动采集。

首轮短采集通过的最低条件：

```text
[UDP] Listening on 192.168.33.30:4098 ...
[CAP] ... drops=0
[DONE] Saved <non-zero bytes>
[POST-ANALYSIS] capture_integrity=PASS
```

接着才做 60 秒海面基线采集：

```bash
python tools/awr2944_capture_once.py \
  --cfg Config/shore_sea_400m_v0_raw_adc.cfg \
  --duration 60 \
  --analysis-max-range-m 450 \
  --post-analyze
```

## 采集前必须记录的环境元数据

每个 30–60 秒 run 应有同名或同目录的人工记录，至少包括：

- 雷达经纬度/岸线位置、安装高度、方位角和俯仰角；
- 距岸的 range 参考、拍摄方向照片或视频；
- 时间、潮位、风速/风向、主观海况/浪高等级、降雨/雾；
- 雷达 CFG 的 Git 哈希或文件副本、DCA 固件/packet delay、Pi 网络和缓冲设置；
- `adc_data_*.json` 的 packet count、dropped packets、saved bytes；
- 空场正常海面、已知目标、油/无油控制条件的明确标签。

建议按距离段组织固定岸基海试：`10/20/30 m` 标定与安装排查、`50/75/100 m` 基线、`150/200/250 m` 中距离、`300/400/500 m` 边界探索。先证明 400 m 正常海面回波相对系统噪声可重复，再讨论油膜差异。

## 不要忽略的物理与验证风险

1. **instrumented range 不等于 detection range。** 468 m 只说明本 profile 的 beat-frequency 覆盖，不能推导 400 m 海面、船只或油膜可检测。
2. **海面回波不是点目标。** 海况、掠射角、雷达高度、天线主瓣照射位置、潮位和近岸多径都可能让 400 m 能量变化远大于油膜效应。
3. **当前 raw decoder 是 AWR2944P 的 real-only 候选合同。** 新 profile 的采集应先检查 `.bin` 长度是否接近 `3,145,728 bytes/frame`，并用 metadata 检查 frame rate 与零丢包；不应把当前图当作 AoA 或标定速度。
4. **DCA lane 拨码必须单独确认。** `lvdsStreamCfg` 不会改变 DCA1000 物理 lane mode。先保持当前已验证通路，变更拨码后重新做短数据完整性试验。
5. **角反射器只用于链路/距离验证。** 金属角反射器在 400 m 的成功不证明油膜成功；油膜实验还需要正常海面与可控油/替代物的对照、盲测标签和误报统计。

## 验收门（V0）

| Gate | 条件 | 通过后的含义 |
|---|---|---|
| G0 — 采集通路 | DCA 4096 全部成功、非零 BIN、`drops=0` | 仅证明设备、网络和存储可工作。 |
| G1 — 远距离海面可观测 | 在固定安装与已记录海况下，`300–450 m` 正常海面能量相对热噪声可重复 | 建立是否值得继续油膜试验的基础；不证明目标检测。 |
| G2 — 目标/对照可分 | 已知金属参考目标或受控水面条件在 repeat runs 中有可重复差异 | 可开始量化 SCR/SNR、变化幅度与假警密度。 |
| G3 — 油膜判别 | 正常海面与已知油膜条件下的盲测，按海况分层统计 `P_D`、`P_FA` 与面积误差 | 才能讨论油膜存在/区域估计能力。 |

## 官方依据

TI 当前 AWR2944P 产品页/数据表说明该器件覆盖 76–81 GHz、4 TX / 4 RX、最大 ADC sampling rate 45 MSps、最大 IF bandwidth 20 MHz、AWR2944P 典型 TX power 14 dBm、典型 RX noise figure 10.5 dB，并提供 4-lane Aurora LVDS raw ADC 接口。[AWR2944P product page](https://www.ti.com/product/AWR2944P)；[AWR2944P datasheet](https://www.ti.com/lit/ds/symlink/awr2944p.pdf)

这些器件规格支持本配置的 ADC/IF 选择，但不构成海面 400 m 或油膜识别性能保证。
