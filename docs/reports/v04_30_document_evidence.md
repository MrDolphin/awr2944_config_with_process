# V0.4.30 原理图/装配/层叠文档证据审计

## 实际提取结果

本阶段对用户资料目录中的文档执行了可复现审计：

```text
PDF   3 个
SchDoc 25 个
总计  28 个
```

其中 22 个文档出现了 `TX`、`RX`、`RF`、`ANT` 或 `AWR2944` 字符串。

最有价值的 PDF 文本证据来自：

```text
PROC113D(001)_Sch.PDF
```

第 2 页提取到：

```text
50 ohm GCPW traces to antenna
TX1 TX2 TX3 TX4
RX1 RX2 RX3 RX4
```

这支持一个重要判断：`TX1–TX4/RX1–RX4` 不只是 PCB 网络表中的随机名称，而是在原理图 PDF 中与“到天线的 50 ohm GCPW 走线”一起出现，因而可提升为“文档支持的 RF 通道候选”。

第 21 页还出现：

```text
TX0_P TX0_N TX1_P TX1_N TX2_P TX2_N TX3_P TX3_N
```

这些更像芯片/高速差分接口或射频通道引脚标签，需要结合具体原理图页和网络拓扑区分，不能仅凭名称确定。

## SchDoc 解析边界

`.SchDoc` 是 Altium 原生二进制格式。本阶段采用 UTF-8/UTF-16 字符串扫描，只用于发现可能的标签。扫描会产生大量类似 `RFxxxxxx` 的内部唯一标识，因此这些结果统一标记为：

```text
binary_token_candidate
```

它们不能替代引出的引脚、网络和连通性信息。要继续提升证据等级，需要使用 Altium 导出 ASCII、网表或 PDF 原理图页进行引脚级解析。

## 产物

结果目录：

```text
simulation/hardware/awr2944pev/v04_30_document_evidence/
```

- `document_evidence.json`：每个文档的 SHA-256、页数、提取状态和候选 token；
- `document_rf_token_candidates.csv`：候选 token、PDF 页码、原文片段和证据等级；
- `document_evidence_summary.json`：汇总状态；
- `output_analysis.md`：边界说明。

## 结论

当前可以把 `TX1–TX4/RX1–RX4` 标记为：

```text
document_supported_rf_channel_candidate
```

但不能标记为：

```text
confirmed_antenna_phase_center
confirmed_array_coordinate
```

V0.4.30 因此完成了“网络名—PCB 铜区—原理图 RF 走线文字”的三源交叉支持，但真实相位中心、绝对阵面坐标、通道幅相和 AoA 校准仍需封装资料、Altium 网表/导出和实测数据确认。
