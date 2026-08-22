# V0.4.136 五工况已知角批量准入

发现 manifest：5 个；成功处理：0 个；失败：5 个。

## 当前结果

本次使用 V0.4.135 的五个 manifest 模板执行批处理。当前 5 个工况均因缺少 `capture.bin` 未进入解码，属于预期的真实采集前 dry-run。

## 通过条件

每个工况必须先补齐真实 capture.bin、manifest 字段、CFG/DCA1000 配置、通道顺序证据和 TI 校准，再重新运行本阶段。只有成功处理数量达到目标覆盖，才可计算坐标变换 RMSE。

## 证据边界

本阶段没有生成硬件 AoA 结论；`hardware_aoa_validated=false`。
