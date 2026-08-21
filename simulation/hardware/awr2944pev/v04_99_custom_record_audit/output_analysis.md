# V0.4.99 自定义 AWR record 文件结构审计

扫描文件：3 个。

## 结论

record_01.bin、record_02.bin、record_03.bin 均包含重复的 8 字节 AWR magic 和 48 字节候选帧头，不符合当前 V0.4.91 直接读取 payload-only DCA1000 BIN 的输入假设。

当前只确认了文件结构和序号连续性，尚未确认 48 字节字段含义、payload 是否为 ADC IQ、RX/TX 排列或 chirp/sample 边界。因此 `standard_dca_payload_ready=false`。

## 下一步

需要将这些 record 文件与采集脚本的元数据、实际 CFG 和 UDP 保存模式对应起来；确认后再写专用解包器，不直接删头或猜测 IQ 维度。
