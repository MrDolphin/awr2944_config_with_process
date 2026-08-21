# V0.4.96 采集 manifest 自动收口

本工具只补写真实文件的 SHA-256 和采集状态，不覆盖目标真值、机械姿态、通道顺序或校准结论。

发现 manifest：30；原始 IQ：0；CFG：0；两者同时存在：0。

两类文件同时存在的工况会标记为 `captured_pending_validation`，下一步仍需运行 V0.94 和 V0.92。
