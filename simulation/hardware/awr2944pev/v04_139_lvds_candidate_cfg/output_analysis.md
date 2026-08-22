# V0.4.139 LVDS 候选 CFG

本阶段只在输出目录生成候选 CFG 副本，在当前 656-sample 3D CFG 原文中插入 `lvdsStreamCfg -1 0 1 0`，不修改正式 CFG。

## 重要边界

该命令来自旧 DCA1000 参考流程，当前候选只保留了原 profileCfg/frameCfg/TDM 配置，尚未通过 TI SDK、当前固件版本或真实板卡验证。它可以用于下一步 operator-reviewed 短采集准备，不能直接宣称 DCA1000 一定收到 ADC 数据。

## 验证

先对候选 CFG 运行 V0.4.137；随后由操作者确认板卡、DCA1000 网络和固件版本，再决定是否发送。
