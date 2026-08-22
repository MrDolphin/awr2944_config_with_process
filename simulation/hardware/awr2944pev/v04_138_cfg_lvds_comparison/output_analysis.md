# V0.4.138 CFG 与 LVDS 参考配置对账

目标 CFG：`D:\hp-laptop\USV\awr2944_sea_clutter_v02\Config\profile_3d_3Azim_1ElevTx_awr2944P.cfg`
参考 CFG：`D:\hp-laptop\USV\awr2944_config_and_process_with_trace\mathlab\PostProc\profile-2944.cfg`

## 关键结论

- 目标 CFG 是否含 `lvdsStreamCfg`：`False`。
- 参考 CFG 是否含 `lvdsStreamCfg`：`True`。
- 目标 profileCfg：`0 77 186 7 57.14 0 0 70 1 656 13349 0 0 158`。
- 参考 profileCfg：`0 77 34 6 66 0 0 60 0 256 5000 0 0 30`。
- 目标 frameCfg：`0 3 16 0 656 100 1 0`。
- 参考 frameCfg：`0 0 128 0 256 200 1 0`。

## 不能直接复制参考 CFG

参考 CFG 的 LVDS 命令证明旧流程曾启用 DCA1000 输出，但它的采样点数、frame/chirp 组织、天线几何和其他命令与当前 3D CFG 不同。直接整文件替换会破坏当前 AWR2944P 3D AoA 的维度/映射合同。

## 建议动作

先由 TI SDK/板卡实际版本确认适用于当前 AWR2944P profile 的 LVDS 参数，再在副本 CFG 中加入并重新计算 SHA-256；完成预检后先做短时中心角度采集，不要覆盖原始 CFG。
