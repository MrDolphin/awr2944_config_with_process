# V0.1 平面海面几何与相对功率

状态：已通过 Python 自动测试和 MATLAB GUI 人工验收。

设计要求、坐标系、方向图假设及 HDF5 契约见
[`docs/plans/2026-08-18-awr2944p-v01.md`](../../../docs/plans/2026-08-18-awr2944p-v01.md)。

新结果必须分别保存到：

```text
results/python/<run_id>/
results/matlab/<run_id>/
```

Python CLI 和 MATLAB adapter 都会拒绝已存在的 `run_id`。运行根目录包含
设计快照、环境、配置、摘要和验收记录；HDF5 位于 `data/`，图位于
`figures/`。需要进入版本控制的小型回归摘要先人工复核，再复制到
`baselines/<baseline_id>/`。

旧的 `simulation/output/v01` 含有 Python 图片和 MATLAB HDF5 的混合结果，
只作为历史证据保留，不再作为新的对比基准。
