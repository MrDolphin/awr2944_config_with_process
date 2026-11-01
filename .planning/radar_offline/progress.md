# 离线开发执行日志

## 2026-11-01：建立续跑计划

- 核对目标分支、提交及未跟踪原始数据；未触碰采集文件。
- 建立 R01–R06 顺序任务和计划表内的执行概要栏。
- 尚未执行 R01–R06 的开发或测试；下一轮从 R01 开始，并以实时状态重新核对。

## 2026-11-01：调度修复

- 首次触发时 `automation-8` 从指定模型的 `cron` 变为无模型字段的 `heartbeat`，已暂停，未执行 R01。
- 建立 `automation-9` 为 `projectless` 本地 cron，配置为每 2 小时运行且指定 `chatgpt-web/gpt-6-sol`；运行时必须切换到明确的雷达融合工作树，不得在默认目录开发。
- 在计划表增加模型和额度双重门槛；创建后配置核对通过，首次触发后的状态仍待验证。

## 2026-11-01：调整为每小时

- `automation-9` 的本地配置不存在，应用更新接口返回该任务不存在；未假定它仍在运行。
- 新建并启用 `automation-3`，配置为每小时一次的 `cron`，模型 `chatgpt-web/gpt-6-sol`；更新计划表的自检 ID。
- 这是调度配置修改，不是 R01–R06 的开发验收；首次触发后的保留情况仍待核对。

## 2026-11-01：R01 测试先行

- 核对额度：5 小时窗口剩余 71%，周窗口剩余 32%；`automation-3` ACTIVE、每小时、GPT-6 Sol。融合分支 HEAD `97e40dc`，未跟踪原始采集未触碰；未见其他活跃聊天改同一功能。
- 先建 `test/test_offline_range_axis.py`，覆盖正常、缺失、冲突、TLV2 长度不一致、无效/哈希不符、非实数 ADC。
- 首次误用 `python -m unittest test.test_offline_range_axis`，因 `test/` 非包而报模块不存在；改用 `python -m unittest discover -s test -p test_offline_range_axis.py -v`。此时预期失败：`ModuleNotFoundError: tools.offline_range_axis`，证明先写测试，未重复原错误命令。
- 实现纯模块 `tools/offline_range_axis.py` 后，7 个聚焦测试通过。再添加极值输入回归，先得到 1 项失败（非有限米数未拒绝），补有限值检查后 8 项全部通过；`git diff --check` 通过。
- 只读核对 1、1.5、2 m 三组原始采集的 `metadata.json` 与首帧 TLV2：均为同一实际 CFG SHA-256、512 格，理论格宽约 0.027915 m，名义奈奎斯特无歧义距离约 14.293 m，点云输出范围 0–3.5 m。未据此断言 10/200 m 探测。
- 全套 `python -m unittest discover -s test -v`：314 项，20 fail、4 error、2 skip；输出明确有本机 Python 缺少 `opencv-python` 的校准测试失败，另见网页 Playwright 初始化失败。未逐项归因全部 24 项，不能报告全绿。`pip show` 确认 NumPy 已安装但 OpenCV/pytest 未安装；本轮以聚焦相关测试验收纯模块，并如实报告限制。
- 双轴只读审查：规范轴无明确 AGENTS.md 违规，指出计划状态需同步；需求轴发现未核对活动 `chirpCfg` 与畸形 `cfarFovCfg` 可误给米轴。先加 2 类失败测试，再补活动 chirp 覆盖/冲突及 FOV 命令校验；另加重复 chirp 冲突的失败测试并修复。
- 最终 `python -m unittest discover -s test -p test_offline_range_axis.py -q` 为 11/11；`test_render*.py` 5/5；`test_register_scan_capture.py` 5/5。新模块及测试 `py_compile` 和 `git diff --check` 通过。三组采集的实际首帧 TLV2 均为 512 格，均返回理论米轴；未编辑采集。
- R01 已完成，R02 尚未开始；全套测试环境失败仍是后续完整验收的限制，不能报告全绿。
