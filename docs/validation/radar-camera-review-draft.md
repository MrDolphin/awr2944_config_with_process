# Radar-Camera Web Fusion 评审摘要（现场验收前）

评审代码基线：`codex/radar-camera-web-fusion` 分支的 `1f0c512`；后续提交若修改运行代码，需要更新本页测试记录。本页只汇总已取得的离线证据，不代表树莓派或现场验收通过。

| 门禁 | 当前证据 | 结论 |
| --- | --- | --- |
| 本地单元与模拟测试 | Windows 工作树中运行 `.\.venv\Scripts\python.exe -m unittest discover -s test -p test_*.py`，135 项通过；最近一次聚焦会话校验测试 8 项通过 | 本地通过 |
| 静态差异检查 | `git diff --check origin/feat/codex...HEAD` 无空白错误；差异统计 62 个文件；搜索结果中 `/dev/video0` 仅出现在测试样本，旧 Pi 路径仅出现在历史文档及迁移说明 | 已检查，需在评审时关注分支原有差异 |
| 浏览器模拟 | 本地 Chrome/Playwright 使用拦截的相机响应验证图像显示、帧元数据及 404 状态；见提交 `c8d240a` 的测试记录 | 模拟通过 |
| 离线录制包校验 | 使用生成的测试会话验证索引、JPEG 引用、同步比例、偏移 p95 和十帧回放清单；损坏文本与无效门限有回归测试 | 本地通过，真实录制包未验证 |
| Pi 雷达单独部署 | 尚未上电、部署或启动 | 未运行 |
| Pi 相机单独运行 30 分钟 | 尚无实测帧率、温度及内存曲线 | 未运行 |
| Pi 雷达加相机静态运行 10 分钟 | 尚无真实匹配率、延迟、丢帧及断线恢复数据 | 未运行 |
| 真实 60 秒录制和十帧回放 | 尚无现场会话文件 | 未运行 |
| 静态空间标定 | 尚无真实标定板和独立验证网格 | 未运行 |
| 监督下的云台扇区扫描 | 尚无操作员、线缆与急停检查、实测姿态和动态误差 | 未运行 |

现场首先在操作员确认上电和安全条件后，从 `/home/pi/camera_web_fusion` 核对 `git rev-parse HEAD` 与 `git status --short`，执行 `systemctl cat radar.service` 记录旧服务命令，然后按 [部署检查清单](../../deploy/DEPLOYMENT_CHECKLIST.md)进行雷达单独验收。每道门分别复制并填写[现场记录模板](radar-camera-template.md)；真实会话在 PC 上运行 `python radar_camera_session_validation.py <session_dir>` 后人工核对十帧图像。未完成这些步骤前，不应将本地测试结果写为 Pi 验收结论。
