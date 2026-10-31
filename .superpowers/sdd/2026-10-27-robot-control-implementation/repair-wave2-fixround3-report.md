# Wave 2 修复第 3 轮报告

基线代码 `35ed464`。安全请求按类型、认证代次、session 分槽，同槽只合并较新的序号；不比较不同 session 或代次的序号。优先排空 estop，每个槽保留一个请求，上限 64。超过上限立即撤销运动资格、触发关闭，标记运行失败并禁用自动重启（仓库 service 的退出码 78 约束），已排队安全工作按既有有界关闭路径处理；最终硬退出仍依赖服务管理器。过载后不承诺每个急停锁存，安全处置依赖 ROS disable 成功或 ROS heartbeat 到期。

ROS adapter 明确返回 `accepted: false` 时视为已知拒绝，向请求方报告 `SAFETY_REJECTED`，保留后续安全请求；IPC 断开、异常或超时仍触发故障撤销和关闭。真实 Arbiter 不允许已禁用代次的急停复活，新代次必须重新认证。生产速度和 reset 仍 unavailable，不代表 ROS 数据、导航或浏览器链路已验收。

Linux 测试失败源于等待 `connected` 这一无法保证可观察到的瞬态；旧测试的 `start` 抛错时 runner 未进入 finally。现用显式创建 runner、观察失败连接关闭和新 epoch 恢复，finally 始终执行关闭。该测试假设和清理修正不是生产 half-open 故障修复的声明。

## 验证

先用真实 IPC 和 Arbiter 运行同 session acquire seq 5 后 estop seq 4、6 的回归：修改前 `mode == idle`，断言失败；修改后同时覆盖跨 session、首个 IPC 在途延迟并明确拒绝、队列填满和溢出故障，共 8 项通过。现有相邻断连、代次切换及被阻塞 stop 的 IPC 测试继续通过。

- `.venv/bin/pytest -q -W error tests/integration/test_shutdown.py tests/integration/test_agent.py`：78 passed（最终新增参数化覆盖前运行）。
- `.venv/bin/python -m pytest tests/unit tests/integration -q -W error::ResourceWarning -W error::pytest.PytestUnhandledThreadExceptionWarning -W error::pytest.PytestUnraisableExceptionWarning`：510 passed in 22.30s（本机）。
- `.venv/bin/ruff check .`：通过；`.venv/bin/ruff format --check .`：160 文件通过；`.venv/bin/mypy --strict src`：70 文件通过。
- 尝试无路径的 `pytest -q -W error` 时，仓库 e2e 与 unit 同名测试模块发生 pytest 收集冲突（2 errors，1 skipped）；上面的既定 unit/integration 质量门使用明确路径并已通过。

本机未执行 Linux、真实 ROS 图或浏览器端到端验收；Linux 由主控复验。超长 schedule 窗口的枚举上界仍 OPEN。
