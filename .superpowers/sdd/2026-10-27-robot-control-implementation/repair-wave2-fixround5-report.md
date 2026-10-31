# Wave 2 修复第 5 轮：急停持久受理的绝对时限

基线 `6da530f`。本轮只处理[第 4 轮复审](repair-wave2-fixround4-review.md)发现的安全锁等待未计入受理期限，以及修复该问题时暴露的清理取消交错；既有合约和背景见[第 4 轮报告](repair-wave2-fixround4-report.md)。生产改动限 `src/senior_control/agent/main.py`，回归限 `tests/integration/test_estop_recovery.py`。

收到当前连接、当前机器人与 epoch 的 estop（或停止请求溢出）后，同一个事件循环的绝对 deadline 覆盖获取安全锁和持久 `mark` 的等待。收到后立即阻止 Agent 运动资格、丢弃缓存速度并停止心跳续期；在截止之前未持久提交时进入安全故障，启动独立的 disable 和退出 78 路径，不向该请求发送成功响应。取得锁与取消同时发生时释放锁；已进入安全故障后，不接受迟到的写入结果作为新的队列受理。

旧重放清理持锁时，新收据的计数不会因为该收据被取消或超时而遗忘。旧清理的底层工作尚未结束时保持安全锁，避免迟到删除覆盖后续标记；如清理期间收到新意图，在清理确实结束后、有界补写，再释放锁。清理协程被取消以及补写期间再次取消均不应跳过这一步。正常关闭对清理工作作有界等待；仍在后台的清理不被当成已经完成。普通断线的处理不写持久安全标记。

0.5 秒是持久受理的等待窗口，不是物理制动时间或保证 disable IPC 在墙钟 0.5 秒内完成：等待首次写入期间，ROS 依靠既有 TTL/heartbeat 停车；超时后独立发起 disable。事件循环调度、IPC 和物理制动延迟仍需目标环境验证。首次标记提交前存储彻底失效时，不能声称该意图已持久受理，也不能保证掉电后记住它。若底层清理永久不结束且反复忽略取消，后台清理会继续持锁以防迟到删除；Agent 的等待有界，但 Python 异步运行器最终等待后台任务可能延迟进程退出，无法在此声称该极端故障下 78 退出时间有界。生产 velocity/reset 仍 unavailable；无新增 ROS hold、reset 或绕过配置。单 Agent SQLite owner 假设不变。

## 测试与自审

环境：macOS，Python 3.12.15。下述 Unix IPC 测试需要允许绑定本机 Unix socket；未启动开发服务。实际 Linux/Python 3.10、ROS 图和实车未由本轮验证。

- RED：在基线 `6da530f` 上，真实 IPC 复现旧清理持锁并尝试补写、新请求持续等待超过缩放预算；锁等待后实际 `mark` 被阻塞也超过总预算；独立持锁测试在 `safety=0.1` 时 0.15 秒内未观察到 disable；新收据取消后旧清理可删除已有标记。均先观察到针对预期行为的失败。普通沙箱最初因 Unix socket `PermissionError` 不能运行前两项，允许本地 socket 后才完成上述真实 IPC RED。
- RED：在本轮初版上进一步复现取消补写导致 marker 丢失；清理在取消后继续运行且清理工作再次被取消时，可在补写之后迟到删除；补写中再次取消清理工作也可丢失 marker。修复后对应回归通过。通过临时移除锁取消分支的释放语句确认取消与锁授予同时发生的测试会因锁泄漏失败，再恢复修复。
- 定向套件初次运行 `125 passed, 1 failed`：正常重放回归暴露初版等待清理与取消信号使用 `ALL_COMPLETED`，导致正常清理仍等待超时；改为 `FIRST_COMPLETED` 后复验通过。该失败与修复均属于本轮，不是历史基线结果。
- 最终代码：`.venv/bin/python -m pytest tests/integration/test_estop_recovery.py tests/integration/test_safety_intent_store.py tests/integration/test_shutdown.py tests/integration/test_agent.py tests/unit/test_arbiter.py tests/unit/test_arbiter_recovery.py -q -W error::ResourceWarning -W error::pytest.PytestUnhandledThreadExceptionWarning -W error::pytest.PytestUnraisableExceptionWarning` → `130 passed in 9.35s`。范围包括绝对锁期限、剩余写入预算、清理/补写取消和迟到工作、已提交 marker 及重启、重放恢复、64 槽积压、IPC 和普通断线。
- `.venv/bin/ruff check .` → `All checks passed!`；`.venv/bin/ruff format --check .` → `163 files already formatted`；`.venv/bin/mypy --strict src` → `Success: no issues found in 70 source files`；`git -c core.whitespace=cr-at-eol diff --check` → 无输出、退出 0。
- 本轮未重跑完整 unit/integration、colcon 或 ROS node 套件；由主控在 Linux 复验，不能将基线测试结果标作本轮结果。
