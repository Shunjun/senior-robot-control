# Wave 2 复审修复第 1 轮

基线 `5e1db06`，对应 `repair-wave2-review.md` 的四项发现；本报告随修复提交。范围仍不包括 ROS 消费端、真实导航、源时钟 challenge 和急停 reset fence。

1. **心跳截止时间。** Agent ledger 保存经 ROS 确认的原始绝对单调时钟 deadline（不超过本地配置窗口），过期不再具备运动或任务资格。连接变更、心跳失败、关闭会推进本地 generation，等待中的旧 ROS ACK 不得重授权；单调时钟回退拒绝。新增 deadline、过期 ACK 及失败与 ACK 竞态测试。
2. **任务故障与取消。** 导航 start 的异常先持久化 `cancel_pending` 并尝试取消；确认取消后记 `needs_review`，未知取消保留 `cancel_pending`。正在运行的任务需要取消时先持久化同样状态，只有取消获确认才标为 paused；超时/失败继续重试，即使连接恢复也不能丢取消义务。`cancel_pending` 跨重启保存并阻止新任务 claim，确认后才允许后续触发。仅捕获导航端封装的可恢复失败；数据库错误及后台自身异常仍向 supervisor 传播并撤销资格。新增 start 失败后后续 occurrence、取消失败后连接恢复、未知 start/取消、重启保持与阻塞等测试。
3. **请求上限。** 普通 inflight 上限 64；额外保留一个 stop/estop 槽位，超过容量直接中止当前会话，由 session 清理取消工作并禁用 epoch，不为每条拒绝新建任务。心跳处理独立于这些任务；若保留的安全槽位也满，关闭会话并禁用运动。测试模拟 500 次超额请求并验证槽位上界。
4. **执行记录查询。** 周期扫描由数据库筛选所有 active（含 cancel_pending）及当前 `_scan` 窗口的去重记录，分成 active 和窗口两个查询，分别强制使用状态索引和发生时间索引；旧库每次 prepare 时补建索引，EXPLAIN QUERY PLAN 测试验证均为索引 SEARCH；最多返回 4096 条，超限明确报错、后台 fail closed，不静默丢失去重记录。SQLite 原有 occurrence 唯一约束保持。3000 条旧历史只返回 active 与窗口中的记录，4097 条窗口记录触发上限测试。历史全量读取 API 仍存在但不由周期扫描调用。

5. **Supervisor 关闭。** 主控在 Linux/Python 3.10 观察到 heartbeat 的 `wait_for` 在完成边界吞掉外部取消后继续长时间 sleep（CPython issue 86296）。heartbeat、drain、uplink 均检查关闭标志，uplink 在 connect 返回和 retry sleep 前也检查；shutdown 有单调的有界等待，超时保留尚未结束任务并报错，运动资格立即撤销。确定性交错测试模拟内部 heartbeat 已完成、外层 cancel 被吞；另一测试验证忽略取消的后台工作在截止时间内明确报错。超时并不声称 Python 能强制终止拒绝取消的协程；调用方必须处理失败。Linux Py3.10 全套复验仍待主控执行。

TDD 已先观察四项行为 RED，再实现并通过对应回归。`ruff check .` 和 `ruff format --check .` 均通过（160 文件），`mypy src` strict：70 文件通过；全依赖普通 `pytest tests/unit tests/integration -q`：485 passed；同范围附加 `-W error::ResourceWarning -W error::pytest.PytestUnhandledThreadExceptionWarning -W error::pytest.PytestUnraisableExceptionWarning`：485 passed。Linux Humble、真实 ROS IPC 消费者、网络和实车仍未验证。特别地，长时间停顿造成的超大时间扫描窗口仍可能产生大量 schedule 计算；当前数据库读取有 4096 条上界，未声明整个调度计算在任意窗口长度下均有界，后续应为扫描时间跨度设计显式上限和跳过记录语义。
