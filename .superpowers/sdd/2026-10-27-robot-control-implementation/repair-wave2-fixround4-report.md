# Wave 2 修复第 4 轮：急停受理与持久安全故障

基线 `4d2d5cb`；本轮代码及报告在本报告所在提交。范围是 Agent、Unix IPC 和 ROS 侧真实 Arbiter 的急停竞态及其测试，不扩展前端、SLAM 或未完成的 ROS consumer。

## 受理与恢复合约

- 收到当前认证 socket、当前 robot/epoch 的急停后，Agent 立即暂停本地运动资格、任务运动资格和向 ROS 的心跳续期，并停止转发已缓存的速度。随后以有界等待把单条安全意图写入本车 SQLite；**只有提交成功后才进入安全队列并可能给出成功响应**。此时才称为持久受理：连接中断、进程重启、未确认 IPC、队列积压或关闭期限到达均不会抹掉该意图。停止请求占满 64 槽并溢出时也先写入同一安全故障标记，然后关闭并禁止自动重启。
- Arbiter 只对已激活的当前 server epoch 检查急停：同 epoch 的 heartbeat 已过期、迟到 heartbeat 已禁用代次或 adapter 已 disable，都不会丢弃序号有效的急停；会锁存、清除运动。未激活的 epoch 0、未到达 ROS 的将来 epoch、已被新 epoch 取代的旧 epoch 和同 session 重放均不获得该授权。IPC 现在传递 Arbiter 的拒绝原因：明确的 `conflict` 才可作为重放/旧 epoch 处理；未激活或未知结果保留故障并关停。
- 每次急停受理都在同一异步锁内幂等写入持久标记，不以缓存判断数据库状态；已明确拒绝请求的清理也持有该锁。只有当前所有已受理急停均被明确判定为重放/已替代且无并发待受理急停，才可以清除；成功锁存、未知结果、超时、取消和溢出均保留。正常重放不会永久污染后续会话；如果中途崩溃，尚未确认的重放也会保守留下标记。
- Agent 每次启动在连接云端、授予 ROS heartbeat 或运动资格前读取标记；存在或读取失败时退出 78。成功急停也保留该标记，普通断线则不创建标记并仍可自动重连。当前没有清除此标记的生产 reset，也没有配置开关绕过；后续须先实现由操作者明确执行并验证最新 estop fence 的受控恢复，再同时定义 ROS 锁存和持久标记的解除顺序。正常关闭期限内处理不完时，保留标记并取消剩余 worker；若取消未及时完成，停止继续提交新工作并保留有界关闭失败语义。
- Agent 安全状态有独立版本号，升级现有 robot 数据库时保留原操作日志与 domain `user_version`；未知的未来 agent schema 被拒绝，组装入口返回 78，不会循环自动恢复。仓库 `deploy/senior-agent.service` 配置了 `RestartPreventExitStatus=78`。

## 明确边界

- 等待首次 SQLite 写入期间，Agent 资格立即撤销，但这不是 ROS 即时 disable：先前已送达 ROS 的命令依靠既有 TTL/heartbeat 期限停止。写入失败或达到默认 0.5 秒 deadline 后，通过独立 IPC 发起 disable、拒绝成功响应并退出 78。0.5 秒是软件等待界限，不是物理制动或零延迟保证；ROS 实际停车延迟须在目标环境测量。
- 如果存储在首次标记提交前不可写，无法声称该急停已持久受理，也无法凭不可用的存储保证掉电后仍记得它。退出 78 可禁止当前 service 的故障自动重启；机器重新启动、人工强制启动、数据库回滚或损坏并非持久标记已经提交的证明。此情形须现场确认及处理，不能把新 epoch 当成已完成恢复。
- 生产 `drive.velocity` 和 `safety.reset` 仍 unavailable；测试不是实车制动保证，也不代表真实 ROS 图、传感器、网络半开、浏览器、SLAM 或导航端到端验收。单 Agent 持有 robot 数据库和本机 0600 IPC 的部署假设不变。

## 验证

- RED：真实 IPC 将急停置于在途后使 heartbeat 过期，原实现返回 idle；阻塞 IPC 后重启 Agent 原可接入新 epoch。纯 Arbiter 另复现 epoch 0 未初始化误锁存，以及旧 epoch 与未来 epoch 拒绝原因未区分。均补回归并在实现后通过。
- RED：持久受理后取消在途急停，原修订会先清掉 marker；已改成只有明确 `conflict` 才清除。64 槽积压总关闭期限后 worker 原仍活跃并可能重开 IPC，补回归后改为保留持久故障并停止工作。并发回归中临时去掉接收锁可确定性复现“已锁存新急停但 marker 被旧重放清除”，恢复锁后通过。进一步复现删除已提交但清理协程被取消、另一个急停已到达的交错；清理中断且存在待受理请求时先有界补写标记，然后故障关闭，回归通过。
- `.venv/bin/python -m pytest tests/unit tests/integration -q -W error::ResourceWarning -W error::pytest.PytestUnhandledThreadExceptionWarning -W error::pytest.PytestUnraisableExceptionWarning`：524 passed in 23.13s；该次运行在最后补充未知 schema 退出 78、epoch 0 拒绝及清理取消交错修正之前。之前一次为 522 passed、1 failed，发现并修复上述关闭后 IPC 资源问题。
- 最终代码的定向严格复验：`.venv/bin/python -m pytest tests/integration/test_estop_recovery.py tests/integration/test_safety_intent_store.py tests/integration/test_shutdown.py tests/integration/test_agent.py tests/unit/test_arbiter.py tests/unit/test_arbiter_recovery.py -q -W error::ResourceWarning -W error::pytest.PytestUnhandledThreadExceptionWarning -W error::pytest.PytestUnraisableExceptionWarning`：121 passed in 8.26s。覆盖过期/disable、未知及新旧 epoch、跨 session、64 槽急停积压、纯 stop 溢出、阻塞 IPC、取消、存储等待和失败、标记清除竞态、原有安全合并及普通重连。此结果覆盖本次最终代码。
- `.venv/bin/ruff check .`：通过；`.venv/bin/ruff format --check .`：163 文件通过；`.venv/bin/mypy --strict src`：70 文件通过。`git -c core.whitespace=cr-at-eol diff --check`：通过，仓库部分既有 Python 文件使用 CRLF。

本机未执行 Linux/Python 3.10、ROS 图或实车测试；这些仍需后续独立复验。
