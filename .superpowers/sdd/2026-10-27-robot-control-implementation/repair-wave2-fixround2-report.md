# Wave 2 复审修复第 2 轮

基于 `e23196f`（文档链接提交 `b8249d3`）。本轮修复三项安全竞态、Agent 关闭边界及 CLI 日志配置；本报告随修复提交。生产远程速度和安全 reset 仍明确 unavailable，ROS 状态、导航、地图消费者、任务终态、源时钟 challenge 与 reset fence 均未实现或未验收。

- **缓冲急停：** 有界安全工作器保留 stop 与 estop 槽，优先执行 estop，普通过载及会话清理不取消已接收的安全工作。响应发送不阻塞后续安全 IPC。断连时独立 disable IPC 先撤销运动，同时有界等待安全工作；无法确认急停时进程报失败并使用 78 阻止服务自动重启。当前 epoch 的急停即使经过 heartbeat deadline 仍可锁存；新 epoch 生效后旧 epoch 不能锁存或撤销新会话。真实 Unix IPC/Arbiter 回归覆盖同批 stop→estop、stop IPC 阻塞时 estop 与断连，以及后台失败时独立 disable 不被阻塞。重复同类请求以有界方式合并，连接关闭时不保证每个请求都收到网络响应。
- **任务连接连续性：** 每个 claim 在同一数据库事务中绑定 Agent 本地 continuity generation；失去资格、断线或 deadline 到期同步推进 generation，并异步持久化该 generation 的活动任务为 cancel_pending。旧代次的迟到持久化回调不能标记新代次任务。调度下一次扫描也核对 generation，start 前后和写 running 前后均核对；迟到 start ACK 不得恢复任务，并尝试取消，未确认时保留 cancel_pending。进程重启时，之前 dispatching/running 改为 cancel_pending 而非可重派状态；后续 ROS cancel 合约必须能对同一 execution_id 幂等确认已停止或从未存在。测试覆盖断线重连跨扫描、阻塞 start、阻塞 cancel、旧回调和进程重启。真实 ROS 动作尚未接入。
- **后台故障：** supervisor 后台失败立即触发 Agent 受监督关停，撤销本地资格及 generation，停止发送 heartbeat；通过独立 disable IPC 禁用 ROS epoch。已在途 ACK 必须再次核对 generation、link 和资格才可能续租，失败或旧 ACK 不恢复资格。真实 IPC/Arbiter 的后台故障测试通过；仍需 Linux ROS 图和实机验证。
- **有界关闭：** Agent 对 inflight、失联持久化和各清理步骤使用有界等待；未结束协程保留引用并报告 runtime failure，而非声称强制终止。安全工作未确认时禁止服务自动重启。Python 无法强制终止吞取消的协程，`asyncio.run` 的最终清理仍可能受它影响；部署必须通过进程管理器的停止期限与最终强制终止提供外部上界，真实进程关闭故障注入仍待验证。
- **日志：** Agent CLI 在实际运行前显式配置 `senior_control` 命名空间 allowlist JSON formatter；重复配置不会叠加 handler，既有子 logger 的 level、disabled 和 propagate 状态被设为可向父 logger 传播，避免 Linux 环境中子 logger `propagate=False` 导致 INFO 消失。模块导入本身不配置日志。测试 fixture 显式保存、设置并恢复相关子 logger 状态；不通过降低断言来掩盖配置问题。Linux 主控已定位父 logger 和叶 logger 的实际状态，修复后的 Linux 全量结果仍待复跑。

测试已先复现缓冲急停、迟到启动、扫描间隙重连、后台故障、迟到 heartbeat ACK、日志传播和顽固清理等失败，再实现对应修复。完整严格命令 `.venv/bin/python -m pytest tests/unit tests/integration -q -W error::ResourceWarning -W error::pytest.PytestUnhandledThreadExceptionWarning -W error::pytest.PytestUnraisableExceptionWarning`：502 passed in 23.71s；`.venv/bin/ruff check .`、`.venv/bin/ruff format --check .`（160 文件）与 `.venv/bin/mypy src`（70 文件、strict）均通过。中途定向套件出现旧测试仍期望故障后同一连接可续租而失败，已改为验证同代次不能恢复并确保测试清理；重新验证通过。过长调度扫描窗口的 schedule 迭代仍无总跨度上界，已在首轮报告列为 OPEN；不得将整个调度系统描述为所有资源均有界。

异常重启时，原 dispatching/running 实例恢复为 `cancel_pending` 而不是直接视为可派发；后续导航取消 RPC 必须对同一 execution_id 提供幂等、可确认的取消或确定不存在应答。进程内不合作协程不能被 Python 强杀；部署层必须配置有限停止超时及最终进程终止，本轮只保证 Agent 协程的等待有界且报告失败。
