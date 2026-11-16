# 第11项：运行故障隔离盘点与最小实施、测试提案

日期：2026-11-16。状态：只读设计，未实施、未运行套件、未启动服务/ROS/车辆，未提交推送。

基线：`fix/degraded-drive-speed`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da` **加当前工作树全部既有修改**；交付目标仍为原 Draft PR26。本文件是本任务唯一新增文件。三保护文件 `ros2/src/senior_robot/senior_robot/native_receipt.py`、`tests/test_native_receipt_diagnostics.py`、`docs/reviews/2026-11-16-software-estop-owner.md` 只读，未修改或暂存。

依据：上级 `.orca-dispatch/2026-11-16-startup-constraints/runtime-fault-isolation.md`、`车辆启动约束审查与处理方案.md` 第11项，以及当前 `docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md`。用户要求主Agent除自身真正致命故障或主动stop外持续运行，局部故障如实上报；运动失效仍必须停止，不能靠断云代替停车。下述“可复现”指静态调用链和现有测试刺激可推导，**不是本轮已运行的故障注入证据**。

## 1. 真实调用链与退出入口

### 1.1 实车与Docker/native监管

`docker_start_helper.main → supervise`（`src/senior_control/docker_start_helper.py:310`）：

1. 获取 `.agent-service.lock`，创建本run学习环境与唯一 `LearningService`；准备 `service.sock`、`agent.sock`，注册信号。
2. `_motion_command → ros2 launch senior_robot safety_motion.launch.py`，独立进程组；另起一个 `agent_adapter/CommandNode` 保留Unix peer PID身份。
3. **先** `_wait_for_adapter`：最多35秒、identity/robot/domain/peer PID+UID/socket inode验证。adapter退出、超时、身份不符均抛异常。**之后**才创建主Agent子进程。
4. 通过既有启动成功条件后一次清理旧学习文件；独立camera launch最后启动，camera启动OSError或退出只报告并收掉camera组，已经局部隔离。
5. 等待stop、Agent、motion launch、adapter；除camera外任一完成即离开循环。motion或adapter退出返回1，Agent退出传播其退出码。
6. 任一路径进入 `finally`，依次终止Agent、adapter、motion组、camera组，再取消waiters、按inode删除本方socket、关闭service IPC及学习服务、移除信号。`_terminate_motion_stack` 还会清理launcher已死后的本方组内幸存进程，不能删除这项资源保护。

所以“运动退出→Agent终止”无需Agent内部出错即可发生；“adapter无法启动→没有Agent云端上报”是启动顺序直接造成的。`profile=None` 仍支持任意单子进程监管，是现有兼容入口，不能把该入口的正常child退出传播误改成永不结束。

### 1.2 launch范围

`safety_motion.launch.py:_nodes` 在include之前注册无 `target_action` 的 `OnProcessExit(on_exit=stop_required)`，首个被该全局处理器接收的退出会发 `Shutdown`，且不区分退出码。

可见范围包括两个WHEELTEC include内的进程，以及显式 `scan_receipt_node`、`scan_adapter`、`safety_scan_ingress`、`scan_motion_node`、`safety_node`；配置navigation时也include其进程。不是只盯一个“必要驱动”，正常结束的一次性辅助子进程也没有排除。外部WHEELTEC launch源码不在本仓库，不能臆称其内部没有自己的shutdown策略；实施时需在目标安装环境核验，不能以本文件静态结论替代。

仿真 `simulation_motion.launch.py` 未见同类全局退出处理器；其include模拟器、共享safety节点并在同一launch内启动 `agent_adapter`，最终仍受外层 `ros.wait()` 连锁影响。launch只看见子进程，**看不见safety Python进程内部native输出子进程的直接退出**；后者应继续由现有控制owner监测和投影，不能假装全局退出handler已经覆盖。

### 1.3 仿真supervise

`simulation_cli.main → supervise`（`src/senior_control/simulation_cli.py:246`）：学习服务 → ROS仿真launch → `_wait_for_adapter`（验证robot identity）→ Agent → 成功后学习清理 → READY通知。`FIRST_COMPLETED(stop.wait, ros.wait, agent.wait)` 后非主动stop返回1；`finally` 先 `_stop(agent,12)` 再 `_stop(ros,10)`，清video临时目录、信号和学习服务。因此仿真ROS启动/readiness异常也会发生无Agent报告或带走Agent。其 `_stop` 目前处理直接进程，实车有显式进程组幸存者清理；仿真整组回收是否完整须单独验证，不在提案中宣称已保证无孤儿。

### 1.4 Agent初始化与运行

`agent.main.main → AgentConfig → composition.run_agent`：

- 配置/凭证错误在main返回78；随后打开主数据库、准备tasks/waypoints/journal。journal初始化失败返回78。
- `run_agent:336` 在日志store、Agent、WebSocketConnector运行前调用 `motion.verify_robot`。不可达的IpcError返回1，其他identity校验错误返回78；所以仅把外层顺序改成“Agent先起”仍然无法在线。
- 校验后才创建state/map/navigation/workspace等独立IPC clients、任务runtime/settings和日志服务，最后创建Agent并安装信号。
- `Agent.run → Supervisor`：心跳、云连接uplink、`tasks`及可选`logs`后台各有task；`Agent.run` 等 `_stop`。`Supervisor._watch` 记录后台正常提前结束或异常 → `_fault` 全局revoke → `Agent._supervisor_fault` 置runtime_failed并 `request_shutdown`。
- `task_loop` 的 `state.read` DomainError已降为不可用context；但 `runtime.tick` 异常向外传播，最终通过上述watch链下线Agent。`TaskRuntime.tick` 有已存在的NavigationFailure处理、持久取消义务与skip补写，不能在外面catch-all后无条件继续tick，把持久化失败变成成功或重复派发。
- `_control` 对stop/estop交付异常调用 `_supervisor_fault`；`_drain_safety` 对当前代次estop非CONFLICT DomainError、`_safety_finished` 对取消/异常、`_settle_safety` 对失败/超时均调用 `_safety_fault`。后者同时置 **auth_rejected** 和runtime_failed再shutdown，错误地把IPC交付未确认表达成认证失败。
- 安全容量耗尽也触发上述shutdown；另一容量分支直接置auth_rejected/runtime_failed并shutdown。`_drain_intent` 的IPC异常则关闭当前云link，造成一次云重连，即使云本身仍正常。
- `_session` 遇AuthRejected主动shutdown；云ConnectionError/OSError走既有uplink重连。session finally禁用旧epoch、丢意图、结算安全交付并等待inflight；`_wait_inflight` 超时再次shutdown。这可能把局部取消不响应升级成整个主Agent退出。
- `request_shutdown` 立即撤销准入、设置stop；`run` finally禁用运动、取消Supervisor后台并 `_release` 关link/clients/cleanup。`run_agent` 将auth_rejected **或** runtime_failed返回78；故多种普通故障还会阻止外部服务重启。独立部署 `deploy/senior-agent.service` 为 `Restart=on-failure` / `RestartPreventExitStatus=78`，不能用新重启机制掩盖该问题。

### 1.5 日志和已有正确隔离

`composition` 的 `AgentLogHandler` 只采集 `senior_control` Python日志；文件明确说明未接 `/rosout`。父supervise的 `print(stderr)`、ROS launch/驱动输出不会自动进入此handler。故“主Agent还在线”也不等于“ROS退出原因已能从云日志查到”。

日志store打开的OSError/ValueError已局部降级；`AgentLogConnection.run` 自带独立认证日志WebSocket及既有重连，普通连接异常不会结束控制连接，日志写入/跟随已有有界队列、gap与容量保护。`logs` task若意外返回/未捕获异常仍会被通用watch升级为Agent退出。不要复制日志连接或再造日志存储owner。

`SnapshotState.collect` 已将ROS state读取DomainError投影为 `ipc_unavailable`、禁用manual/tasks/runtime能力、保留最后采样标记；这应直接复用。`Agent._keep_snapshot_fresh` 已对采集失败只跳过一轮；但tasks/home数据库读取在ROS try之外，数据库异常时会一直无新快照，不能宣称此时云状态仍完整健康。

`IpcClient.request/send` 失败丢连接且不重发，后续请求才重新建连；不需要新增重试器。`agent_adapter` 的真实异常可传播退出，finally关闭server并release；CommandNode来源退役、身份代次、EOF处理保持。native `motion_output_guard.cpp:poll` 在私有输出EOF/非法包时先stop再shutdown，自身健康时到deadline停止；它若自身被杀，物理停车依赖下游驱动/车辆看门狗，不能把“有代码”写成已验证整车停车。规范也明确下游物理行为另行验证。

## 2. 与用户需求逐项对应的真实问题

| 编号 | 现状/复现刺激 | 违反需求的可见结果 |
| --- | --- | --- |
| B1 | 实车launch内任一被全局handler监听的进程退出（包括0码）；handler直接Shutdown | 放大局部故障，连带健康观测进程退出，再由外层杀Agent |
| B2 | 实车motion或adapter wait完成；仿真ros wait完成 | 即便Agent正常仍进入finally终止Agent，云和Python日志同时结束 |
| B3 | ROS创建OSError、adapter超时/退出/身份不符 | 两个supervise尚未创建Agent；直接运行Agent也在verify_robot提前返回，无法报告启动故障 |
| B4 | tasks后台抛异常或logs意外结束 | 通用watch全局revoke并shutdown，模块故障被当作主进程致命故障 |
| B5 | stop/estop IPC异常/超时、安全队列溢出、交付task失败 | Agent退出且可能标认证失败；“未确认停车”被扩大成失去全部观测 |
| B6 | 手动意图发送IPC失败；旧session取消不响应 | 前者主动断健康云链；后者超时关闭Agent，而不是隔离失效执行与保留报告 |
| B7 | 父监管检测退出但只print；ROS进程内部退出日志 | 当前云日志源收不到这类事实，无法证明故障原因上报 |

这些是调用链上的确定行为，不是要求每个传感器失效都立即完全停车。方向感知、轮速退化与真实执行故障的判断继续归现有运动owner；第11项不另写健康阈值或把辅助camera故障当作驾驶故障。

## 3. 最小修改边界与职责提案（未实施）

默认方案只删除连坐、保留真实失败、阻断受影响功能、接通故障事实，不增自动重启/恢复服务，不重放运动。

### 3.1 launch和两种supervise

- 删除实车无target的“任何退出→Shutdown”处理器及退役注释。退出可观察且记录name/PID/returncode；不新增“满足若干条件才放过辅助进程”的复杂判断，也不逐一把所有节点都列为required。节点本身无法继续运行仍真实退出。
- 两个supervise在创建ROS/adapter之前创建Agent报告面；Agent创建失败才进入整体最终回收。ROS/adapter创建、readiness失败进入“局部功能不可用”，保留Agent、service控制入口及既有学习owner；不重复spawn，不盲目重试。身份不匹配仍拒绝该IPC，绝不接管/删除外来socket。
- 待等待集合中local child退出时读取一次退出码、记录事件、移除已完成waiter并清理其本方残留资源，继续等Agent与主动stop。不能保留done waiter造成忙循环。运动组失效可回收该失效组的幸存进程；不顺带结束健康Agent/camera/学习服务。正常code0的意外提前退出也如实报告“已结束”，不能当健康运行。
- 整体finally只由主动stop、Agent自身结束/无法启动或监管自身真正不可继续执行触发。保留当前有限期限终止、锁、socket inode/peer保护和资源关闭次序；`profile=None` 单child兼容语义不变，若实施产生兼容代码按仓库约定标 `//Compatible`。
- READY/service status拆清“Agent报告面已启动”和“ROS可用”；不把Agent子进程存活宣称已连云，也不因ROS失败拒绝报告面readiness。学习旧文件清理仍只在**现有完整启动成功条件**下触发一次，不能因Agent提前创建就提前清理；run、claim、唯一owner与完成结果恢复握手保持现实现。

### 3.2 Agent：取消启动依赖，缩小故障影响

- 从 `run_agent` 的全局退出前置条件移除ROS identity成功；保留identity校验为**运动/ROS业务准入**。身份未验证或不匹配时，云连接、日志和不可用状态可运行，控制调用明确UNAVAILABLE，不能因为“以后可能恢复”直接放行。后续已有IPC请求/状态轮询可重新验证当前真实身份，再恢复数据可用性，不新增后台重试器，不把旧验证跨连接沿用。
- 让watch回传**模块名和结束/异常事实**；Supervisor继续拥有云心跳和既有uplink重连，模块故障不再统一调用request_shutdown或全局revoke。logs故障只撤销日志能力、保留stderr诊断；tasks循环失败停止该模块并撤销/收束其任务执行权限，保留未知/待取消持久事实，不自动重启循环、不把正在执行任务写为完成。
- 任务执行失去owner时，必须通过现有任务撤销/暂停、来源屏障收束运动；IPC不可达时报告“收束未确认”，由车端失效停止机制兜底。不能只停调度却让已派发故障任务无owner继续运动。若现有任务owner无法表达该局部故障，应补最小故障入口，不把生命周期迁入Supervisor。
- stop/estop未确认、容量失败和安全drain失败走本地执行不可用/交付未确认；撤销相关manual准入、丢弃未发意图，保持已接受安全事件的既有优先级和有界结算。不伪造estop锁存，不新增Agent急停owner，不用auth_rejected表示本地失败。仍只在唯一车端owner确认后回成功；不新增重发/恢复后自动补发运动。
- `_drain_intent` IPC失败不主动关闭健康云link；记录故障并撤销该manual来源，后续旧authority不续命。`_wait_inflight` 的旧代次工作超时记录并隔离straggler，保留现有deadline和结果代次屏障；不能通过解除上限让工作无限堆积。此处不增加重试或任务重放。
- 不用“忽略全部Exception”维持在线：业务端将已知IPC错误变为DomainError；后台未预期异常记录堆栈并结束失效模块；顶层真正无法继续执行的异常仍可传播。`runtime_failed` 不再同时代表模块故障与全进程退出码，需明确只给真正退出失败使用或在退出码处拆开，防止普通故障经78再次锁死部署。

### 3.3 故障上报与日志：只补生命周期缺口

推荐只增加一个**父监管→Agent的单向、有界生命周期事件通道**（继承pipe，非新常驻服务），两个supervise共用同一小接口。父方是本方子进程“spawn失败/readiness失败/真实退出”的唯一事实生产者，Agent只接收并投影，不成为第二进程监管owner。事件限模块、阶段、PID、退出码/错误摘要、run标识；不含凭证，不携运动指令。读写不阻塞事件循环，有限帧/缓冲、溢出报告缺失，EOF只标监管观测不可用，不触发Agent退出。保留stderr兜底。

launch内部退出由其观察回调记录固定结构的退出事实，经父方捕获转入同一通道；只观察，不发Shutdown。实现不得把任意stdout文本解析成可信控制状态，原ROS日志仍保留在ROS_LOG_DIR/journal。此范围只保证生命周期故障摘要能进入现有AgentLogHandler/VehicleLogService；不扩成全量ROS日志采集系统，不能声称已提供完整rosout历史。pipe故障不影响运动owner或云link，也不自动恢复服务。

Agent把事件写入现有日志服务，并在既有快照轮询中投影模块不可用原因；IPC失败继续复用 `RuntimeObservation(status="ipc_unavailable")`。若当前协议容纳不了父监管模块事实，只做必要的模块故障字段扩展，不造第二套健康评分。最后成功采样只能带原时刻展示，不能伪造新采样或健康、清除未知急停状态；ROS状态恢复可显示新事实，但不自动发放manual authority或继续任务。

这条通道是为补B7的具体建议，未实现或压测；没有它，仅“Agent提前启动+删除shutdown”还不能满足父监管故障云日志可见性。独立直接运行Agent时不依赖父通道存在，ROS业务仍按已知不可用报告。

## 4. 真正fatal：现有证据与不能擅自新增的政策

| 条件 | 现有证据 | 建议边界 |
| --- | --- | --- |
| 用户主动stop / SIGTERM / SIGINT | service IPC stop设置event，Agent信号调用request_shutdown | 正常整体退出，仍完成有界运动撤销/回收，不是故障分类 |
| 主Agent子进程真正死亡、解释器/事件循环无法继续 | 外层 `process.wait/agent.wait`；未处理顶层异常传播 | 本报告面已不存在，外层回收本方执行资源并保留stderr/退出码，不自动再建Agent |
| Agent无法创建 | `create_subprocess_exec` OSError，无进程可报告 | 外层stderr记录，回收已建资源；不能承诺不存在的云连接 |
| 缺有效设备身份/凭证或非法云URL | AgentConfig/main返回78 | 无法建立合法报告身份，允许启动失败；与ROS identity mismatch严格区分 |
| 主数据库/operation journal初始化失败 | Database进入、tasks/waypoints.prepare可抛；journal.prepare失败返回78 | 目前全Agent共用该存储，初始化失败确实阻止现有composition构建；保留为现有启动失败边界，不伪造可写/成功。是否另建无数据库报告模式属后续政策，不在此重构 |
| 运行期某任务SQL异常、日志磁盘满、IPC断开 | task tick可抛；日志store已有局部处理；IPC错误已有DomainError | 无证据证明解释器/云报告面不能继续，不能称fatal；相应模块停用并报告 |
| 安全请求不确认、容量耗尽、取消不响应 | 当前_safety_fault/ _wait_inflight直接shutdown | 当前代码有退出行为，**没有证明主进程已无法报告**，应移除这种fatal等同关系 |
| AuthRejected | Agent._session主动shutdown，Supervisor有auth_failed并停止该uplink循环 | 保留不拿无效凭证无限重试，但这不是本地故障，也不证明主进程必死。建议留本地在线待处理、明确云认证不可用；凭证更新后的重连政策尚未授权，不新增轮询/重配服务，提交负责人裁定 |
| 监管自身不可继续运行、进程级资源耗尽/未处理BaseException | 最外finally仍会回收；未见专门OOM/fatal分类器 | 不新增“每个异常都fatal”判断器；主循环确实不能运行时允许传播/退出，尽可能记录。不要声称SIGKILL/OOM必能写最后一条日志 |

另有需要实现前核对的控制契约：任务后台永久失效时现有owner如何完成暂停/撤销；straggler超时后如何阻断其迟到副作用；第三方驱动/native输出同时死亡时下游物理看门狗能否停车。前两项应优先复用既有来源屏障和任务持久未知状态，若确需新控制政策要单列负责人决定，不能在“保Agent在线”名义下放行运动或新增自动恢复。第三项需要目标环境验收，不以本地mock代替。

第10项新心跳仅负责连接在线性。不得恢复ROS heartbeat、RTT/长断联运动资格、五次heartbeat恢复等旧链；学习记录一写多读、同run恢复、原始时间proof和来源代次全部沿用当前工作树，第11项不混入毫秒迁移或总体架构重构。

## 5. 文字测试清单（本轮不创建测试、不运行）

以下均先用可控fake child/events/IPC端口确定因果，不按任意sleep证明“还活着”。断言Agent任务未结束、同一云link/epoch仍处理heartbeat或新只读请求，并能查询相应故障日志/快照；“未退出”单独不算验收。这里的stop须区分服务主动stop与普通车辆停车命令。

| 用例 | 输入/刺激 | 预期 | 需求或实际bug依据 | 可复用测试位置 |
| --- | --- | --- | --- | --- |
| T1 launch辅助退出 | scan_motion/辅助进程分别0和非0退出；另测include子进程 | 只产生退出事实，不发全局Shutdown；健康节点保留，来源失效按现有owner处理 | B1，无target handler且无退出码区分 | `tests/unit/test_motion_launch_profile.py`（新增定向launch结构/回调测试）；目标ROS行为另验 |
| T2 关键运动进程退出 | 实车motion launch或adapter各自退出；仿真ros退出 | 事件含退出码，Agent/云/日志保持；失效本方组回收；不自动spawn | B2 | `tests/unit/test_docker_start_helper.py` fake Child；`tests/integration/test_vehicle_cli.py` supervise fakes |
| T3 ROS创建失败 | motion/ros subprocess创建抛OSError | 先存在Agent报告面，故障快照与日志可查；服务仍能stop | B3 | 同T2；保留Agent spawn失败真实退出负例 |
| T4 adapter启动异常 | readiness超时、启动后退出、identity/domain/peer不符各一项 | Agent在线、ROS控制UNAVAILABLE，无外来socket删除/接管，不放行新运动 | B3，现有identity保护不可削弱 | docker helper既有 `test_identity_or_domain_mismatch_prevents_agent_readiness`、`test_matching_robot_id_does_not_establish_process_ownership`；vehicle_cli matching adapter测试 |
| T5 直接运行Agent无adapter | verify_robot遇ENOENT/EOF/timeout及错误identity | 云/logs先可运行，状态ipc_unavailable；identity不合法仍禁止调用 | run_agent当前返回1/78 | `tests/unit/test_agent_config.py`、`tests/integration/test_agent_snapshots.py`（新增composition级fixture） |
| T6 运行中状态IPC断开再恢复 | state.get EOF/超时，后续返回合法新代次/新采样 | 同一云连接继续；不可用期间无旧健康续命；现有后续请求重连，恢复仅新状态，无自动运动 | 已有SnapshotState降级与IpcClient drop可复用 | `tests/integration/test_agent_snapshots.py`、`tests/integration/test_task_runner.py`、`tests/integration/test_agent.py:test_failed_collect_does_not_end_the_keepalive` |
| T7 手动IPC失败 | intent send失败，随后迟到旧authority/旧seq再到达 | 保云link；manual退役且旧动作不复活，无自动申请authority，无重发样本 | B6及当前manual协议 | `tests/integration/test_manual_wire.py`、`tests/integration/test_agent.py`、`tests/unit/test_heartbeat_wire.py`（仅在线性回归） |
| T8 stop/estop未确认 | 交付EOF/timeout/明确拒绝；急停drain取消/失败 | 无成功回执、明确unconfirmed；Agent在线且非auth_rejected，受影响运动关闭；已接受事件不被普通stop吞掉 | B5；唯一急停owner规范 | `tests/integration/test_shutdown.py:test_real_transport_failure_reports_unconfirmed_without_success_reply`、`test_blocked_estop_transport_has_bounded_release_and_explicit_unconfirmed_log`、`test_disconnect_keeps_buffered_estop_while_stop_ipc_is_blocked` |
| T9 安全容量耗尽 | 填满安全pending后再送新请求 | 明确容量错误，已有accepted事件仍有界结算；不误标认证失败/不杀Agent、不无界积压 | B5 | `test_shutdown.py:test_capacity_fault_then_immediate_release_delivers_every_accepted_estop`、`test_agent.py:test_overload_bounds_work_and_reserves_stop_capacity` |
| T10 tasks后台异常 | runtime.tick抛数据库/未预期异常；另测正常提前返回 | tasks故障可见，停止新派发并收束原执行；云/logs继续，未知取消不标成功，无自动重启或重放 | B4；tick持久义务 | `test_runtime.py:test_background_failure_revokes_motion`、`test_shutdown.py`后台故障测试、`test_task_runner.py:test_runtime_propagates_database_failure`（保留底层传播） |
| T11 已有业务局部失败 | 导航start拒绝/取消未知、普通操作DomainError | 维持现有业务错误与任务持久结果，Agent在线；不把普通失败升级global fault | 用户需求；已有任务恢复语义 | `test_task_runner.py:test_runtime_recovers_after_navigation_start_failure`、`test_failed_start_with_unknown_cancel_remains_pending_until_confirmed`、`test_agent.py:test_failed_commit_is_never_reported_as_success` |
| T12 logs独立失败 | 日志WS断开、store磁盘满、logs task意外结束分别刺激 | 控制云保持；日志能力不可用/gap真实可见，stderr可诊断；不自动重启意外结束模块 | B4/B7；既有独立日志链 | `tests/integration/test_vehicle_logs.py:test_real_sqlite_full_records_loss_and_recovers`、`tests/integration/test_settings_logs_wire.py`、`test_runtime.py` |
| T13 生命周期日志闭环 | 父方ROS spawn/readiness/exit事件、launch子退出；在adapter前注入 | 事件进入现有云日志query/follow，带模块阶段退出码；快照无健康伪装；无token泄露 | B7与启动前上报需求 | docker helper/vehicle_cli fakes接 `test_settings_logs_wire.py`、`test_vehicle_logs.py`日志fixture |
| T14 事件通道故障/洪峰 | 帧过大/非法、buffer满、EOF，Agent正常运行 | 有界丢弃及显式缺失/观测不可用，心跳/stop不阻塞，不关闭Agent、不触发运动；只接本方run | 最小通道必要边界，防新增连坐 | 新增同上述supervise与Agent集成测试；复用 `tests/unit/test_runtime_diagnostics.py` 有界诊断方法 |
| T15 云断线与恢复 | 仅cloud link断开，ROS保持；重连后送旧epoch | 仅既有uplink重连，拒旧manual；已接受任务按当前规范自主运行，不引入heartbeat运动资格 | 心跳10与任务规范 | `test_agent.py:test_dropped_link_disables_its_epoch`、`test_task_runner.py:test_running_task_survives_connection_generation_changes_between_ticks`、`tests/integration/heartbeat_test_support.py` |
| T16 straggler | session退役后业务task吞cancel，最后尝试旧epoch副作用 | 有界隔离、资源不增长；Agent可观测；旧任务不能写出新成功或发运动 | B6；不能把移除shutdown变成旧工作复活 | `test_agent.py:test_disconnect_cancels_inflight_old_epoch_operations`、`test_agent_release_is_bounded_for_cancellation_resistant_work`、`test_shutdown.py` |
| T17 主动stop与重复信号 | service stop、Agent SIGTERM/SIGINT；含局部已故障状态 | 正常结束，先撤销运动，有界清理本方child/IPC/日志/学习服务/信号，锁可再获取；外来资源保留 | 用户主动stop例外，既有cleanup不退化 | `test_docker_start_helper.py:test_control_stop_reaps_owned_child_and_preserves_data`、`tests/ros/test_agent_adapter_shutdown.py`、`test_shutdown.py:test_sigterm_revokes_motion_before_cleanup` |
| T18 真fatal退出 | Agent spawn失败、Agent非0退出、journal初始化失败、无合法设备凭证 | 失败码与stderr真实；仅本方资源回收，不假称云在线、不自动新建owner | 第4节现有证据 | docker helper单child失败测试、vehicle_cli failed-start测试、`tests/unit/test_agent_config.py` |
| T19 认证拒绝 | connector抛AuthRejected | 不无限重试；认证不可用有本地事实；是否保持主进程按负责人裁定后的政策测试，不能混为ROS故障 | 现有退出行为与fatal证据缺口 | `test_runtime.py:test_auth_failure_does_not_retry_forever`、`tests/integration/test_agent_pairing.py` |
| T20 输出失效停止 | 控制owner退出→native私有EOF、样本deadline到期；另杀native本身 | 前两者现有guard输出0并拒旧数据；后者必须目标驱动看门狗停车取证；Agent仍在线报告未知/失效 | 规范独立停车；不能靠杀Agent证明停车 | `tests/unit/test_native_output_protocol.py`、`test_motion_core_stop_fallback.py`、`test_motion_core_handoff.py`；真实ROS/车后续验收 |
| T21 学习与时间回归 | Agent提前创建但adapter失败；同run控制进程重启，完整成功启动再清理 | 失败不提前清旧文件；唯一owner、完成结果恢复/消耗握手不变，不重放中间动作；原始time proof不续命 | 本任务明确保护既有学习与时钟实现 | `tests/unit/test_learning_cleanup.py`、`test_learning_record_owner.py`、`test_learning_client.py`、`test_learning_frozen_reference.py`、`test_control_clock_domains.py` |
| T22 不忙循环/不遗孤 | 子进程退出后保持Agent运行若干轮，再主动stop；launcher先死child仍存 | 完成waiter移除，无CPU忙循环；只回收本方存活child，wait/task/fd有界 | 修改FIRST_COMPLETED的必要负例；仿真整组回收缺证 | docker helper/vehicle_cli fake waiter测试；Linux受控子进程组验证（后续授权验证，非启动ROS） |

旧测试中 `test_docker_start_helper.py:test_verified_adapter_precedes_agent_and_agent_stops_first`、`test_handshake_failure_reaps_adapter_without_starting_agent`、`test_vehicle_cli.py:test_simulation_waits_for_matching_adapter` 固化了旧启动先后；应改为Agent报告面先起、运动校验仍强制。`test_shutdown.py:test_background_failure_revokes_ros_heartbeat_and_stops_agent` 及 `test_background_fault_disables_ros_while_safety_ipc_is_blocked` 名称/断言包含退役ROS heartbeat或Agent连坐，应改为局部功能撤销+Agent云日志持续，不恢复旧链。底层异常传播、已接受急停不丢、真实停止/用户stop、有界资源测试继续保留，不能整组删除以获得绿灯。

## 6. 交付边界

本轮仅提供调用链盘点、最小方案和文字用例。实施建议限launch、两种supervise、Agent composition/lifecycle及必要生命周期事件投影接线；不改运动算法、学习结果owner、时间proof、心跳资格或全量日志架构。上述真实Linux peer/进程组、ROS launch及车辆看门狗均尚未验收；这里没有新增通过数，也不声称故障隔离已经修复。AuthRejected待机政策及若现有来源屏障不足时的任务收束政策需要负责人明确，不能自行增加恢复条件或服务。
