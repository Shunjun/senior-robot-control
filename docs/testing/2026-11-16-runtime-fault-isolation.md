# 第11项局部故障隔离：实现与集中验证

2026-11-16。状态：本轮实现及本地集中验证完成，待负责人亲审；未提交、暂存、推送或部署，未启动开发服务、ROS、仿真或车辆。

工作区为 `fix-degraded-drive-speed`，分支保持 `fix/degraded-drive-speed`，HEAD保持 `593e8f13f385892f0bd62cf75c593f7b07efd2da`。交付仍归原 Draft PR26。本轮起点是该HEAD加任务开始时的全部既有脏改动，不能用HEAD diff替代本轮增量。

## 交付与基线

- [启动快照到最终版本的精确增量](2026-11-16-runtime-fault-isolation-owned.diff)：只含本轮19个Python文件（9个运行/launch文件、10个测试文件）。新增文件以 `/dev/null` 为起点；既有文件以启动时实际字节为起点。
- [逐文件起点/最终SHA-256与保护文件核验](2026-11-16-runtime-fault-isolation-fingerprints.json)：含9个运行/launch文件在最终测试开始时冻结的SHA，已逐个比对最终代码一致。
- [最终pytest日志](2026-11-16-runtime-fault-isolation-evidence/pytest-final.log)、[Ruff](2026-11-16-runtime-fault-isolation-evidence/ruff-final.log)、[严格mypy](2026-11-16-runtime-fault-isolation-evidence/mypy-final.log)。
- 完整起点快照保存在本机 `/tmp/fault-isolation-start/files`，另有全工作树起点SHA、git status及HEAD；它是本机临时审阅材料，持久交付依赖上面的精确增量和指纹。

三保护文件的最终SHA与启动SHA逐字节一致：`ros2/src/senior_robot/senior_robot/native_receipt.py`、`tests/test_native_receipt_diagnostics.py`、`docs/reviews/2026-11-16-software-estop-owner.md`。核对启动清单后，其他既有文件变化仅为增量中列出的16个文件；另新增3个Python文件及本报告/证据。未改运动算法、学习结果owner、时间proof、原始TTL或心跳10的新实现。

## 最终行为和职责

### launch与外层监管

`safety_motion.launch.py` 删除任意子进程退出就发全局Shutdown的回调。实车和仿真launch均用同一 `report_ros_exit` 记录PID、退出码等生命周期事实，不解析任意stdout、不授予控制权限。

两种supervise先创建Agent报告面，再在独立启动task中创建ROS/adapter。ROS创建OSError、readiness/identity失败和运行期退出都生成故障事件、回收本方受影响资源，保持Agent及已有stop入口。完成waiter及时从集合移除，不在已完成wait上空转；不会重新spawn失败模块。camera仍按原局部职责隔离。

实车motion、adapter、camera均为本方独立进程组；adapter换组不改变其exec后PID，原Unix peer PID/UID检查保留。仿真ROS也改为本方独立进程组，复用现有组终止/幸存者收尾逻辑。用户stop、Agent创建失败/自身退出、真正顶层不能继续运行时仍进入有界整体finally；`profile=None`任意单child语义保留。

学习旧文件清理仍要求原来的完整启动成功条件（adapter验证成功、相应子进程及Agent存活、未stop）；没有因Agent提前创建而提前清理。本run学习服务和稳定锁的生命周期保持。

### IPC准入与故障上报

`run_agent` 不再在ROS identity检查失败时提前返回1/78。所有composition中的ROS clients在**每条新连接**先做identity握手，再允许业务请求或专用intent stream；失败丢连接，后续已有请求/轮询才重新建连，不新增重试服务或重发原请求。

实车还要求现有owner记录匹配当前socket inode/UID及本run标识，且连接前后socket身份一致。父监管仍先做原PID/UID/robot/domain校验，再写本run证明。当前run标识复用既有启动claim身份，没有新增run owner或学习状态写者。旧run遗留的有效socket记录也不能让提前启动的Agent绕过本次peer校验；保留旧列表格式的stale socket清理兼容，代码标注 `//Compatible`。外来活跃socket仍不接管、不删除。

新增 `runtime/lifecycle_events.py` 是父监管/launch到Agent的单向pipe接口，单帧不超过2048字节、非阻塞原子写，缓冲满不阻塞owner，后续事件报告丢失计数并保留本地日志。Agent读端有帧长限制，EOF/坏帧报告观察通道不可用；不会下线Agent或发出运动。它没有常驻服务、控制命令或第二日志owner。

事件经现有Agent日志handler、VehicleLogService及日志连接提供query/follow；快照在 `vehicle_ui.module_faults` 提供模块事实，`ipc_unavailable`继续使用现有投影。报告元数据与内部失败模块集合分离，通道上的ready事实不能恢复已失败本地模块。未新增前端展示组件或全量rosout采集；ROS原始日志仍归原ROS_LOG_DIR/日志环境。

### Agent局部失败收束

- manual intent传输失败保持健康云link，丢弃旧queued intent、拒绝该失效authority后续输入，并用现有 `control.release` 请求退役该authority；没有重放速度或自动申请新authority。
- stop/estop交付未确认、安全pending满、drain失败不再设置auth_rejected或请求全局shutdown。错误回执/未确认日志保留；已接收安全事件继续有界结算，不伪造成功或车端锁存。
- 安全交付故障复用原 `disable_motion(epoch)` 撤销手动来源，同时调用现有任务owner的pause/revoke，保留未知撤销义务；不靠杀Agent停车。两项分别有界，捕获原epoch，迟到清理不读取一个新的epoch去撤销新手动来源。
- 安全交付失败本身不新增永久任务准入锁存；任务暂停、来源栅栏、未知回执及显式继续仍由既有任务owner决定。真正tasks后台失败则停用该driver，阻止其新派发/继续，经同一owner收束当前执行，不自动重启。
- logs后台失败只报告并撤销对应能力，控制云继续；没有新增日志存储或重试owner。现有日志连接内部的重连逻辑保持。
- session旧工作取消不响应时有界隔离，保留原inflight容量和deadline；retired task不能重新调用准入，也不能向新/旧已退役link发送成功结果。异常task取出并记录堆栈，不吞异常后继续运行失效业务。

AuthRejected、凭证/配置错误、主数据库/journal初始化失败的现有退出策略保持；未增加无数据库报告系统、凭证热更新或认证待机模式。进程级致命异常仍可传播。普通本地故障不再借runtime_failed/78停止整个报告面。

## 直接回归依据与实际覆盖

主要用例见 `tests/integration/test_runtime_fault_isolation.py`；其中supervise组合用例运行真实 `composition.run_agent`、Agent、Supervisor、SQLite日志服务、真实Agent日志协议/grant/query，只替换OS子进程与远端socket边界。验收断言同时检查同一link/epoch继续心跳、故障快照和可查询日志，没有只以“进程未退出”作为通过条件。

| 实际bug/要求 | 刺激与直接断言 | 验证位置 |
| --- | --- | --- |
| 原全局launch退出Shutdown | 正常0码及非0退出都只产生生命周期事实；两个launch绑定观察回调且无Shutdown调用 | `test_launch_exit_only_reports_lifecycle` |
| 原motion/adapter/ROS退出会带走Agent | 实车motion退出、adapter退出、仿真ROS退出；Agent同link/epoch心跳与故障快照继续、真实日志wire查询到退出事件；组清理调用到位且不再spawn | `test_supervise_local_fault_keeps_actual_agent_and_log_transport` 的exit/adapter_exit |
| 原ROS创建/readiness失败时没有Agent | 实车/仿真spawn抛OSError和identity拒绝；已存在实际报告面，故障日志可查，后续主动stop可正常结束 | 同上spawn/identity |
| 原composition无IPC/身份不符返回1/78 | 缺socket或foreign identity，真实composition继续出不可用快照；foreign server只收到identity，未收到业务请求 | `test_agent_config.py:test_composition_starts_reporting_without_adapter_identity` |
| 不沿用旧身份或旧owner记录 | 重连后identity拒绝阻止业务；intent连接也先握手；无记录、错误记录、旧run记录均拒绝，当前run记录才可访问 | `test_runtime_identity.py` |
| 原manual IPC失败关闭云link | send抛IpcError后同link继续心跳；release交付、同authority新seq不再发送 | `test_manual_transport_failure_retires_authority_without_cloud_disconnect` |
| 原estop未确认/容量满会认证失败并下线 | 真实Agent接收安全事件，交付IpcError或填满pending；无成功回执、非auth_rejected、心跳继续；实际durable任务owner进入revoking，原plan不重复 | `test_safety_delivery_fault_preserves_cloud_and_never_claims_success` |
| 已接受急停仍须有界结算 | 满容量拒绝额外请求，原accepted事件逐个交付；stop/estop并发、阻塞IPC和shutdown drain保持原语义 | `test_shutdown.py` 相关原测试及更新后的非连坐断言 |
| tasks/logs后台异常 | module factory只运行一次，失效task结束，Agent继续心跳；真实任务owner根据撤销proof进入paused或保留revoking/active effects | `test_failed_background_module_stops_without_stopping_report_plane`、`test_task_failure_uses_actual_durable_pause_owner` |
| 旧session straggler | 旧工作吞取消，连接已切epoch2后才继续；新运动准入拒绝、不发迟到成功、占用原有inflight直到完成，Agent继续报告 | `test_retired_straggler_cannot_issue_new_motion_or_reply`及原overload/取消测试 |
| 观察通道EOF | 真实pipe关闭，Agent观察故障并继续同link心跳；ready不能清除本地failed模块 | EOF及ready防复活测试 |
| stop和Agent自身退出仍需收尾 | 两种supervise在故障后stop并回收本方资源；模拟Agent wait退出78，外层传播78；单child入口另用真实Python子进程验证退出传播 | supervise Agent退出用例、`test_docker_start_helper.py`真实子进程测试 |
| 心跳、普通业务和持久性回归 | 原手动TTL、任务离线自主/幂等/普通失败、云重连、日志隔离和磁盘满、学习清理/共享记录/同run恢复、控制时钟及输出失效用例 | 本轮集中命令列出的既有套件 |

退役测试只移除旧启动先后、旧verify返回码和全局连坐要求；有效身份、外来资源、accepted安全事件、期限、持久任务及日志权限测试保留。日志诊断测试改用新的按模块异步故障回调并继续验证堆栈脱敏；任务wire回执测试明确建立当前link/epoch，以符合新迟到回执屏障。

## 最终验证

执行入口为本机 `/tmp/fault-isolation-start/final-check.sh`，最终测试开始前冻结9个运行/launch文件SHA，结束后与最终文件、交付指纹逐一匹配。主要命令为：

```sh
PYTHONPATH="$PWD/tests/integration:$PWD/tests/unit" .venv/bin/pytest -q -rs -W error::RuntimeWarning \
  tests/integration/test_runtime_fault_isolation.py tests/unit/test_runtime_identity.py \
  tests/unit/test_agent_config.py tests/unit/test_docker_start_helper.py \
  tests/integration/test_vehicle_cli.py tests/unit/test_learning_cleanup.py \
  tests/integration/test_runtime.py tests/integration/test_shutdown.py tests/integration/test_agent.py \
  tests/integration/test_manual_wire.py tests/integration/test_task_ownership.py \
  tests/integration/test_task_runner.py tests/integration/test_agent_snapshots.py \
  tests/integration/test_settings_logs_review.py tests/integration/test_settings_logs_wire.py \
  tests/integration/test_vehicle_logs.py tests/unit/test_heartbeat_wire.py \
  tests/unit/test_websocket_link.py tests/unit/test_native_output_protocol.py \
  tests/unit/test_motion_core_stop_fallback.py tests/unit/test_logging_diagnostics.py \
  tests/unit/test_learning_record_owner.py tests/unit/test_learning_client.py \
  tests/unit/test_learning_frozen_reference.py tests/unit/test_control_clock_domains.py
```

最终结果：**449 passed，7 skipped，33.28s，退出码0，无RuntimeWarning**。Ruff检查本轮19个Python文件通过；严格mypy检查7个非ROS运行模块通过；19文件AST解析通过。`git -c core.whitespace=cr-at-eol diff --check` 在本轮既有文件范围通过；该临时选项用于保留原测试文件CRLF，不把既有换行风格转换成整文件噪声。

早期沙箱内fixture绑定Unix socket遭EPERM，不当作产品故障；获准对本地fixture在沙箱外执行后验证真实socket行为。中间轮修正了测试root未resolve、旧回调/连接fixture假设，以及实现中直接release失败标记、Agent已死仍启动本地模块和取消窗口中未await协程的问题；最终轮覆盖后者且启用RuntimeWarning为错误。未做RED基线回滚：按本任务要求保留既有工作树、完成修改后集中验证，精确起点代码保存在启动快照/增量中。

## 真实限制与待亲审事项

1. 7项跳过：1项Linux SO_PEERCRED/PID实测、6项Linux SOCK_SEQPACKET/native输出协议实测。当前macOS结果不能证明目标Linux、ROS Humble、DDS、插件ABI或实车行为。
2. ROS/adapter/camera进程在新组合测试中使用可控child fixture，验证监管的真实等待/分支、组回收调用、报告与stop；没有实际启动ROS，也未完成真实ROS进程组幸存者回收和第三方WHEELTEC内部launch策略验收。
3. 车辆独立看门狗、native本体死亡后的物理停车、真实驱动失效制动仍需目标环境取证。IPC不可达时只能保留撤销未确认，不能写成已物理停稳；未新增强杀健康控制栈来伪造停车确认。
4. 生命周期通道只补本方spawn/readiness/退出摘要，不宣称云端已有全量rosout。真实云TLS/远端部署未验证；测试远端边界为内存link/log socket，Agent、composition、日志协议与本地SQLite为真实实现。
5. 真正主数据库/journal无法初始化、AuthRejected等现有退出边界未扩展；运行期数据库故障可能使原业务快照采集持续不可用，此时仍保留连接心跳和独立日志诊断，不伪造数据库结果或健康状态。
6. 最终为作者自审，未启用额外worker；负责人尚未亲审。此次结论是本轮相关本地验证通过，不是整个仓库CI、ROS集成或整车验收通过。

根方案在当前允许写入目录之外，本轮未修改它。供负责人更新第11项的实际状态文本：

> 第11项局部故障隔离代码已完成，待负责人亲审：删除launch任意子进程退出连坐和两种supervise终止Agent链，ROS/adapter不可用时保持Agent报告面，保留每连接identity及实车本run peer证明；后台模块、manual及安全交付故障局部收束，复用唯一任务/急停owner与原来源屏障，不自动重启或重放运动，AuthRejected保持原策略。启动快照到最终精确增量、逐文件SHA及证据见 docs/testing/2026-11-16-runtime-fault-isolation.md；本地集中449通过/7项Linux缺证跳过，Ruff19文件及严格mypy7模块通过，三保护文件SHA不变。真实Linux/ROS/下游物理停车与部署未验收，未提交推送，不代表可驾驶或整车完成。
