# 运动清理返修：DDS 组合资产与删除场景去向

基线 `0f9100315767f015a697a35d39b25de3c0f62843`，原分支 `fix/degraded-drive-speed`。按负责人收窄意见，本次仅补现有单测/native socket 未覆盖的真实 DDS 差异；不恢复旧测试体系。旧断言从 `8858514` 实际函数读取，包含混合文件删除的测试；完整旧函数/断言索引保存在本轮本地证据中，以下合并相同语义。

## 唯一新增组合资产

`tests/ros/test_final_command_dds.py` 用一个参数化用例集中覆盖五种差异：stop barrier 丢失中间代次后拒绝旧帧、同名称重复 publisher、其他名称重复 publisher、唯一但错误名称 publisher、native guard 丢失。

实际链为 **真实 CommandNode publisher → DDS RMW GID/sole-writer 原生 guard → SafetyGate fence → 实际 MotionCoreOutput**；显式选择有效仿真 profile，现行 ManualIngress 准备/neutral 激活/接纳后生成 FinalCommand。不替换 guard、publisher graph、fence 或 owner 行为，只在实际 owner 方法外记录收到的帧/撤销原因。先证明非零样本已到 owner，再断言屏障/错误来源后的旧样本不再到达，guard 丢失确实触发 owner 撤销。

没有提供传感器或车辆：owner 仍执行实际反馈/感知约束并维持零输出。本资产证明命令 transport 到 owner 的身份/代次/撤销组合，不证明实际非零运动、车体停车或完整导航终态。夹具显式使用仅测试的 3 秒 command TTL，并逐次断言旧帧尚未过期，以免 DDS discovery 延迟把身份/代次拒绝退化成超时测试；没有修改产品默认值。PersistentEstop 仅用于 CommandNode 构造前置，不新增持久化重启用例。

同类产品清理仅删除 relay 两个无作用 `time.monotonic()` 表达式（Ruff 无未使用 now 局部变量），更新 SafetyGate 已失真的 unset-limit 说明为 profile 必选和共享核心连续制动。没有更改算法、参数、Docker/DNS、配对、1 米角扫掠或启动分类容差。

## 按真实断言语义整理的去向表

“已有”表示具体合同已有消费者/测试，**不宣称全部测试在本轮重跑或所有层级等价**。只为仍实际使用且有明确覆盖差异的真实 DDS 链新增资产。

| 原测试/场景 | 判定与具体去向 |
| --- | --- |
| `test_final_command_dds`：旧帧跨 stop barrier、重复/伪造 publisher、guard 丢失 | 保留差异；迁入本次小型真实 DDS 参数化资产，现行 FinalCommand/ManualIngress/profile 必选 |
| 同文件：急停重启持久化 | 已有 `test_estop_latch_state.py`、`integration/test_estop_recovery.py`；不重复恢复 ROS 重启测试 |
| 同文件：相同指令刷新不能延长原始 geometry deadline | 旧 SafetyTicket/worker 退役；原 deadline 已有 `test_motion_core_exit_deadline.py`、`test_navigation_command_clock.py`、`test_constraint_scheduling.py`；不再复制 ROS 版本 |
| 同文件：逐速度 blocked/forwarded 回执 | 旧逐速度 drive.feedback 无消费者，退役；不迁回。现用退休/回收证明不是该协议 |
| `test_motion_watchdog_latch`：bare zero 不能清旧 slip latch | 旧 slip latch/复核接口退役；现行 state-machine/controller 的故障/急停恢复合同已有 `test_motion_state_machine.py`、`test_motion_core_handoff.py` 和持久急停测试；不重复新增 |
| `test_future_source_clock`：未来数据 FIFO、八条公平预算、progress gap/requalification、过期后重放 | 旧 Fusion progress 队列和全链资格门退役；当前通用边界已有 `test_deferred_sources.py`、`test_motion_sources.py`、`test_sensor_timing.py`、`test_egress_receipt.py`。未来时钟不按旧队列资格解释，不逐条复制 ROS 版本 |
| `test_motion_topology`：新 epoch/invalid 撤销旧流量、重放/回滚/来源替换、Fusion restart | MotionEvidence/旧 Fusion pending 门退役；当前 source epoch、原始身份、时间及传输边界由 `test_motion_sources.py`、`test_native_motion_observation_adapter.py`、`test_egress_receipt.py`、原生 receipt/scan matcher 资产负责；命令 DDS 差异只在本次集中补齐 |
| `test_motion_receipt`：回调不得刷新原始收包年龄、坏 receipt 和失败 frontier | 旧 MotionEvidence frontier 接口退役；原始年龄/拒绝/重放已有 `test_sensor_timing.py`、`test_unified_motion_runtime.py`、`test_estimation_review_contracts.py`，双 capture 来源不删除、不重新打时标 |
| `test_motion_progress_continuity`：连续窗口、静止重资格、积分 applied-command、八条预算、Trigger requalify | 全属旧 progress/slip/资格链专用合同，退役；不迁回新估计逻辑。原始时间、源身份和新 reducer/控制器自己的恢复合同仍由现用测试负责 |
| 同文件：在途非零与零 ACK 顺序 | 零 Twist 不是车体静止证明；命令代次屏障差异迁入本次 DDS 资产；实际连续制动已有 `test_motion_controller.py` |
| `test_velocity_chain`：冻结时钟、原 deadline、迟到异步结果、缺观测输出、未盖章导航 | 原 deadline/输出验证已有 `test_motion_core_exit_deadline.py`、`test_navigation_command_clock.py`、`test_constraint_scheduling.py`、`test_motion_core_handoff.py`；旧 `/safety/command_in` 镜像、unset gate、MotionEvidence reason 及 applied-command/slip 比较退役 |
| `test_scan_executor_deadline`：生产 oversized self_mask、旧 gate worker 阻塞 | 生产 self_mask 已无入口、旧 worker 已退役；离线 self_filter 消费者保留；现用 ingress 有界与异步约束的实际合同由 `test_scan_ingress.py`、`test_constraint_scheduling.py` 负责，不恢复旧夹具 |
| `test_motion_shutdown`：信号/重复中断、错误抑制、子进程清理、活跃 publisher 退出 | 已有 `test_motion_lifecycle_restoration.py`、`test_receipt_process_shutdown.py`、`test_scan_matcher_process_shutdown.py`、`integration/test_shutdown.py`；退出 deadline 已有专用测试。撤回本次拟恢复的整个文件，不重复建立新关闭体系 |
| `test_velocity_uncertainty`：旧 PSD/covariance 资格、旧极值误差扫描 | 旧协方差资格门及对应数值结论退役；当前统一估计/误差/停车风险合同见 `test_unified_estimator.py`、`test_directional_constraints.py`、`test_estimation_review_contracts.py`，不恢复旧资格模型 |
| `test_review_motion`：MotionEvidence 有界字段、旧 first-latch 日志；横向信息 | 旧 wire 字段/日志格式退役；现用 SCF3 消息、原始双 capture、独立横向信息与风险合同保留在 `test_motion_guard_to_owner.py`、`test_unified_motion_runtime.py`、`test_directional_constraints.py`、`test_estimation_review_contracts.py` |
| `test_simulation_manual`：真实运动/停止；observer 16条/3ms | 现用 opt-in `test_motion_core_acceptance.py` 使用新协议验证运动/释放制动；旧 observer 私有算法无生产消费者，随 harness 退役。不将已有资产视为本机已执行 |
| `test_simulation_navigation`、混合 `test_navigation` 中 prepare 期间连接 epoch 改变即终止 | 旧租约导航断网停/连接代次驱动任务合同被最新自主任务离线继续替代；实际 task_epoch/接管/回执见 `integration/test_task_ownership.py`、`integration/test_manual_wire.py`；不迁回旧断网取消 |
| `test_motion_live`：旧连续窗口、requalify、旧 cap 资格、wheel/command stall latch | 专属旧设计断言退役，不恢复旧 live 体系；现行正常运动 opt-in 资产、真实 ingress-loss/异常证据分析仍在。真实环境完整验收仍 Draft，不能用静态断言或纯模块数目当实物结果 |
| `test_fixed_stand_fixture` 及 fixed-stand SDF/bridge/profile | 只由被删旧 live harness 消费；旧 physical_cap_context 资格与无现行消费者的夹具退役，不为保留而恢复启动注册；通用 `motion_stand_geometry.py` 未删 |
| `benchmark_motion_coverage`、`motion_qualification.launch`、`motion_progress_test_helpers` | 都是旧 evaluate_motion/progress/镜像 harness 专用消费者，退役；不复制旧 benchmark 数字、不恢复 overlay 或共享工具旧接口 |
| `e2e/test_tasks`：断网 PAUSE/重连不恢复 | 与最新任务离线继续冲突；已有 `integration/test_task_ownership.py::test_runtime_runs_offline_and_completes_same_identity`，不保留旧 TaskRunner 断言 |
| `integration/test_agent` 删除的旧云校时/期限、ROS heartbeat ACK 延迟 | 旧服务器期限/ROS运动心跳机制退役；原始 steady TTL、认证连接与后台故障关闭由现用 `test_manual_wire.py`、`test_agent.py`、`test_shutdown.py` 负责 |
| `integration/test_shutdown` 删除的旧 arbiter lease/drive IPC/epoch/reset | 原 lease、旧 velocity IPC/自动 reset 恢复退役；当前 ManualIngress 原始 context/不可复开、旧控制者隔离、持久急停及关闭顺序在 `test_manual_wire.py`、`test_manual_authority.py`、现存 shutdown 测试 |
| integration task takeover、runtime/video drive_slot | 手动接管已在 `test_manual_wire.py::test_native_acquire_consumes_durable_task_retirement_before_ipc`；有界合并/settlement/视频隔离有现用 wire 用例。Supervisor drive_slot 无消费者，退役 |
| integration CLI requalify-motion 的十个函数 | 被删命令/Trigger 专用的校验、超时、域检查、私密输出合同退役；通用部署/profile/身份检查未删除 |
| 五个 ROS 文件引用已删旧 active_fusion 夹具 | 后续收集返修已逐项删除这五个退役文件（15 个函数），消除真实收集失败；未恢复旧门或增加重复资产，详情见下节 |

终态 retired/task_retired/task_collected 的生产语义和 `test_task_terminal_release.py`、`test_terminal_release_wire.py`、`test_terminal_release_collection.py`、`test_terminal_native_channel.py`、`test_terminal_collection_capacity.py` 保留。本次不额外复制持久 settlement/回收/重启/容量测试，也不把前轮已记录的基线失败改称通过。

## 本轮验证

没有启动 ROS、服务、VM、Docker、仿真或车辆；没有执行此前拒绝的宽范围 integration/e2e 提权命令。未重跑无变化整套 unit/integration/web。DNS 调查另存只读证据，不纳入本清理提交，不修改产品连接或配置。

- 相关纯模块回归（两处生产文件与该次运行时指纹一致）：`.venv/bin/python -m pytest tests/unit/test_heartbeat_wire.py tests/unit/test_drive_challenge.py tests/unit/test_native_motion_observation_adapter.py tests/unit/test_unified_motion_runtime.py tests/unit/test_sensor_timing.py -q --tb=short`：**30 passed / 0.46s**，没有重复无变化套件。
- 最终资产纯模块检查：`.venv/bin/python /private/tmp/motion-dds-review-smoke.py`：通过实际 ManualIngress neutral 激活/接纳、原始期限和 FinalCommand schema 字段核对；只替换 ROS 消息分配，不执行 ROS/DDS。131 个被删测试的实际旧断言索引生成并校验数量。
- `.venv/bin/python -m pytest tests/ros/test_final_command_dds.py -q --tb=short`：**5 skipped / 0.22s**，未开启 opt-in，缺本机 ROS，不计为真实 DDS 通过。
- `.venv/bin/ruff check src/senior_control/cloud/relay.py ros2/src/senior_robot/senior_robot/safety_node.py tests/ros/test_final_command_dds.py` 通过；`git -c core.whitespace=cr-at-eol diff --check` 通过。

实际 DDS 待在隔离、同版 Linux ROS 工作区运行：`SENIOR_FINAL_COMMAND_DDS=1 SENIOR_MOTION_DOMAIN=173 python -m pytest tests/ros/test_final_command_dds.py -q`（173 仅示例，需选择无人使用的隔离域）。本次未执行此命令，Linux 消息生成、guard/RMW/scan matcher 链接、SCF3/823 与 ScanMotion 双 capture 同版重建及实车仍 Draft。

最终本轮三个源码/测试文件的规范化 LF SHA-256：`de602b167f9f00e01b24376080dbd633c8737e651498453444e844293bbd5bd8`；精确脚本、日志、逐文件原始指纹和旧断言索引见 `/private/tmp/vehicle-motion-dds-review-0f91003/`，本地提交 SHA 见其中 commit.json。该指纹只含本轮自有文件，避免混入共享工作区原生接收 Agent 的独立修复。

## 悬空测试导入收尾（基线 a380a555）

此前仅说明合同退役却留下旧 fixture import，会使 ROS 测试收集失败。本次实际删除文件，未新增替代接口或重复测试：

| 文件/函数数 | 读取断言后的判断 |
| --- | --- |
| `test_scan_shape_epoch_interleavings.py` / 5 | 全部断言旧 `_progress`、`_deferred_scan`、`_scan_frontier` 的 reserved future epoch/失败 frontier；不存在现行独有入口。通用 epoch/重放/回滚已有 motion_sources、deferred_sources、egress_receipt 合同测试，命令 DDS 身份差异在保留的 final_command_dds 资产 |
| `test_scan_shape_structural.py` / 2 | residual/valid_rays/noncanonical stamp 驱动旧 Fusion progress 资格；该消费者已删。原始来源时间检查仍在 sensor_timing/egress_receipt，不能恢复旧资格门以消费这些字段 |
| `test_scan_shape_qualification.py` / 5 | covariance/response/字段长度触发旧资格阈值及 retired/pending progress 断言；旧阈值合同退役，现行原始 capture、质量、身份与时间消费者/测试不改 |
| `test_invalid_source_receipt.py` / 1 | 无日期 invalid 在旧 Fusion `_scan_frontier` 建立 failure frontier 的专用合同；原始 receipt 不刷新年龄的现用合同在 sensor_timing、egress_receipt、unified_motion_runtime |
| `test_deferred_immutability.py` / 2 | 旧 Fusion `_on_scan` 对 mutable ScanMotion 的专用 future FIFO 快照和旧 epoch 丢弃；该入口/队列已删。没有为了复制旧快照语义给现行估计增加兼容路径 |

实际验证（仅静态/收集，不执行测试、不启动 ROS）：

- `.venv/bin/python /private/tmp/vehicle-motion-import-cleanup-a380a55/check_imports.py`：解析全部 **201 个**测试 Python 文件，核查本轮所有被删生产/测试模块的普通 import、from import、常量动态 import；**无悬空引用**。并确认五个文件已移除，两个禁止修改的原生接收文件保留任务开始时字节哈希。
- `.venv/bin/python -m pytest tests/ros --collect-only -q`：**134 tests collected / 0.45s**，无收集错误；本机缺 ROS 的模块可在 importorskip 边界跳过，不能把收集成功宣称 ROS 环境运行通过。
- `git -c core.whitespace=cr-at-eol diff --check`：通过。

本轮只提交这五个删除及本文更新；不修改 native_receipt.py 或 tests/test_native_receipt_diagnostics.py，不运行大套、不部署、不推送。Linux/ROS 同版构建、DDS 实际执行和实车仍未验证。原始五文件、逐文件哈希/函数清单、精确检查脚本与收集日志保存在 `/private/tmp/vehicle-motion-import-cleanup-a380a55/`，最终本地版本见其中 commit.json。
