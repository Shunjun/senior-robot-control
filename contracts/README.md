# senior-contracts：云车 WebSocket 契约

独立 Python 发行包，源码 `src/senior_contracts`，只依赖 Python ≥3.10/Pydantic。共享范围按真实 WebSocket 校验、编解码及必要嵌套引用确定，不导入 ROS、车辆执行或管理业务实现。内部值类、异常、驾驶配置和本地运行状态不因界面展示同形字段而成为共享类型。

本次基线 `feat/vehicle-architecture-refactor@3dfd1e0`；代码与记录对应本文件所在提交。依据[最新版确认稿](../docs/architecture/ROS2与车端架构重构确认稿.md)及[计划的云车共享契约范围](../docs/superpowers/plans/2026-11-19-vehicle-architecture-refactor.md)。本次未搬云/管理子项目、未迁后续运动包、未修改守卫或控制规则。

## 实际生产链

- 云：`cloud/app.py` 的 `/ws/agent`、`/ws/control` → `cloud/relay.py` 的 `Hub.handle_agent_frame/handle_control_frame` → `validate_frame`、`serialize_envelope` → socket `send_text`。
- 车：`agent/websocket.py:WebSocketLink.recv/send_response/send_snapshot/send_video` 直接使用共享编解码；`agent/main.py` 的分发和回复、`agent/snapshots.py:snapshot_envelope` 产生实际消息。
- 日志：`/ws/logs` → `cloud/logs.py:LogRelay.viewer` → `/ws/agent/logs` → `agent/logs_wire.py:AgentLogConnection.serve`；两端都校验共享 `LogRequest.query`。
- 云端快照缓存独立归 `cloud/snapshots.py`，只存共享 `SnapshotPayload` 的深拷贝；relay 不再导入 agent 快照实现。车端仍拥有 `LiveState/RobotSnapshot` 和上行投影。

## 保留类型与消息索引

下表列出所有自行定义的共享类型及必要别名/约束；`wire.py` 中的 private adapters 只是这些消息的校验帮助对象。

| 文件/类型 | 实际云车消息与嵌套边 |
| --- | --- |
| `wire.StrictModel/EnvelopeBase` | 以下消息共同校验结构；`_received_steady` 仍是本地 PrivateAttr，不上线 |
| `HeartbeatRequest` | 车端 heartbeat.request → 云校验并构造 heartbeat.reply → 车端校验原字段及 nonce |
| `EmptyPayload/SimpleEnvelope` | control.acquire/release、drive.stop、safety.estop；云亦合成退场 release |
| `ManualIntentPayload/ManualIntentEnvelope` | drive.intent 的 authority/gesture/sequence/linear/turn/gear；controller_generation 规则保持 |
| `RecoveryPayload/RecoveryEnvelope` | safety.estop.reset、safety.fault.recover 的 revision |
| `SettingsApplyPayload/SettingsGetPayload/SettingsResultPayload` | settings.apply/get/result 的原请求形状 |
| `SettingsEnvelopeBase/SettingsGetEnvelope/SettingsApplyEnvelope/SettingsResultEnvelope/SettingsEnvelope` | 设置请求的身份、代次与判别联合 |
| `LearningStartPayload/LearningStatusPayload/LearningStartEnvelope/LearningStatusEnvelope` | learning.start/status；attempt 与 start_request 查询身份 |
| `LearningStartReceipt/LearningActiveModel/LearningStatus` | response.data 的学习分支实际验证；active_model 嵌套 LearningActiveModel |
| `TaskExecutionCommandPayload/TaskExecutionEnvelope` | task.execution.command，原 pause/resume/cancel/takeover 请求 |
| `OperationPayload/OperationEnvelope/OPERATION_TYPES` | task.create/update/delete/enable、home.set、navigation.goal/return_home、waypoint.create、map.export/import/clear、operation.query |
| `ShortText/MapRefPayload/WorkspaceRequest/WORKSPACE_REQUEST_TYPES/WorkspacePayload/WorkspaceEnvelope` | task.get、waypoint.list、map.manifest/tile；map_ref、grid/tile/cursor 约束 |
| `HomePayload/TaskSummaryPayload/ScanPointPayload/RobotPosePayload` | snapshot 的 home（再嵌 map_ref）、tasks、scan_points、robot_pose |
| `SnapshotPayload/SnapshotEnvelope` | 实际车端状态上行与云端缓存投影；route_progress 直接嵌共享 RouteProgress |
| `TaskNavigationState/TaskCommandReceiptPayload/PreparedAuthorityPayload` | response.data 的 navigation/query、task.execution.command、control.acquire 分支实际校验 |
| `ResponsePayload/ResponseEnvelope` | 车端回复及云端错误回复；ok/error、status/data 约束与省略规则不变 |
| `SdpPayload/CandidatePayload/VideoSdpEnvelope/VideoCandidateEnvelope/VideoCloseEnvelope/VideoEnvelope` | video.offer/answer/candidate/close；只含信令，媒体不走本 envelope |
| `Envelope/Frame` | 实际编解码判别联合及其类型别名 |
| `validate_envelope/validate_frame/serialize_envelope` | 原上线编解码、目标 robot 校验、严格字段与长度限制 |
| `HEARTBEAT_WINDOW_SECONDS/HEARTBEAT_INTERVAL_MS/DEVICE_REVOKED_CLOSE_CODE` | 两端现有心跳/关闭语义 |
| `MAX_MESSAGE_BYTES/MAX_SDP_CHARS/MAX_CANDIDATE_CHARS` | 原 65536/16384/512 上限；内部 safe integer 和 adapters 仍用于同一校验 |
| `errors.ErrorCode` | ResponsePayload.error 的稳定字符串枚举；不是共享异常实现 |
| `navigation.RouteProgress/RouteStatus/MAX_ROUTE_WAYPOINTS` | snapshot.route_progress；active_waypoint_index 原 `<64` 约束仍需该常量，所有现有进展/身份字段保留 |
| `logs.LogRequest/LogQuery/Level/LOG_WIRE_BYTES` | 日志 WS 的 action、嵌套查询字段/等级及原 5MiB 通道上限；共享 Query 无筛选方法 |

`settings`/workspace 回复的 `data`、`snapshot.vehicle_ui` 和日志回复仍是原字典 JSON。字段不变，但不因其由本地对象 `model_dump` 产生而把完整业务类、缓存、执行许可或持久化服务放进共享包；本次不新增强制 DTO 或扩大/收紧这些旧字典消息的接受规则。

## 唯一车内归属

| 从共享包移出的类型 | 唯一实现位置 |
| --- | --- |
| Velocity、BodyTwist、Pose2D、MapRef、SensorHealth、OperationStatus、OperationResult、数值校验帮助 | `src/senior_control/models.py`，等待相应功能模块后批迁移 |
| DomainError | `src/senior_control/errors.py`；ErrorCode 仍显式转发共享枚举 |
| FrozenModel、DrivingSettings | `src/senior_control/settings/models.py`；配置字段和校验原样保持 |
| RoutePlan、RouteRejected | `src/senior_control/navigation.py`；RouteProgress 则显式转发共享类型 |
| BodyReferenceState、RuntimeObservation、MotionRuntimeState | `src/senior_control/protocol.py` 的本地接口部分；原 schema alias/字段/校验保持 |
| LogQuery.matches、日志状态/存储/脱敏 | `vehicle_logs/models.py` 的本地 Query 子类及原 service/store；字段只在共享 DTO 定义一次 |
| CloudSnapshot/SnapshotCache | `cloud/snapshots.py`，不再由 agent 导出云状态 |

`contracts/values.py` 与 `configuration.py` 已删除，不留反向导入车内模块的兼容壳。旧本地 `senior_control.models/errors/protocol/navigation/settings.models` 入口仍可用。真正 WS 类型的历史本地路径只显式转发同一对象并标注 `//Compatible`。

日志入口将共享查询转成本地 Query 子类，保留原 `matches` 规则（gap 永远可见）和流式/查询行为；共享类不带筛选业务。云缓存保留原 `clock-synced > stale_after`、offline 标记、原 last_synced_at、连接/几何/路线/准备状态投影、legacy navigation_ready 缺失省略及浏览器 owner 元数据清理；不产生任何控制许可。

## 范围与记录

实际 **30 文件：产品21（新增2、修改17、删除2）、测试5、配置构建0、工具文档4**，位于已预报26–34内。新增为共享日志 DTO 与云私有快照缓存；测试只迁原缓存输入边界/导入并加强同一既有消息场景断言，没有新驾驶情景。工具文档为本 README、安装工具和同步的确认稿/计划。

Ruling：本次先把不属于云车共享的现有声明交还原车内入口，避免为了数个值类提前创建新的 core 包；正式 ROS owner 落位仍归后续职责迁移。共享类型变窄不改变线上字段，整个应用未部署。

## 验证证据

环境：macOS arm64；CPython 3.12.15、pytest 8.4.2、ruff 0.17.3、uv 0.12.23。`PY` 为 `/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python`，pytest 使用当前工作区源码与 contracts 源码路径。

```sh
"$PY" -m pytest -q tests/unit/test_protocol.py tests/unit/test_websocket_link.py tests/unit/test_live_map_geometry.py tests/integration/test_agent_snapshots.py tests/integration/test_cloud.py tests/integration/test_agent.py tests/integration/test_settings_logs_wire.py tests/integration/test_settings_logs_review.py tests/integration/test_vehicle_logs.py tests/integration/test_task_ownership.py -k 'not test_directory_log_database_does_not_abort_composed_control' --disable-warnings --maxfail=1
```

结果 **196 passed, 1 deselected**（12.48s）：覆盖实际 cloud relay/Agent 的编解码、快照过期、RouteProgress/任务嵌套响应、authenticated settings/logs wire、筛选/脱敏及原 WS owner 行为。WebSocket 测试夹具需要临时 loopback socket，沙箱原先 EPERM，获测试权限后运行；没有启动开发/车辆服务。

```sh
"$PY" -m pytest -q tests/unit/test_settings.py tests/unit/test_startup_learning_chain.py tests/unit/test_native_output_protocol.py tests/unit/test_unified_motion_runtime.py --disable-warnings --maxfail=1
```

结果 **67 passed, 6 skipped**（4.45s）：验证还原的本地值/配置/runtime 入口没有改变原运动链/40字节及停止语义；6项为原 Linux-only 协议场景，本机未执行。

### 明确登记的旧失败

整组首次执行在 `test_settings_logs_review.py::test_directory_log_database_does_not_abort_composed_control` 失败：旧 mock 只承认 identity/disable_motion/exit，不承认 Agent 当前发送的 learning_capability，导致无快照。没有改测试或生产代码绕过它。用原 `3dfd1e0` 的完整归档在 `/tmp/contract-convergence-baseline` 独立复现 **1 failed**，调用栈确认实际使用该归档源码：

```sh
git archive 3dfd1e0 | tar -x -C /tmp/contract-convergence-baseline
"$PY" -m pytest -q -c /tmp/contract-convergence-baseline/pyproject.toml /tmp/contract-convergence-baseline/tests/integration/test_settings_logs_review.py::test_directory_log_database_does_not_abort_composed_control --disable-warnings --maxfail=1
```

最终命令中的 1 deselected 正是该已复现旧失败；不宣称全量绿。迁移 RED 为原缓存用例转到云 owner 路径时的 ModuleNotFound；实现后同场景通过。

### 安装与依赖闭包

```sh
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/contract-convergence-wheels
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build contracts --offline --out-dir /tmp/contract-convergence-wheels
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv pip install --python /tmp/vehicle-batch-one-venv/bin/python --reinstall --no-deps /tmp/contract-convergence-wheels/*.whl
cd /tmp
/tmp/vehicle-batch-one-venv/bin/python -I /Users/zhangshunjun/workspace/orca/senior-robot-control/vehicle-architecture-refactor/tools/verify_batch_one_install.py
/tmp/vehicle-batch-one-venv/bin/senior-robot --help
```

实际 wheel 构建安装成功；隔离前缀已具备原安装依赖和未改的 control wheel，Pydantic 2.14.3。离开仓库后共享编解码/日志请求/RouteProgress 的旧入口身份验证通过，内部类实际归车内模块，本地入口与 CLI help 正常；另以 `python -I` 检查单独导入 shared wire/logs 不加载 senior_control/senior_robot/rclpy，导入云缓存不加载 agent，均通过。

对本批 Python 做 ruff check/format --check，`uv lock --check --offline` 与 `git diff --check`；锁和依赖未变化。无 ROS、ament_index_python 或 colcon，安装工具明确 native installation UNVERIFIED；本次 native/ROS/IPC 实现未修改，未做实车/真实ROS/部署或全量套件验收。

兼容补充证据：在基线完整归档与当前源码的隔离 Python 子进程（分别设置 CHECKOUT_ROOT/PYTHONPATH，并断言实际 wire.__file__ 位于对应目录）计算 `TypeAdapter(Frame).json_schema()`、`HeartbeatRequest.model_json_schema()`、`LogRequest.model_json_schema()`，排序 JSON 后逐项比较完全相同；结合上述实际编解码/relay原回归，保留原字段和约束，不把schema比较当作运行链替代。

### 兼容复核中的实际 bug 回归

移走车端快照转换后发现原 `snapshot_from_envelope` 构造 `MapRef` 的隐式校验不能丢：`map_id=" "` 的共享快照虽能通过原 WS 字段模型，原云路径在缓存前会抛 `ValueError("invalid map reference")`。已在完整 `3dfd1e0` 归档的原转换函数复现拒绝，并在迁移实现上以 `test_cloud_cache_preserves_original_map_identity_validation` 看到 RED（DID NOT RAISE）后补回同一缓存入口校验，覆盖主地图和 Home 地图引用；没有把这条校验扩到其他 WS 消息或让共享包导入车内 MapRef。

这是本次迁移引入并在交付前修正的实际兼容 bug，新增1项定向回归，文件总数仍30；不是新增驾驶功能或无关故障矩阵。修正后快照/缓存定向组79通过，完整已选组196通过、1项已证明基线旧失败明确排除；重新构建并安装 manager wheel 后，同一离仓库安装/依赖闭包检查通过。首次收敛提交为 `6f4a235`，该兼容修正作为后续小提交交付，审阅范围应以 `3dfd1e0..最终HEAD` 为准。
