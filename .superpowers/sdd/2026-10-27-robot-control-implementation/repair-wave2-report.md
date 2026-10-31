# Wave 2：云连接、心跳与车端组装

基线 89c8665。提交号见本报告所在提交。代码仅供继续集成和评审，**尚不能验收实车遥控、复位、导航、地图或任务执行**。

## 已实现与验证范围

- Agent 入口从显式 CLI/环境装配一个数据库、任务存储、航点存储、操作日志、WSS 连接、独立 IPC 客户端和任务扫描。缺配置返回 78，认证拒绝和协议错误不无限重试；正常重连不自动恢复旧控制或旧请求。任务扫描保存墙钟边界，重启将离线区间记为跳过，启动时原运行实例进入 needs_review。
- 云数据库通过 migration 5 为每车持久递增连接代次。ASGI 测试重建应用并使用同一数据库重新认证，验证代次递增、旧代次心跳被拒绝。心跳请求／应答绑定车、代次、nonce、发送时间，Agent 验证往返时间和云车时钟误差，只有收到并经 ROS IPC 确认、仍在本地 deadline 内的应答才授予资格。丢应答/半开连接会停止续期并关闭连接；云端会在心跳过期后关闭连接并将缓存标为离线。实际 TCP 握手单测、Hub→Agent→Unix IPC→真实 Arbiter 的隔离链路和半开测试通过；不代表真实 ROS 图验收。
- ROS 适配器生产入口要求独立 server heartbeat，本地单调期限到期撤销旧运动，新的代次须重新 control.acquire。旧代次撤销不应影响已接受的新代次。单管理员多页面回归证明当前认证代次下其他页面可急停且不取得租约。
- 重现跨页面急停前排队的 reset 后到达会错误解除急停；生产路径暂时将 **safety.reset 返回 unavailable**，直至协议实现急停 fence。受控测试 seam 中的旧行为不能作为生产验收。
- 浏览器源时钟没有可验证的误差上限，不能从 expires_at 证明产生时间。生产装配将 **drive.velocity 返回 unavailable**，CLI 没有绕过选项；实验性受控测试使用显式已验证时钟 seam。已实现的到达年龄校验拒绝过远未来时间戳，将已耗时间计入 ROS 本地 deadline，但它不能证明浏览器源时钟。下一波需控制页面取得绑定控制 socket、robot、epoch、发放单调时间的短期 server challenge；云在发放时固定 deadline，验证 challenge 后附认证 deadline，车端以已测得云车偏差和不确定度换算并扣除误差，不能按收到时间重开 TTL。此波未修改 web。
- ROS 地图/定位/状态/任务导航 consumer 尚未实现。它们缺失或拒绝时返回 unavailable，快照标记 execution_status=unavailable、无活动地图/可运行任务；Home 不凭空初始化。任务导航普通命令也返回 unavailable。测试中的 FakeState 与受控 clock seam 不是真实机器人数据。已知任务运行尚无动作终态回读，不能宣称任务完成；导航 waypoint 坐标分发也尚未闭环（见下）。
- 修复两处测试 SQLite 连接所有权，并增加重复进入拒绝和同一 app 多次 lifespan 重开测试；异常中途确保关闭。全依赖严格检查另暴露视频测试的编码线程事件循环未关闭，已增加显式关闭、线程回收和测试资源所有权，未增加视频功能。增加 Linux/Python 3.10 ROS extra 的 NumPy <2 约束，锁文件解析到 1.26.4，其他平台/版本保持独立解析。已在 CI 安装 safety/cloud/agent/video/ros extras；Linux Humble cv_bridge 实际导入复验仍待执行。

## Agent 启动方式

安装 `.[safety,agent,ros]`（服务端另装 `.[cloud]`，视频测试装 `.[video]`）。先使用 `python -m senior_control.agent.main --pair --cloud-url wss://<cloud>/ws/agent --device-name <name> --credentials-file /etc/senior-control/device.json`，交互式输入一次性配对码；HTTPS 成功后原子创建 0600 凭据文件，已存在时拒绝覆盖。正常运行设置 `SENIOR_CLOUD_URL`、`SENIOR_DATABASE_PATH`、`SENIOR_AGENT_SOCKET`（后两者为绝对路径），运行 `python -m senior_control.agent.main --credentials-file /etc/senior-control/device.json`；`SENIOR_CREDENTIALS_FILE` 也可指定该文件。亦可显式提供 `SENIOR_ROBOT_ID` 与 `SENIOR_DEVICE_TOKEN`，或使用 `--robot-id`、`--cloud-url`、`--database-path`、`--socket-path` 和 `--token-file`。该命令会常驻，须由服务管理器负责启停。缺 ROS 消费端不会制造成功状态。

## 下波 ROS IPC 合约（v=1，Unix socket 0600，4 字节大端长度 + JSON，帧上限 8192）

下列请求都含 `v:1` 和 `type`，响应含 `v:1` 和 `ok`；未实现必须返回 `{v:1,ok:false,error:"unsupported"}` 或 `unavailable`。进程采用独立客户端分别处理控制、状态/Home、地图及导航，IPC 客户端请求超时 1 秒；地图/导航实现还须保持各自 deadline 与取消语义。

- `server_heartbeat`: 请求 `connection_epoch:int, deadline:float`，deadline 是与 ROS 同一主机单调时钟坐标；返回 `ok:true` 仅表示当前有效代次已接受该有界期限。超时、旧代次、deadline 已过或超过 ROS disconnect 上限必须拒绝并保持停车；不能由 ROS 自行续期。此处理器已实现，供联调。
- `state.get`: 成功响应字段 `sampled_at:float`（同主机单调时钟、采样距 Agent 检查 0–0.5 秒）、`localized:bool`、`safety_clear:bool`、`manual_driving:bool`、`map_id:string|null`、`reference_revision:int|null`、`execution_status:string`。两个 map 字段同时为空或同时有效。应由真实定位、map、仲裁和安全链派生；不得使用 Gazebo ground truth 或默认 ready。缺失、陈旧、不一致返回 unavailable。
- `pose.get`: 成功响应 `sampled_at:float, healthy:true, map_id:string, reference_revision:int, anchor_id:string, x:float, y:float, yaw:float, anchor_x:float, anchor_y:float, anchor_yaw:float`，采样时限同上、数据有限且 anchor 非空。真实定位及其一致 map/anchor 是权威来源；Agent 只在验证后写本车数据库 Home。需保证 map/anchor 在采样及提交期间仍有效，否则拒绝。无真实定位返回 unavailable。
- `navigation.task_start`: 请求 `execution_id, task_id, map_id, reference_revision, waypoint_ids`；仅当 ROS action 已按该 execution_id 接受对应任务时响应 `{v:1,ok:true,execution_id:<相同值>}`。`navigation.task_cancel`: 请求 `execution_id`，确认取消后返回同样的匹配 id；超时/未知结果不得伪造成功。**当前请求仅含 waypoint ID，ROS 没有权威数据库读取通道；下波必须扩展为由 Agent 校验同 map/revision 的航点姿态并随请求传递，或提供有界、授权的反向查询，完成前不可启用。** 还需定义 execution_id 绑定的运行/终态查询或推送、重复请求和取消竞态协议，再将结果写入 TaskStore；当前没有终态消费实现。
- `map.export`: 请求 `operation_id, map_id, reference_revision, path`；`map.import`: `operation_id, path`；`map.clear`: `operation_id, map_id, reference_revision`。返回 `ok:true` 必须意味着对应操作实际完成并保证事务/幂等语义；仅接受受限目录内路径，配合 Agent 现有名称验证。地图事务、包传输和 SLAM 桥尚未接入，不能因为请求已接受就返回成功。
- `drive`, `stop`, `estop`, `command` 传 `envelope` JSON 文本；`drive` 额外传 `received_at`（已扣除排队年龄的本地主机单调时间）。`disable_motion` 含 `connection_epoch`；`exit` 请求 ROS 适配进程退出。导航 command 与 reset 在生产返回 unavailable。后续安全 reset 必须带实时 estop revision/nonce，来自用户看到最新 fence 后发出的显式动作；每次 estop 都推进 fence，验证失败不能清锁或取得租约。

## 验证与未验收

- 配对：通过真实 ASGI 配对 API 测试一次性码兑换、权限 0600、已有文件拒绝覆盖和失败不留下半文件；CLI 测试确认交互读取配对码和目标路径。
- `.venv/bin/python -m pytest tests/unit tests/integration -q`：470 passed；相同范围加 `-W error::ResourceWarning -W error::pytest.PytestUnhandledThreadExceptionWarning -W error::pytest.PytestUnraisableExceptionWarning`：470 passed。`ruff check .`、`ruff format --check .` 均通过（159 文件）；`mypy src` strict：70 文件通过；`uv lock --check --offline` 与全 extras `uv sync --offline --frozen` 通过。`pytest tests/ros/test_velocity_chain.py tests/ros/test_navigation.py -q`：7 skipped（本机无 ROS），不视为验收。
- TDD RED 覆盖发送成功误续资格、重复进入数据库、跨重启 epoch、心跳身份/时钟、ROS heartbeat、半开、任务调度与重启扫描、多页面急停和旧 reset。实现后对应测试通过。
- 此前全 extras 严格 warnings 一次 464 passed；一次并行执行质量门时 `test_slow_encoder_does_not_delay_stop` 以 0.101 秒超过 0.1 秒门限，测试失败时未清理导致随后视频测试出现未释放任务告警，已补 finally。另一次为 465 passed、1 error，暴露视频编码循环未关闭，已修复并补回归；最终结果以以下复验为准。
- 仍需 Linux Humble/Python 3.10 全依赖重同步与 cv_bridge 导入、ROS 图真实节点、ROS 时钟冻结、设备撤销／重连、真实网络半开、任务终态/取消、地图事务、传感器、SLAM、Nav2、视频和受控实车联调。服务端多 worker 跨进程旧连接抢占尚未验证；数据库恢复到旧备份可能回退 epoch，需部署规定和更强代次防回退措施。

复审首轮修复及后续验证见同目录 [repair-wave2-fixround1-report.md](repair-wave2-fixround1-report.md)。
