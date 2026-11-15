# 云端 HTTP 与 WebSocket 协议

范围：Task 13 的单管理员认证、设备配对和多车中转。云端是单进程、单 worker。

## 启动

`create_app(settings, *, database, admin_password_hash, allowed_origins, limits=None)` 是同步工厂，调用时不打开数据库、不启动任何任务。ASGI lifespan 才会打开未打开的 `Database`，并运行迁移 1–3（机器人域表）加云端迁移 4（`cloud_session`、`cloud_pairing_code`、`cloud_robot`）。云端使用自己的数据库文件。

`admin_password_hash` 由 `senior_control.cloud.auth.hash_password` 生成（`scrypt$n$r$p$salt$hash`），格式不合法时 `create_app` 抛 `ValueError`。系统没有注册路由，管理员只能由配置产生。

数据库只保存哈希：会话令牌、CSRF 令牌、配对码、设备令牌都以 `settings.secret` 派生的 HMAC-SHA256 摘要保存。

## 显式限额（`CloudLimits`）

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `login_failures_per_client` / `login_failures_global` | 5 / 30 | 窗口内登录失败上限，按客户端地址与全局 |
| `pair_failures_per_client` / `pair_failures_global` | 10 / 60 | 配对失败上限 |
| `failure_window_seconds` | 300 | 失败计数窗口 |
| `tracked_clients` | 4096 | 限流器记录的客户端上限 |
| `session_ttl_seconds` | 28800 | 会话有效期 |
| `pairing_code_ttl_seconds` / `max_live_pairing_codes` | 600 / 20 | 配对码有效期与未兑换数量 |
| `max_body_bytes` | 4096 | HTTP 请求体上限，声明长度和流式读取都检查 |
| `max_ws_message_bytes` | 65536 | WebSocket 单帧上限，等于 `protocol.MAX_MESSAGE_BYTES` |
| `outbox_frames` | 32 | 每个 socket 的发送队列上限 |
| `max_controllers_per_robot` | 8 | 同一机器人的浏览器连接上限 |

限流状态保存在进程内存中，进程重启后清零。

## HTTP

错误体统一为 `{"detail": "..."}`。

| 方法与路径 | 认证 | 成功 | 失败 |
| --- | --- | --- | --- |
| `POST /api/session` `{"password"}` | 无 | 200 `{"csrf_token"}`，设置会话 Cookie | 401 密码错误；429 被限流并带 `Retry-After`；403 Origin 不在允许列表 |
| `DELETE /api/session` | 会话 + CSRF | 204，清除 Cookie，关闭该会话的控制 socket | 401、403 |
| `POST /api/pairing-codes` | 会话 + CSRF | 201 `{"code","expires_in"}` | 401、403、429 未兑换的码太多 |
| `POST /api/devices/pair` `{"code","name"}` | 配对码 | 200 `{"robot_id","device_token"}` | 409 码无效、过期或已用；429 被限流 |
| `GET /api/robots` | 会话 | 200 `{"robots":[{"id","name","online","reconnecting"}]}` | 401 |
| `DELETE /api/robots/{id}` | 会话 + CSRF | 204，设备凭据作废，关闭该机器人的所有 socket | 401、403、404 |

- 会话 Cookie 名为 `__Host-session`，属性为 `Secure; HttpOnly; SameSite=Strict; Path=/`。
- 写操作（`POST`、`DELETE`，配对兑换除外）必须带 `X-CSRF-Token`，值为登录响应中的 `csrf_token`。请求若带 `Origin` 头，必须在允许列表内。
- 配对兑换用一条带条件的 `UPDATE ... WHERE redeemed_at IS NULL` 并在同一事务内插入机器人行。两个并发兑换只有一个成功，另一个得到 409。
- 请求体超过 `max_body_bytes` 返回 413。
- `online` 表示当前代次的连接仍打开，且云端在过去 2 秒内接受过该代次的心跳。已在线设备失联后，`online` 立即为 `false`，`reconnecting` 在 60 秒宽限内为 `true`。收到新代次心跳立即恢复在线，超过宽限仍无心跳则显示离线。云端无法观察未抵达的尝试。60 秒覆盖默认的前五段退避（1、2、4、8、16 秒）、五次各 5 秒的握手超时以及心跳余量。失败但未完成首次心跳的连接不会延长宽限。

## WebSocket

所有帧都是 `senior_control.protocol` 的版本化 `Envelope` JSON 文本，单帧不超过 `max_ws_message_bytes`。

### `/ws/agent`

- 认证：`Authorization: Bearer <device_token>`。会话 Cookie 不能打开此 socket。令牌缺失、未知或已作废时拒绝握手。已建立连接的设备凭据被撤销时，云端以 4403 关闭连接。Agent 将握手 HTTP 401/403 或关闭码 4403 视为认证失败。
- 同一机器人的第 n 个存活 agent socket 拥有 epoch n，握手响应头 `x-connection-epoch` 返回该值。新 socket 连上时旧 socket 被关闭（4000），旧 socket 之后的帧丢弃。
- Agent 每秒发送一个 `heartbeat.request`，云端回复 `heartbeat.reply` 并回显机器人、连接代次、随机值和发送时间。心跳间隔、回复超时和宽限均用单调时钟计时。Agent 在 1 秒内未收到匹配回复会关闭连接并重试。500 ms 往返可以通过在线探测，驾驶资格仍取决于命令时钟、有效期和 ROS 确认。
- Agent 可发送心跳、状态快照、操作响应及视频信令。消息必须属于自己的机器人及当前连接代次。旧 socket 的迟到帧不改变当前连接的状态。

### `/ws/control?robot_id=<id>`

- 认证：会话 Cookie 与 `Origin` 头。`Origin` 缺失或不在允许列表时握手前关闭（1008）。设备令牌不能打开此 socket。机器人未知或已作废、浏览器连接数已满（1013）同样拒绝。
- 浏览器帧发给 agent 前的检查：
  - 帧不合法或超限：关闭连接（1007 / 1009）。
  - `robot_id` 与连接的机器人不同，或帧类型是 `response`：回 `ResponseEnvelope`，`ok=false`，`error=validation`，不转发。
  - 一般命令已过期（`expires_at`）：回 `error=timeout`。手动意图按车端本地 TTL 验证；控制准备、释放和停止不依赖浏览器时钟。
  - 机器人没有在线 agent：回 `error=offline`。
  - agent 发送队列已满：回 `error=conflict`。
  - 其余帧原样转发给当前 epoch 的 agent。
- `drive.intent` 是单向流：上列业务拒绝直接丢弃样本，不发逐条错误或成功回执；非法帧仍按协议关闭连接。
- 拒绝响应回显原帧的 `robot_id`、`session_id`、`connection_epoch`、`request_id`、`seq`。

### 手动意图与急停

- 云端在控制连接建立、Agent更换或离线时独立发送 `control.connection`：`v=1`、`type=control.connection`、`robot_id`、`connection_epoch`（离线为0），无运行报告也可建立来源身份。该身份通知与运行状态快照分离。
- 浏览器通过 `control.acquire` 准备车端手动 authority；`drive.intent` 只发送 authority_id、gesture、sequence、linear、turn 和 gear。车端在本地收包时绑定 TTL，过期或退役 authority 的迟到数据不能复活。
- 云端认证并按原始 socket 分配 controller_generation，获取/释放等控制事件的回执按原 socket/会话/请求关联，意图没有逐条回执。浏览器不能提交可信来源代次。断开控制 socket 时清除待发意图并向车端发送该来源的 release。
- 浏览器时钟不参与驾驶资格。心跳用于连接活性与状态；自主任务继续使用自己的执行代次及本地 Nav2 样本 TTL，断网不取消已接受任务。
- `safety.estop` 可在浏览器还未收到状态快照时发送，使用已认证的当前车端代次。软件急停仅由最终输出侧 owner 在当前进程内锁存，命令回执表示请求受理，实际状态由新鲜 `/motion/runtime` 单向发布；完整启动不读取或写入旧状态文件，硬件状态不参与软件锁存或复位许可，恢复使用带 revision 的 `safety.estop.reset` / `safety.fault.recover`。

### 有界队列

每个 socket 有 `outbox_frames` 个槽位。`drive.intent` 只保留最新待发送帧，替换直接丢弃旧值，不生成运输或执行回执。网页按100ms持续发送当前意图，不等待逐条响应；socket已有积压时丢弃本次样本并在下周期采样。Agent及单向 `manual_intent` IPC 接收各保留一个最新待交接值，本地原始接收时间与300ms断流期限不因排队重置，停发仍会使旧目标到期。急停和停止优先，视频最先让出容量；过载请求拒绝并回 `conflict`，无法继续发送的 socket 关闭（1013）。

浏览器到 ROS 底盘的同版运行验收仍需目标环境执行。

## Agent 运行时（`senior_control.runtime.supervisor`）

`Supervisor` 是异步上下文管理器，构造时不启动任务。它拥有独立心跳、后台任务和上行重连循环，不导入 ROS，不传递速度样本。

- `request_ros(name)` 调用注入的 `call_ros`，慢调用不阻塞心跳。
- 后台任务失败会通知 Agent 收束连接和本地 authority。
- 认证拒绝停止重试，其他连接错误按有界指数退避重连。

### 运行状态报告与运动证据

`vehicle_ui` 是持续状态报告，不以运动许可为发送条件。ROS 发布器继续按控制周期发布运行状态（含急停、故障）；CommandNode 将最新未撤销的可信样本放入 IPC `motion_runtime`，额外返回只读 `runtime_observation`。Agent 即使未收到运行状态或 IPC 不可用，也发送 UI 帧。云沿现有 snapshot 通道转发，既有 controller 身份隔离不变。

UI 增加 `observation`：`status` 为 `current / missing / revoked / ipc_unavailable`，`last_sampled_steady` 为最后可信接收样本的原始单调采样时间（没有则 null），`age_seconds` 为它距生成 UI 帧的年龄。该时间不是 UTC，不用于浏览器时钟计算。网页从帧接收时刻继续累加年龄。IPC 断开期间 Agent 保留该历史时间，不能据此恢复许可；从未收到可信样本时明确报告 missing。

状态不可观测时 `mode` 和 `confirmed_estop_status` 为 `unknown`，速度及 manual authority 为 null，`allowed_actions` 为空；不得用零速、静止或急停已解除替代未知。能力列表描述接口支持，不代表运动许可；设置和日志仍可独立提供。任务投影继续由任务生命周期管理。历史速度只可按原有 stale / 最后测量标记展示，不能成为操作证据。

运行状态不再具有固定150ms `valid_until`，报告的 sequence、年龄、缺失或健康情况不作为手动命令准入。网页手动入口只使用独立连接身份，保留车辆/来源/序号与authority隔离；恢复事件仍携带车端revision，最终输出控制循环判断实际运动及复位条件。原始运动证据期限和来源撤销仍由对应车端职责管理。状态历史与未知的展示不得被解释成车辆已停止或已解除急停。

该版本网页、云端、Agent和ROS适配器需要同版部署，不保留旧逐条意图回执链。状态报告不会替代通信身份通知，也不会续期本地运动目标。

软件急停状态仅为 `clear` / `software_latched`（展示断流可为 `unknown`）。退役的 `hardware_latched` 或硬件观察字段不是操作许可，不接入软件急停；旧格式状态包拒绝解析。重复请求保留请求级幂等，复位后不重放旧请求；Agent 的在途请求和 CommandNode 的交付确认不是第二份急停状态。
