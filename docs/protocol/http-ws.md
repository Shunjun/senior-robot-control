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
| `GET /api/robots` | 会话 | 200 `{"robots":[{"id","name","online"}]}` | 401 |
| `DELETE /api/robots/{id}` | 会话 + CSRF | 204，设备凭据作废，关闭该机器人的所有 socket | 401、403、404 |

- 会话 Cookie 名为 `__Host-session`，属性为 `Secure; HttpOnly; SameSite=Strict; Path=/`。
- 写操作（`POST`、`DELETE`，配对兑换除外）必须带 `X-CSRF-Token`，值为登录响应中的 `csrf_token`。请求若带 `Origin` 头，必须在允许列表内。
- 配对兑换用一条带条件的 `UPDATE ... WHERE redeemed_at IS NULL` 并在同一事务内插入机器人行。两个并发兑换只有一个成功，另一个得到 409。
- 请求体超过 `max_body_bytes` 返回 413。

## WebSocket

所有帧都是 `senior_control.protocol` 的版本化 `Envelope` JSON 文本，单帧不超过 `max_ws_message_bytes`。

### `/ws/agent`

- 认证：`Authorization: Bearer <device_token>`。会话 Cookie 不能打开此 socket。令牌缺失、未知或已作废时握手前关闭（1008）。
- 同一机器人的第 n 个存活 agent socket 拥有 epoch n，握手响应头 `x-connection-epoch` 返回该值。新 socket 连上时旧 socket 被关闭（4000），旧 socket 之后的帧丢弃。
- agent 只能发送 `response` 帧，`robot_id` 必须是自己的机器人，`connection_epoch` 必须等于自己的 epoch。epoch 不符的帧丢弃；非 `response` 帧或非法帧关闭连接（1008 / 1007）。合法帧转发给该机器人所有浏览器 socket。

### `/ws/control?robot_id=<id>`

- 认证：会话 Cookie 与 `Origin` 头。`Origin` 缺失或不在允许列表时握手前关闭（1008）。设备令牌不能打开此 socket。机器人未知或已作废、浏览器连接数已满（1013）同样拒绝。
- 浏览器帧发给 agent 前的检查：
  - 帧不合法或超限：关闭连接（1007 / 1009）。
  - `robot_id` 与连接的机器人不同，或帧类型是 `response`：回 `ResponseEnvelope`，`ok=false`，`error=validation`，不转发。
  - 已过期（`expires_at`）：回 `error=timeout`。
  - 机器人没有在线 agent：回 `error=offline`。
  - agent 发送队列已满：回 `error=conflict`。
  - 其余帧原样转发给当前 epoch 的 agent。
- 拒绝响应回显原帧的 `robot_id`、`session_id`、`connection_epoch`、`request_id`、`seq`。

### 有界队列

每个 socket 有 `outbox_frames` 个槽位的发送队列。`drive.velocity` 只保留最新一帧；队列满时先丢旧的速度帧，`drive.stop` 与 `safety.estop` 可挤掉其他非安全帧，其余帧被拒绝并回 `conflict`。接收方太慢导致队列溢出的浏览器 socket 会被关闭（1013）。

链路存活由 WebSocket 层的 ping 负责，由进程入口（Task 20）通过服务器的 ping 与最大消息大小配置设置。

## Agent 运行时（`senior_control.runtime.supervisor`）

`Supervisor` 是异步上下文管理器，构造时不启动任何东西。它拥有心跳任务、后台任务和上行重连循环，不导入 ROS，也从不调用 `publish_cmd_vel`。

- `request_ros(name)` 调用注入的 `call_ros`，心跳在独立任务中运行，不被慢调用阻塞。
- `submit_drive(vx, wz)` 同步，只有一个待发送槽：新的提交（包括 `(0, 0)` 停车）直接替换槽里尚未发出的值，不排队。已经进入 `drive_sink` 的那一次调用可以完成，被替换的旧值不会再发出。运动被撤销后返回 `False`。
- 任一后台任务抛错或提前结束，运动被撤销并清空队列。`drive_sink` 抛错同样撤销。
- `connect` 抛 `AuthRejected` 时状态变为 `auth_failed`，撤销运动，不再重试。其他异常以指数退避重试（`retry_delay` 起，上限 `max_retry_delay`）。
