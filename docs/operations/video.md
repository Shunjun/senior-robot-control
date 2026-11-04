# 车端视频：已认证的 WebRTC 与 TURN

本文说明视频会话的边界、TURN 凭据、状态含义，以及跨 NAT 的 relay-only 验收步骤。这台 macOS 没有启动 coturn、浏览器、ROS 或云端服务。`deploy/coturn/turnserver.conf.template` 是模板，不是运行中的服务。验收步骤需要 Linux 上的 coturn 和两个不同 NAT 后的真实客户端，目前都没有执行。

## 边界

- 信令复用已认证的 `/ws/agent` 和 `/ws/control`。不开独立的视频端口，也没有未认证入口。
- `VideoSession` 在车端按单个观看者创建，构造时带上云端已验证的连接身份（`role` 和所选 `robot_id`）。每次 `offer`、`add_ice_candidate`、`turn_credentials` 都会重新核对：连接必须是 `control` 角色，请求的机器人必须等于所选机器人，也必须等于本车。
- 视频帧不经过 `Supervisor.submit_drive`，也不占那个唯一的待发速度槽。帧的路径是：相机 → 一个只保留最新帧的槽 → 对端。对端拥塞时丢帧并计数，心跳和停止不受影响。
- 车上没有相机时，会话报告 `unavailable`（原因 `no_camera`），不生成帧，不显示假画面。`perception_node` 目前不发布相机话题，所以现在车端应传 `source=None`。
- `aiortc` 在可选依赖 `video` 里。`import senior_control` 不会导入它。只有 `AiortcPeerFactory.open` 才导入，缺少时报告 `aiortc_not_installed`。安装：`uv sync --group dev --extra video`。

## 信令消息

下面是会话方法对应的消息，信令载体由已认证的 WebSocket 提供：

| 方向 | 内容 | 会话方法 |
| --- | --- | --- |
| 控制端 → 车端 | 请求 ICE 服务器 | `turn_credentials(now)` |
| 控制端 → 车端 | SDP offer | `offer(robot_id, SessionDescription)`，返回 answer |
| 控制端 → 车端 | 追加的 ICE candidate（trickle） | `add_ice_candidate(candidate)` |
| 车端 → 控制端 | 状态 | `status()` |
| 任一端 | 结束 | `close()` |

`/ws/control` 与 `/ws/agent` 的信封目前只允许速度、控制、操作、快照和响应这些类型，信令消息还没有对应的信封类型。把这些消息装进信封并经 `Hub` 转发，需要改 `protocol.py` 和 `cloud/relay.py`，也需要具体的车端 WebSocket 客户端，二者都不在本任务范围内，保留为后续工作。

`aiortc` 的限制：它在 `setLocalDescription` 返回前收集完本地 candidate，并放进 answer，所以车端本地 candidate 随 answer 一次给出，不再逐条 trickle。控制端到车端的 candidate 可以逐条追加。

## TURN 凭据

`senior_control.vision.turn` 使用 coturn 的 REST 临时凭据方案：

```text
username   = "<过期时间戳>:<session_id>"
credential = base64(HMAC-SHA1(共享密钥, username))
```

- `TurnConfig` 要求至少一个 `turn:` 或 `turns:` 地址、至少 32 字节共享密钥、有效期不超过 3600 秒。默认 600 秒。
- `session_id` 是 `VideoSession` 生成的随机不透明值，不含登录会话令牌。coturn 日志里只会出现它。
- `verify_turn_credentials` 复现 coturn 的检查：格式正确、`now` 小于过期时间、HMAC 一致。到点即失效，篡改过期时间或换密钥都失败。
- 共享密钥只存在于云端（或签发方）和 coturn 主机。模板里是占位符 `__TURN_SHARED_SECRET__`，部署时替换，不提交真实值。
- 签发的凭据对象不在 `repr` 中显示 `credential`。

## 状态

`VideoSession.status()` 返回真实测得的值，不伪造视频。

| 状态 | 含义 |
| --- | --- |
| `idle` | 有相机，还没有协商 |
| `unavailable` | 没有相机（`reason=no_camera`） |
| `connecting` | 已协商，连接还没建立 |
| `live` | 连接已建立，最近一帧在 `max_frame_age_s` 内，并且最近一次推送也在该时间内 |
| `stalled` | 连接已建立，但没有新鲜帧或推送卡住（`reason=no_fresh_frames`） |
| `relay_failed` | relay-only 模式下 ICE 失败，TURN 不通或凭据无效 |
| `ice_failed` | 非 relay-only 模式下 ICE 失败 |
| `closed` | 会话已关闭 |

字段 `frame_age_s` 是当前时间减去最近一帧的时间戳，没有收到过帧时为空。另有 `frames_sent`、`frames_dropped`、`buffered_frames`（最多 1）、`bitrate_bps` 和 `codec`。超过 `max_frame_age_s` 的陈旧帧丢弃并计数，不当作实时画面推送。

## 限制

`VideoLimits` 的每个界限都是命名字段：

- `max_bitrate_bps`：默认 1 500 000。`AiortcPeerFactory` 通过调低 `aiortc.codecs.vpx` 和 `aiortc.codecs.h264` 的 `MAX_BITRATE` 实现，这是进程级设置，只降不升。
- `max_sdp_bytes`（16 KiB）、`max_candidates`（64），单条 candidate 最长 512 字符。
- `ViewerBudget(maximum)`：多个会话共享的观看者上限。每个会话同一时间只能有一个对端，重复 `offer` 返回 `VideoBusy`。
- 编解码：从客户端 offer 的视频段里取它实际声明的 codec，按车端偏好 `VP8`、`H264` 选第一个。没有交集时返回 `NoCommonCodec`，不开编码器。answer 只保留选中的 codec。
- `close()` 幂等，会取消泵任务、关闭对端释放编码器、归还观看者名额。协商失败也会释放这些资源。

## relay-only 跨 NAT 验收

这些步骤需要 Linux 主机上的 coturn 和两个不同 NAT 后的真实端点，还没有执行。

1. 在 Linux 公网主机上用模板生成 `turnserver.conf`，替换所有 `__NAME__` 占位符。共享密钥用 `head -c 48 /dev/urandom | base64` 之类的随机数据，不要写进仓库。开放 UDP/TCP 3478、TCP 5349 和 UDP 49160-49200。启动 coturn，确认 `turns:` 证书有效。
2. 云端和车端用同一个密钥构造 `TurnConfig`，会话以 `relay_only=True` 创建。
3. 车端在家庭 NAT 后，控制端浏览器在另一个 NAT（例如手机热点）后。浏览器的 `RTCPeerConnection` 配置 `iceTransportPolicy: "relay"`，`iceServers` 用 `turn_credentials` 返回的值。
4. 完成 offer、answer 和 candidate 交换。通过标准：
   - `status().state == "live"`，`frame_age_s` 小于 1 秒，`frames_sent` 持续增加；
   - 浏览器 `getStats()` 中被选中的 candidate pair，本地和远端 candidate 的 `candidateType` 都是 `relay`（`chrome://webrtc-internals` 也能看到）；
   - 抓包或 coturn 日志里没有两端之间的直连 UDP。
5. 过期：把系统时间推进到凭据过期之后（或等待 TTL），新建分配被 coturn 拒绝，状态进入 `relay_failed`。重新调用 `turn_credentials` 得到的新凭据可恢复。
6. 故障：停掉 coturn，`status().state` 变为 `relay_failed`，页面显示“中继不可用”，不显示旧画面。
7. 带宽：用 `VideoLimits.max_bitrate_bps` 的两倍压力观察，`bitrate_bps` 不超过上限。
8. 拥塞：用 `tc netem` 限制上行带宽，确认心跳不中断，`drive.stop` 在一个心跳周期内到达车端。
9. 吊销：信令接入 `Hub` 之后（见“信令消息”一节的后续工作），在云端撤销机器人或登出，确认控制连接关闭会调用会话的 `close()`，`frames_sent` 不再增加，coturn 上对应分配在 TTL 内释放。目前 `VideoSession` 没有接到 `Hub` 的关闭事件，这一步还不能验收。

模板里的 `denied-peer-ip` 禁止中继到私有地址。如果验收时车端和 coturn 在同一个内网，需要按实际网络临时放开对应网段，并在验收后恢复。
