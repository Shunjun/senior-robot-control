# 地图包

一个地图包是 zip。根上有 `manifest.json` 和 `waypoints.json`，稳定层在 `occupancy/occupancy.json`，可续建位姿图在 `pose_graph/`，栅格图在 `grid/map.pgm` 和 `grid/map.yaml`。YAML 写出 `image`、`resolution` 和 `origin`。没有 `pose_graph/` 或栅格图的包不能切换地图。包里没有任务，也没有车辆配置或凭据。

当前生产 fixed cut 的 `manifest.json.format_version` 为 `2`；保留的 V1 writer/parser 用于明确的旧档迁移。`file_hashes` 是除清单外每个成员的 SHA-256 十六进制摘要。`content_hash` 覆盖决定行为的清单字段，包括 `source_robot_id`、`map_ref`、航点、地理参考和 `startup_pose`，不把 `file_hashes` 自己算进去。改这些字段而不重算 `content_hash` 会被拒绝。航点里的 `source_robot_id` 还必须和清单一致。`backend_version` 是 `slam_toolbox 2.6.10`。同时写入的后端标记还有 `ros_version`（`humble`）、`slam_version`（`2.6.10`）和 `coordinate_convention`（`right_handed_xy_yaw`）。

`map_ref` 是这张图的身份：

- `map_id`
- `reference_revision`

`stable_layer` 固定为 `occupancy/occupancy.json`。这份文件是长期占据层，不是位姿图栅格。

`anchors` 的每一项有 `anchor_id`、`available`、`x`、`y`、`yaw`。没有可用位姿时三个坐标是 `null`，`available` 为 `false`。

没有地理参考时，`georeference` 只有 `{"status": "unreferenced"}`。有参考时才写入 `datum`、`height_datum`、`translation_m`、`rotation`、`heading_convention`、`quality`、`source` 里实际有值的字段。

`waypoints` 与 `waypoints.json` 的 `waypoints` 使用同一组字段：`waypoint_id`、`kind`、`x`、`y`、`yaw`、`map_id`、`reference_revision`、`anchor_id`、`source_robot_id`。回家点的 `kind` 是 `home`，其余是 `waypoint`。导入时每个航点的地图身份必须和 manifest 的 `map_ref` 一致。

`tasks_included` 固定为 `false`。任务定义留在机器人本地，不进包。

`startup_pose` 固定为 `null`。`last_pose_hint` 只是位姿图给出的上一姿态，形如 `{ "x", "y", "yaw" }`，没有则是 `null`。加载图时可以把这个提示交给后端，它不是已知的开机位姿，导入成功也不表示已经重定位。

`source_robot_id` 是导出这张图的机器人。另一台机器人导入后，包内回家点保持 `pending_confirmation`，在确认之前不算已经采纳。清除地图时，这条 Home 保留原来的地图、航点和来源，但不再处于采纳状态。

切换时先把当前位姿图和占据层备份到 history，再加载新图。history 和 active 都保留位姿图文件。文件替换之后会 fsync 该文件和父目录。加载失败时先按备份恢复会话和占据层，导航保持挡住，然后才把 `rollback_confirmed` 记为真。`last_pose_hint` 仍然不是已知开机位姿。

## V2 权威 cut（2026-11-26）

生产链路使用同一个 NativeSession / ActiveMap owner。`occupancy.json` 的 V2
信封包含完整 `fixed`（C3 watermark、绝对 O/F/U、局部 support/free tiles、legacy
mask、Home/航点和 profile 来源）、`odom_owner`、`pose_graph_prefix`、
`review_required`、可选的原始 V1 `legacy_audit_b64`，以及覆盖其余字段的 SHA-256
`checksum`。`fixed.localization_role` 固定为 `localization_only`：raw graph 只能定位，
不能把已撤掉的占据重新画回固定层。完整水位包括 graph sequence、anchor digest、
fixed/evidence revision、原 ScanKey 和 change_sequence；恢复的新会话保留历史证据和
单调 revision，但清空 live evidence_key，使用新 boot/source/R/floor，Q 不写入长存档。

checkpoint 的 `version` 同步为 2。`checkpoint.json` 保持不超过 4096 字节，记录稳定层
SHA-256 和有界 `members.json` 的 SHA-256；后者绑定所有 graph 成员。先验证完整成员、
版本、checksum 和 graph proof，再安装 cut；损坏 V2 不回退 V1 或 raw。原有
`durable_replace` / `durable_move_tree` / fsync 以及 catalog 最终指向顺序保留。

`load_snapshot` 显式分派 V1/V2。保留的 `SlamSnapshot.from_json/read` 只读 V1，
会拒绝 V2；新 native 会话只有在当前 catalog 指向实际 V1 cut 时才调用
`migrate_legacy`，允许一次标为 legacy-derived 的 native 三态输入。原始 binding 精度、
removed 和 Home 的完整 V1 字节保存在审计成员中；原档不覆盖。生产 checkpoint 迁移还
记录原权威成员清单 digest，独立 helper 无外层档案时使用原稳定层字节 digest。
这些是历史来源，不是新的物理判稳证明。

V2 PGM 逐格编码为 O=0、F=254、U=205；行号 `row` 对应 dense 的
`y=height-1-row`。`map.yaml` 使用该 cut 的 resolution/origin、`negate: 0`、
`occupied_thresh: 0.65`、`free_thresh: 0.196`。不能直接把 dense free=0 当作黑像素。
workspace、选图恢复和导出均读取同一权威 cut，生产 `map.export` 从管理 owner 的已验证、
固定租约 checkpoint 导出到原 `map-transfer` 目录，不再路由到不支持导出的 native RPC。

V1 writer 保留用于旧安装/迁移 fixture；它不是新 fixed 生产写入路径。Python 集成验证
不能替代 ROS adapter/native 三进程、Nav2 StaticLayer 消费或物理 profile 取证。
