# 地图包

一个地图包是 zip。根上有 `manifest.json` 和 `waypoints.json`，稳定层在 `occupancy/occupancy.json`，可续建位姿图在 `pose_graph/`，栅格图在 `grid/map.pgm` 和 `grid/map.yaml`。YAML 写出 `image`、`resolution` 和 `origin`。没有 `pose_graph/` 或栅格图的包不能切换地图。包里没有任务，也没有车辆配置或凭据。

`manifest.json` 的 `format_version` 是 `1`。`file_hashes` 是除清单外每个成员的 SHA-256 十六进制摘要。`content_hash` 覆盖决定行为的清单字段，包括 `source_robot_id`、`map_ref`、航点、地理参考和 `startup_pose`，不把 `file_hashes` 自己算进去。改这些字段而不重算 `content_hash` 会被拒绝。航点里的 `source_robot_id` 还必须和清单一致。`backend_version` 是 `slam_toolbox 2.6.10`。同时写入的后端标记还有 `ros_version`（`humble`）、`slam_version`（`2.6.10`）和 `coordinate_convention`（`right_handed_xy_yaw`）。

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
