# 地图包

一个地图包是 zip。根上有 `manifest.json` 和 `waypoints.json`，稳定层在 `occupancy/occupancy.json`，位姿图在 `pose_graph/`。包里没有任务，也没有车辆配置或凭据。

`manifest.json` 的 `format_version` 是 `1`。`file_hashes` 是除 `manifest.json` 以外每个成员的 SHA-256 十六进制摘要，键是包内相对路径。`backend_version` 是 `slam_toolbox 2.6.10`。同时写入的后端标记还有 `ros_version`（`humble`）、`slam_version`（`2.6.10`）和 `coordinate_convention`（`right_handed_xy_yaw`）。

`map_ref` 是这张图的身份：

- `map_id`
- `reference_revision`

`stable_layer` 固定为 `occupancy/occupancy.json`。这份文件是长期占据层，不是位姿图栅格。

`anchors` 的每一项有 `anchor_id`、`available`、`x`、`y`、`yaw`。没有可用位姿时三个坐标是 `null`，`available` 为 `false`。

没有地理参考时，`georeference` 只有 `{"status": "unreferenced"}`。有参考时才写入 `datum`、`height_datum`、`translation_m`、`rotation`、`heading_convention`、`quality`、`source` 里实际有值的字段。

`waypoints` 与 `waypoints.json` 的 `waypoints` 使用同一组字段：`waypoint_id`、`kind`、`x`、`y`、`yaw`、`map_id`、`reference_revision`、`anchor_id`、`source_robot_id`。回家点的 `kind` 是 `home`，其余是 `waypoint`。导入时每个航点的地图身份必须和 manifest 的 `map_ref` 一致。

`tasks_included` 固定为 `false`。任务定义留在机器人本地，不进包。

`startup_pose` 固定为 `null`。`last_pose_hint` 只是位姿图给出的上一姿态，形如 `{ "x", "y", "yaw" }`，没有则是 `null`。加载图时可以把这个提示交给后端，它不是已知的开机位姿，导入成功也不表示已经重定位。

`source_robot_id` 是导出这张图的机器人。另一台机器人导入后，包内回家点保持 `pending_confirmation`，在确认之前不算已经采纳。
