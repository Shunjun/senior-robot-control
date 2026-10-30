# 验收矩阵

任务 21 把可在开发机复核的控制逻辑与必须在 Linux 仿真栈、浏览器或实车上执行的检查分开。场景文件只是测试夹具，验收人员自行启动所需进程。

| 范围 | 验收点 | 自动化入口 | 所需环境 |
| --- | --- | --- | --- |
| 驾驶安全链 | 只有 `safety_node` 发布 `/cmd_vel`，停发后停车 | `tests/e2e/test_drive.py`、`tests/ros/test_velocity_chain.py` | Linux、Humble、运行中的完整仿真栈 |
| 后部遮挡 | 倒车限速、限距，并要求地图自由走廊；实时障碍优先 | `tests/e2e/test_recovery.py` | Python；实际运动另需 Linux 仿真 |
| 长期地图 | 箱体确认、清除后重载仍为空；人不写入 | `tests/e2e/test_maps.py` | Python |
| 任务断线 | 运行中暂停，重连不恢复、不补跑 | `tests/e2e/test_tasks.py` | Python |
| Home | 数据库重开后仍使用原 Home | `tests/e2e/test_home.py` | Python |
| GPS/航向 | 只产生定位信息，不发布速度 | `tests/e2e/test_gps.py` | Python；ROS 话题复核需 Linux |
| 地图导入 | 无 GPS 可用人工提示；有 GPS 也须匹配确认；跨车 Home 待确认 | `tests/e2e/test_maps.py`、`tests/integration/test_map_operations.py::test_cross_robot_home_requires_confirmation` | Python |
| 浏览器驾驶 | 显式 `base_url`，桌面与手机横竖屏，真实仿真控制链 | `web/e2e/control.spec.ts` | 已运行的云端、网页和仿真栈、Chromium |

## Linux 复现

先按 `README.md` 启动 Gazebo、雷达适配、安全门、仲裁、导航、建图、云端和车端代理。四个新增世界仅作为独立场景使用，不由测试启动：

```sh
python3 -m pytest tests/ros tests/e2e --simulation-profile config/simulation.yaml -q
```

`--simulation-profile` 为每次测试分配临时 SQLite 数据库、两个互异的保留端口，并固定场景 seed `210202610`。缺少 Linux、ROS 或运行中的仿真栈时，相应测试跳过，不能计为通过。

浏览器检查不启动 Vite，也不安装浏览器：

```sh
PLAYWRIGHT_BASE_URL=https://control.example.test \
SENIOR_CONTROL_PASSWORD='从安全渠道读取' \
npm --prefix web run test:e2e -- --project=chromium
```

当前 `web/package.json` 尚未提供 `test:e2e` 脚本和 Playwright 直接依赖，因此该命令在依赖补齐前属于未执行项。

## 故障注入入口

以下注入要在同一完整 Linux 栈中记录开始、故障发生、停止确认和恢复后的状态。开发机测试只覆盖其纯逻辑护栏，不等同于全链路注入。

- 半开连接：停止心跳但保持 TCP；参考 `tests/unit/test_arbiter.py::test_half_open_connection_expires`。
- 进程重启：在已接受操作后重启车端代理；参考 `tests/integration/test_task_runner.py::test_crash_after_dispatch_never_redispatches_unknown`。
- 时钟暂停与回退：暂停 `/clock` 后回退；参考 `tests/ros/test_velocity_chain.py` 和 `tests/unit/test_schedule.py::test_backward_clock_jump_skips_without_dispatch`。
- 损坏上传：篡改地图包成员但保留旧哈希；参考 `tests/integration/test_map_operations.py::test_corrupt_bundle_preserves_active_map`。
- GPS 跳点：注入 jump 标记并保持本地定位健康；参考 `tests/unit/test_localization.py::test_reverse_curve_slip_jump_rejected`。
- 感知过期：停止新扫描直至超过最大年龄；参考 `tests/unit/test_guard.py::test_stale_sensor_stops`。
- 视频拥塞：限制发送缓冲并确认控制心跳继续；参考 `tests/integration/test_video.py` 的 congestion 测试组。
