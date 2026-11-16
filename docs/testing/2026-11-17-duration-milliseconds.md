# 时长毫秒迁移审阅包（前任 ctx_021291 + 本轮 ctx_732f7dad）

分支 `fix/degraded-drive-speed`，HEAD `593e8f13f385892f0bd62cf75c593f7b07efd2da`。未建分支，未暂存，未提交，未推送，未启动 ROS、仿真或车辆。

这是从迁移前正文到当前工作区的整项内容增量，覆盖前任与本轮。`/private/tmp/senior-ms-takeover-ctx2f5ee105/round-report.md` 只覆盖接管后的 31 个文件，不能当作整项迁移完成。

## 基线与不可恢复的拆分

迁移前正文与 SHA 在 `/private/tmp/senior-ms-ctx021291/before/` 和 `before-sha.json`。本轮继续改代码前，正文与 SHA 另存于 `/private/tmp/senior-ms-rework-ctx732f7dad/`。

接管快照 `/private/tmp/senior-ms-takeover-ctx2f5ee105/` 只有 SHA，没有正文。已经在接管时变脏的文件，无法把前任 hunk 与本轮 hunk 逐行拆开。这里不编造那次拆分。可审的是下面这份合并 diff 和逐文件最终 SHA。

- 内容 diff：`docs/testing/2026-11-17-duration-milliseconds-owned.diff`（188840 字节）
- 指纹：`docs/testing/2026-11-17-duration-milliseconds-fingerprints.json`
- 命令与原始结果：`docs/testing/2026-11-17-duration-milliseconds-commands.txt`
- JUnit / pytest / ruff / mypy：`docs/testing/2026-11-17-duration-milliseconds-evidence/`

内容有变的文件 115 个。每个文件的 `before_sha256` 与 `final_sha256` 在指纹的 `owned_content_files`。三份保护文件相对迁移前正文未改：

| 文件 | SHA256 |
| --- | --- |
| `ros2/src/senior_robot/senior_robot/native_receipt.py` | `53d08230fb94dce972ed72dbf6fb16532d3b34aea4e5ed7191772a9bd0ef910a` |
| `tests/test_native_receipt_diagnostics.py` | `5c33cba2555e0c0b9b0096a025620fbfdb9ef86ef03bfee53e485dc43c440408` |
| `docs/reviews/2026-11-16-software-estop-owner.md` | `cf258266704168979f6485658e6c8e08d18bd444e6a1175cfc2f848ff5cab4d1` |

`native_receipt.py` 继续读取 `default_policy().hard_age_s`。该属性是 `hard_age_ms / 1000`，配置不接受 `hard_age_s`。

## 不归入本迁移功能的增量

43 个文件只发生 CRLF 到 LF，归一化后正文相同。名单在指纹 `newline_only_excluded`，不进入 owned diff，也没有回改。

故障隔离指纹 `docs/testing/2026-11-16-runtime-fault-isolation-fingerprints.json` 的 19 个文件，迁移前正文 SHA 全部等于该项最终 SHA。功能实现已经在迁移起点里。其后进入 owned diff 的只有毫秒参数后续修改：

- `src/senior_control/agent/composition.py`
- `src/senior_control/agent/main.py`
- `src/senior_control/docker_start_helper.py`
- `src/senior_control/runtime/ipc.py`
- `src/senior_control/runtime/supervisor.py`
- `tests/integration/test_runtime.py`
- `tests/integration/test_runtime_fault_isolation.py`
- `tests/integration/test_shutdown.py`
- `tests/integration/test_task_ownership.py`

其余 10 个故障隔离文件，包括两份 launch，当前 SHA 仍等于该项最终 SHA。对照全部内容 hunk 后，没有另一组“收尾 3 文件”的非毫秒功能差可以单独归出来。三份保护文件与这 19 个文件是不同集合。

心跳指纹里 13 个文件的迁移前正文已等于心跳交付 SHA；`main.py`、`supervisor.py`、`test_runtime.py`、`test_shutdown.py`、`test_task_ownership.py` 的迁移前正文已经新于心跳指纹，因为故障隔离在心跳之后、毫秒迁移之前进入了正文。学习面评审 22 个文件的迁移前正文已等于该评审最终 SHA。这三项功能不记成毫秒迁移新做的功能。现有回归结果见下面的 pytest。

## 契约收尾

`RETIRED_PARAMETERS` 仍是读入前剥离的旧键名单，注释写明它们不是生效消费者。名单保留 `command_ttl_seconds`、`disconnect_seconds`、`heartbeat_seconds`、`scan_max_age`、`max_age_seconds`。生效默认仍是 `command_node.command_ttl_ms=300`、`scan_motion_node.max_interval_ms=400`。

产品 URDF `ros2/src/senior_sim/urdf/senior.urdf.xacro` 与迁移前正文相同，SHA256 `2082281fbdfbdb2281a226fcc111de2351df0191facb061eabf56e5742b99cd4`。插件 remap 保持：

- `/drive/actuator_target:=/control/actuator_target`
- `/drive/odom:=/wheel/odom`
- `/drive/odom/timed:=/wheel/odom/timed`

`tests/unit/test_sim_contract.py` 的 `test_drive_remapping_uses_humble_plugin_configuration` 改为断言这三项，并断言 remap 文本不含 `cmd_vel` 或 `Twist`，launch 不含 `--controller-ros-args`。URDF 没有改回旧 Twist 路径。`controllers.yaml` 的 `cmd_vel_timeout: 0.3` 是控制器超时字段，仍保留。

`tools/vehicle_motion_fixture.py` 原先只替换 `/drive/cmd_vel_unstamped:=/cmd_vel`，对当前 URDF 会因找不到该行而失败。现改为把唯一执行 remap `/drive/actuator_target:=/control/actuator_target` 改到计划里的隔离 topic，里程计两条 remap 原样留下。产品 URDF 不被改写。对应现有测试已按这个契约更新。

`NavigationReferenceConfig.max_age_s` 上限 5 秒，字段保持秒。`hardware_reference.py` 曾改成不存在的 `config.max_age_ms`，mypy 报 `attr-defined`。现已改回：位姿期限直接比较 `max_age_s`；`sensor_current` 的毫秒参数接收 `max_age_s * 1000` 一次。

只用于比较两个毫秒输入的校验，不再先除 1000 再比：语义协调器、`qualify_registration`、`SemanticSynchronizer` 的入参检查、`RawCaptureAdmission`、`EgressAdmission`、`DepthAdmission`、`DepthPairBuffer`。5 秒上限写成 5000 毫秒。`SemanticSynchronizer._age` 仍保存 `hard_age_ms / 1000`，现有断言 `_age == 1` 通过。算法里和单调秒、`Age.seconds`、asyncio 比较的 `/1000` 保留，时长数值未改。

## 旧名分类

有效外部或保留秒制：`constraints` 的 `lateral_stop_s`；`GeoLocalizer`、occupancy、evidence、`hardware_navigation.max_age_s`；`DeviceProfile.observation_max_age`；`scan_receipt_admission.hpp` 的 `max_age_seconds=0.5`；`scan_ledger_max_age_s=3`；`history_s=2`、`horizon_s=12`、`diagnostics_period_s`、`snapshot_seconds`、`max_frame_age_s=2`、地图编辑期限、5 秒在线窗口。Nav2、SLAM、diffdrive 的秒接口未改名、未改值。`scan_motion_node.cpp` 把 `sensor_hard_age_ms / 1000` 存进比较用的 `max_age_`，错误文本标签仍写 `scan_max_age=`。

只读算法投影：`SensorTiming.hard_age_s`。Arbiter、Agent、Supervisor 内部秒字段由毫秒配置除以 1000 得到。`SafetyFreshness.max_age` 与扫描戳比较仍是秒；交给毫秒接口时乘 1000 一次。

历史证据与剥离名单：`docs/reviews/` 与 `docs/superpowers/` 里的旧秒名保留为历史说明。`RETIRED_PARAMETERS` 与 `config.py` 对 `disconnect_seconds` 的 `pop` 只丢弃旧输入。测试里的 `future_wait_s`、`soft_age_s`、`lag`、`lateral_stop_s` 用于断言拒绝或字段不存在。

遗漏收口见下一节。产品 JSON/YAML 中没有再发现可被模型接受的旧时长输入键。`tests/ros` 与需要 `colcon` 的节点仍没有编译证据。本机不能 import `rclpy`。`hardware_reference.py` 的全文件 strict mypy 仍会碰到既有缺注解和 ROS stub；`max_age_ms` 属性错误已经消失。

## 验证

命令、退出码、失败节点和跳过原因在 `2026-11-17-duration-milliseconds-commands.txt`。摘要：

| 运行 | 结果 | 退出码 |
| --- | --- | --- |
| 定向 205 | 200 passed，5 failed，0 skipped | 1 |
| fixture + drive remap 9 | 9 passed | 0 |
| 年龄与 fixture 后续 79 | 79 passed | 0 |
| learning gap 诊断 5 | 5 failed | 1 |
| 其余受影响单元/集成 456 | 455 passed，1 skipped | 0 |
| 本机 C++ 传感器策略 2 | 2 passed | 0 |

上表里的 5 个失败都来自当时仍在 `tests/unit` 的 `test_learning_gap_debug.py`。失败日志保留。该过期探针的处置见文末，正式学习周期测试当时 5 项通过，本轮未改该文件，不重复执行。同批定向运行里，startup learning、heartbeat wire、runtime 与 fault isolation 测试通过。

跳过 1 项：`test_aiortc_loopback_delivers_a_real_frame`，本机没有 `aiortc`。

Ruff check 与 format 对 95 个迁移 Python 文件通过。mypy strict 对 22 个产品模块通过；fixture 在 `--follow-imports=skip` 下通过。

依赖 ROS 的部分仍未验证：没有 `colcon`，没有 `/opt/ros`，`import rclpy` 失败。`tests/ros` 中 6 个内容有变的文件没有执行。只依赖本机 `c++` 的两份传感器策略测试已在下一节编译并执行。

先前终端中的 552 项输出没有保存进仓库，本报告不把它当作可核验的通过记录。

## 未入 owned diff 的测试与诊断探针

`test_shared_sensor_cpp.py` 与 `test_native_sensor_age_boundary.py` 在迁移前正文里就已经是旧调用，所以上一版内容 diff 没有它们。全库 `tests/` 与 `tests/support` 再搜旧名后，仍在调用现行接口的只有这两份：`Policy(.3,1.,5.)` 把 0.3 秒和 1 秒送进毫秒构造，第一份还调用已不存在的 5 参 `age_ns`。第二份在错误阈值下 `summary` 断言会让进程 abort。其余命中是拒绝旧键或检查字段不存在：`test_settings.py`、`test_common_sensor_composition.py`、`test_sensor_timing.py` 的 `soft_age_s` 构造、`test_rgbd_runtime.py` 的 `future_wait_s`。`header_consumer.cpp` 只检查 `sizeof(Policy)`。

两份测试已改为 `Policy(300.,1000.,5.)`。诊断周期仍是 5 秒。现行 `age_ns(measured_ns, received_steady, now_steady)` 的年龄是 `llround(now_steady*1e9)-measured_ns`。原 5 参成功例的 0.9 秒由稳态差 0.7 秒加上采样差 0.2 秒组成，现写成 `age_ns(4800000000ULL,5.,5.7)==900000000`。反向时钟现写成 `age_ns(1200000000ULL,1.2,1.1)`，仍要求抛出 `invalid_argument`。软/硬阈值观察、非法构造、`try_age_ns` 的 noexcept 与 9 个无效时钟、队列 threshold 与 warning/recovery 断言都保留。产品头文件没有为旧 5 参或秒制构造加兼容入口。

命令：

`NO_COLOR=1 .venv/bin/pytest -q -p no:cacheprovider tests/unit/test_shared_sensor_cpp.py tests/unit/test_native_sensor_age_boundary.py --junitxml=docs/testing/2026-11-17-duration-milliseconds-evidence/pytest-native-sensor-cpp.xml`

结果：2 passed，2.171s，退出码 0。JUnit 与日志在同一证据目录。编译参数仍是测试里的 `-std=c++17 -Wall -Wextra -Werror`，第二份还有 `-fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all`。mypy strict 对这两个文件通过。

`tests/unit/test_learning_gap_debug.py` 的循环是 `for i in range(1, 21)`，步长 0.05 秒，最多推进 1.0 秒，断言却要求 3 秒动作 `completed`，并检查 `deadline - started == 3.0`。保留的失败日志里 `action_consumed` 为 true，`action_elapsed_s` 约为 0.65，状态是 `moving`，阶段只有 `sampling` 与 `moving`。这与 1 秒窗口内动作尚未走完一致。正式 `tests/unit/test_startup_learning_cycle.py` 使用 `range(1, 301)`，覆盖 15 秒，并保留同一 3 秒期限断言。该诊断文件已从 `tests/unit` 删除。删除前正文在 `docs/testing/2026-11-17-duration-milliseconds-evidence/test_learning_gap_debug.py.txt`，SHA256 `8da79ce17436d1708b0a973cd34e19ae6ec2d5ebfa26e1b813efe27e193b5d9f`。`pytest-learning-gap.log` 与 `pytest-learning-gap.xml` 仍保留。没有给它加 skip，也没有改短周期断言或产品学习时长。

当前内容 diff 116 个文件，另加上述删除。两份本机 C++ 测试的最终 SHA：

| 文件 | 最终 SHA256 |
| --- | --- |
| `tests/unit/test_shared_sensor_cpp.py` | `5347108b1355326487b53b8a11ae0c5fa869b01fdb99d07824f4434eccb00376` |
| `tests/unit/test_native_sensor_age_boundary.py` | `3e9c6f385fc4dde804f7d4e099eca3bc101b44e2a13853f4b5abd50fd3e1c2fd` |

三份保护文件相对迁移前正文仍未改。URDF 仍与迁移前正文相同。
