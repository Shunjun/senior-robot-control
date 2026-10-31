# ROS 安全拓扑修复 wave 1

基线 5923581。Nav2 与脱困的裸 Twist 因无动作代次及生成时间暂不接收；手动速度由仲裁器按连接和单调时钟有效期选择，安全门输入由仲裁发布。急停、接管、失联、取消及重新获取均清除旧速度。脱困模式需调用 `begin_recovery` 明确切换；当前尚无运行时调用者，默认拒绝脱困速度。租约释放后新会话可获取，保留每会话代次序号；适配器停用后必须以更高代次明确获取才重新启用。CommandNode 的业务时钟不覆盖 rclpy 内部时钟。

安全门改用稳态定时器，查询扫描时间的 TF 并计算实际激光坐标；扫描、带时间戳的车体速度均需新鲜且时间递增，缺失、超时、时钟回退或 TF 无效停车。odom 配置统一 `/wheel/odom`，验收节点名统一 `safety_gate`。

## 证据

- RED `pytest tests/unit/test_arbiter.py -q`：2 failed, 10 passed（缺少自主输入和释放后新会话拒绝）；实现后 12 passed；追加恢复与取消重放测试并确认失败后修复。
- RED `pytest tests/integration/test_shutdown.py::test_disabled_adapter_requires_new_epoch_acquire_to_reenable -q`：1 failed；修复后 1 passed。
- `pytest tests/unit -q`：236 passed。
- `pytest tests/integration/test_shutdown.py -q -k adapter`：8 passed, 9 deselected。
- `pytest tests/unit tests/integration -q`：416 passed, 1 skipped, 1 failed, 4 errors；五个失败均为本环境禁止 Unix socket bind 的 PermissionError（test_ipc_frames_are_versioned_and_length_limited、test_shutdown_disables_motion_before_ros_exit、test_sigterm_revokes_motion_before_cleanup、test_cancel_propagates_and_releases_resources、test_ipc_motion_sends_validated_envelopes_and_the_epoch）。
- `pytest -q`：收集失败，e2e 与 unit 的 test_home.py、test_recovery.py 同名冲突，1 skipped、2 errors；项目 README 指定按 unit/integration 范围运行。
- `pytest tests/ros/test_velocity_chain.py tests/ros/test_navigation.py -q`：5 skipped，本机无 rclpy；不视为通过。新增真实 TF 和冻结 ROS 时间稳态定时器检查供 Linux 执行。
- `python -m compileall -q src/senior_control/safety ros2/src/senior_robot/senior_robot`：退出码 0。

## 未验收

Linux Humble 构建、启动及完整速度链、时钟冻结/恢复、真实 TF、底盘 watchdog、测量源质量和硬件停车均未在本轮执行。独立扫描匹配/运动估计的 ROS 供应者尚未接入；安全门没有新鲜质量合格的 TwistStamped 时保持零输出。脱困控制器、动作取消与网络 Agent 仍未组装，不据此声称可自动导航或实机安全。Nav2 原始 Twist 无来源时间戳，网络或 ROS 中间层排队后的旧消息无法根据消息自身时间识别，需 Linux 故障注入验证并在系统组装时处理。安全限值与测量源须现场标定和验收。

## 质量门与 Linux 回归补充

- Linux Humble 上复跑 b7f1559 的 ROS 节点测试：6 passed、1 failed。失败为 TF 测试触发未配置的 `footprint_xy` 参数抛 `ParameterUninitializedException`；这是真实 RED。已将未配置的 footprint 与 self-mask 数组视为空配置，仍由安全评估拒绝运动，并增加默认配置的 ROS 回归检查。修复后的 Linux 测试尚待复跑。
- `ruff check .`：All checks passed。
- `mypy src`（项目配置 strict）：Success: no issues found in 65 source files。
- `pytest tests/unit tests/integration/test_shutdown.py -k 'not ipc_frames and not shutdown_disables_motion_before_ros_exit and not sigterm_revokes_motion_before_cleanup and not cancel_propagates_and_releases_resources and not ipc_motion_sends_validated_envelopes_and_the_epoch' -q`：248 passed、5 deselected；排除项为上述本环境禁止 socket 的测试。

## 独立复审修复

复审指出裸 Twist 无法绑定动作来源：旧任务的消息可在 resume 后被当作新速度。新测试 `test_unstamped_autonomous_command_cannot_drive_after_resume` 在 b7f1559 真实失败；当前拒绝纯核心无代次自主速度，并取消 CommandNode 裸 Twist 订阅，Nav2/脱困运动不可用，待实现源端时间戳和代次桥后再启用。ROS 订阅回归尚待 Linux。

旧 epoch 的 disable 不再撤销新租约，只有仲裁接受的 epoch 推进水位；exit 永久关闭。`test_stale_disable_does_not_disable_new_epoch`、`test_rejected_epoch_does_not_block_valid_reenable`、`test_exit_permanently_rejects_later_acquire` 修复前均失败。前文将裸自主速度描述为已接入属于 b7f1559 状态，以本节为准。

复审修复后验证：`ruff check .` 全通过；`mypy src` 65 个源文件无问题；本地单元加非 socket 的 shutdown 测试 252 passed、5 deselected。旧停用与被拒绝高 epoch 的两项测试在修复前均失败，修复后通过；退出后禁止重新启用测试亦先失败后通过。Linux ROS 测试尚待重跑。

`ruff format --check` 检查本轮 14 个 Python 文件：13 个已格式化，`tests/unit/test_recovery.py` 一处既有格式不符；从基线 5923581 提取的同文件同一位置复核仍不符，未改动无关内容。

## 第二轮复审修复

复审指出，停用新 epoch 先于 acquire 时没有记住停用水位；较新 epoch 的停用还会被误认为过期事件，不能撤销旧租约。新增 `test_disable_before_acquire_blocks_same_epoch_until_new_handshake` 和 `test_newer_disable_revokes_older_lease_and_rejects_its_own_epoch`，在 bc494ac 运行结果 `2 failed`，分别实际出现同 epoch 错误获准及旧速度仍为 0.2。修复后记录已接受和已停用的独立水位：较新或相同的停用撤销当前租约，较旧停用无副作用，重新启用须明确获取更高 epoch。水位仅保存在进程内，不声称跨重启持久化。

验证：`pytest tests/integration/test_shutdown.py -k 'not ipc_frames and not shutdown_disables_motion_before_ros_exit and not sigterm_revokes_motion_before_cleanup and not cancel_propagates_and_releases_resources and not ipc_motion_sends_validated_envelopes_and_the_epoch' -q` 为 17 passed、5 deselected；`ruff check .` 通过；`ruff format --check` 两个本轮 Python 文件通过；`mypy src` 65 文件无问题。排除项仍是本环境的 Unix socket 限制。
