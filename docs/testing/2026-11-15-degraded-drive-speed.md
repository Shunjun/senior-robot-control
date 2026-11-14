# 退化手动驾驶比例限速修复（代码审阅通过，Draft 交付）

审阅修订：上一版把旧 `degraded_cap` 解释为物理能力边界的判断已撤回；以下公式和范围已修正。增量调查、RED/GREEN 及 ROS 构建缺口见 `2026-11-15-degraded-drive-speed-review.md`。

当前交付状态：总负责人已亲读累计和增量生产 diff、新增模块、测试及证据，代码审阅通过，并授权提交本修复分支、推送及向 main 创建 Draft PR。仅认可模块验证；Linux ROS 消息生成、guard 完整构建及实车验证仍未完成，不合并。以下测试结果保留原执行时点事实，本轮只更新文档状态，不重复跑测试。

## 基线与范围

- 启动时 HEAD 与协调者已 fetch 的 origin/main 均为 `ad1c0c6e5764f436d0d25ec6622a9ff9d32afd09`，从该基线开发。
- 分支：`fix/degraded-drive-speed`；仅将本次新建、未开发的 `Shunjun/fix-degraded-drive-speed` 改名，未修改其他分支。
- 工作区：`/Users/zhangshunjun/workspace/orca/senior-robot-control/fix-degraded-drive-speed`，Orca 当前工作区也已回读确认相同 SHA、分支和 origin/main baseRef。
- 只实现本轮批准的退化满杆速度及其配置链路，不修改运行中的实车配置、部署 profile 数值或其他已讨论限制。没有启动开发服务、ROS、Gazebo 或硬件；前两阶段回报时尚未提交、推送或建 PR，现已获负责人代码审阅通过及 Draft 交付授权，保留工作区。

## 实际变化与关键文件

1. `settings/models.py`、`settings/store.py`：新增 `degraded_linear_mps` 默认 0.3，产品偏好范围 0–25/18 m/s；复用驾驶设置版本、停稳校验、事务保存与回读。只允许完整旧五字段驾驶组补入新默认并在首次加载时持久化；旧请求回执可读，其他缺字段仍拒绝；兼容处均标 `//Compatible`。
2. `agent/manual_control.py`、ROS `command_node.py`：保留归一化线性意图、所选档位/倒车满杆上限及保存的退化偏好，沿原手动期限发送，不延长 authority 或 deadline。
3. 新增 `motion/manual_target.py`、扩展 `motion/contracts.py`、ROS `motion_core_output.py`：在输出 owner 根据当前 source mode，每 tick 生成比例映射目标，再交给原方向候选、轮能力约束、平滑轨迹及最终核验。正常模式保持档位映射；退化目标为 `x * min(档位上限, 用户退化设置, 正常车体/轮能力 cap)`，倒车的档位上限已额外取倒车上限。角速度映射维持原行为。
4. `FinalCommand.msg`、`final_command_frame.hpp`、`final_command_guard.cpp`、Python `final_command_frame.py`：增加三项元数据，验证归一化范围/有限值/非负上限/速度一致性，完整载荷参与原生重复帧判断；私有帧从 SCF2/799 升至 SCF3/823，旧帧拒绝，barrier/导航元数据必须为零。
5. `web/src/api/vehicleState.ts`、`features/settings/VehicleSettings.tsx`：在原车辆驾驶设置入口新增“退化模式线速度上限”，显示 km/h，默认 1.08，正确转为 SI 保存并回读；明确实际速度仍受档位、倒车和车端能力限制。无新增全局 CSS 或内联多属性 props 类型。
6. 新增 `tests/unit/test_degraded_manual_speed.py`，扩展设置迁移/保存回读、手动 wire、原生 C++ roundtrip 与前端设置测试。现有手工构造 CommandNode 的两个共享测试夹具补上 main 已存在的 `_motion_control_configured=True`，否则会在新逻辑前因缺属性失败；仅修改夹具，不改产品放行逻辑。其余测试夹具同步新增消息字段。

## 能力边界与偏好职责

`degraded_cap` 经 main 原始文档和提交核实是旧策略值，没有独立物理保证；本次手动退化上限只由 `degraded_linear_mps` 负责，正常 `cap`、轮能力和方向约束继续有效。默认正反向 25%/50%/100% 分别映射 0.075/0.15/0.3 m/s；档位、倒车或用户设置更小则同比降低。自定义值可高于旧 0.15 策略，不被它再次截断。当前硬件 JSON 数值保留不动，导航继续使用旧策略，明确不属于本次手动修复；保存的设置值不宣称实际车速。

## 命令与结果

Python 使用目标仓库现有虚拟环境解释器，只读复用依赖并以 `PYTHONPATH=src` 导入本工作区代码；执行时临时 `.venv` 链接已清理。前端依赖复制到本工作区，未在原工作区运行构建或改配置。

- `PYTHONPATH=src .venv/bin/python -m pytest tests/unit/test_degraded_manual_speed.py tests/unit/test_motion_core*.py tests/unit/test_motion_guard_to_owner.py tests/unit/test_motion_controller.py tests/unit/test_motion_trajectory.py tests/unit/test_directional_constraints.py tests/integration/test_manual_wire.py tests/integration/test_vehicle_settings.py ros2/src/senior_motion_msgs/test/test_final_command_frame.py -q`：**282 passed**，涵盖原生 C++ sender→Python decoder→实际 gate fence/owner、方向约束、急停、过期、控制平滑等。当时新增模式切换/非法 wire 的最后五项尚未添加。
- 添加最后五项并增强原生元数据重复/非法值检查后，`pytest tests/unit/test_degraded_manual_speed.py tests/integration/test_vehicle_settings.py ros2/src/senior_motion_msgs/test/test_final_command_frame.py -q`：**113 passed**。
- 最终 `pytest tests/unit/test_degraded_manual_speed.py tests/integration/test_manual_wire.py -q`：**61 passed**，包括输入百分比经手动入口→消息→输出目标的衔接、障碍首因断言与新切换测试。
- 修改的 Python 源码、设置模块及新增/扩展核心测试执行 `python -m ruff check ...`：通过；七个相关源文件 `python -m mypy src/senior_control/agent/manual_control.py src/senior_control/motion/manual_target.py src/senior_control/motion/contracts.py src/senior_control/settings`：通过。
- `cd web && npm test -- --run`：**13 files / 166 tests passed**。`npm run typecheck`、`npm run build`：通过，Vite 90 modules。
- `git diff --check`：通过。C++ roundtrip 用本机编译器、C++17、`-Wall -Wextra -Werror`，通过；没有宣称编译完整 ROS guard。

前端第一次尝试复用只读 node_modules 链接时因 Vite 临时缓存写权限失败，改为工作区独立依赖副本后测试及构建通过。初始手动/启动回归暴露 main 测试夹具缺现有 `_motion_control_configured` 属性，按上述夹具补全后相关回归通过。

## 验证覆盖与剩余限制

- 低/高档、正反向 25%/50%/100%，默认及自定义退化偏好；档位/倒车上限更低；正常能力边界更低；正常模式档位比例：通过合成入口与实际约束候选测试；审阅修订补测见增量报告。
- 正常转退化及恢复的加速度连续约束、障碍仍阻止候选、手动期限、急停、轮能力与既有控制回归：通过定向纯逻辑/合成场景测试。
- 旧持久化缺新字段迁移、旧 receipt 回读、保存/回读/数据库重开保持、负数/NaN/Inf/布尔/字符串/超产品范围拒绝：通过集成测试；网页 1.08 km/h 默认及自定义 0.72 km/h→SI 保存通过。
- 本机没有 `colcon` 或 `ros2`，未执行 Linux ROS 消息生成/全 guard 构建、DDS 运行、Gazebo 或实车；这些不是上述模块测试已经完成的验收。
- 部署需同版重建并一起更新 `senior_motion_msgs`、原生 guard 和 Python 适配；旧 frame 不兼容是明确拒绝，不通过默认值模拟兼容。
- 总负责人已完成实际累计与增量 diff、全部新增文件及证据审阅；本次获准提交、推送及向 main 发 Draft PR，不合并，不宣称 ROS 或实车完整验收。
