> **2026-11-22 当前状态：** ROS 通用生命周期已迁到 senior_sensor_ingress.ros_lifecycle；SafetyGate 等控制入口由本包提供。 下文 B 阶段记录中的旧包名、转发路径和保留承诺属于当时历史，不作为当前安装说明。当前清单及迁移步骤见 [ROS 包归属](../../PACKAGE_OWNERSHIP.md)。

# senior_motion_control

B4 从 `ebb8c6c` 迁移既有控制职责；原数值/截止/权限和停止算法保留。功能包拥有实际控制节点、仲裁、速度计算、调度、控制报告和输出，实车与仿真使用同一实现。不是新守卫或新导航算法交付。

## 实际目录与入口

```text
ros2/src/senior_motion_control/
  CMakeLists.txt, package.xml, pyproject.toml
  scripts/{safety_node,command_node}         # 实际 main 的 ROS 安装入口
  native/{motion_output_guard.cpp,final_command_guard.cpp,final_command_frame.hpp}
  senior_motion_control/
    __init__.py
    safety_node.py                         # 原 SafetyGate、接收/控制组合和原停止屏障
    command_node.py                        # 原 CommandNode、FinalCommand 和任务终端握手
    motion_core_output.py                  # 控制轮次编排、学习请求应用和输出期限
    vehicle_arbitration.py                 # 唯一目标/代次/状态机/退场证明 owner
    vehicle_speed.py                       # 原净距限速、加减速及转向计算
    arbiter.py, state_machine.py, cadence.py # 原本地仲裁、权限状态与绝对周期
    control_inputs.py                      # 原隔离接收子进程、有界公开快照
    motion_lifecycle.py                    # 原 executor、元数据、CLI 关闭
    motion_profile.py                      # 原显式参数装配
    vehicle_diagnostics.py, runtime_reports.py # 控制报告 owner 与原隔离报告进程
    vehicle_capabilities.py                # 原能力门面；未接入仍结构化不可用
    motion_output.py, final_command_frame.py, native.py # 已交付输出、帧与定位
  test/{test_final_command_frame.py,final_command_frame_sender.cpp}
ros2/src/senior_motion_estimation/senior_motion_estimation/optional_motion_inputs.py
                                              # 已有 opt-in 观测适配，无新 GPS 能力
ros2/tests/
  test_{motion_cli_lifecycle,motion_ros_twist_output,motion_guard_to_owner}.py
  test_{navigation_command_clock,manual_command_generation,control_cadence_gate}.py
  test_{terminal_release_wire,terminal_release_collection,terminal_native_channel}.py
  test_{launch_recovery,startup_visibility}.py
```

上述 11 个原 ROS 控制模块来自 `senior_robot/senior_robot/` 同名文件；arbiter 来自
`src/senior_control/safety/arbiter.py`，state_machine/cadence 来自 `src/senior_control/motion/`。
这些旧路径只作显式 `//Compatible` 转发，类/函数与 canonical 实现保持身份相同。
既有 optional_motion_inputs 归运动估计；只替换其原依赖入口，没有开启 GPS/RGB-D 或更改算法。
legacy motion_core_perception 的限速出口也转同一速度实现。

实车 `senior_robot/launch/safety_motion.launch.py` 和仿真 `simulation_motion.launch.py` 的
SafetyGate 由 `senior_motion_control` 安装入口启动；各自的设备、参数、隔离、恢复、topic 和监管不变。
原 agent_adapter 继续拥有命令节点的 IPC/peer identity 组合，通过 Compatible 旧入口取得同一 CommandNode；
不另外启动独立命令节点。原 senior_robot console 命令名仍安装并直接调用 canonical main。
导航控制器的私有输出、任务与地图/定位/导航实现和 launch 仍留原位，不复制或提前迁移。

## 公开接口与状态归属

| 接口/owner | 输入 → 输出 | 状态及保护 |
|---|---|---|
| CommandNode / Arbiter / FinalCommandFence | 原认证手动、现有导航私有 Twist、回执 → 原 FinalCommand/终端协议 | 原 authority/generation/GID、不可刷新截止、旧 scope 拒绝和独立身份退场规则 |
| CommandArbitration / ArbitrationPorts | 已过真实 FinalCommand 栅栏的命令、带 boot/generation 的学习请求 → 当前目标与原事件 | 唯一拥有 target/machine/input generation/退场证明；ports 只暴露 boot、只读状态回调及明确 trace/学习中断/清零/恢复完成效果，不接收可写 MotionCore context |
| SpeedCalculation | 原请求、前次输出、实测轮速、感知净距、dt → 原 Velocity | 刹停距离、斜坡、方向及转向限速公式未变 |
| MotionCoreOutput | 原轮次时钟、公开估计快照、学习结果、净距 → 原输出候选/截止 | 仅编排；内部显式访问 arbitration/learning/sensor_access/diagnostics owner，无 OwnedState；旧读视图与少数历史仲裁写入口显式 Compatible，不复制状态 |
| MotionSensorRuntime / ControlInputs | B3 frozen 四字段视图及原精简 reference → 控制 reader | 单槽/原大小限额、观测时间和代次不刷新，接收 owner 持有历史；原 spawn/父死亡/恢复及关闭保留 |
| LearningRuntime / CoreDiagnostics | 原学习输入/返回请求、只读 failure_report → 控制应用/报告 | 学习仍在感知持有业务状态；控制报告只读取公开失败事实，诊断故障隔离与原有界异步报告不变 |

旧 OwnedState 类为历史代码留原位，但 canonical 控制不再使用；旧 MotionCore 属性变为显式 owner 视图。
学习/缓存/诊断的兼容视图只读，内部不经过跨职责可写描述符。
MotionCore 的 complete tick 决策树按 owner 属性位置归一后与原实现核对，原 12 份非边界实现的 AST 除导入外相同。

## ABI、安装与构建闭包

原两个 C++ guard、FinalCommand v3 帧、MotionOutput 和 native_executable 本批未修改。
`=ddddQ` 40 字节顺序仍为 vx、wz、原绝对 deadline 秒、computed monotonic 秒、sequence；
deadline=0 停止包、原停止屏障、发送失败撤通道、原 guard 截止和 sole `/cmd_vel` 发布者保留。
没有新增 250ms 协议、守卫、补偿、GPS、探索或底盘发布者。

ament_cmake_python 安装全部 canonical Python 模块；CMake 安装两个真实 CLI 和原 native guard。
package.xml 补 rclpy/sensor_msgs/std_msgs/diagnostic_msgs/tf2_ros 以及感知/估计/接入依赖；不反向依赖 senior_robot。
根项目、uv.lock、Docker COPY 和 CI 安装闭包补已存在的 control SDK，不新建发行项目。
wheel 只验证无 ROS 值/算法与原本地入口，**不含 native 程序，不能代替 ROS/ament 安装**。
ROS runtime/optional 入口还需要原根 safety 依赖及实际接入包；无 ROS SDK 默认导入不加载它们。

## B4 实际范围

62 个逻辑文件：产品 35、测试及辅助 15、配置构建 9、工具文档 3；迁移按逻辑单计。
其中新增 17、修改 34、测试重命名 11，无额外逻辑删除；新旧测试不复制。
11 份 ROS 接口/组装测试从 tests/unit 归 ros2/tests，两个现有跨云车 IPC 集成仍原位更新真实源路径；
既有控制资源辅助及学习链只调整真实报告/速度实现和学习 owner 补丁，不添 controller/_inputs 等假属性。
未触及的历史根测试仍待随所属后批迁移，不能据此宣称测试已全部归包。

## 命令、结果与限制

环境：macOS 26.4 arm64、CPython 3.12.15、pytest 8.4.2、Pydantic 2.14.1、NumPy 2.5.4、
Shapely 2.2.0、uv 0.12.23、ruff 0.17.3、Apple clang 21.0.0。
pytest 使用原主仓库 `.venv/bin/python`；以下均从本功能工作区执行，安装导入检查从 `/tmp` 执行。

- RED：先改现有 CLI 生命周期测试读取 canonical 源路径，在实现迁移前因文件不存在失败。
- 主定向：`python -m pytest ros2/tests/test_startup_learning_chain.py ros2/tests/test_motion_core_perception.py ros2/tests/test_manual_command_generation.py ros2/tests/test_navigation_command_clock.py ros2/tests/test_motion_cli_lifecycle.py tests/unit/test_native_output_protocol.py ros2/src/senior_motion_control/test/test_final_command_frame.py tests/unit/test_sim_contract.py tests/unit/test_motion_launch_profile.py tests/unit/test_task_terminal_release.py::test_terminal_identity_does_not_revoke_current_manual_or_touch_output_frontiers -q`：78 passed、6 skipped；六项为本机不支持 Linux SOCK_SEQPACKET 的原生运行用例。通过原净距/速度、三秒/迟到/接管/恢复、真实当前 core 终端不撤新手动、私有导航指令、原 C++ 确定时钟和帧字段、实车/仿真装配契约。
- ROS 原入口：`python -m pytest ros2/tests/test_native_motion_executor.py ros2/tests/test_scan_ingress.py -q`：10 skipped，缺 rclpy，未冒充实际 DDS/节点运行通过。
- 旧组合：`python -m pytest ros2/tests/test_terminal_release_wire.py ros2/tests/test_terminal_release_collection.py ros2/tests/test_terminal_native_channel.py tests/unit/test_task_terminal_release.py tests/unit/test_navigation_core_output.py ros2/tests/test_control_cadence_gate.py tests/unit/test_motion_state_machine.py -q`：33 passed、9 failed、35 errors；完整 ebb8c6c archive 在 /tmp/b4-baseline 用对应旧测试路径复现同样结果。旧 controller 夹具使 35 项 setup 错误和一项 wire 失败，另三项 cadence 仍用退役 prefetch/缺实际 trace 传输的夹具，五项 state_machine 仍断言旧恢复策略；不改变产品去迎合旧规则。
- 补充装配：当前 `test_motion_ros_twist_output`、`test_motion_guard_to_owner`、`test_launch_recovery`、`test_startup_visibility`、`tests/integration/test_manual_wire.py`、`tests/integration/test_shutdown.py` 与同一当前 core 终端用例组合执行：81 passed、11 failed、1 skipped；完整基线同名用例/原路径同样结果。失败为旧 mark/controller/LearningClient 夹具、四项 AST 固定切片误取 FinalCommandFence、三项 manual_wire 缺既有 _output_incarnation。沙箱内首跑曾因本地 Unix bind 限制而报 27 failed/3 errors；允许测试自有 IPC 后上述 81/11/1 为真实结果，不把权限错误归为迁移行为。未添加假属性或删弱断言。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --offline` 和 `uv lock --check --offline`：65 包，新增已存在的 senior-motion-control 本地 SDK 依赖。
- 同缓存 `uv build ros2/src/senior_motion_control --offline --out-dir /tmp/b4-wheels`、对 senior_motion_estimation/senior_perception 和根项目 uv build：sdist/wheel 通过；`uv pip install --offline --python /tmp/vehicle-batch-one-venv/bin/python --reinstall --no-deps /tmp/b4-wheels/*.whl` 后从 /tmp 导入 arbiter/state_machine/cadence/speed 与原本地接口，类型身份一致且不加载 rclpy；`uv pip check` 兼容。
- 定向 ruff check/format、git diff --check、原算法/控制 tick AST、同级依赖及真实 launch/安装声明核对通过。已执行作者独立 diff 自审，仍等待负责人亲审。

未验证：无 colcon、CMake、rclpy、ament_index，实际 ROS/ament 构建/安装、native 定位、DDS/TF、
真实接收子进程、Linux SOCK_SEQPACKET、实车/仿真及建图导航运行均待对应环境。
SDK、原生 stub harness、AST 与入口契约不替代实际 ROS 验证；旧失败仍保留并交后续所属批次处理。
未启动开发/实车服务、部署或推送；后续功能与新算法不在本提交。
