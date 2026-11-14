# 已替代运动路径清理：证据与验证

本次基线为 fetch 后的 `origin/main = aff459211e518744ff287f986bef724e4ad027c0`，提交标题 `Merge branch 'Shunjun/vehicle-real-control-wiring'`。工作树初始 HEAD 与此一致，且干净、没有独有提交。仅在本次新工作树首次编辑前，将 Orca 初建的 `Shunjun/remove-legacy-motion` 改为授权的 `refactor/remove-legacy-motion`，随后 `merge --ff-only origin/main` 返回 Already up to date。

工作目录：`/Users/zhangshunjun/workspace/orca/senior-robot-control/remove-legacy-motion`。读取了 `/Users/zhangshunjun/Wheel 机器人/AGENTS.md`；本仓库没有额外 AGENTS.md。未操作其他工作树、旧 Agent 或 VM，没有复制未合并导航代码，没有 cherry-pick。用户后续已明确授权“直接提 PR”，据此提交并推送本功能分支，创建回 main 的 PR，不自动合并。

参考盘点 `/private/tmp/current-main-safety-parameters-20261114/{INDEX,REPORT}.md` 保留原样；其 `454cbfe` 基准仅作线索，本次判断以实际 `aff4592` 入口、导入与调用为准。未调整安全参数、增加检测或重设计模型，未启动开发服务、ROS、Gazebo 或硬件。

## 删除前的消费者证据与现行职责

| 删除对象 | aff4592 消费者证据 | 现行路径与删除边界 |
| --- | --- | --- |
| `safety/recovery.py` 整个旧地图辅助倒车控制器 | 全仓 `RecoveryController/RecoveryBounds/MapCorridorEvidence` 引用中，代码消费者只有 `tests/unit/test_recovery.py`、`tests/e2e/test_recovery.py`；无 ROS/Agent/CLI/工具导入、launch 或 console script | 当前核心由 `motion/constraints.py`、`motion/controller.py`、`motion/state_machine.py` 和 `MotionCoreOutput` 分别负责方向许可、执行、停车/恢复；Nav2 的两份 behavior tree 只做规划/跟随与 costmap 清理，未接该旧倒车器。删除未接线实现不等于新增地图辅助倒车能力 |
| `motion_estimator.py` 的 `supports_degraded_reverse`、`degraded_reverse_gate`、`body_displacement` | 除自身定义外，只有上述旧控制器和 `test_motion_estimator.py` 引用；其中 gate 始终返回不允许，仅遗留 deferred 结果 | 保留 MotionEstimator、MotionEstimate、body_twist_delta 及实际实时消费者；移除仅为旧倒车控制器服务的助手及 SafetyDecision 导入 |
| `motion/estimate_adapters.py::from_motion_estimate` | 除定义外，只有 `test_motion_sources.py` 一项专用回归调用；无 ROS/Agent/工具导入 | `NativeObservations.laser` 直接将真实 ScanMotion 的原区间、位移、协方差接入 DeltaObservation；随后 snapshot → TemporalResolver → MotionFusion，不再经旧 MotionEstimate 平均速度转接 |
| `AdmissionOwner.assess`、`source_modes.select_reference` 和 `MotionAssessment` | assess 的调用全部在 `test_motion_sources.py`；selector 只被 assess 调用；assessment 仅属于这条旧出口 | 现行 `NativeObservations.snapshot` 和回放工具使用 `AdmissionOwner.snapshot` 的 admission journal，消费 TemporalResolver 和数值融合结果；保留仍被 journal 使用的 SourceView 和核心使用的 SourceMode |
| `ros2/src/senior_robot/config/hardware.yaml` | 全仓精确路径引用仅 setup.py 的 data_files 和 2026-10-27 历史计划；无 launch 选择或运行配置读取 | 实车 `safety_motion.launch.py::_nodes` 读取显式 hardware_motion_profile、safety_parameters，并经 `real_runtime` 校验；用于 doctor 的根目录 `config/hardware.example.yaml` 仍保留。仅删无消费者的 ROS 包内示例副本及安装注册 |

上述证据按基线源码的全仓文本检索、Python import/定义检查、现行 launch/CLI 逐项核对；不以 `legacy` 文件名、注释或 Compatible 标记作为删除理由。

## 精确删除与变更清单

整文件删除 5 个：

- `src/senior_control/safety/recovery.py`
- `src/senior_control/motion/estimate_adapters.py`
- `ros2/src/senior_robot/config/hardware.yaml`
- `tests/unit/test_recovery.py`
- `tests/e2e/test_recovery.py`

局部删除：

- `src/senior_control/safety/motion_estimator.py`：上述三个旧倒车助手和死导入，估计器算法不变。
- `src/senior_control/motion/source_modes.py`：旧 MotionAssessment/选择器及死导入，保留 SourceMode/SourceView。
- `src/senior_control/motion/source_admission.py`：旧 assess 出口及其导入；snapshot/准入/事件记录算法不变。
- `ros2/src/senior_robot/setup.py`：删除包内 hardware.yaml 的 data_files 注册；console scripts 全部保留。
- `tests/unit/test_motion_estimator.py`：删除只测旧 gate 的手工包装估计测试；来源、退化、协方差、IMU、时间对齐、SE(2) 速度和 loop jump 回归保留，改为直接断言真实估计器结果。
- `tests/unit/test_motion_sources.py`：删除旧 estimate adapter 和同窗口拼接选择器的专用测试；其余来源准入、恢复、过期、时钟、序号/代次、协方差回归改查现行 snapshot/SourceView。测试显式启用有限 journal，不改变生产配置，不新增测试矩阵。
- `src/senior_control/motion/observations.py`：文档字符串由 assess 更新为 snapshots。
- `README.md`、`docs/testing/matrix.md`：移除已删除倒车器的当前能力/执行入口描述，指向仍在的方向约束与场景契约；历史结果报告和历史设计/计划保持原样。

## 保留项与理由

| 保留内容 | 当前消费者及不能删除的原因 |
| --- | --- |
| ROS `safety_node.py` 中旧安全算法和 `async_guard`/guard/DeviceProfile | `simulation_cli` 启动 `simulation_motion.launch.py` 只传 qualified_reference=true，默认 motion_core_profile 仍为空，因此默认仿真实际继续走旧安全算法。整体删除必须先迁移并验收仿真，不能在清理中改默认选择、移除感知/停车能力 |
| ROS `motion_fusion.py` 及旧 motion_runtime/estimator/slip | 两份运动 launch 都启动它；除 native observation bridge，它还发布 MotionEvidence、提供 `~/requalify_progress`。`simulation_cli` 恢复接口真实调用该服务，旧安全门仍消费 evidence。需要另行迁移进展证明/恢复接口后才能删 |
| 全部 simulation_motion*.yaml、motion_core_profile 条件分支、candidate clock/domain 门 | 默认仿真和显式 candidate 都活跃；flat_lateral/rear_mask 由实际 launch 条件选择。未擅自合并配置或开启共享核心替换默认路径 |
| 显式旧 real profile 的 Agent-only 启动分支 | `docker_start_helper`、CLI 和现行硬件操作指南约定未配 motion_profile/safety_parameters 时仍提供 Agent 与非运动功能，并拒绝运动。删除会破坏真实升级兼容 |
| 协议/手动租约/时钟/仲裁兼容及旧任务适配 | 仍有旧 peer、独立调用方和相应运行分支，非“已无消费者”的运动实现；本次不改变协议或任务生命周期 |
| 根目录 simulation.yaml、hardware.example.yaml、hardware-motion.example.yaml、hardware-safety.yaml/JSON schema | doctor/run_checks、实车指南和显式 profile 读取仍消费；不因文件名接近已删 ROS 包副本而连带删除 |
| RGB-D 诊断转换、rear_legacy_oracle 与等价回归 | 前者是保留的诊断能力；后者验证当前 rear geometry 行为，未被控制器替代，均不属于旧运动输出控制器 |
| 历史设计、计划、测试结果及 /private/tmp 安全参数盘点 | 它们是当时实现/结果的证据。保留其中旧路径和旧命令，不冒充当前可执行入口；尤其 `docs/testing/results.md` 的 2026-10-30 结果不改写 |

## 引用与安装检查

最终检查遍历 `src/ros2/tests/tools` 所有 Python AST：已删除的两个模块无 import，六个退休函数/类型名（from_motion_estimate、select_reference、MotionAssessment、三个倒车助手）无定义或导入。`AdmissionOwner.assess` 无代码调用。全文检索剩余旧路径引用均为保留的历史设计/计划/报告或本清理说明。

执行 setup.py 的声明捕获检查，逐一验证所有 data_files 文件和 console_scripts 对应 Python 模块存在；通过。没有把资源缺失留在安装注册中，没有为旧 RecoveryController/adapter 保留多余启动入口。

`git -c core.whitespace=cr-at-eol diff --check` 通过；两个受改文件原本为 CRLF，保留换行格式。未修改任何仍保留的 JSON/YAML 参数值、launch 或协议实现。

## 验证结果与限制

环境为本工作树独立 `.venv`，Python 3.12.15，通过锁文件离线安装 safety/agent/cloud 和 dev 依赖，不共用其他工作树环境。

- 全仓 `.venv/bin/ruff check .`：通过。
- 受改 Python 的 `.venv/bin/ruff format --check`：7 文件通过（含 observations.py）。
- `.venv/bin/mypy src/senior_control`：149 个源码文件通过。
- 定向既有回归：**265 passed，1 failed，21.41s**。包含 estimator/runtime、motion_sources、fusion/temporal/replay、native observation adapter、core chain/controller/state machine、directional constraints、launch/profile、sim contract、deployment config。
- 唯一失败为 `test_native_motion_observation_adapter.py::test_humble_missing_gid_accepts_only_expected_unique_simulation_publisher`，其 `object.__new__(NativeObservationRosBridge)` 夹具没有注入 profile，回调读取 `self.profile.wheel_publisher` 时 AttributeError。该测试与 adapter 源码逐字节等于 aff4592；单独执行同样为 2 passed/1 failed。本次不借清理修补 main 的活跃适配行为或测试夹具，不能宣称全绿。
- `tests/e2e/test_occlusion.py`：**1 passed**。最初受沙箱禁止临时端口 bind 影响，按权限机制在沙箱外复跑通过；该测试只检查场景契约，没有启动仿真或服务。
- 全仓格式检查：**483 文件中 3 个已有格式失败**，分别为 safety_motion.launch.py、agent_adapter.py、command_node.py；三者没有本次 diff，且取 aff4592 原始内容经相同 formatter stdin 检查均 exit 1。未把无关格式修复混入清理。
- 最终 AST/注册检查通过；未执行 ROS C++ 构建、真实节点运行、Gazebo、硬件、VM 或浏览器验收，不将纯回归作为实车/仿真运动验收。

原始定向回归日志、基线格式验证日志、完整实际 patch 与删除清单交付在 `/private/tmp/remove-legacy-motion-review-aff4592/`。完整 patch 包括本说明；用户后续授权直接创建 PR，提交与推送仅限本功能分支。
