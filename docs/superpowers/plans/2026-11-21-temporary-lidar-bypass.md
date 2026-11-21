# Temporary Lidar Bypass Implementation Plan

**Goal:** 在固定分支完成可逆的全部雷达诊断旁路，保留非雷达控制语义。
**Architecture:** 单一严格模式选择器；正式服务传播环境，ROS 参数交叉校验；在资源所有者处关闭雷达，在控制所有者处跳过雷达约束。
**Execution:** Orca worker 顺序实施，范围已授权；开发 worker 不连接车端或部署，代码亲审后由负责人部署，用户本人操作车辆，负责人只做 OS 级 CPU 采样、不主动动车。用户明确无需补测试，覆盖新增测试/TDD 条款，不新增或改写测试。

### Task 1: 统一模式与资源

新增 `src/vehicle_core/lidar_diagnostic.py`；修改 `senior_control/src/vehicle_manager/service.py`、`ros2/src/senior_robot/senior_robot/{platform_runtime,real_runtime,agent_adapter}.py`、`ros2/src/senior_robot/launch/{safety_motion,mapping}.launch.py`、`ros2/src/senior_navigation/launch/navigation.launch.py`、`ros2/src/senior_localization/senior_localization/localization_node.py`、`config/hardware-safety.yaml`。读取严格布尔配置、传播并校验环境；停止学习和雷达启动链；独立入口及 IPC 明确不可用，保留 run/claim 身份。

### Task 2: 控制旁路

修改 `ros2/src/senior_motion_control/senior_motion_control/{safety_node,motion_core_output,vehicle_speed}.py`、`ros2/src/senior_motion_estimation/senior_motion_estimation/motion_observation_adapter.py`、`ros2/src/senior_perception/senior_perception/learning_runtime.py`。停掉逐扫描工作和匹配接收，保留 50ms 快照；显式无雷达速度计算仅移除距离 cap，保留原 deadline、物理限制和非雷达状态。

### Task 3: 静态验证与回退说明

内存 `ast.parse` 全部变更 Python，`git diff --check`（现有 CRLF 文件使用 `core.whitespace=cr-at-eol`），有现成 ruff 时仅检查变更文件；不安装依赖、不启动服务、不运行全套。亲查正常 false 路径与临时 true 热路径、核对文件范围，记录环境限制和 vendor 源码可见性。按负责人最新指示，交付此工作区供负责人亲审后提交/推送，worker 不连接车端或部署。

负责人部署时分别记录部署前停车基线、部署后稳定停车和用户实际操作窗口的起止时间、版本与模式；部署、重启和回退保持停车，用户操作结束后停车再采样，不由负责人发送运动指令。整机 CPU 与进程 CPU 分开，使用同一时间窗口内 OS 计数/进程 CPU 时间的增量并注明归一化口径；PID 变化拆分窗口，不用 `ps` 生命周期累计均值代替窗口负载，不增加埋点、工具或测试。当前仍尚未部署、未实测。

**Review Focus:** 跨进程一致性、真实资源停止、非雷达约束不变、旧 run/claim 不开启学习、地图/定位/导航明确不可用；配置 false 与环境同步后重启正式入口即可回退。产品新增 1、修改 13，配置修改 1，文档新增 2，共 17 文件，测试 0。
