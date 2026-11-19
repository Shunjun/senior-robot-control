# 运动估计功能包

状态：仅可被 ROS 包工具发现的 `ament_cmake` 架构骨架；没有节点、算法或运行能力。
本批不提供假可执行入口，不接生产 launch，不从 `senior_robot` 导入业务实现。

## 职责和公开边界

- 输入：轮速、雷达运动与可用 IMU，未来可选 GPS 速度观测。
- 输出：融合车速与连续本地运动历史（含测量时间和不确定性）。
- 业务归属：时间对齐、来源权重、观测一致性与轮滑；不拥有仲裁、限速或全局定位。
- 实际迁移源：`ros2/src/senior_robot/senior_robot/motion_sensor_runtime.py、motion_observation_adapter.py 与现有 senior_scan_matcher 对接`。当前实现继续留原位置，保持既有实车/仿真链可用。

公开接口要写明单位、坐标系、测量时间、来源/帧身份及缺失语义。
ROS 消息留 ROS transport；管理程序复用现有 Unix IPC；云车消息才归 `contracts/`。
具体话题、服务、动作或进程内接口以及直接包依赖在对应迁移批次确认，
本批不臆造协议或互相依赖。缺失功能留模块实现阶段，不能伪造可用结果或运动指令。

## 源码、入口、接口和测试布局

当前实际文件只有 `CMakeLists.txt`、`package.xml` 和本 README。
以下是迁入实现时的职责落点，尚未创建空源码或测试文件：

| 位置 | 后续职责 |
|---|---|
| `src/` | 功能节点适配及 C++ 内部算法；复杂逻辑按业务模块分开 |
| `senior_motion_estimation/` | 需要 Python 实现时的包；节点入口、内部算法和状态按职责分模块 |
| `include/senior_motion_estimation/` | 需要跨包复用的 C++ 公开只读接口 |
| `launch/`、`config/` | 本功能确有需要的启动/参数；组合选择仍归实车和仿真入口 |
| `test/` | 迁入原同名算法、接口及生命周期用例；不复制无关场景 |

这些目录在迁入真实文件时加入构建/安装声明，不为骨架添加假入口。
测试复用方向：原运动估计/观测适配、纳秒时间边界及连续历史回归。
当前没有故障处理或运行参数；迁入后由本模块明确不可用、停止/撤销及质量降级行为，
迁移阶段先保持原行为，算法改造另行验收。

## 最低验证与完成条件

`colcon list --base-paths ros2/src` 应发现 `senior_motion_estimation`；
ROS/ament 环境可执行 `colcon build --base-paths ros2/src --packages-select senior_motion_estimation`。
仅静态 XML/CMake 检查不代表 ROS 构建成功。完整完成条件是节点、算法、公开接口、
状态和原回归一同迁入，实际生产与仿真组合共用该包，安装后定位与关闭路径验证通过。
