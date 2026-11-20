# 批次 1：独立契约与原生控制输出

基线 `feat/vehicle-architecture-refactor@a445b1b`；依据`docs/architecture/ROS2与车端架构重构确认稿.md`。仅实现本批，不开始后续控制规则、管理目录或导航算法迁移。

## 实际文件清单（实施前记录）

产品：迁移 `src/senior_control/{models,errors,protocol}.py` 到 `contracts/src/senior_contracts/{values,errors,wire}.py`，原位置保留显式同类型转发；拆 `navigation.py` 的 DTO 到 `contracts/src/senior_contracts/navigation.py`，执行保留原处；拆 `settings/models.py` 的 DrivingSettings/FrozenModel 到 `contracts/src/senior_contracts/configuration.py`，设置业务保留；新增 contracts 包 init；迁移 ROS `motion_output.py`、`final_command_frame.py` 及消息包 native 的两个 guard 和 frame.hpp 到 `senior_motion_control`，旧 Python 两路径只显式转发；新增控制包 init；修改 `safety_node.py` 实际导入/可执行查找及 `vehicle_arbitration.py` 帧契约导入。

测试：迁移 `senior_motion_msgs/test/{test_final_command_frame.py,final_command_frame_sender.cpp}` 到控制包 test；更新 `tests/unit/{test_native_output_protocol,test_launch_recovery,test_motion_guard_to_owner,test_terminal_native_channel}.py` 的真实源码路径；增加/使用批准的安装导入契约检查，不新增驾驶场景。

配置构建：新增 `contracts/pyproject.toml`、控制包 `CMakeLists.txt/package.xml/pyproject.toml`；修改根 `pyproject.toml/uv.lock`、消息包 `CMakeLists.txt`、机器人 `package.xml`、CI `.github/workflows/backend.yml` 与 `deploy/Dockerfile.cloud`，按实际安装验证修订。

文档工具：纳入 docs/architecture 新版确认稿；本清单；contracts 和控制包 README；根与 ROS README；批次验证记录和必要安装验证工具。

迁移按一个逻辑文件计，兼容旧路径与迁移实现在交付清单中单列但不双计同一迁移；预测范围仍为用户已获预报40–66逻辑文件，范围扩大前问负责人。

## 决策与执行记录

- Ruling：采用任务授权的当前功能工作区，不创建新分支；负责人承担最终独立审阅，不派其他 worker。
- Ruling：显式兼容转发保留生产旧导入；新契约只依赖标准库/Pydantic，导航DTO不导入QualifiedRoute；旧pickle路径可经相同类转发解码，类身份不复制。
- 批次1保留40字节ABI、原截止及停止屏障，不提前实现新250ms接收计时。
- native_receipt 暂留 senior_motion_msgs，rclcpp运行依赖也暂时保留，后批归 senior_sensor_ingress；消息包不可宣称已完全只含消息。
- 全轮仍需管理子项目、感知/估计/控制其余节点、建图定位实现、既有导航后置迁移；本批不将能力占位视为已实现。

## 最终安装目录

```text
contracts/
  pyproject.toml README.md
  src/senior_contracts/{__init__,values,errors,wire,navigation,configuration}.py
ros2/src/senior_motion_control/
  CMakeLists.txt package.xml pyproject.toml README.md
  senior_motion_control/{__init__,motion_output,final_command_frame,native}.py
  native/{motion_output_guard.cpp,final_command_guard.cpp,final_command_frame.hpp}
  test/{test_final_command_frame.py,final_command_frame_sender.cpp}
```

实车/仿真 launch 本批不复制或新增节点；原 launch 都调用同一 SafetyGate，它直接导入新 MotionOutput/native 路径。senior_robot package.xml 声明新功能包依赖。消息包的 native_receipt 及相应 rclcpp/export 依赖明确暂留。

## 最终源 → 目标/原位修改清单

| 原实现 | 唯一实现目标 | 原位置处理 |
| --- | --- | --- |
| `src/senior_control/models.py` | `contracts/src/senior_contracts/values.py` | 同对象显式转发 |
| `src/senior_control/errors.py` | `contracts/src/senior_contracts/errors.py` | 同对象显式转发 |
| `src/senior_control/protocol.py` | `contracts/src/senior_contracts/wire.py` | 同对象显式转发 |
| `src/senior_control/navigation.py` 的 DTO | `contracts/src/senior_contracts/navigation.py` | DTO 转发，QualifiedRoute 执行留原处 |
| `src/senior_control/settings/models.py` 的 FrozenModel/DrivingSettings | `contracts/src/senior_contracts/configuration.py` | 配置值转发，设置业务留原处 |
| `ros2/src/senior_robot/senior_robot/motion_output.py` | `ros2/src/senior_motion_control/senior_motion_control/motion_output.py` | 同实现显式转发 |
| 同目录 `final_command_frame.py` | 新控制包 Python 同名文件 | 同实现显式转发 |
| `ros2/src/senior_motion_msgs/src/{motion_output_guard.cpp,final_command_guard.cpp,final_command_frame.hpp}` | 新控制包 `native/` 同名文件 | 删除旧源，字节完全相同 |
| `ros2/src/senior_motion_msgs/test/{test_final_command_frame.py,final_command_frame_sender.cpp}` | 新控制包 `test/` 同名文件 | 删除旧源，只更新 Python 路径 |

其余产品改动：contracts/control 的 `__init__.py`；新控制包 `native.py` 安装位置解析；SafetyGate 的实际导入/程序查找；vehicle_arbitration 的帧契约导入。

测试原位路径改动：`tests/unit/test_native_output_protocol.py`、`test_launch_recovery.py`、`test_motion_guard_to_owner.py`、`test_terminal_native_channel.py`。其中 test_launch_recovery 的旧 LearningClient monkeypatch 目标更新为已在 a445b1b 迁入的 vehicle_learning；原断言未删未弱化，防止旧夹具地址掩盖本批回归。

构建/安装共10文件：根 pyproject.toml/uv.lock、contracts/pyproject.toml、新控制包 CMakeLists.txt/package.xml/pyproject.toml、消息包 CMakeLists.txt、机器人 package.xml、backend.yml、Dockerfile.cloud。根 uv.sources 的本地 contracts 路径支持原 uv sync；CI 同时构建/安装三个 wheel；Docker COPY contracts，旧云 namespace 和管理安装入口不变。

工具/文档共8文件：本清单、新版确认稿、contracts README、控制包 README、根 README、ROS README、`tools/verify_batch_one_install.py`、批次验证报告。

本批实际产品22、测试6、配置构建10、工具文档8，共46个逻辑文件；纯移动的旧/新路径折算一次，保留在原处且职责变化的兼容转发文件独立计入。Git 原始旧/新路径共51个（含3个 C++ 与2个测试的移动前后路径）。均在预报40–66与各类别区间内，无额外模块实现。

## 记录与边界

- 契约只依赖标准库/Pydantic；`wire` 直接依赖契约 DTO，不再依赖 QualifiedRoute；公开旧类/函数只转发同对象，非仓库 wheel 验证覆盖了类型身份。
- Native 三文件逐字节等于 a445b1b，只移动归属；Python MotionOutput 仅将安装程序位置提到 native.py，不改 publish/停止协议。
- 离线构建首次缺缓存，获准获取构建与安装依赖后完成；隔离安装位于 /tmp，没有写入系统或父仓库虚拟环境。
- Linux/ROS 安装和现场仍未验；wheel 不包含 native 程序；既有两项 ProcessWorker 旧夹具错误单列在验证报告，未伪造属性使其通过。
- 单独恢复协议测试因沙箱阻止临时 Unix socket，使用测试权限重跑；不启动开发服务、车辆服务或部署。
- 自审：控制程序只剩新包一份源；旧 Python 路径显式转发；根确认稿只在 docs/architecture 一份；最终独立审阅由派发负责人执行，未创建辅助 worker。

执行完成：契约/输出职责迁移、构建安装接线、实际wheel安装和定向验证均已记录；代码自审没有发现需改变原算法的事项，待负责人独立亲审。最终证据见 `../testing/2026-11-19-vehicle-batch-one.md`，121通过/10跳过为两组相加，两个已知旧夹具错误单列，不宣称全量通过。
