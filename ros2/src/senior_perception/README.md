# 感知功能包

B2 首片以 `eb4a147` 为基线迁移已有雷达接入、解码、原始/补点观测与相关几何。
首片已迁雷达实现；下文 B2 第二片继续迁入学习 owner/服务与原矩形净距。速度、RGB-D 和其他完整感知职责仍待后批。
首片实际 48 个逻辑文件（迁移单计）：产品 25、测试/夹具 12、配置构建 8、工具文档 3。
其中 10 个原测试/辅助文件迁移，ROS conftest 保留旧入口转发同一实现，无新增场景。
没有新增算法、顶层项目、控制输出或运行功能占位。

## 实际目录与生产入口

```text
senior_perception/
  CMakeLists.txt, package.xml          # ament 安装同一 Python 功能包
  pyproject.toml                      # 无 ROS SDK wheel；不新增顶层项目
  senior_perception/
    __init__.py, py.typed
    scan_adapter.py                   # 原 ScanAdapter 节点类；过滤/建图候选与状态
    safety_scan_ingress.py            # 原 SafetyScanIngress 节点类；安全扫描候选
    raw_safety_observation.py          # RawSafetyObservation、PredictedReturn
    scan_gap_compensation.py           # 原三次真实采集种子过期与补点实现
    scan_decode.py                    # 原始/避障两种解码入口
    scan_access.py                    # 原物理帧身份和两个缓存槽
    self_filter.py                    # 原 ScanObservation/FilteredObservation、射线分类
    geometry_bounds.py                # 原精确几何的保守包围盒剪枝
  test/
    test_self_filter.py
    test_rear_geometry_equivalence.py
    rear_legacy_oracle.py              # 原冻结几何 oracle，未复制
ros2/tests/
  test_scan_gap_compensation.py        # 实际补点/原始隔离和缓存消费者
  test_motion_core_perception.py      # 原净距/速度消费者回归
  test_scan_adapter.py, test_scan_ingress.py
  test_scan_exact_stamp.py, test_scan_epoch_live.py
  test_scan_native_end_to_end.py
  conftest.py                         # 原生接收/封装夹具的唯一实现
```

源码目标均来自原 `senior_robot/senior_robot/` 的同名文件（scan_decode 原名
vehicle_sensor_decode，scan_access 原名 vehicle_sensor_access），以及原
`src/senior_control/safety/{self_filter,geometry_bounds}.py`。
旧路径保留标注 `//Compatible` 的显式重导出，同一类/函数不复制实现。
原 test 文件迁入对应包/ROS 工作区；尚未迁出的 ROS 测试通过旧 conftest 转发使用同一夹具。
原几何 fixture JSON 仍由其他未迁用例共用，保留在原 tests/fixtures，不复制数据。

实车和仿真仍选择现有 `senior_robot` 的 scan_adapter、safety_scan_ingress 可执行入口。
这两个模块现在只组装 `run(功能包节点类, cli=True)`，沿用原有界生命周期、信号与关闭顺序。
ROS 功能包不反向导入 senior_robot，不复制 executor，也不启动第二套 ROS 循环。
节点类通过本包的 ament Python 安装，senior_robot 清单新增本包运行依赖。
包内原几何与学习用例通过 ament_cmake_pytest 的 perception_algorithms 目标注册到 colcon test。
实车 safety_motion 和仿真 simulation_motion 的 launch、参数、重启监管和扫描重映射保持原值；
既有 `/scan` 建图/导航链继续沿用原候选/接入程序，原生 scan_receipt 仍归 senior_sensor_ingress。

SafetyGate 直接使用本包原始观测和 ScanGapCompensator；MotionCore 组装本包 SensorAccess。
旧 motion_core_perception 门面及原学习 owner 从本包公开解码入口消费结果；
原净距、速度与模型采用/学习控制仍留原 owner。本批只改学习 owner 的解码 import，未搬迁其业务状态或方法。
原覆盖/外部特征/学习视图也使用本包同一射线类型和几何函数。

## 公开契约与判断归属

| 接口 | 输入/输出与时间、来源含义 | 判断与状态 owner |
|---|---|---|
| ScanAdapter | 原 `/safety/raw/scan` BoundedRawLaserScan → `/perception/scan_candidate` BoundedFilteredScan、原诊断状态 | 来源/GID/代次、原时限与 TF 资格由节点原实现持有；只读消费原学习模型 |
| SafetyScanIngress | 同一有界原始接收 → `/safety/scan_candidate` BoundedSafetyScan | 来源/撤销与 body frame 变换由节点原实现持有 |
| RawSafetyObservation、PredictedReturn | 原消息、接收/测量单调秒、ROS 秒；补点保留原采集 seed_capture，不写回原 ranges | frozen 观测容器；消息按只读约定共享，来源/时钟元数据不重造 |
| scan_observation / avoidance_observation | 同物理帧原始 ranges / 原始 ranges 的独立派生副本 → FilteredObservation | 原始结果不含补点；避障仅替换合法缺测位置；沿用原几何过滤 |
| ScanObservation / filter_self | 雷达距离米、角度弧度、sensor_to_body 米/弧度、原 stamp/clock_domain/reference_frame → body 坐标射线与显式未知/自身/外部证据 | 本模块沿用原 self-mask 和精确射线规则，不新增感知资格/边界 |
| SensorAccess | 原 capture、来源/序号、测量/接收时刻、frame/TF、设备尺寸和 ranges → 原帧缓存键与独立 raw/avoid 槽 | 本对象持有原缓存身份/槽，学习 owner 继续执行原模型相关缓存消费；不迁学习 owner |
| ScanGapCompensator | 原始扫描、本地测量位姿历史、当前单调时刻 → 同类观测和独立 predictions | 本对象持有原种子与同帧结果；三次新采集过期、异常回退原始数据、原时限/来源变化清理均不变 |

`SampleTime`、帧龄策略、RawCapture、DeviceProfile/Pose2D、学习只读接口等仍用现有公开模块；
本片不重新分配这些 owner，也不将它们加入 WS contracts。
纯算法保留 NumPy/Shapely 和现有根 Python 配置/值接口；ROS 清单声明原实际运行依赖，
Python wheel 本身仅打包源文件，几何/ROS 执行仍需要原 safety/ros 运行依赖环境。
默认导入包、RawSafetyObservation 与 SensorAccess 不加载 rclpy 或 Shapely。

## 构建、安装与验证记录

根 pyproject/uv.lock 增加现有 ROS 包的 SDK 路径依赖，使根部兼容入口在安装后仍能找到同一实现。
云镜像 COPY 和 backend CI wheel 列表同步该本地构建源，不改部署或运行入口。
CI 保留已迁同名非 ROS 用例的收集；root pytest 的测试 helper 路径仅用于兼容尚未迁出的用例。

环境：macOS 26.4 arm64、CPython 3.12.15、pytest 8.4.2、Pydantic 2.14.1、uv 0.12.23、ruff 0.17.3。
使用原主仓库 `.venv/bin/python`；构建/安装操作仅在本地 /tmp 产物与隔离环境，不启开发/实车服务。

- RED：迁移原 test_self_filter import 后，运行同名用例在 senior_perception 未实现时按 ModuleNotFoundError 失败。
- `python -m pytest ros2/src/senior_perception/test ros2/tests/test_scan_gap_compensation.py ros2/tests/test_motion_core_perception.py ros2/tests/test_scan_adapter.py ros2/tests/test_scan_ingress.py ros2/tests/test_scan_exact_stamp.py ros2/tests/test_scan_epoch_live.py ros2/tests/test_scan_native_end_to_end.py tests/unit/test_motion_launch_profile.py tests/unit/test_sim_contract.py tests/ros/test_mapping.py -q`：38 passed、14 skipped；跳过项需要 rclpy/真实 ROS，包含实际 TF、来源替换、扫描/建图接线与生命周期。
- 另以 `python -m pytest ros2/src/senior_perception/test ros2/tests/test_scan_gap_compensation.py ros2/tests/test_motion_core_perception.py tests/unit/test_body_return_geometry_only.py -q`：30 passed、3 errors；三个原几何用例跳过构造器后设置 learning 委托属性，缺 learning owner。完整 eb4a147 archive 中单独运行 test_body_return_geometry_only 得 12 passed、相同 3 errors，未为迁移修改原学习逻辑或该旧夹具。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --offline`：解析 63 包，仅增加 senior-perception；`uv lock --check --offline` 核对锁。
- 同缓存 `uv build ros2/src/senior_perception --offline --out-dir /tmp/b2-wheels` 与根 `uv build`：sdist/wheel 构建通过；用 `uv pip install --offline --python /tmp/vehicle-batch-one-venv/bin/python --no-deps --reinstall /tmp/b2-wheels/*.whl` 更新隔离安装，从 `/tmp` 运行 tools/verify_batch_one_install.py，通过实际安装 SDK 导入/归属检查，未加载 ROS；原生安装查找仍因无 ament 未验证；uv pip check 检查隔离环境 13 个包依赖通过，撤回的 runtime 项目未参与安装闭包。
- 定向 ruff check/format、diff 检查与兼容入口身份/实现迁移核对通过。原始扫描、补点、TF/时间、缓存/截止算法沿用原实现，没有改变 guard 40 字节/停止协议。

本机无 colcon、CMake、rclpy/ament，未执行真实 ROS 构建、节点导入/安装定位、DDS/TF、
实车/仿真/建图导航运行；静态/无 ROS SDK 验证不代替它们。ROS Humble 环境需
`colcon build --base-paths ros2/src --packages-up-to senior_robot senior_perception` 后，
先执行 `colcon test --packages-select senior_perception`，再执行已迁 ROS 回归及原建图/导航入口回归，单独登记实测。没有新增测试场景、部署或推送。


## B2 第二片：学习与矩形净距

基线 `3b61157`，只迁既有学习业务与旧矩形净距；不做单次解码、运动补偿、新守卫或其他包迁移。
实际 40 个逻辑文件：产品 28、测试/辅助 8、配置构建 2、文档 2；迁移原用例，不新增无关场景。

### 实际源、目标与状态 owner

以下目标均在 `senior_perception/` Python 包内，旧路径是标注 Compatible 的同一实现重导出。

| 目标 | 原生产源 | 业务/接口归属 |
|---|---|---|
| learning_record.py | src/senior_control/motion/learning_record.py | 原冻结记录、标识、几何与持久模型 DTO，字节上限和序列化约束不变 |
| startup_learning.py | src/senior_control/motion/startup_learning.py | 原三秒窗口、动作首次提交、一次消耗、接管与阶段投影 |
| learning_owner.py | src/senior_control/runtime/learning_owner.py | 本 run 持久记录、版本及恢复；沿用原提交 worker |
| learning_store.py | 同目录 learning_store.py | 原原子落盘、锁、权限/常规文件检查与读取 |
| learning_service.py | 同目录 learning_service.py | 原 Linux 私有 IPC、peer 身份、有界队列及 owner/计算生命周期 |
| learning_compute.py | 同目录 learning_compute.py | 原独立 spawn 计算与只读批次、结果、原时间/来源和迟到处理 |
| learning_wire.py | 同目录 learning_wire.py | 原车内类型与校验，Unix IPC 不转成云车 WS 契约 |
| learning_client.py | 同目录 learning_client.py | 原只读/动作客户端、采集器、轮询与命令 FIFO |
| learning_cleanup.py | 同目录 learning_cleanup.py | 原 run 文件清理与锁/目录边界 |
| learning_runtime.py | senior_robot/vehicle_learning.py | 学习采用、过滤、缓存消费与失败事实；新只读控制输入/请求接口 |
| collision.py | senior_robot/vehicle_collision.py | 原 DirectionalClearance/CollisionPrediction，旧矩形/时限结果 |

服务、存储、计算、记录、startup、客户端、清理及净距这十份业务 AST 与基线一致（仅 imports 与兼容日志类别除外）。
学习 Runtime 的边界调整见下节；原模型拟合/可见性函数继续复用现有公开实现，不在本片重写算法。
实际生产 `docker_start_helper` / `simulation_cli` 仍在原启动、监管和退出时机创建/启动/关闭 LearningService，
只改为直接引用本功能包；SafetyGate 的 LearningFrames、雷达只读客户端也引用同一 canonical 实现。
没有新增服务入口、顶层发行项目或绕过原监管的后台服务。原控制 guard、40 字节协议及 ROS launch 保持。

### 学习只读输入、控制应用请求

`LearningControlInput` 是 frozen 值快照：输出 boot、输入 boot、generation、mode/task_id、
目标来源/不可变 Velocity、恢复标记和轮次。它不携带 mutable target、machine 或 output。
`LearningPorts` 只提供原观测/配置的只读 getter、所属扫描缓存及 trace/close 生命周期接口，
不传控制 context。getter 保留原读取时机与动态配置/拓扑观察，不固化启动时的传感器状态。

学习返回 frozen `LearningMotionRequest`（target 或 retire），携带输出/输入 boot 与 generation。
速度仍为原前向 m/s、零角速度，期限由原 Startup/action 规则解释。
控制的 CommandArbitration.apply_learning_request 同步核对当前身份，负责 TASK_START/TASK_RETIRED、
目标和原输出清零；其现有命令/锁存规则保留，旧学习退场不能撤较新的权限。
成功应用 target 后才通过学习 owner 的 applied 接口确认原 reason，不跨 owner 写学习状态。
请求仅用于同一控制调用的同步返回/应用，不是排队权限、持久授权或新的运动线协议。

LearningRuntime 不导入 senior_robot，也不读写 context.target/machine/_output 或发控制事件。
MotionCore 组装 ports、采样输入并调用仲裁应用结果；学习不能直接改控制状态。
原三秒期限、首次提交前后区别、动作消耗一次、手动接管及同 run 重建不复活、
迟到计算可提交模型但不续运动，以及原始/补点缓存隔离均保留。

### 测试位置、命令与结果

迁入本包 test/：test_learning_{client,cleanup,record_owner,compute,dual_clock,service_linux}.py，
以及唯一 learning_test_support.py；真实控制组装用例迁入 ros2/tests/test_startup_learning_chain.py。
原手动接管用例扩展为先取只读快照/学习退场结果，再用真实 prepared 命令更新权限，
证明学习返回时未改目标且迟到旧请求不能改新目标/事件序号。断言未删弱，未新增无关场景。
包内 ament pytest 目标与 backend CI 同步收集这些原用例。

环境：macOS 26.4 arm64、CPython 3.12.15、pytest 8.4.2、Pydantic 2.14.1、NumPy 2.5.4、Shapely 2.2.0、uv 0.12.23、ruff 0.17.3。
使用原主仓库 .venv/bin/python；基线 archive 位于 /tmp/b22-baseline。

- RED：扩展原接管用例先运行，因 MotionCore 没有只读学习输入接口失败；完成请求接线后通过。
- `python -m pytest ros2/src/senior_perception/test/test_learning_client.py ros2/src/senior_perception/test/test_learning_cleanup.py ros2/src/senior_perception/test/test_learning_record_owner.py ros2/src/senior_perception/test/test_learning_compute.py ros2/src/senior_perception/test/test_learning_dual_clock.py ros2/src/senior_perception/test/test_learning_service_linux.py ros2/tests/test_startup_learning_chain.py ros2/tests/test_motion_core_perception.py ros2/tests/test_scan_gap_compensation.py -q`：75 passed、1 skipped；跳过的是 Linux abstract SEQPACKET / SO_PEERCRED 实际服务/线程/spawn 集成，不是以静态检查代替它。
- `python -m pytest tests/unit/test_task_terminal_release.py tests/unit/test_terminal_native_channel.py -q`：6 passed、27 errors；旧 helper 仍替换不存在的 MotionCoreOutput.ProcessWorker，错误在 setup 发生，完整 3b61157 基线同命令得到相同 6 passed、27 errors。这些原保护和断言均保留，未为通过旧夹具添假属性；旧权限保护还由前述接管用例的真实控制请求路径覆盖。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build ros2/src/senior_perception --offline --out-dir /tmp/b22-wheels` 和根 `uv build`：sdist/wheel 构建通过；`uv pip install --offline --python /tmp/vehicle-batch-one-venv/bin/python --reinstall --no-deps /tmp/b22-wheels/*.whl` 后从 /tmp 导入 canonical/旧路径的 LearningRecord、StartupLearning，身份一致且不加载 ROS。服务仍需原 safety 依赖与 Linux 运行环境。
- 逐份 AST 核对十份未改业务实现、兼容入口身份与无反向 senior_robot 导入；定向 ruff check/format、git diff --check 通过。Root 运行保护、启动时机和持久文件名/格式/权限/锁未改。

未验证：实际 ROS 构建/安装、实车或仿真运行；本机无 colcon/CMake/rclpy/ament。
实际 Linux 私有 IPC/peer credential/计算子进程全链未运行；已有 Linux 用例保留待目标环境执行。
原终端/原生身份通道用例被基线夹具阻断，不宣称该全链已通过。
没有启动开发/实车服务、部署或推送。后续算法与其他功能包须另批实施。
