# 车端控制、导航与建图重构计划

> 当前执行顺序：先交整体架构骨架，再按完整功能迁移已有实现，缺失功能在所属模块阶段实现，最后整链联调。骨架与实现迁移分开提交。

**目标**　将控制、感知、定位、建图和导航拆成可独立运行测试、独立评估算法的功能模块。

**架构**　同一功能的 ROS 接入、算法、状态和公开接口放在同一功能目录。模块通过公开契约交互，各自持有业务状态。实车和仿真共用功能模块，车端管理层负责组装、调度和监管。

**技术基线**　ROS 2 Humble、现有 Python/C++ 实现与本地 IPC。固定功能主分支 `feat/vehicle-architecture-refactor` 最初从 `main@cfbc5e4` 分叉；本骨架批次基线为 `c438948`，直接提交同一功能分支。

**设计依据**　以 [架构确认稿](../../architecture/ROS2与车端架构重构确认稿.md) 和用户最新的骨架先行、目录及命名要求为准。原讨论方案与 `docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md` 仅提供历史背景。云项目外层本批采用已确认的可逆名称 `cloud/`，新增 Python 导入名使用 `vehicle_manager` 与 `cloud`。

## 模块、接口与判断归属

接口名称是实现计划中的契约名。阶段一固定消息字段、传输、单位、坐标系、测量时间及缺失语义，供阶段二并行开发。模块边界与进程边界分别确定。

| 模块 | 消费 → 产出 | 负责人需要列出的判断 | 现有迁移起点 |
|---|---|---|---|
| 传感器接入与解码 | 原始帧 → 只读观测，含帧身份、来源、测量时间、坐标系及标定版本 | 来源、格式、时间戳结构、标定和解码完整性。保留合法历史观测 | `ros2/src/senior_sensor_ingress`、`safety_node.py`、`motion_core_perception.py` |
| 自车运动估计 | 轮速、雷达相对运动、已有 IMU、可选 GPS 速度 → 本地连续速度与位姿增量历史，含时间和不确定性 | 时间对齐、来源权重、观测一致性和轮滑 | `motion_sensor_runtime.py`、`ros2/src/senior_scan_matcher` |
| 回波学习 | 原始观测、动作结果 → 模型快照、学习状态和运动请求 | 几何匹配、模型采用、恢复、异常和动作结束 | `motion_core_output.py` 中学习成员、`src/senior_control/motion/startup_learning.py`、`src/senior_control/runtime/learning_*.py` |
| 碰撞预测 | 已解码观测、本地连续运动历史、适用的回波模型 → 当前净距／碰撞风险和不确定性 | 自车运动补偿、动态障碍不确定性、历史不足时的保守计算 | `motion_core_perception.py` |
| 指令仲裁 | 手动、导航、学习请求及安全状态 → 选定目标和控制权状态 | 来源、代次、顺序、接管、取消、急停、释放及旧任务退场 | `motion_core_output.py` 的命令、仲裁和退场逻辑 |
| 速度计算 | 选定目标、融合车速、碰撞结果、上次输出、控制参数 → 候选速度和原因 | 加减速、限速和避障决策 | `motion_core_perception.py` 的限速实现 |
| MotionCore | 控制节拍和各模块结果 → 调用速度计算、提交本轮指令 | 调度顺序与本轮输入组装 | `motion_core_output.py`、`safety_node.py` 的控制定时器 |
| 输出守卫 | 速度、序号、诊断用计算时刻 → 唯一最终 `/cmd_vel` | 包质量、顺序、最新候选、本地断流 | `ros2/src/senior_motion_control/senior_motion_control/motion_output.py`、`ros2/src/senior_motion_control/native/motion_output_guard.cpp` |
| 地图定位 | 实时雷达、持久地图、本地运动、可选 GPS 位置 → 地图位姿、质量和定位状态 | 匹配质量、漂移纠正、全局重定位、手动初值局部微调 | `src/senior_control/localization`、`ros2/src/senior_slam_bridge` |
| 地图管理与建图 | 观测、可信地图位姿、地图操作 → 地图版本、活动地图、独立 Home、实时障碍与持久地图结果 | 地图切换生命周期、观测变化证据、持久更新条件 | `src/senior_control/mapping`、`ros2/src/senior_slam_bridge` |
| 导航 | 原始目标、活动地图、定位结果及实时障碍 → 路径、探索状态、运动意图 | 已知路径可达性、可达前沿、探索进展、重规划、任务结果 | `src/senior_control/navigation.py`、`src/senior_control/hardware_navigation.py`、现有 Nav2 接入 |
| GPS 适配 | 未来接收机 → 标准位置和可选速度观测 | 数据格式、解类型及设备质量字段 | `src/senior_control/localization/geo.py` 用于核对坐标契约 |
| 统一诊断 | 模块事件 → 可筛选、可关联的调试记录 | 启动选择模块／事件／帧采集、容量和丢失计数 | `motion_output.py` 追踪、`src/senior_control/motion/diagnostic_pump.py`、`vehicle_logs` |

ROS Python 文件简称均位于 `ros2/src/senior_robot/senior_robot/`。每个最终功能包的 README 列出实际节点／进程、接口、输入单位与时间基准、每项校验的归属、失败行为和独立验证方式。统一诊断负责序列化、写盘和轮转，业务模块提交结构化事件，控制路径用有界非阻塞入口。按帧 ID、任务／轮次和速度序号串联数据、判断、经过时间与线程 CPU 时间。

### 重复逻辑收敛原则

同一业务规则由一个模块负责。同一输入、同一语义且结果可安全复用的判断或计算，优先由该模块执行一次并提供结果。优先收敛雷达逐点解码、坐标转换与同条件过滤，以及分散的统一帧龄判断。只读原始点可按帧复用，运动补偿仍按本轮位姿计算。控制与诊断可复用同一轮的运动状态快照。

简单、耗时可忽略的局部操作保留在使用处。不同方向、不同输入或不同业务语义所需的判断分别保留。第一阶段接好数据和职责边界，第二阶段再按模块消除实际重复并定向验证。

### 云车共享契约的范围

- [x] 按实际生产消息逐项核对 `contracts/` 中的类型。只有云端与车端 WebSocket 消息实际引用的类型留在共享契约，包括消息所需的嵌套类型。
- [x] 仅供单个车内模块使用的类型归该模块。车端管理与 ROS 之间的 Unix socket 消息归本地 IPC 接口，ROS 消息、守卫帧及执行状态按各自实际使用方归属。若车内状态也需要云端展示，由车端转换成云车协议所需的字段。
- [x] 迁移验收记录每个保留类型对应的云车消息，核对共享契约无车内执行或 ROS 依赖。复用现有协议测试检查线上消息字段和序列化兼容性，并定向验证受迁移影响的本地调用。

## 第一阶段　骨架先行，再迁移既有行为

### 当前实际布局与能力状态

```text
contracts/src/senior_contracts/              # 已实现：实际云车 WS 消息与嵌套 DTO
senior_control/                             # 本批骨架：独立车端管理项目
  pyproject.toml, README.md
  src/vehicle_manager/__init__.py           # 导入标识，无生产执行实现
  tests/README.md
cloud/                                     # 本批骨架：独立云项目，可逆目录名
  pyproject.toml, README.md
  src/cloud/__init__.py                     # 导入标识，无服务入口
  tests/README.md
ros2/src/
  senior_motion_control/                   # 已实现：两个 guard、帧、MotionOutput
  senior_sensor_ingress/                   # B1：native_receipt 程序与客户端已归此包，ROS 实测待补
  senior_motion_estimation/                # B3：既有观测/融合/位姿与公开只读快照已迁
  senior_perception/                       # B2 首片：雷达/解码/补点/相关几何已迁，其他职责待续
  senior_localization/
  senior_mapping/
  senior_navigation/
  senior_robot/                            # 既有实车组合，生产入口保持
  senior_sim/                              # 既有仿真组合，生产入口保持
ros2/tests/                               # B1 已迁入两份原 ROS 接收/关停用例
src/senior_control/                        # 尚未迁完的现有生产实现，继续工作
```

五个新增 ROS 骨架各只有 `CMakeLists.txt`、`package.xml`、`README.md`。
它们是可发现的 ament 包，尚无节点、算法、launch、运行参数或业务返回值。
源码、公开接口与测试的后续落点写在各包 README，真实文件迁入时才创建/安装。
没有重复创建现有 `senior_motion_control`，没有新增根目录 `runtime` 或其他技术发行包。
默认生产构建/launch 不引用新骨架；根项目安装配置与已有服务入口不切换。

### 实际批次、源目标、依赖与完成条件

| 批次与状态 | 实际源 → 目标 | 依赖与验收条件 |
|---|---|---|
| C0 已落地：共享契约收敛 | 原协议消息 → `contracts/src/senior_contracts/`；云缓存 → 现有 `src/senior_control/cloud/snapshots.py`；车内配置/状态 → 原所属本地模块 | `3dfd1e0` 后在 `6f4a235`、`c438948` 收敛；核对实际云车 WS 校验/序列化、嵌套 RouteProgress、日志 DTO 与原线上省略/接受规则；详见 contracts README |
| C1 已落地：控制输出归包 | 原消息包两个 guard 与旧 Python 输出实现 → `ros2/src/senior_motion_control/` | `3dfd1e0`；保留原 40 字节 ABI、截止/停止屏障与实车/仿真唯一输出调用，安装与环境限制见原验证记录；不代表新 250 ms 协议已实现 |
| B0 当前：整体骨架 | 无实现搬迁 → 本节实际布局中的两个 Python 项目和五个 ROS 功能包 | 本批只验构建声明、仓库外导入、ROS 包元数据与包发现；骨架提交先由负责人审阅，再继续 B1；ROS/ament 缺失必须登记，不能静态冒充实测 |
| B1 已迁实现，ROS 实测待补：原生接收 | `senior_motion_msgs/src/native_receipt.cpp`、`senior_robot/senior_robot/native_receipt.py` → `senior_sensor_ingress/src/` 与其 Python 客户端 | 依赖 B0；从 51b25e6 单独交付，程序逐字节迁移，Python 旧入口转发同一客户端；SampleTime/帧龄策略、进程与诊断继续使用原 owner 公开接口；原接收/关停用例移入 ros2/tests，35 项非 ROS 定向通过、10 项 ROS 跳过，真实 typesupport/安装/接收/关停验证尚缺，详见 ros2 README |
| B2 首片已迁，完整职责待续：感知 | `senior_robot` 的 sensor decode、collision、learning、`motion_core_perception.py`，及 `src/senior_control/safety/`、`vision/` 中对应执行实现 → `senior_perception` 内部模块 | 依赖 B1；首片从 eb4a147 迁雷达节点、解码、原始/补点观测、帧缓存与自身几何到 senior_perception；原组合入口只组装同一节点，38 项通过、14 项 ROS 跳过；第二片继续迁学习 owner/服务/存储/计算与原矩形净距，并以只读控制输入和返回请求隔离仲裁；速度及其他感知职责待后批；详见功能包 README |
| B3 已迁既有行为，ROS 待验：运动估计 | `motion_sensor_runtime.py`、`motion_observation_adapter.py` 与现有 `senior_scan_matcher` 调用 → `senior_motion_estimation` | 依赖 B1/B2 的既有接口；从 155e2b9 迁入观测适配、准入/时序/融合、运动与连续位姿 owner，ControlInputs 经 frozen 决策快照替代字典写入；127 项通过、2 项 ROS 跳过，两个 retired 激光组合用例已登记；GPS/新算法仍留实现阶段 |
| B4 已迁既有控制职责，ROS 实测待补 | `vehicle_arbitration.py`、`vehicle_speed.py`、`motion_core_output.py`、`control_inputs.py` 及现有执行控制逻辑 → `senior_motion_control`；`senior_robot`/`senior_sim` 只选设备/参数/组装 | 依赖 B2/B3 公开结果；MotionCore 只编排，仲裁/速度/业务状态由所属 owner 持有；先迁旧行为并验手动/学习/导航权限退场、原停止屏障及实车/仿真实际装配；新截止/断流算法在该模块阶段另批实现 |
| B5.1–B5.3 已迁既有Python/原生与持久事务，缺失功能待续 | `src/senior_control/localization/`、`mapping/`、`localization_node.py`、`senior_slam_bridge` 实际调用与 `senior_robot/launch/mapping.launch.py` 组合 → `senior_localization`/`senior_mapping` 及原组合入口 | 依赖接入和本地运动接口；先保留并迁移已有实时建图/地图管理/重定位，随后在本轮实现缺失地图操作、独立 Home、可信定位协作；复用原存储/地图/定位回归，新增功能测试先单独审定；切图未确认不导航或持久写图 |
| B6.C1–C3 云与M1a设置/M1b日志已迁，镜像实测与管理余项待续 | 现有 `src/senior_control/agent/`、`runtime/` 中管理职责、settings、vehicle_logs、CLI/监管 → `senior_control/src/vehicle_manager/`；现有 `src/senior_control/cloud/` 及云专用依赖 → `cloud/src/cloud/` | 公开 IPC 和业务 owner 稳定后按生产链迁移；云端只依赖共享 WS 契约，不导入车端快照实现；逐项复用编解码/relay、日志设置 wire、快照过期、监管/隔离回归，核对锁、CI wheel、安装与现有启动入口；完成前原根项目继续承载生产实现 |
| B6.M2a SQLite引擎已迁，domain及Home协调仍原位 | 原 `storage/database.py` 连接引擎、`migrations.py`、`core_schema.py` → `vehicle_manager/storage/` | 旧Database Compatible子类只装配原DiagnosticLock/Home协调，连接实现唯一；Migration/core入口同一对象，原生产同连接/写锁、SQL/备份/取消不变；domain schema/map_persistence仍在原边界，不把地图/任务执行打入manager安装闭包 |
| B6.M2b journal SQL数据已迁，值归属仅盘点 | 原agent/journal的AGENT_JOURNAL_STATEMENTS → `vehicle_manager/storage/agent_schema.py` | 旧入口同一tuple，journal执行owner仍原位；无import的新数据模块不拉起agent/ROS，不搬MapRef/Pose2D/OperationResult/DomainError；下一片候选先迁标准库纯schedule/lifecycle，完整任务驱动依赖另核 |
| B6.M2c 标准库纯任务核心已迁 | 原tasks/schedule、lifecycle → `vehicle_manager/tasks/` | 旧对象显式Compatible同一身份，旧公开包名字不变且按需加载；模型/驱动、MapRef/Pose2D/错误及ROS边界不迁；复用原时间/状态机和服务层锁外dispatch及离仓库安装验证 |
| B6.M2d 纯任务模型/Runner已迁 | 原tasks/models、runner → `vehicle_manager/tasks/`及只读MapReference形状 | 原运行时MapRef及比较保留，旧类/常量/函数同一身份；仅经确认扩两个旧具体MapRef调用点的Compatible静态cast，不作运行时校验或改准入；Store/Runtime/地图/ROS/IPC实现不迁 |
| B7 后置：既有导航迁移，再实现新导航 | `src/senior_control/navigation.py`、`src/senior_control/hardware_navigation.py`、`ros2/src/senior_robot/senior_robot/navigation_adapter.py` 与现有 Nav2 对接 → `senior_navigation` | 既有导航在此前批次保留可用；依赖 B3/B4 的运动接口和 B5 的地图/定位接口稳定，先迁原目标/进度/取消/装配行为，再实现前沿探索等新算法；所有运动仍经共同控制，导航新算法不得前置 |
| B8 待联调 | 各功能包与原实车/仿真组合、车端/云端独立项目 | 依赖各前项验收；实际 ROS 图、构建安装、仿真/实车与地图导航整链验收，未验能力逐项登记，不靠骨架或占位宣称可用 |

每批实施前重新核对实际文件清单及测试候选，预报范围后再动手。迁移先复用现有算法和同名用例；
算法变化与缺失功能在第二阶段相应模块内单独实现，不把新算法混入目录搬迁。
已有建图和导航在新实现完整接线验收前继续从原入口提供能力。

### 测试按所属项目收纳

管理、云端与 ROS 工作区分别拥有 `senior_control/tests/`、`cloud/tests/`、`ros2/tests/`。
功能包自身 C++/算法单元测试留所属包 `test/`；ROS 工作区级接口、实车/仿真组装和跨功能链回归归 `ros2/tests/`。
B0 骨架阶段这些新目录只有职责 README；B1 已随接收实现迁入两份原 ROS 用例，其余后续随实现迁入并更新发现/安装路径，
不复制测试或先撤掉现有生产链覆盖。根目录 `tests/` 中现有用例也需逐项明确所属项目，不能作为各项目长期混放目录。

### B0 骨架验证记录

基线 `c438948`，本批 25 文件：产品标识 2、测试代码/数据 0、配置构建 12、工具文档 11。
其中 `ros2/tests/README.md` 按用户追加的测试归属要求补齐；没有算法、运行入口或测试场景变化。
新增 24 文件，修改本计划 1 文件，没有删除/重命名文件。B0 提交当时接收迁移仅为未提交草稿，不属于骨架提交；后续进度见 B1。

环境：macOS 26.4 arm64、CPython 3.12.15、uv 0.12.23、ruff 0.17.3。

- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/vehicle-architecture-skeleton-wheels`：sdist 和 wheel 构建通过。
- 同一命令将构建目录换为 `cloud`：sdist 和 wheel 构建通过。
- `uv venv /tmp/vehicle-architecture-skeleton-venv` 后，`uv pip install --offline --python /tmp/vehicle-architecture-skeleton-venv/bin/python /tmp/vehicle-architecture-skeleton-wheels/*.whl`：两个 wheel 安装通过；从 `/tmp` 使用该解释器导入 `vehicle_manager`、`cloud`，确认来自该环境 site-packages，元数据无运行入口或业务依赖。
- 用 CPython `tomllib`/`xml.etree.ElementTree` 检查两个 pyproject 源码布局、五个新 ROS 清单的唯一包名/ament 类型、CMake project/ament_package 声明及无 executable/install 目标：通过；当前 ROS 清单共 12 个包名且无重复。这是静态元数据检查。
- `ruff check senior_control/src cloud/src`、`ruff format --check senior_control/src cloud/src`、`git diff --check`：通过。
- `colcon`、`cmake`、`rclpy`、`catkin_pkg` 本机均不可用；未执行实际 colcon 包发现、ROS/ament 构建/安装、ROS 测试或生产/仿真运行。没有启动服务、部署或推送。

本批仅对骨架作上述验证，没有将此前接收迁移草稿的测试证据计为骨架生产能力验证。
所有本轮自建根 runtime 草稿及其依赖、安装/锁、进程、executor、诊断变更已按文件精准撤回；
原 owner 与已有实现保持，B0 提交当时接收迁移另外保留；B1 随后按收窄范围继续。

### 决定、待定与禁止绕行

已决定：根目录 `senior_control/`、可逆云目录 `cloud/`、现有 `contracts/`，
新增 Python 导入名 `vehicle_manager`/`cloud`，上述同级 ROS 包名与现有实车/仿真组合入口。
待定：管理/云内部模块拆分及配置/部署入口细节，除现有协议外的新本地 IPC/ROS transport 选择，
跨项目通用诊断与进程资源的最终复用边界。它们目前保留现 owner，必须按真实调用者另行定案。
不得因待定而再建根目录 runtime 项目，也不得放进云车 WS contracts。

## 第二阶段　按模块实现并行分支，执行各自测试

完成对应骨架和既有行为迁移、公共接口冻结后，下表工作才可按已确认依赖安排。依赖其他模块结果的算法先使用已冻结契约的固定输入回放。实际联接在阶段三完成。子任务发现必须变更契约时，先由负责人更新公共契约并同步使用方。

| 子任务 | 实现与交付 | 定向验证 |
|---|---|---|
| A 感知与学习 | 同一物理帧按帧身份和标定版本解码一次。学习复用原始观测，避障在共享解码结果上派生补点视图。保持真实观测与补点来源区分。复用相同过滤结果；仅补点或模型变化时重算受影响的结果。学习模块独立管理模型、恢复及动作申请。 | 同帧进入多条消费者路径只解码一次。补点不进入学习原始观测。复用启动学习链已有覆盖，迁移其调用边界。 |
| B 自车运动融合与 GPS 契约 | 用轮速、雷达运动、可用 IMU 和 GPS 观测综合估计。处理观测时刻、质量和一致性，输出连续本地运动历史。GPS 位置采用 `NavSatFix` 与关联的解状态，区分无解、单点、RTK 浮点、RTK 固定，携带测量时间、天线坐标系和协方差。速度为可选带时间与协方差的向量。定位持有地理原点、地图对齐和天线外参。单点与 RTK 按质量进入融合，GPS 缺席时继续使用现有观测。交付契约和回放接入，硬件驱动留待设备安装。 | 回放轮速与雷达不一致、GPS 缺席及单点／RTK 降级，检查融合结果、连续性和质量。核对未来生产者可替换模拟输入。 |
| C 碰撞预测与速度计算 | 使用带测量时间的观测和连续自车运动历史，把障碍变换到当前车体坐标系，再计算净距与风险。处理转向、动态障碍与历史不足。速度算法消费融合车速与风险。合法传感器观测保留为历史证据，其信任程度由使用算法决定；传感器时间不随候选速度向后传播。 | 独立回放 150 ms 前的真实时间语义。直行夹具以 1.00 m 初始净距、0.12 m 自车位移得到 0.88 m；用有转动的夹具验证坐标变换。报告观测年龄与算法耗时。 |
| D 输出守卫与控制节拍 | Python 与 C++ 一起迁移协议，移除合并绝对截止及停止时间屏障。守卫按序号发布最新合格包，用自己的单调时钟从新指令被接受时计断流。默认控制 20 Hz，阈值为五个配置周期，即 250 ms。断流发零，下一条更新的合格包可恢复。计算起点只用于延迟诊断。上游仲裁落实取消、急停和学习结束，必要时主动送零。真实连接断开与协议错误的保护独立保留。 | 用现有 C++ 确定时钟 harness 验证默认 250 ms、队列取新、重复或乱序不续计时、慢计算包仍可发布、断流发零和新包恢复。Python 协议测试核对两端字段与节拍配置，记录计算、发送、接收、发布各段延迟。 |
| E 地图管理与定位 | 支持实时建图、新建、保存历史、切换和删除；保存继续建图和重定位所需数据及每图 Home 位姿。Home 默认建图开始车位，可拖动修改位置和朝向。切图尝试自动匹配；失败仅在界面将标记放到目标地图 Home，提示待校准，暂停导航和持久写图。用户给出车位和朝向后局部扫描匹配微调；匹配可信才提交真实位姿。将实时障碍、持久地图分开，用稳定结构和独立运动证据纠偏，环境变化由多帧与可见性证据确认后更新地图。 | 回放切图自动定位、失败转 Home 待校准与手动微调。用车被挪动和车辆障碍驶离的录制场景核对纠偏与地图更新由不同模块处理。覆盖地图操作持久化与 Home 独立性。 |
| F 导航与前沿探索 | 保留原始任务目标。已知路径可达时直接规划；目标在未知区或暂不可达时，从已知可通行区域选可达前沿，实时建图并重规划，目标可达后回到原始目标。探索评分考虑进展、重复与绕路成本，预算和评分作为导航策略参数。导航依据定位质量、地图版本和当前障碍结果，输出意图。 | 用地图夹具验证目标外、到前沿、地图扩展及切回原目标。覆盖没有可达前沿的停机理由与切图后旧路线失效。 |
| G 统一诊断与地图交互 | 实现启动选择模块、事件和传感器帧，集中落盘与关联。地图界面实现新建／保存／切换／删除、Home 编辑、手动位姿和待校准流程。界面先在功能原位置提供交互预览，确认后接入真实接口并移除临时代码。 | 验证按启动配置采集、帧与输出关联及拥塞丢失计数。界面回归验证待校准标记、Home 编辑和车位编辑独立。 |

每项先只读盘点候选测试，由负责人审核后落实。按仓库约定控制新增／改写场景，优先复用旧用例。每项只执行模块定向检查，报告命令、版本、通过结果和未验证项。负责人亲审 diff 和证据，按依赖顺序正常 merge 回固定功能主分支。当前任务直接在固定功能主分支交付；后续分支/并行安排另行派发，整车可用性由第三阶段验收。

## 第三阶段　联调和验收

- [ ] 移除阶段一仅用于迁移的旧入口、兼容路径和测试占位组装。核对实车与仿真组装共用模块，核对 ROS 图和底盘唯一发布者。
- [ ] 联调车辆运动链。覆盖手动、导航和学习意图的控制权切换、显式停车、Python 停更后的守卫断流停车和收到新指令后恢复。检查观测时间留在使用算法内、速度包无传感器截止字段。
- [ ] 联调地图和导航。覆盖切换地图、自动及手动重定位、Home 待校准、地图外目标探索到达、动态障碍与位姿纠偏。回放模拟 GPS 缺席、普通定位和 RTK 模式变化。
- [ ] 对比重构前后同输入的解码次数、感知与控制耗时、传输至守卫延迟和输出周期。报告车端运行中的经过时间与线程 CPU 时间，保留可复现日志。
- [ ] 运行受影响的跨模块回归、构建和仿真端到端验收，再安排实车验证。GPS 硬件未安装期间，报告只覆盖契约与回放。实车验证单独记录部署版本及现场结果。
- [ ] 整理验证和限制，推送固定功能主分支并更新同一 PR。按项目约定由用户合入 main。

### 共享契约收敛实施记录

功能分支以 `3dfd1e0` 为基线按上述云车实际消息范围收敛；逐类型生产调用、车内归属、实际文件范围和定向验证见 [contracts README](../../../contracts/README.md)。云缓存由 cloud 私有模块拥有，车内接口未升级为云车消息；后续运动功能包迁移尚未开始。本记录不代表整车或全部旧测试已验收。


### B1 原生接收迁移记录

基线 `51b25e6`，固定功能分支直接提交。本批仅原生接收与客户端，
不包含此前撤回的根 runtime 或后续感知迁移。
实际目录、生产调用、公开入口身份、依赖与完整验证证据见
[ROS 接收迁移记录](../../../ros2/README.md#b1原生接收归传感器接入包)。
现有时间/帧龄、进程/诊断 owner 保留原位；实际 ROS/ament 安装和原生接收仍需对应环境续验。
已迁入两份原 ROS 接收用例到 `ros2/tests/`，无新增场景；受影响的激光发布原夹具失败在基线复现并单独登记。
负责人亲审本批后才继续后续职责迁移。


### B2 首片记录

从 `eb4a147` 接续 B1，本片迁移雷达接入、解码、原始与补点观测、帧身份缓存和对应几何。
[senior_perception README](../../../ros2/src/senior_perception/README.md) 列出实际源/目标、公开输入输出、
节点/算法/接口/测试落点、真实组合与安装依赖及验证命令版本结果。
原实车/仿真可执行入口只调用迁入节点，沿用既有生命周期；功能包不反向导入组合程序。
旧 Python 接口显式 Compatible 转发同一实现；学习 owner 和原缓存/模型采用逻辑、净距/速度、
地图/导航入口与扫描接线不改业务行为。新算法和剩余感知模块仍留各模块后续实施。
测试按所属功能包/ROS 工作区迁入原用例，原始/补点与缓存消费者的原回归保持；
真实 ROS 运行未验证，三个旧几何夹具错误已在完整基线复现，不能用 SDK/静态检查声称 ROS 验收。


### B2 第二片记录

基线 `3b61157`，现有 senior_perception 内续迁学习 Runtime、记录/owner/service/store/compute/wire/client/cleanup、
StartupLearning 和原矩形净距；实际来源/目录与公开契约见 [功能包 README](../../../ros2/src/senior_perception/README.md#b2-第二片学习与矩形净距)。
生产实车/仿真监管入口在原时机引用 canonical 服务，根部旧入口只作 Compatible 转发；
不新建顶层包、不变协议、不引入补偿/单次解码/新守卫。
学习只接 frozen 控制值与观测 ports，返回带 boot/generation 的请求；控制 owner 核对并应用仲裁和目标，
学习不再直接修改控制 context 的 target/machine/output，功能包也不反向依赖组合包。
原三秒窗口、限期、一次消耗、接管与持久恢复保持。75 项定向通过、1 项 Linux 全链跳过；
旧终端保护 27 个 setup 夹具错误在完整基线复现并登记，未删弱原用例。
实际 Linux IPC/spawn 与 ROS/实车/仿真运行待对应环境，SDK/AST 不替代实测。
实际文件范围与完整命令、版本及结果以功能包 README 为证；后续职责与新算法仍按批次审阅后实施。


### B3 运动估计迁移记录

基线 `155e2b9`，既有 ROS 轮速/雷达运动/IMU 桥、原始类型、准入事件/前沿、时序/融合、
运动与连续位姿估计迁入已有 senior_motion_estimation，单一 canonical 实现保留 Compatible 旧入口。
[功能包 README](../../../ros2/src/senior_motion_estimation/README.md) 列实际目录、公开只读快照、真实进程/入口、
构建安装闭包与完整命令/版本/结果/限制。接收 owner 保留历史，控制 reader 只应用原四字段视图；
没有 runtime.__dict__ 写入、时间重造或新估计步骤。原元数据标记函数由既有接入包公开，避免反向依赖组合包。
实车/仿真、scan_matcher、建图/导航接线和隔离/关闭保持；旧夹具按真实当前接口修正或如实登记，未添假属性/弱化断言。
实际 ROS/Linux 接收全链与两个退役控制组合用例未验证，不以 SDK/静态检查替代。后续控制、导航及新 GPS/算法另批处理。


### B4 控制职责与组合迁移记录

基线 `ebb8c6c`；原控制节点、仲裁、速度、调度、报告及隔离接收组合迁入已有 senior_motion_control。
原本地 arbiter/state_machine/cadence 归该功能包，已有 opt-in 观测适配补归运动估计，不启用新 GPS。
控制包安装真实 CLI；实车/仿真 launch 选择同一 SafetyGate，原 agent 的 IPC/peer 组合和 Compatible 命令入口保留。
仲裁通过明确 effects/只读 ports 代替可写 MotionCore context；调度内部显式使用 owner，移除 OwnedState 透传。
学习仍消费 B2 只读输入并返回请求，B3 公开有界快照保持；失败报告补只读公开入口。
原 40 字节、截止、停止屏障、sole cmd_vel、手动/现有导航退场以及建图/导航链不变。
实际 62 逻辑文件与目录、公开契约、完整命令/版本/结果见 [功能包 README](../../../ros2/src/senior_motion_control/README.md)。
78 项主定向通过，六项 Linux 运行跳过；十项实际 ROS 用例缺环境跳过；旧终端/组装夹具和旧恢复规则失败
在完整 ebb8c6c 归档对照登记，不补假 controller/_inputs/传输属性，不弱化断言。
ROS/Linux/实车/仿真运行待补；B5 地图定位建图、管理子项目/独立云及 B7 后置既有导航依赖顺序不变，
不提前交新守卫、GPS 或导航算法，不新建顶层项目；负责人亲审后再推进下一批。


### B5.1 定位接入与 SLAM Python 适配迁移记录

基线 `7f1d603`；geo/heading/relocalization、原固定导航参考配置与真实 HardwareReference、
既有最小 localization_node 归 senior_localization，原 SLAM Python adapter 归 senior_mapping。
agent_adapter 直接组装 canonical HardwareReference，默认 mapping launch 选择 canonical adapter；
旧入口显式 Compatible，原未启动定位壳不启用，原 stock/native 分支、scan/topic、单图 owner、
测量时间/来源/GID/质量/地图代次与真实导航准入不变；两份生产文件只变一个导入/包名。
实际 45 逻辑文件（产品22、测试6、配置13、工具文档4），完整目录、公开接口、命令版本和结果见
[定位包记录](../../../ros2/src/senior_localization/README.md#b51-范围与验证记录) 与
[建图包记录](../../../ros2/src/senior_mapping/README.md)。77 项通过，2 项实际 ROS 节点检查因缺环境跳过；
三项目 wheel/仓库外安装通过，ROS/ament/Linux/实车仿真运行未验证，不以静态代替。
B5 后片依次处理原地图存储/事务与管理数据库端口、原 native SLAM 后端及其 ROS transport 安装，
随后单独实现缺失地图操作、每图 Home 和可信定位协作；保留原可用导航，B7 导航迁移与新算法仍后置。
仿真总 launch 仍在 senior_robot，CLI/simulation_rgbd/include 及 senior_sim 平行装配收尾另批预报，
本片不夹带仿真装配、原生后端、地图事务、Home 或新算法。


### B5.2 地图持久与事务迁移记录

基线 `a34f485`；8 份原地图实现及公开值归已有 senior_mapping，保留原路径 Compatible；
MapOperations 业务实现唯一，旧 owner 类是薄构造子类而非同一类对象，值类型仍保持同一身份。
只注入原 connection/write_lock、schema prepare 与两项同事务 Home 写入接口；管理侧原 Database、
迁移引擎、任务/Home 状态和连接所有权保留，schema 版本/SQL/路径、原锁顺序和备份不变。
地图仍先停再等 DB，graph guard/reconcile、取消/幂等、同 ref 新代次和 Home 防覆盖保护保留，
无全局默认工厂、无新顶层项目，Home effect 不加锁/新连接/提交。
实际 55 逻辑文件（产品37、测试6、配置8、工具文档4），完整目录、接口、命令/版本/结果与限制见
[地图包记录](../../../ros2/src/senior_mapping/README.md)。真实 SQLite 主定向225通过、2实际ROS跳过，
原地图合成持久验收4通过；SDK安装/类型身份/继承业务方法、依赖锁和格式检查通过。
现有地图读取、定位及导航保持，未装配写入IPC仍未装配；不提前实现每图Home/新地图操作。
后片原生SLAM/ROS transport/候选工具、仿真总装配、缺失功能和B7后置导航仍需各自核定；
本机ROS/ament/Linux/车辆运行缺口据实保留，不以AST或SDK代替实测。


### B5.3 原生 SLAM 运行归包记录

基线 `4ec3d12`；16份原生实现与runtime preflight归已有senior_mapping，三个运行入口同处lib/senior_mapping。
18份原单元/探针一同迁移，35文件逐字节相同；原6份消息/服务留senior_slam_bridge原名原字段，
其仅生成/导出ROS transport，运行依赖单向mapping→bridge，无旧包运行shim。
原默认完整native构建与2.6.10私有库要求保留并显式声明，实际launch只改native分支运行包，
stock、单live owner、原扫描/TF/receipt/capture及候选诊断隔离和关闭行为不变。
实际46逻辑文件（产品17、测试19、配置6、工具文档4），完整树与命令版本/结果/限制见
[地图包B5.3记录](../../../ros2/src/senior_mapping/README.md#b53-原生-slam-迁移) 和
[接口包职责](../../../ros2/src/senior_slam_bridge/README.md)。6个host C++单元程序通过，Python72通过/1缺ROS跳过；
真实Karto/rclcpp编译缺头已登记，SDK构建安装不替代ROS typesupport、隔离安装/私有库及DDS验证。
未做地图写入IPC、新地图/Home/定位协作、仿真总装配或导航；这些后批仍须各自核定验收。

### B6.C1 云业务归包记录

基线859c954；九份原云业务归已确认cloud/src/cloud，唯一业务实现不依赖车端/ROS实现。
原senior_control.cloud.app保留生产bootstrap签名，通过局部回调注入同连接/锁、原schema版本1–5、
原设置与诊断owner；不注册全局provider，诊断帧龄与DB/Home/任务owner仍原位。
Hub/Outbox/LogPeer/LogRelay旧构造是显式Compatible薄子类（非同一类），值和缓存保持同一身份。
原生产镜像CMD仍走旧装配，Docker只补云源码COPY，现有.dockerignore排除本地依赖及旧uv锁兼容缺口仍待C3。
实际42逻辑文件：产品22、测试12、配置5、文档3；完整实际树、源目标、接口和验证见
[云包记录](../../../cloud/README.md) 与 [迁入用例/共享夹具](../../../cloud/tests/README.md)。
原定向三组144项通过；四项原控制夹具失败在859c954独立归档复现；云源码11份类型检查通过，
全链38项类型错误与基线逐条相同；云/根/契约wheel构建和仓库外纯云/旧bootstrap安装检查通过。
未启动服务、部署或推送，不宣称云独立生产部署已完成，B5原生ROS缺口仍保留。

后续依赖仍须逐片审阅/预报：C2确认通用诊断复用边界及云独立DB/schema/config设施，
保留原SQL历史和外部格式；C3再验证真实Docker上下文、uv/锁、独立镜像与生产入口闭包。
管理子项目按settings/logs→storage/task/Home→agent连接/分发→CLI/监管的真实引用分片，
先处理公开本地接口和ROS反向引用，不能盲搬原config/models/diagnostics或让ROS执行依赖管理实现。
只用既有senior_control/src/vehicle_manager、cloud/src/cloud及ROS功能包，不增加根runtime。
已可用地图/定位/导航链保持；缺失地图/Home/定位功能留对应模块实现，B7先迁既有导航再做新算法。


### B6.C2 独立云设施记录

基线ee735b0；cloud.application提供真实独立factory/lifespan，CloudSettings、同连接SQLite、
原备份/取消/回滚迁移和云实际使用的队列/锁诊断由云进程持有；原注入兼容入口保持不变。
全部原v1–v5 SQL按运行值UTF-8字节保留为安装资源快照，14语句逐条SHA256核对，
不复制任务/Home执行代码，不改路径/格式/版本，不迁车端Age或ROS执行职责。
实际19文件：产品8、测试6、配置2、文档3，无删除或其他项目迁移；详见
[云包C2记录](../../../cloud/README.md#c2-独立云设施与应用装配)。
设施/设置原回归114项通过、跨端既选集160项通过（有重复不相加），四项旧控制夹具失败本轮仍复现；
17份云源码类型检查、云wheel构建、无车端/ROS依赖的纯云安装与两次真实SQLite lifespan通过。
原旧bootstrap/诊断队列兼容检查继续通过；不启动开发/实车服务，不部署或推送。
C2已确定采用云内最小队列/等待诊断，不建立根runtime、不放入WS契约，车端通用诊断最终迁移边界仍待后续管理片。
C3下一步须审阅预报后再接生产镜像、原环境/数据卷/反代及锁/上下文，不能以C2独立factory通过替代镜像或部署验证。
既有地图/定位/导航链不变，管理独立安装按前列真实引用切片，导航算法仍后置。


### B6.C3 生产镜像与独立锁记录

基线05a2d8b；Docker只COPY并frozen安装cloud与contracts，入口cloud.bootstrap调用独立factory，
server锁定Uvicorn0.32.1/WebSockets15.0.1，uv0.12.23与CI/本地一致；根锁原68包只补extra元数据，
云独立20包锁与根锁公共版本一致，不迁管理、不改WS/SQLite业务。
原环境、8000、UID/GID10001、DB路径/卷/只读及Caddy代理、Jenkins/Compose真实调用与保护保留。
实际14文件：产品1、测试4、配置6、文档3；详见
[云包C3记录](../../../cloud/README.md#c3-生产镜像与独立锁接线)。
原部署/云回归35通过、Jenkins替身保护通过，仓库外Python3.12.15/3.10.22 frozen安装与真实双lifespan、
WebSockets实现导入及拦截server的原参数检查通过，未监听/启动服务。
真实Docker构建已尝试但本地daemon不可连接，未产镜像，镜像内一次性检查与Linux3.10.15仍未验证，
不能以静态声明或宿主安装宣称通过；没有启动Docker、部署、推送或进行下一片迁移。
后续Docker条件满足时补本批镜像验证；管理项目继续按原真实依赖切片审阅预报，导航算法仍后置。

### B6.M1a 设置owner迁移记录

基线0928aef；原设置model/service/store及公开入口归已建senior_control/src/vehicle_manager/settings，
旧路径显式Compatible同一对象；公开models导入不加载service/store/原Database，服务类公开入口按需加载。
SettingsDatabase仅为原connection/write_lock只读接口，未迁DB/诊断/日志或改事务、锁顺序、版本、DEBUG timer与发布时序。
根→manager→Pydantic/aiosqlite，没有manager反向导入Root/ROS/cloud/WS契约；云独立镜像/锁不改。
实际20逻辑文件（产品11、测试1、配置4、工具文档4），实际树、接口与命令见
[manager记录](../../../senior_control/README.md)；原设置62项、含跨端/手动装配选择75项通过（不相加）。
退役控制manual speed文件50失败在0928aef独立归档同样复现；全链38项类型错误也与基线逐条相同，不弱化原保护。
manager6份源码类型检查和wheel/完整旧SDK安装检查通过，独立7包与原SDK24包pip check通过；
没有真实Linux/ROS/DDS/实车仿真验证，没有启动开发服务、部署或推送。
后续M1b先核对真实日志诊断端口并保留原owner，按已盘点切片另行审阅预报；不提前搬日志、通用runtime或导航。

### B6.M1b 车辆日志owner迁移记录

基线ec7c031；原日志models/store/service/wire归已有vehicle_manager/vehicle_logs，旧公开入口显式Compatible。
值、Store、订阅与辅助函数同一对象；旧Service为保留原签名的薄构造子类，非同一类，所有业务方法继承canonical实现。
只声明实际使用的LogDiagnostics端口并由根装配注入原计数/锁/队列/bind_async，诊断状态仍由原owner持有；
原独立SQLite/MEMORY journal、锁、授权/撤销、过滤/游标/脱敏、队列界限/overflow及只读设置策略保留。
根→manager→原WS contracts与第三方；无manager反向实现导入、无新顶层项目或诊断复制，云镜像/contracts不改。
实际19逻辑文件（产品12，其中1仅更新包说明；测试1迁移、配置2、工具文档4），在预报总18–20范围内；
实际目录、公开端口、命令版本、早期超时/基线对照及最终结果见[manager M1b记录](../../../senior_control/README.md#m1b-车辆日志owner迁移)。
最终原完整定向72通过（3.76s），未改时限/断言；manager12份源码类型通过，原全链38错误与ec7c031归档逐条相同。
manager/根wheel构建、仓库外纯manager8包与旧完整SDK24包安装及真实兼容/诊断owner检查通过，根69包版本不变。
真实Linux/ROS/ament/车辆运行未验证，原native/SLAM/Docker及旧控制夹具缺口仍登记；不开服务、不部署、不推送。
后片仍按真实依赖核对DB/任务/Home→agent连接/分发→CLI/监管与技术设施最终边界，不能借日志端口搬整套runtime。
既有地图/定位/导航保持；缺失模块功能留所属模块实现，导航算法最后。

### B6.M2a SQLite引擎与core schema迁移记录

基线d06f18c；三份SQLite技术实现归已有vehicle_manager/storage，旧Database标Compatible且保留签名，
薄构造注入原诊断锁并保留原Home协调状态，旧类与canonical非同一类，连接方法直接继承唯一实现。
Migration/migrate/CORE_MIGRATIONS/migrate_core同一对象转发；原同连接写锁、版本顺序、PRAGMA、路径、
SQLite备份及失败/取消回滚保留，core SQL原样，domain schema与map_persistence/任务/Home/ROS均不迁。
实际13逻辑文件：产品7、测试2、配置0、工具文档4，即新增4/修改8/迁移1/删除0，未改依赖配置或锁。
原SQLite/连接所有权22通过；含原设置与地图事务的139通过；原共享Home guard/历史迁移6通过，子集不相加。
manager16份源码类型通过，原全链38项类型诊断与M1b记录逐条相同；22份相关Python格式/静态检查通过。
manager/根wheel构建及仓库外纯manager8包/旧完整SDK24包安装、真实SQLite迁移/备份/失败回滚/core与重开通过。
实际目录、公开边界、完整命令版本及限制见[manager M2a记录](../../../senior_control/README.md#m2a-sqlite引擎与core-schema)。
真实Linux/ROS/ament/DDS/SLAM及车辆运行未验证，原安装工具继续标UNVERIFIED，未启服务/部署/推送。
后续按只读清单先核对必要的车内纯值/异常/agent SQL数据与包入口，再迁纯任务决策、持久任务驱动、航点和Home；
具体纯值模块名与地图端口尚须各片审阅，不把待定落点写成已实现，manager不反向导入Root/ROS或地图执行。
原IpcHomePort生产链与未装配HomeService区别保留，不借目录迁移开启新Home或地图操作，导航算法仍最后。

### B6.M2b journal SQL数据与后续依赖盘点

基线3ffeaf7；原两条AGENT_JOURNAL_STATEMENTS声明及UTF-8字节/顺序迁入manager/storage/agent_schema，
旧journal显式Compatible导入同一tuple，执行AST、事务/恢复/去重和owner不动。
实际6文件：产品2、测试与验证1（原安装工具）、文档3、配置0，即新增1/修改5/删除及迁移0。
原SQLite任务接受与journal定向30通过（1.37s），manager17份源码类型检查、三份Python格式/静态检查通过；
原SQL源码及241/199字节两份运行值hash核对通过，无反向实现导入。
manager/根wheel构建、仓库外独立manager8包与旧SDK24包安装及旧新tuple身份/原保护通过。
真实ROS/Linux/车辆运行未验证，未启服务/部署/推送；详见[manager M2b记录](../../../senior_control/README.md#m2b-journal-schema数据)。

四类值只读盘点显示：MapRef横跨管理与建图/定位，Pose2D另横跨运动估计/感知，OperationResult横跨地图/定位与管理操作确认，
DomainError真实参与管理和ROS的构造/捕获/码分支；不把这些值归管理实现或扩到WS契约，本轮保持原归属与身份。
下一片候选仅tasks/schedule、lifecycle→vehicle_manager/tasks，两个原模块均仅依赖标准库，原入口同一实现，
并让旧tasks包按原名字按需公开避免纯导入牵出executor；候选12文件：产品6/测试2/配置0/工具文档4。
原test_task_lifecycle迁入manager/tests，混合TaskRunner的test_schedule保留原位置，只改Schedule导入；
复用原DST/时钟跳变、重复request、旧epoch、未知revoke/迟到ack和锁外dispatch用例，具体输入/预期见manager记录。
此前先迁local_values/errors的建议不再作为执行安排；后续TaskStore重建MapRef需原构造注入、Pose2D经本地端口只读，
错误构造/catch及RouteRejected、地图参考准入尚需精核，完整任务驱动片再重估，不沿用旧预算直接扩大。
不让ROS反向依赖管理实现，不新建顶层项目/全局工厂；既有地图/Home/导航与IPC保持，导航算法最后。

### B6.M2c 标准库schedule/lifecycle迁移记录

基线301fd53；canonical两份源码除换行规范外文本一致，旧模块显式Compatible转发同一类/值/函数，
旧tasks包保留原__all__并按明确名字公开，未迁models/runner/store/executor仍使用原实现。
纯导入不加载这些owner、Database/mapping/ROS，实际请求原Store/Executor/Runner仍同一类，不改生产装配。
实际12逻辑文件：产品6、测试2、配置0、工具文档4，即新增3/修改8/测试迁移1/删除0。
原纯状态机测试随实现迁入manager/tests，混合test_schedule留原位；只修canonical导入和旧模块patch归属，原保护不变。
原时间/DST/时钟跳变、重复request/接管/迟到/重启/断链与服务层锁外dispatch选择集129通过（0.75s）。
manager/根wheel与仓库外独立manager8包/旧SDK24包安装、按需隔离及公开对象身份检查通过，九份Python格式/静态检查通过。
原lifecycle:318类型错误随实现迁入，manager20份源码检查仍有该1项旧错误；全链212份源码38项，归一迁移路径后与基线逐条相同。
完整命令/版本、首轮夹具修正及CRLF检查方式见[manager M2c记录](../../../senior_control/README.md#m2c-纯schedule与lifecycle)。
没有真实ROS/Linux/车辆验证，原缺口保留；未启服务、部署或推送，不移动共享值或提前实现导航。
下一片先只读精核models/runner的消费侧只读MapRef接口、原实例/相等性及生产入边；候选范围12文件
（产品7/测试1/配置0/工具文档4），待负责人审阅预报后才实施，不沿用旧完整任务持久/驱动预算。
Store._task重建、DomainError构造/catch/码、RouteRejected和参考准入仍后续精核；原地图/Home/导航保持，导航算法最后。

### B6.M2d 纯任务models/runner与两处类型接线

基线3f4b6bb；models/runner迁入manager，MapReference仅声明map_id/reference_revision只读属性，
原MapRef值与SQL/dispatcher构造路径不动，原对象、冻结模型与比较/扫描/准入算法保留，旧公开入口同一对象。
只读注解与旧具体MapRef签名冲突后，经确认从12扩至14逻辑文件：产品9/测试1/配置0/工具文档4，
新增3/修改10/测试迁移1/删除0；额外两处为Executor回调和agent IpcNavigation航点读取的Compatible局部cast，
仅静态视图，不是验证/转换；不继续改WaypointStore或整组回调签名，不扩大业务行为。
原test_schedule迁入manager/tests，原断言/patch owner保持；初选216通过，批准接线后原扩选218通过（2.61s）。
manager23份源码仍原lifecycle:318一项，初始全链40项中的两处新类型冲突已消除，最终215份源码38项与M2c逐条相同。
manager/根wheel、纯manager8包/旧SDK24包安装、公开身份/原对象/无反向依赖和真实安装后两调用点核对通过；
十三份Python格式/静态检查及按原CRLF diff检查通过；命令/版本与实际目录见[manager M2d](../../../senior_control/README.md#m2d-纯任务models与runner)。
真实ROS/Linux/车辆及Python3.10本轮未验，原工具保留UNVERIFIED；不开服务、不部署、不推送。

下一片只读候选：S1先TaskStore，复用manager SQLite同连接/写锁及原schema准备、MapRef构造、DomainError构造端口，
不导入Root/地图执行，不改TASK_STATEMENTS/boot/claim/scan；候选10–12文件（产品4–5/测试2–3/配置0/工具文档4）。
S2再lifecycle_store/service，同事务Admission/receipt/effect/取消settle与锁外dispatch、原agent_schema及只读Pose形状，
候选12（产品6/测试2/配置0/工具文档4）；S3最后executor/runtime/navigation_port，错误继承/catch和RouteRejected、
参考准入/真实导航效果端口先精核，条件16–21（产品9–12/测试3–5/配置0/工具文档4），均须负责人审阅预报后实施。
候选原SQLite失败/未知/回滚/恢复、双连接CAS/旧epoch、未知仅查询、接管迟到与新手动隔离的输入/预期见manager记录；
不使用全局工厂、不提前搬值或导航算法，现有地图/Home/导航及IPC继续保留。
