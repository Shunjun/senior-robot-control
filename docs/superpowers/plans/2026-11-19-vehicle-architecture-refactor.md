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
| B4 待迁：控制完整职责与组合 | `vehicle_arbitration.py`、`vehicle_speed.py`、`motion_core_output.py`、`control_inputs.py` 及现有执行控制逻辑 → `senior_motion_control`；`senior_robot`/`senior_sim` 只选设备/参数/组装 | 依赖 B2/B3 公开结果；MotionCore 只编排，仲裁/速度/业务状态由所属 owner 持有；先迁旧行为并验手动/学习/导航权限退场、原停止屏障及实车/仿真实际装配；新截止/断流算法在该模块阶段另批实现 |
| B5 待迁并实现：定位与建图 | `src/senior_control/localization/`、`mapping/`、`localization_node.py`、`senior_slam_bridge` 实际调用与 `senior_robot/launch/mapping.launch.py` 组合 → `senior_localization`/`senior_mapping` 及原组合入口 | 依赖接入和本地运动接口；先保留并迁移已有实时建图/地图管理/重定位，随后在本轮实现缺失地图操作、独立 Home、可信定位协作；复用原存储/地图/定位回归，新增功能测试先单独审定；切图未确认不导航或持久写图 |
| B6 待迁：管理与云独立安装 | 现有 `src/senior_control/agent/`、`runtime/` 中管理职责、settings、vehicle_logs、CLI/监管 → `senior_control/src/vehicle_manager/`；现有 `src/senior_control/cloud/` 及云专用依赖 → `cloud/src/cloud/` | 公开 IPC 和业务 owner 稳定后按生产链迁移；云端只依赖共享 WS 契约，不导入车端快照实现；逐项复用编解码/relay、日志设置 wire、快照过期、监管/隔离回归，核对锁、CI wheel、安装与现有启动入口；完成前原根项目继续承载生产实现 |
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
