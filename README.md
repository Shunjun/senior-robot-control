# Senior 小车远程控制

面向 WHEELTEC senior_4wd_dl 的车端 ROS 2 程序、自托管云端和中文网页控制台。阶段一至阶段四已有代码并完成复审，包含控制基础、地图与任务、云端中转、车端代理、视频、网页和自托管部署资产。Linux 上尚未执行 `safety_node`、完整速度链、Nav2、`slam_toolbox` 和任务 21 端到端套件；此前任务 2 的仿真记录和任务 3 的后部遮挡扫描记录保留在本文及 `docs/testing/2026-10-27-linux-simulation.md`，未据此扩展其他通过结论。

速度只走一条链：导航、遥控或脱困进入控制仲裁，再进入独立安全门，最后由安全门发布 `/cmd_vel`。Nav2 和脱困的裸速度暂不接入仲裁：它们没有来源代次和生成时间，无法排除旧动作的迟到消息。`command_node` 独占发布 `/safety/command_in`，自主运动需待具备来源代次和时间戳的桥接实现后才能启用。仿真底盘把 `/cmd_vel` 接到差速控制器，控制器自己还有 0.3 秒命令超时。单独启动 `command_node` 时没有网络入口；车端代理可通过独立的 `agent_adapter` 使用本地 IPC，运行链仍需 Linux 验证。

## 已完成的功能

- 版本化命令信封。校验车辆、会话、连接代次、序号和数值；拒绝 NaN、错误车辆和超大消息。心跳、失联和速度有效期集中配置，单位是米、弧度、秒。
- Ubuntu 22.04 上的 Humble / Gazebo Fortress 仿真。输出雷达、RGB-D、IMU、轮里程计和独立真实位姿。命令停发后底盘看门狗把速度归零。这项已在虚拟机上跑通，记录见 `docs/testing/2026-10-27-linux-simulation.md`。
- 自车过滤。车体后方附件回波被去掉，且不会变成后方空闲；附件外的近墙仍保留。仿真世界是 `rear_occlusion.sdf`。历史地图辅助倒车在脱困模块里：实时障碍否决历史空闲，缺边界则拒绝动作。后部遮挡仿真已跑通；脱困本身还没有在 Linux 上跑过。
- 独立安全门 `safety_node`。按车体矩形预测扫掠，硬边界拦截，预警区允许已观测空闲方向的倒车。传感器过期、扫描运动来源不对或打滑时停车。速度命令按车端单调时钟超时后发零。逻辑有单元测试；这个节点还没有在 Linux 上作为进程跑过。
- 控制仲裁 `command_node`。每台车一份租约。优先级是锁定急停、手动、脱困、导航。手动接管停掉导航，松开后不自动续跑。旧连接和旧序号不能把过期速度重新激活。它只发布 `/safety/command_in`。仲裁器单元测试已通过；整条 ROS 速度链还没有在 Linux 上跑过。
- 有界脱困和 Nav2 适配。速度、距离、时间和次数由调用方给出，缺一项就拒绝。导航适配等待目标终态，接受取消本身不算结束。`navigation.launch.py` 把 Nav2 输出重映射到 `/control/navigation_cmd`，但目前仲裁器不接收无代次的裸速度。单元测试已通过；这个 launch 和 lattice 还没有在 Linux 上加载。
- 长期占据层。重复观测才确认障碍。自由观测不覆盖几何锚点。清空代价地图不能代替清除这一层。
- SLAM 适配。位姿图按后端写入的内容保存，长期占据层是单独的 `occupancy.json`。网格变大不改变地图参考。Humble 2.6.10 上，反序列化没有返回对象就算恢复失败。`slam_toolbox` 还没有在本机加载。
- 地图包导入、导出和清除。一次操作先把日志写稳，再切换位姿图和占据层。跨车导入的 Home 保持待确认，不会覆盖已经采纳的 Home。格式见 `docs/protocol/map-bundle.md`。
- 每台车一个 Home，位姿存在导航点表里。返航先等上一个目标终态。断线只取消断线前已经接受的目标，重连不自动再发。
- GPS 配准和被动航向。没有绝对航向时只给出位置。`navsat_status` 小于 0 的定位被拒绝。RTK 只认显式的 `fixed` 或 `float`。`localization_node` 不发布速度，也不订阅 NavSatFix。`tests/ros/test_relocalization.py` 还没有跑过。
- 本地任务。车端 SQLite 保存任务定义、版本和执行历史。一次扫描最多派出窗口里最新的一次，更早的触发记为跳过。断线不启动、不补跑；执行中断线暂停，重连不自动续跑。一个进程只准备一个 `TaskStore`。规则见 `docs/protocol/scheduling.md`。

## 仓库布局

| 路径 | 内容 |
| --- | --- |
| `src/senior_control` | 协议、配置、存储、安全、地图、导航点、定位和本地任务 |
| `ros2/src/senior_sim` | 仿真模型、世界和 `simulation.launch.py` |
| `ros2/src/senior_robot` | 雷达过滤、安全门、仲裁、Nav2 适配、SLAM 适配、定位节点 |
| `config/simulation.yaml` | 仿真外形与话题，供 Python 配置读取 |
| `config/hardware.example.yaml` | 实车配置示例，不能当作已核对的实车参数 |
| `docs/protocol/v1.md` | 当前命令类型 |
| `docs/protocol/map-bundle.md` | 地图包格式 |
| `docs/protocol/scheduling.md` | 本地任务调度 |
| `docs/superpowers/plans/2026-10-27-robot-control-implementation.md` | 实施计划 |

## 各端部署与启动

有两个互不替代的 Python 环境。开发机上的 `.venv` 跑单元测试和类型检查，不包含 ROS。Ubuntu 上的 `.ros-venv` 继承系统里的 `rclpy`，再安装本仓库，供 `colcon` 生成的节点入口使用。用系统自带的 `colcon` 构建时，生成脚本的解释器找不到 `senior_control`。

### 开发机：控制库

需要 Python 3.10 或更新版本，以及 [uv](https://docs.astral.sh/uv/)。macOS 只做这项，不启动仿真。

```sh
uv sync --frozen --group dev --extra safety
uv run ruff check .
uv run ruff format --check .
uv run mypy src/senior_control
uv run pytest tests/unit tests/integration -q
```

`safety` 额外依赖提供 Shapely，自车过滤和扫掠会用到它。这条命令不启动 ROS，也不连接小车。

环境只读检查不启动仿真、硬件或 ROS 守护进程：

```sh
python3 tools/detect_environment.py
```

在已 source ROS 的 Ubuntu 上，它报告已安装的 ament 包和被动可见的话题。未 source 时只报告缺失项。

### 仿真端：Ubuntu 22.04

安装 ROS 2 Humble 和 Gazebo Fortress 的官方二进制，包含 `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control`、`controller_manager`、`diff_drive_controller`、`joint_state_broadcaster`、`robot_state_publisher`、`xacro`。无桌面的机器还需要 `xvfb`、`xauth`、`mesa-utils`。已测试环境是 Ubuntu 22.04.5 aarch64、Humble、Fortress 6.18.0。

在仓库根目录：

```sh
source /opt/ros/humble/setup.bash
python3.10 -m venv --system-site-packages .ros-venv
.ros-venv/bin/pip install -e ".[safety,ros]"
rosdep update --rosdistro humble
rosdep install --from-paths ros2/src --ignore-src -r -y --rosdistro humble
cd ros2
../.ros-venv/bin/python -m colcon build --packages-select senior_sim senior_robot
source install/setup.bash
```

每次开终端都要重新 source Humble 和 `ros2/install/setup.bash`。仿真和测试使用同一个未占用的域，避免接到别的 ROS 图：

```sh
export ROS_DOMAIN_ID=73
export IGN_PARTITION=senior_sim
```

带图形界面的主机：

```sh
ros2 launch senior_sim simulation.launch.py headless:=false use_sim_time:=true
```

`headless:=true` 是默认值，Gazebo 以 `-r -s` 运行。在已测试的 Parallels 虚拟机上，没有显示服务器时 Ogre2/EGL 会在初始化时中止。该虚拟机用下面这条命令跑通了传感器契约：

```sh
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1280x720x24' \
  ros2 launch senior_sim simulation.launch.py headless:=true use_sim_time:=true
```

默认世界是安装后的 `baseline.sdf`。后部遮挡世界：

```sh
ros2 launch senior_sim simulation.launch.py headless:=true use_sim_time:=true \
  world:="$(ros2 pkg prefix senior_sim)/share/senior_sim/worlds/rear_occlusion.sdf"
```

仿真起来后，传感器话题包括 `/scan`、`/camera/color/image_raw`、`/camera/depth/image_raw`、`/imu/data`、`/wheel/odom`、`/ground_truth/odom`、`/clock`。真实位姿的 TF 不接入定位。看门狗负对照和传感器契约的完整步骤在 `ros2/README.md`。

### 车端 ROS 进程

扫描来源由独立的 `scan_receipt_node` 进程接收 `/scan`，核对每条消息的发布者身份，并将有界原始扫描交给 `scan_adapter` 和 `safety_scan_ingress`。过滤结果与安全扫描经原生来源核验后分别提供给扫描匹配器和安全门；安全门不直接订阅原始 `/scan`。采集时间和首次接收时间贯穿这些环节，不因转发重新计时。

安全路径要求 `/scan` 恰有一个合格发布者。出现多个发布者时立即撤销扫描资格；当前实现会退休原来源，以及歧义期间实际收到回调的来源。歧义消除后，这些已退休的发布者即使重新成为唯一来源也不会自动恢复，须消除重复发布并重建相应驱动发布者，使其获得新的来源身份。持续无效数据不会证明空间安全。来源切换、停机及恢复需在目标部署环境复验。

仿真组合入口 `simulation_motion.launch.py` 包含扫描接收、过滤、匹配、运动融合和安全进程；它要求显式提供车端代理 socket，仿真参数只适用于测试。`safety_motion.launch.py` 要求显式提供已测量、已标定的安全参数文件；仓库目前没有经过资格化的实车制动、覆盖和相机参数。必需深度覆盖不足时安全门保持停车，不能把仿真参数用于真实底盘。

RGB-D 仿真诊断入口 `simulation_rgbd.launch.py` 默认给其子进程设置本地 Fast DDS 传输配置：UDPv4 加 16 MiB 共享内存段。该容量只针对已测的 640×480 仿真相机；实机或其他图像尺寸须重新验证，不适用于 4G 视频传输。使用 `enable_rgbd_transport:=false` 关闭自动配置，或用 `rgbd_transport_profile:=/绝对路径/profile.xml` 选择配置文件。已有非空 Fast DDS 环境变量或工作目录中的默认配置文件优先于启动参数；空环境变量不会关闭自动配置。选择非 Fast DDS RMW 或关闭开关时不加载配置。未指定 RMW 时通过 ROS 查询实际后端，查询失败则报错。所选配置文件必须可读且为格式正确的 XML；Fast DDS 仍负责校验具体配置内容。配置仅针对 Humble 的 Fast DDS 2.x。此设置只影响该启动入口的子进程。

### 导航、建图与定位

这些入口已经在 `senior_robot` 包里，对应逻辑的单元测试在开发机上通过。Humble 上还没有加载 Nav2 lattice 和 `slam_toolbox`，下面的 launch 也没有执行过。

```sh
ros2 launch senior_robot navigation.launch.py use_sim_time:=true
```

它启动 Nav2 控制器、规划器、行为树和 `navigation_adapter`，不启动仿真。控制器输出重映射到 `/control/navigation_cmd`，当前仲裁器不会接收其裸速度。`safety_node` 仍然是唯一发布 `/cmd_vel` 的进程，需要另外启动。

```sh
ros2 launch senior_robot mapping.launch.py use_sim_time:=true
```

它启动 `slam_toolbox` 和 `slam_adapter`，不发布速度。长期占据层与位姿图分开保存。

```sh
ros2 run senior_robot localization_node
```

`localization_node` 不发布速度，也不订阅 NavSatFix。GPS 样本由调用方注入。本地任务没有单独的 ROS 入口；调度规则在 `docs/protocol/scheduling.md`，执行器调用注入的导航端口，不直接发布 `/cmd_vel`。

### 实车

没有实车 launch，也没有 systemd 服务。`config/hardware.example.yaml` 和 `ros2/src/senior_robot/config/hardware.yaml` 只是待核对的话题与外形示例。`tools/detect_environment.py` 只读，不启动驱动。仿真参数、仿真遮挡多边形和上面的制动示例都不能用来驱动真实底盘。厂商底盘、雷达和相机驱动保留，本仓库不替换它们。

### 云端与车端代理

云端认证、配对、多车中转和车端代理已有实现。先配置 HTTPS 云服务、管理员密码、允许的 Origin 和 TURN；生产网络与硬件参数仍需现场核对。管理员登录后，以返回的 CSRF token 和会话 cookie 调用 `POST /api/pairing-codes`，在车端用一次性配对码兑换身份：

```sh
python -m senior_control.agent.main --pair \
  --cloud-url wss://<cloud-domain>/ws/agent --device-name <robot-name> \
  --credentials-file /etc/senior-control/device.json
```

命令会交互式读取配对码，通过 HTTPS 兑换并原子创建权限为 `0600` 的凭据文件；文件已存在时拒绝覆盖。凭据文件包含 robot ID 和设备 token，不要写入版本库或日志。安装车端依赖 `pip install '.[safety,agent,ros]'`，设置 `SENIOR_CLOUD_URL`、`SENIOR_DATABASE_PATH`（绝对路径）和 `SENIOR_AGENT_SOCKET`（绝对路径），再运行：

```sh
python -m senior_control.agent.main --credentials-file /etc/senior-control/device.json
```

同样支持 `SENIOR_CREDENTIALS_FILE` 指向该文件；也可通过 `SENIOR_ROBOT_ID`、`SENIOR_DEVICE_TOKEN` 提供身份。使用 `deploy/senior-agent.service` 时将环境值放入 `/etc/senior-control/agent.env`，文件由服务用户可读且权限为 `0600`。缺少配置或认证被拒绝时进程退出 78。Agent 在 ROS 状态、地图、导航处理器缺失时返回 unavailable；浏览器时钟尚不能验证，生产远程速度暂不可用。跨页面急停已锁定，安全复位暂不可用，需后续带急停 fence 的协议。任务终态、ROS 全链路、Linux 和实车均未验收；只有 `safety_node` 可以发布 `/cmd_vel`。

### 网页

网页工程位于 `web/`，包含登录、选车、驾驶、地图、任务、地图管理和视频界面。单元测试和静态构建可在开发机执行。浏览器端到端测试只连接显式提供的 `PLAYWRIGHT_BASE_URL`，不启动开发服务器；当前浏览器验收仍未执行。

### 任务 21 验收

验收范围、Linux 复现命令和故障注入入口见 `docs/testing/matrix.md`，本次结果见 `docs/testing/results.md`。`pytest --simulation-profile` 使用临时数据库、互异保留端口和固定 seed。Linux、Gazebo 和浏览器检查在不具备对应环境时保持“未执行”，不能写成通过。实车验收使用 `docs/testing/real-robot-checklist.md`，所有项目默认未勾选。

## 任务路线图

状态按代码和复审记录。Linux 运行栏只把已经在 Ubuntu 上执行并通过的检查标成已验证。

| 任务 | 功能 | 代码 | Linux 运行 |
| --- | --- | --- | --- |
| 1 | 协议、配置、时钟、SQLite 骨架 | 已完成 | 不依赖 ROS |
| 2 | 仿真底盘、传感器契约、底盘看门狗 | 已完成 | 已验证 |
| 3 | 自车过滤与覆盖判断 | 已完成 | 后部遮挡仿真已跑通 |
| 4 | 矩形扫掠、打滑检测、独立安全门 | 已完成 | 节点未跑 |
| 5 | 控制租约、急停锁定、断线停车 | 已完成 | 速度链未跑 |
| 6 | Nav2 导航与有界脱困 | 已完成 | 未跑 |
| 7 | 稳定障碍与自由空间清除 | 已完成 | 未跑 |
| 8 | SLAM、回环与持续建图 | 已完成 | `slam_toolbox` 未加载 |
| 9 | 地图包导入、导出与清除 | 已完成 | 未跑 |
| 10 | 导航点与 Home | 已完成 | ROS 结果等待未跑 |
| 11 | GPS 配准、重定位与被动航向 | 已完成 | 重定位测试未跑 |
| 12 | 本地任务与调度 | 已完成 | 未跑 |
| 13 | 单管理员认证、配对与多车中转 | 已完成 | 不依赖 ROS |
| 14 | 车端代理、任务与地图同步 | 已完成 | 代理进程入口未组装 |
| 15 | RGB-D 几何与可选 YOLO | 已完成 | 未跑真实传感器 |
| 16 | WebRTC 视频与 TURN | 已完成 | 未跑浏览器全链路 |
| 17 | 网页骨架、登录与选车 | 已完成 | 静态测试可在开发机执行 |
| 18 | 地图、驾驶与重定位界面 | 已完成 | 未跑浏览器全链路 |
| 19 | 任务、地图管理与视频界面 | 已完成 | 未跑浏览器全链路 |
| 20 | 自托管部署与硬件检查 | 已完成 | systemd 未启用 |
| 21 | Linux 全链路仿真验收 | 验收资产已添加 | Linux、Gazebo 与浏览器未执行 |

阶段一至阶段四的代码和复审已经完成。阶段退出条件写在实施计划对应章节。下面这些 Linux 检查还没有执行，不能记成通过：`safety_node` 进程、整条 ROS 速度链、Nav2 launch 与 lattice、`slam_toolbox`、返航的 ROS 结果等待、`tests/ros/test_relocalization.py` 和任务 21 端到端套件。任务 5 的 ROS 测试在第一次上 Linux 前，还要先补上订阅发现完成后再发命令的等待；当前测试可能在发现完成前丢消息。
