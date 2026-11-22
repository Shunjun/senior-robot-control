# 实车 Agent 部署


本轮仅迁移管理实现，本文原CLI、module启动名与安装命令保持兼容。
生产平台绑定仍需本项目完整SDK与既有ROS/媒体/IPC owner，不能仅安装vehicle-manager替换当前运行环境；
独立manager的配置解析/计划渲染与旧SDK的help/dry-run检查不代表独立生产部署验收。
仿真总装配launch目前仍由senior_robot安装与调用，归senior_sim的收尾将在后续单独验证。

本页从克隆仓库后的安装开始，分别说明已有 Docker 容器和非容器两种实车部署。云端服务须先按[云端部署](deploy.md)准备；Gazebo 环境使用[仿真部署](simulation.md)，不要把仿真启动命令用于实车。

本页适用于包含 [PR #17](https://github.com/Shunjun/senior-robot-control/pull/17) 联合启停修复与 [PR #18](https://github.com/Shunjun/senior-robot-control/pull/18) 实车控制接入的版本。native 与 Docker 的 `start` 都校验实车参数、加载实际 ROS overlay，启动共享扫描处理、运动融合、安全门和单个 `agent_adapter`，完成身份握手后再启动 Agent。`deploy` 仅生成和注册服务，不启动。选择实车运动配置后，运动 ROS launch 启动厂家底盘（含板载 IMU、EKF、车身 TF）和雷达，相机由同一服务的独立 launch 启动；已有容器仍由现场管理；导航需另外提供有效地图、定位与 Nav2 参数。服务 active 或云端在线都不代表运动能力已就绪。

## 1. 确认运行位置与目录

已有 Docker 容器模式的关系为：

```text
宿主机：独立 Shell CLI → 宿主机 systemd
                                  ↓ docker exec
容器：项目 .venv 中的 Agent ↔ Unix socket ↔ ROS 适配器
```

| 项目 | 已有 Docker 容器模式 | 非容器模式 |
| --- | --- | --- |
| 执行 deploy/start/stop/status 的 CLI | 宿主机 Shell，无 Python | 宿主机 Shell |
| Agent 使用的 Python | 容器内 `<project_dir>/.venv/bin/python` | 本机 `<project_dir>/.venv/bin/python` |
| systemd 服务管理器 | 宿主机；容器内不需要 systemd | 本机 |
| `config.json` 中的 `project_dir` | 容器内实际项目路径 | 本机实际项目路径 |
| 配置、凭据、数据库和 IPC | 一个已有可写 bind mount 内的统一目录 | 一个本机统一目录 |

下面以项目在 `/home/wheeltec/ros2_shared_dir/senior-robot-control`、统一目录在 `/home/wheeltec/ros2_shared_dir/senior-control`、已有容器名为 `ros2` 为例。它们是示例值，不是必须使用的程序安装位置。

CLI 默认统一目录为执行用户的 `~/senior-control`。用 `--control-dir` 选择其他目录。Docker 模式通过已有挂载自动推导对应的容器内路径；两边不必同名，但必须映射同一批文件。它不会自动新增挂载或重建容器。

## 2. 安装独立 Shell CLI

在宿主机项目根目录执行（尚未克隆时先克隆仓库）：

```sh
sudo install -m 0755 bin/senior-robot.sh /usr/local/bin/senior-robot
type -a senior-robot
senior-robot --help
```

单 Shell 文件不编译、不依赖源码相对位置。宿主机应提供 Bash、jq、systemd、GNU timeout，Docker 模式另需 Docker CLI。无 Python 时 help 和语法诊断仍可用；Docker 模式不会调用宿主 Python。Python wheel 已不注册同名 console-script；升级旧环境会移除旧注册项，仍应检查 PATH，避免遗留 venv 脚本遮盖 `/usr/local/bin/senior-robot`。不要盲删其他项目的同名命令。

native 模式的业务和 ROS 在项目 `.venv` 中，Docker 模式在容器项目 `.venv` 中；只准备选用的运行环境。不得把旧虚拟环境简单改名：须使用与 ROS ABI 一致的 Python 重建（Humble 通常为系统 Python 3.10），保留 `--system-site-packages`。native 用配置 `project_dir` 定位 `.venv/bin/python`，旧 profile 缺少该字段时显式设置绝对路径 `SENIOR_ROBOT_PYTHON`；sudo 会过滤环境，应显式传变量或填好 project_dir。

## 3. 创建统一目录和 config.json

在宿主机项目根目录、以目录所属普通用户执行：

```sh
control_dir=/home/wheeltec/ros2_shared_dir/senior-control
mkdir -p "$control_dir"
chmod 700 "$control_dir"
cp config/vehicle-real.example.json "$control_dir/config.json"
```

这是首次安装步骤；已有配置或数据时先检查并保留，不要直接覆盖。编辑 `config.json`，例如：

```json
{
  "kind": "real",
  "project_dir": "/home/wheeltec/ros2_shared_dir/senior-robot-control",
  "cloud_url": "wss://robot.example.com/ws/agent",
  "motion_profile": "hardware-motion.json",
  "safety_parameters": "hardware-safety.yaml"
}
```

`motion_profile` 指向运动响应和误差参数 JSON，`safety_parameters` 指向 ROS 参数 YAML。
相对路径以 **config.json 所在的统一目录** 为基准，不是项目目录，也不是启动命令所在目录。
按上例应把配套文件放成：

```text
/home/wheeltec/ros2_shared_dir/senior-control/
├── config.json
├── hardware-motion.json
└── hardware-safety.yaml
```

从项目目录复制配套文件（已有同名文件时先保留自己的配置，不覆盖）：

```sh
cp -n config/hardware-motion.json config/hardware-safety.yaml \
  /home/wheeltec/ros2_shared_dir/senior-control/
```

Docker 模式可在 config.json 添加 `"container": "ros2"`，参照
[agent.example.json](../../config/agent.example.json)；非 Docker 模式不设置 container。
保持自己已有的 cloud_url、project_dir 和 ros_domain，不要用示例值覆盖现场配置。
相对路径会在容器内按已映射的统一目录解析，因此该目录及两个文件也必须被现有 bind mount 覆盖。

标准实车启动必须同时提供有效运动 profile 与安全参数，缺失时启动前报错；使用共享运动核心。
文件路径或数值格式错误会明确报错。当前配套 JSON 包含用户要求的假设参数，核对由操作人员负责，
说明文字不会作为“是否实测”的认证条件；配置加载成功不表示运动运行验收通过。
更多字段见[逐项参数说明](hardware-motion-parameters.md)。
可选导航须同时提供 navigation_reference 和 navigation_parameters；未配置时导航 unavailable。

实车运动启动会先加载 `vendor_ros_setup`（默认 `/home/wheeltec/wheeltec_ros2/install/setup.bash`），再加载本项目 `ros_setup`。厂家底盘、雷达和相机分别复用 `turn_on_wheeltec_robot.launch.py`、`wheeltec_lidar.launch.py` 与 `wheeltec_camera.launch.py`，从厂家的 `wheeltec_param.yaml` 读取车型、IMU 和雷达型号，相机型号同样读取厂家配置；不启动键盘控制或独立厂家导航。相机在 Agent/IPC 就绪后启动，崩溃会报告视频不可用，不会关闭激光运动链；厂家相机驱动的段错误仍需独立排查。所有本次启动的驱动随服务联合停止；不要同时手动启动相同驱动。当前厂家底盘发布 `/odom`（帧 `odom_combined`）、`/imu/data_raw`，雷达发布 `/scan`。旧配套 YAML 的 `wheel_frame: odom` 自动迁移为 `odom_combined`，话题 `/odom` 不改名。

CLI 不会自动创建此配置。将示例云域名换成实际 WSS 云端地址。可显式添加 `ros_setup` 指向实际 ROS overlay 的绝对路径；默认推导为 `project_dir/ros2/install/setup.bash`。该 overlay 必须包含构建时正确加载的 ROS 底层及厂商 underlay。若驱动使用特定 ROS 域，添加与驱动实际一致的 `ros_domain` 整数；不要照搬仿真域或假定为 42。`project_dir` 在 Docker 模式下指容器内的路径，必须能找到 `.venv/bin/python` 和已安装的本项目。Python 路径不单独配置。不要逐项添加 `credentials`、`database`或 `socket` 的其他路径；CLI 从统一目录推导它们。

| 文件 | 来源与用途 |
| --- | --- |
| `config.json` | 手工创建的运行配置；不含设备 token |
| `device.json` | pair 创建的设备身份和 token，权限 0600 |
| `agent.db`、可能的 `agent.db-wal/-shm` | Agent 的 SQLite 数据 |
| `agent.sock` | ROS 适配器提供的本地通信端点 |
| `agent.env` | deploy 生成的环境变量，引用凭据文件，不写入 token |
| `senior-agent-<目录哈希>.service` | deploy 生成的宿主机 systemd 服务文件 |
| `service.sock`、`.agent-service.lock` | 联合入口的停止端点与单实例锁，native 和 Docker 均使用 |
| `agent.owner.json`、`service.owner.json` | 联合入口记录己方 socket 的文件身份，供安全回收检查 |
| `ros-home/`、`ros-log/` | 联合入口设置的 `ROS_HOME`、`ROS_LOG_DIR` |

systemd 只在宿主机 `/etc/systemd/system` 建立指向目录内服务文件的必要链接。CLI 不再把新配置分散写入 `/etc/senior-control`、`/var/lib/senior-control` 和 `/run/senior-control`。系统日志仍由宿主机 journal 管理。

已有设备应保留并迁移原凭据和数据库，迁移前停止相关写入并做好备份。CLI 不会自动迁移或删除旧目录。旧文件确已删除且没有备份时，需要重新配对；不能凭克隆仓库恢复原设备身份或任务历史。

## 4. 配对设备

Docker 模式先完成第5节容器内安装，并保持共享容器运行。先在云端网页设备列表选择添加设备，生成一次性配对码。在宿主机项目根目录执行，交互式输入配对码：

```sh
senior-robot docker pair --container ros2 \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control \
  --cloud-url wss://robot.example.com/ws/agent \
  --name wheeltec
```

此处 URL 应与 `config.json` 指向同一云端。配对不需要启动硬件或 Agent。以统一目录所属用户配对，确保 `device.json` 与目录属主一致；不要使用 sudo 配对后留下不匹配的属主。文件已存在时 pair 拒绝覆盖，设备 token 不应写入仓库、命令行参数或日志。

native 模式使用 `senior-robot pair`，其余参数相同。Docker 配对以操作者 UID/GID 执行，沿用原私有目录与文件属主约束。Shell 在原凭据/TLS校验通过后才隐藏读取配对码，无法安全关闭终端回显则失败；凭据不进入 argv、环境或日志，原原子防覆盖写入保持。

## 5. 在 Agent 的运行环境安装依赖

### 已有 Docker 容器模式

宿主机需要 Docker CLI 和访问本地 Docker daemon 的权限。部署时容器必须已运行，且统一目录在容器已有的可写 bind mount 内。项目目录也必须能在容器中访问，或已经单独安装在容器中。CLI 的检查不会修改镜像、增加挂载或重建容器。

在宿主机进入已有容器：

```sh
docker exec -it ros2 bash
```

随后这些命令在容器内执行。示例采用 ROS 2 Humble。先确认现场实际 ROS 发行版与 Python ABI；如需厂商 underlay，应在 Humble 后、本项目构建前 source 其实际 `install/setup.bash`，下列命令不猜测厂商路径：

```sh
source /opt/ros/humble/setup.bash
cd /home/wheeltec/ros2_shared_dir/senior-robot-control
python3 -m venv --system-site-packages .venv
uv sync --frozen --no-dev --inexact --extra agent --extra safety --extra ros --extra video

# 通过 rosdep 补齐这些实车 ROS 包缺少的依赖；不安装 Gazebo 仿真包：
rosdep install --from-paths \
  ros2/src/senior_robot ros2/src/senior_motion_msgs \
  ros2/src/senior_sensor_ingress ros2/src/senior_scan_matcher \
  --ignore-src -r -y --rosdistro humble

cd ros2
../.venv/bin/python -m colcon build --packages-up-to senior_robot \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

根项目默认启用dev与vehicle依赖组；`--no-dev`排除开发工具但保留vehicle整车项目组，
上述冻结命令复用根目录同一套`.venv`。现场需使用锁文件对应的uv（当前0.12.23），
不要用仅安装根wheel代替整车同步。根`senior-control` wheel现只声明共享core基础依赖；
manager通过它获得`vehicle_core`，不再由根wheel默认聚合manager/cloud/ROS项目。
//Compatible 旧`senior_control`源码命名空间仍随根wheel分发，供原SDK入口使用；
manager独立逻辑不得导入旧业务模块，完整旧SDK由vehicle组装配。

容器应预先具备对应 ROS 发行版、Python venv、uv、pip、rosdep、colcon 和本机构建工具。`--system-site-packages` 用于访问 ROS 安装提供的 `rclpy` 等模块；不能只在宿主机安装一次 Python 依赖就认为容器也已安装。ROS 包重新构建后，其他运行 ROS 节点的会话仍需加载安装工作空间。

启动服务前执行 [ROS 类型支持加载检查](ros-interface-rebuild.md#启动前检查不创建-ros-节点)。升级包含消息删除或重命名时，先按该页定向备份并重建接口包；普通增量构建成功不足以排除旧生成物和安装残留，无需重复创建 venv 或 pip 安装。

### 非容器模式

本机的 `<project_dir>/.venv` 同时承载 Agent 与适配器。按上面的 ROS 安装与构建步骤在本机准备环境，不执行 `docker exec`。只装宿主机 CLI 依赖不足以运行此模式。

## 6. 准备实车 ROS 适配器和安全链

联合入口负责厂家底盘、雷达、相机、共享运动栈、适配器和 Agent 的生命周期；现场仍需管理导航所需地图与定位来源，并完成下列核验。不要预先启动另一个适配器占用本服务的 `agent.sock`：

- 联合入口从 `device.json` 读取 robot ID 并传给适配器，握手核对身份；不要输出或复制其中的 token。
- 适配器的 `SENIOR_AGENT_SOCKET` 指向运行环境内统一目录的 `agent.sock`，进程用户可读写该目录并能创建端点。
- ROS Domain、话题、TF、时钟与硬件实际配置匹配；安全门是唯一最终 `/cmd_vel` 发布者。

硬件核验见[硬件检查](hardware.md)。实车启动会拒绝缺失或无效的运动配置；仓库没有已经资格化的通用实车制动、覆盖和相机参数，当前共享输出适配仅使用激光自由空间。下面的启动命令不能代替现场标定与运行验收。

## 7. 部署、启动和查看状态

回到宿主机项目根目录。先选择一种部署模式；不要对同一目录同时部署两种模式。

### 已有 Docker 容器模式

```sh
senior-robot docker doctor \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control --container ros2

senior-robot docker deploy \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control --container ros2 --dry-run

sudo senior-robot docker deploy \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control --container ros2

sudo senior-robot docker start \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control

sudo senior-robot docker status \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control
```

不传容器名时读取配置 container，缺省为 `ros2`；`--container` 可显式指定，不改变持久配置，和已有配置冲突会拒绝。Docker 仿真不在本次实现内。

容器停止时 `deploy --dry-run` 与 start 只读 inspect 后明确失败，不 exec、不启动容器、不写配置/unit、不触发 systemctl。运行中的预览用容器项目 `.venv` 完整生成原渲染内容，不写入、不启动。正式 deploy 由容器 Python 使用原防覆盖原子写入器生成绑定目录内文件，Shell 在宿主机 link/reload；失败只回滚本次拥有的文件/确切链接，回滚遇到身份改变会拒绝并报错。共享容器生命周期由管理员独立管理，本项目生成的新 unit 不含 docker start。

start/stop/status 核对部署文件和链接；stop/status 仍可管理已核对的旧 Docker unit。start 另核对当前选择和新 unit 内容，旧自动启动容器的 unit 需先 stop，再按升级章节归档并重新 deploy，不会悄悄改写。容器内无 systemd，宿主机无 Python；服务隔离配置不等同于容器内进程隔离。

### 非容器模式

`config.json` 不设置 `container`，部署时不传 `--container`：

```sh
senior-robot deploy --control-dir /home/wheeltec/ros2_shared_dir/senior-control --dry-run
sudo senior-robot deploy --control-dir /home/wheeltec/ros2_shared_dir/senior-control
sudo senior-robot start --control-dir /home/wheeltec/ros2_shared_dir/senior-control
sudo senior-robot status --control-dir /home/wheeltec/ros2_shared_dir/senior-control
```

服务按目录属主身份，以本机 `<project_dir>/.venv/bin/python` 执行历史命名的 `senior_robot.service_runtime run` 联合入口，并使用 `KillMode=mixed`，以便按顺序关闭子进程。这个模块名不表示 native 模式需要 Docker。

### 两种模式共有的行为

- sudo 可能把 HOME 改为 `/root`，因此 sudo 命令始终显式传 `--control-dir`。
- deploy 与 start 分开，deploy 不启动 Agent，也不会自动启用开机自启。
- 已有生成文件或同名服务时拒绝覆盖。修改配置后不会自动改写已部署 unit；切换部署模式需先停止服务，再由管理员核对旧文件和链接后重新部署。
- status 分别报告进程状态、ROS IPC 身份核验和该次运行最新的云连接事件。active 不是云端已连接、地图有效或运动已获许可的证明。

## 8. 停止与排障

```sh
sudo senior-robot stop \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control
```

Docker 模式改用 `senior-robot docker stop --control-dir ...`，restart 同样加 docker 分组；两者只管理项目服务。

两种模式都先停止 Agent 和本次适配器，再关闭本次共享运动栈及其子进程；Docker 模式保留已有 `run/stop` 接口，同时停止本次 launch 启动的厂家驱动，不停止整个容器或外部独立运行的节点。入口不覆盖活动或未知 socket，只回收身份记录一致、明确属于己方的陈旧 socket；不要盲删 socket、owner 记录或锁文件。

| 现象 | 先检查 |
| --- | --- |
| `senior-robot: command not found` | 安装单 Shell 到 `/usr/local/bin` 并检查 `type -a senior-robot` |
| `Could not read a valid vehicle profile` | `config.json` 是否存在且 JSON 合法；project_dir、云地址、挂载、凭据与权限是否满足条件。当前 CLI 会把多类部署失败折叠成这条通用错误 |
| `device.json` 不存在 | 先配对；克隆仓库不包含设备凭据 |
| 容器内 Python 不可用 | 容器运行状态、容器内 project_dir、`.venv` 与本项目安装是否正确 |
| `Process: activating`、`ROS IPC: unavailable`，日志 `adapter_unreachable` 且约每 5 秒重启 | 先核对实际安装版本及 unit 是否仍是旧 Agent-only 入口；再检查适配器启动日志、ROS overlay、容器内项目路径、`.venv`、目录权限、robot ID 与配置 ROS 域。每 5 秒是失败重启间隔，不是已经上线 |
| ROS IPC unavailable / Agent 无法上线 | 本次启动的适配器是否成功，是否存在其他进程占用，双方路径和权限、robot ID 是否匹配；禁止盲删端点 |
| 服务 active 但不能运动 | 检查实车参数、驱动、共享运动栈及安全许可；导航还需有效地图和定位。启动不会清除安全锁定 |

### 宿主机只读诊断

以下在宿主机执行。先从 deploy 输出或统一目录里的 `.service` 文件名确定本目录对应的真实 unit，替换示例值（不要原样使用示例哈希）。日志可能包含现场路径与设备信息，分享前检查脱敏，不要输出 `device.json` 或完整环境变量：

```sh
unit='senior-agent-替换为实际目录哈希.service'
sudo systemctl show "$unit" -p ActiveState -p SubState -p ExecStart -p User
sudo systemctl cat "$unit"
sudo journalctl -u "$unit" -n 100 --no-pager
stat -c '%a %U:%G %n' /home/wheeltec/ros2_shared_dir/senior-control
# Docker 模式另外检查已有挂载、运行状态；不修改容器：
docker inspect --format '{{.State.Status}} {{json .Mounts}}' ros2
```

现代实车 unit 应指向联合入口。`doctor` 只证明可检查的 Python、目录和挂载条件；ROS 就绪在 start 时核验。`Cloud: last event connected` 只是该次服务运行最近的连接事件，不是持续在线证明。软件急停由最终输出侧唯一 owner 在当前进程内锁存；完整启动未锁存，不读取或写入旧急停文件。文件缺失、损坏或不可读不影响软件急停状态，也不会因此授予运动许可。不要删除现场历史文件。

## 9. 升级已部署服务

仅更新代码不会自动改写 unit。尤其旧 native unit 直接执行 Agent 时，必须重新生成 unit 才能采用联合入口：

1. 记录当前提交、实际 unit 名、控制目录及运行环境映射；停止该服务，确认已无本服务写入，再备份整个控制目录（含 SQLite WAL/SHM、凭据与急停状态），备份也须私有保存。
2. 更新独立 Shell；只在实际运行位置（native 宿主机或 Docker 容器）更新项目 `.venv` 并重建 ROS 包，按第5节加载正确 underlay。Docker 宿主机不安装 Python。
3. 对照 `config.json` 和 dry-run 输出审阅计划。如果需要重新生成，管理员先核对旧服务身份和链接目标；如曾启用则停用它，移除 `/etc/systemd/system` 中指向本目录旧 unit 的确切链接，并将本目录旧生成 unit 和 `agent.env` 归档到私有备份目录。不得操作其他同名前缀服务，不得移除 `device.json`、`agent.db`、急停状态或 socket 来绕过检查。
4. 执行宿主机 `systemctl daemon-reload`，再按第 7 节正式 deploy（它会重新 link 和 reload），审阅生成文件后显式 start。若需要开机自启，由管理员对已核对的实际 unit 另行 enable；deploy 自身不 enable。
5. 检查日志、Process、IPC、Cloud 与现场完整安全链。回退前核对数据兼容性，恢复已记录的代码/环境和生成文件；不要用旧凭据覆盖新配对身份。

老部署可继续显式使用 `--profile /path/to/old.json` 的兼容入口（//Compatible）。`deploy/senior-agent.service` 仅供这条旧路径使用；它的 `/opt`、`/etc`、`/var/lib`、`/run` 布局不是新 `--control-dir` 模式的默认值，不要混用两套说明。

## 工作台只读地图

如需配置同车导出的只读地图包，可在生成的 `agent.env` 中由运维人员补充 `SENIOR_WORKSPACE_MAP_BUNDLE` 与 `SENIOR_WORKSPACE_MAP_SHA256`。文件路径必须是 Agent 运行环境内可读的绝对路径；Docker 模式还需已有挂载覆盖该文件。

Agent 会校验完整小写 SHA-256、地图包内容与来源车辆。只读包可用于显示长期占据层；任务编辑、定位和导航仍依赖实时来源。更新文件后须核对摘要并按服务流程重启 Agent。

## 源码依据与验收边界

入口与统一目录解析见 [cli.py](../../senior_control/src/vehicle_manager/operator_cli.py)，生成 unit / 挂载推导见 [vehicle_deploy.py](../../senior_control/src/vehicle_manager/platform_deployment.py)，联合生命周期、身份和 socket 回收见 [docker_start_helper.py](../../ros2/src/senior_robot/senior_robot/service_runtime.py)。本文是部署操作说明；目标 Linux、厂商 underlay、权限映射、systemd 启停、云连接和整车安全/运动链仍须现场分项验收，不能从静态源码或成功配对推导通过。

升级时从自定义配置中移除已退役的 `estop_state` / `safety_state_path` 参数，并从自定义 motion profile 移除 `hardware_estop` 配置块；新版本不保留这些配置的兼容读取。现场旧 `estop-state.json` 保留原样，不作为启动输入。


## 标准 ROS 时间接口（2026-11-17）

本项目直接接收标准 `/scan`、配置中的 Odometry/Imu 及 Image/CameraInfo，不要求厂家时间补丁或 `/timed` 话题。首次接入使用原始整数 `header.stamp` 和本地 ROS/单调时钟保留测量年龄，排队与控制期限使用单调时钟。最终执行消息为 `geometry_msgs/Twist`，话题固定 `/cmd_vel`，由项目唯一输出 owner 发布；Nav2 使用私有运行话题，厂家底盘/串口 owner 只启动一次。

更新含消息接口变更的版本时按[干净接口重建](ros-interface-rebuild.md)清理已核对的本项目生成物，避免旧安装库掩盖接口变化。不要安装已退役的 sensor/Nav2/simulation 时间 overlay，不修改厂家组件。通用 profile 不再包含 `simulation_only` 字段，仍须提供明确数值依据；导航配置缺失时保持可选导航不可用的原行为。真实 ROS 构建、DDS 和车端只读状态验证须独立完成，便携单测通过不等于现场链路通过。

### restart 与日志

```sh
senior-robot restart --control-dir /absolute/control
senior-robot logs --control-dir /absolute/control --lines 100 --follow
senior-robot docker restart --control-dir /absolute/control
senior-robot docker logs --control-dir /absolute/control --follow
```

restart 必须等原 systemd/联合入口停止成功才启动，停止非零直接返回。logs 读取宿主项目 journal；仿真用 user scope。docker exec 的输出由宿主 unit 收集，不使用只展示容器 PID1的 docker logs；Docker 项目日志可在容器停止时读取，不需要宿主 Python或容器 exec。语法错误返回2，业务错误保持非零，缺解释器/工具127，原系统工具退出码保留（GNU timeout 超时124）；不会把 active 当作 IPC/云/运动资格。
