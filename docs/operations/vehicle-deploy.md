# 实车 Agent 部署

本页从克隆仓库后的安装开始，分别说明已有 Docker 容器和非容器两种实车部署。云端服务须先按[云端部署](deploy.md)准备；Gazebo 环境使用[仿真部署](simulation.md)，不要把仿真启动命令用于实车。

本页适用于包含 [PR #17](https://github.com/Shunjun/senior-robot-control/pull/17) 联合启停修复与 [PR #18](https://github.com/Shunjun/senior-robot-control/pull/18) 实车控制接入的版本。native 与 Docker 的 `start` 都校验实车参数、加载实际 ROS overlay，启动共享扫描处理、运动融合、安全门和单个 `agent_adapter`，完成身份握手后再启动 Agent。`deploy` 仅生成和注册服务，不启动。厂商驱动、TF 和已有容器仍由现场管理；导航需另外提供有效地图、定位与 Nav2 参数。服务 active 或云端在线都不代表运动能力已就绪。

## 1. 确认运行位置与目录

已有 Docker 容器模式的关系为：

```text
宿主机：项目 .venv 中的 CLI → 宿主机 systemd
                                  ↓ docker exec
容器：项目 .ros-venv 中的 Agent ↔ Unix socket ↔ ROS 适配器
```

| 项目 | 已有 Docker 容器模式 | 非容器模式 |
| --- | --- | --- |
| 执行 deploy/start/stop/status 的 CLI | 宿主机项目 `.venv` | 宿主机项目 `.venv` |
| Agent 使用的 Python | 容器内 `<project_dir>/.ros-venv/bin/python` | 本机 `<project_dir>/.ros-venv/bin/python` |
| systemd 服务管理器 | 宿主机；容器内不需要 systemd | 本机 |
| `config.json` 中的 `project_dir` | 容器内实际项目路径 | 本机实际项目路径 |
| 配置、凭据、数据库和 IPC | 一个已有可写 bind mount 内的统一目录 | 一个本机统一目录 |

下面以项目在 `/home/wheeltec/ros2_shared_dir/senior-robot-control`、统一目录在 `/home/wheeltec/ros2_shared_dir/senior-control`、已有容器名为 `ros2` 为例。它们是示例值，不是必须使用的程序安装位置。

CLI 默认统一目录为执行用户的 `~/senior-control`。用 `--control-dir` 选择其他目录。Docker 模式通过已有挂载自动推导对应的容器内路径；两边不必同名，但必须映射同一批文件。它不会自动新增挂载或重建容器。

## 2. 克隆并在宿主机安装 CLI

在实车宿主机执行。已有仓库可直接进入项目根目录；需要使用包含 `--control-dir`、`--container` 和上述联合启动修复的项目版本，不能仅凭分支名判断已经包含修复。

```sh
# 尚未克隆时执行：
git clone https://github.com/Shunjun/senior-robot-control.git /home/wheeltec/ros2_shared_dir/senior-robot-control

cd /home/wheeltec/ros2_shared_dir/senior-robot-control
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[agent,safety]'
.venv/bin/senior-robot --help
```

需要 Python 3.10 或更新版本；系统应提供 `venv` 和 pip。仅克隆不会注册 `senior-robot` 命令，安装项目后入口才会出现在 `.venv/bin` 中。

可选执行 `source .venv/bin/activate`，把该目录加入当前 shell 的 PATH，然后直接输入 `senior-robot`。激活不是必需步骤，本页始终使用显式路径。`sudo` 通常重置 PATH，所以也用 `sudo .venv/bin/senior-robot ...`。

宿主机 `.venv` 只运行 CLI，不要求安装 ROS。不要让宿主机 Python 重建或复用容器的 `.ros-venv`：两边的 Python 版本、ROS 扩展和二进制依赖可能不同。

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

两项都不配置时只启动 Agent/IPC，状态显示 Motion unavailable；填入后选择共享实车运动栈。
文件路径或数值格式错误会明确报错。当前配套 JSON 包含用户要求的假设参数，核对由操作人员负责，
说明文字不会作为“是否实测”的认证条件；配置加载成功不表示运动运行验收通过。
更多字段见[逐项参数说明](hardware-motion-parameters.md)。
可选导航须同时提供 navigation_reference 和 navigation_parameters；未配置时导航 unavailable。

CLI 不会自动创建此配置。将示例云域名换成实际 WSS 云端地址。可显式添加 `ros_setup` 指向实际 ROS overlay 的绝对路径；默认推导为 `project_dir/ros2/install/setup.bash`。该 overlay 必须包含构建时正确加载的 ROS 底层及厂商 underlay。若驱动使用特定 ROS 域，添加与驱动实际一致的 `ros_domain` 整数；不要照搬仿真域或假定为 42。`project_dir` 在 Docker 模式下指容器内的路径，必须能找到 `.ros-venv/bin/python` 和已安装的本项目。Python 路径不单独配置。不要逐项添加 `credentials`、`database`、`socket` 或 `estop_state` 的其他路径；CLI 从统一目录推导它们。

| 文件 | 来源与用途 |
| --- | --- |
| `config.json` | 手工创建的运行配置；不含设备 token |
| `device.json` | pair 创建的设备身份和 token，权限 0600 |
| `agent.db`、可能的 `agent.db-wal/-shm` | Agent 的 SQLite 数据 |
| `estop-state.json` | 实车急停持久状态；缺失或无效时保持锁定，CLI 不重置它 |
| `agent.sock` | ROS 适配器提供的本地通信端点 |
| `agent.env` | deploy 生成的环境变量，引用凭据文件，不写入 token |
| `senior-agent-<目录哈希>.service` | deploy 生成的宿主机 systemd 服务文件 |
| `service.sock`、`.agent-service.lock` | 联合入口的停止端点与单实例锁，native 和 Docker 均使用 |
| `agent.owner.json`、`service.owner.json` | 联合入口记录己方 socket 的文件身份，供安全回收检查 |
| `ros-home/`、`ros-log/` | 联合入口设置的 `ROS_HOME`、`ROS_LOG_DIR` |

systemd 只在宿主机 `/etc/systemd/system` 建立指向目录内服务文件的必要链接。CLI 不再把新配置分散写入 `/etc/senior-control`、`/var/lib/senior-control` 和 `/run/senior-control`。系统日志仍由宿主机 journal 管理。

已有设备应保留并迁移原凭据、数据库和急停状态，迁移前停止相关写入并做好备份。CLI 不会自动迁移或删除旧目录。旧文件确已删除且没有备份时，需要重新配对；不能凭克隆仓库恢复原设备身份或任务历史。

## 4. 配对设备

先在云端网页设备列表选择添加设备，生成一次性配对码。在宿主机项目根目录执行，交互式输入配对码：

```sh
.venv/bin/senior-robot pair \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control \
  --cloud-url wss://robot.example.com/ws/agent \
  --name wheeltec
```

此处 URL 应与 `config.json` 指向同一云端。配对不需要启动硬件或 Agent。以统一目录所属用户配对，确保 `device.json` 与目录属主一致；不要使用 sudo 配对后留下不匹配的属主。文件已存在时 pair 拒绝覆盖，设备 token 不应写入仓库、命令行参数或日志。

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
python3 -m venv --system-site-packages .ros-venv
.ros-venv/bin/python -m pip install -e '.[agent,safety,ros]'

# 通过 rosdep 补齐这些实车 ROS 包缺少的依赖；不安装 Gazebo 仿真包：
rosdep install --from-paths \
  ros2/src/senior_robot ros2/src/senior_motion_msgs \
  ros2/src/senior_sensor_ingress ros2/src/senior_scan_matcher \
  --ignore-src -r -y --rosdistro humble

cd ros2
../.ros-venv/bin/python -m colcon build --packages-up-to senior_robot \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

容器应预先具备对应 ROS 发行版、Python venv、pip、rosdep、colcon 和本机构建工具。`--system-site-packages` 用于访问 ROS 安装提供的 `rclpy` 等模块；不能只在宿主机安装一次 Python 依赖就认为容器也已安装。ROS 包重新构建后，其他运行 ROS 节点的会话仍需加载安装工作空间。

### 非容器模式

本机的 `<project_dir>/.ros-venv` 同时承载 Agent 与适配器。按上面的 ROS 安装与构建步骤在本机准备环境，不执行 `docker exec`。只装宿主机 CLI 依赖不足以运行此模式。

## 6. 准备实车 ROS 适配器和安全链

联合入口负责共享运动栈、适配器和 Agent 的生命周期；现场仍需管理厂商驱动、TF，以及导航所需地图与定位来源，并完成下列核验。不要预先启动另一个适配器占用本服务的 `agent.sock`：

- 联合入口从 `device.json` 读取 robot ID 并传给适配器，握手核对身份；不要输出或复制其中的 token。
- 适配器的 `SENIOR_AGENT_SOCKET` 指向运行环境内统一目录的 `agent.sock`，进程用户可读写该目录并能创建端点。
- `safety_state_path` 指向该目录的 `estop-state.json`，由实车安全流程准备和管理。不要使用仿真的初始化命令来创建实车未锁定状态。
- ROS Domain、话题、TF、时钟与硬件实际配置匹配；安全门是唯一最终 `/cmd_vel` 发布者。

硬件核验见[硬件检查](hardware.md)。实车启动会拒绝缺失或无效的运动配置；仓库没有已经资格化的通用实车制动、覆盖和相机参数，当前共享输出适配仅使用激光自由空间。下面的启动命令不能代替现场标定与运行验收。

## 7. 部署、启动和查看状态

回到宿主机项目根目录。先选择一种部署模式；不要对同一目录同时部署两种模式。

### 已有 Docker 容器模式

```sh
.venv/bin/senior-robot doctor \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control --container ros2

.venv/bin/senior-robot deploy \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control --container ros2 --dry-run

sudo .venv/bin/senior-robot deploy \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control --container ros2

sudo .venv/bin/senior-robot start \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control

sudo .venv/bin/senior-robot status \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control
```

只写 `--container`、不跟名称时，默认容器为 `ros2`。也可以把 `"container": "ros2"` 写入 `config.json`，供 deploy 和 doctor 使用；命令行值优先。start、stop、status 使用已生成的宿主机服务，不能靠给 start 临时添加 `--container` 来切换模式。

`--dry-run` 会只读 inspect Docker、推导路径并打印计划，不创建文件、不启动容器或服务。正式 deploy 还检查凭据、权限和容器内 Python。生成的服务通过 `docker_start_helper run/stop` 联合托管适配器与 Agent；宿主机 systemd 在 start 时通过 `docker start` 确保已有容器运行，二者在容器内执行。容器内不需要 systemd，宿主机不需要第二套 ROS。宿主机服务的隔离配置不等同于对 Docker 内进程的隔离。

### 非容器模式

`config.json` 不设置 `container`，部署时不传 `--container`：

```sh
.venv/bin/senior-robot deploy --control-dir /home/wheeltec/ros2_shared_dir/senior-control --dry-run
sudo .venv/bin/senior-robot deploy --control-dir /home/wheeltec/ros2_shared_dir/senior-control
sudo .venv/bin/senior-robot start --control-dir /home/wheeltec/ros2_shared_dir/senior-control
sudo .venv/bin/senior-robot status --control-dir /home/wheeltec/ros2_shared_dir/senior-control
```

服务按目录属主身份，以本机 `<project_dir>/.ros-venv/bin/python` 执行历史命名的 `senior_control.docker_start_helper run` 联合入口，并使用 `KillMode=mixed`，以便按顺序关闭子进程。这个模块名不表示 native 模式需要 Docker。

### 两种模式共有的行为

- sudo 可能把 HOME 改为 `/root`，因此 sudo 命令始终显式传 `--control-dir`。
- deploy 与 start 分开，deploy 不启动 Agent，也不会自动启用开机自启。
- 已有生成文件或同名服务时拒绝覆盖。修改配置后不会自动改写已部署 unit；切换部署模式需先停止服务，再由管理员核对旧文件和链接后重新部署。
- status 分别报告进程状态、ROS IPC 身份核验和该次运行最新的云连接事件。active 不是云端已连接、地图有效或运动已获许可的证明。

## 8. 停止与排障

```sh
sudo .venv/bin/senior-robot stop \
  --control-dir /home/wheeltec/ros2_shared_dir/senior-control
```

两种模式都先停止 Agent 和本次适配器，再关闭本次共享运动栈及其子进程；Docker 模式保留已有 `run/stop` 接口，不停止整个容器、厂商驱动或外部独立运行的节点。入口不覆盖活动或未知 socket，只回收身份记录一致、明确属于己方的陈旧 socket；不要盲删 socket、owner 记录或锁文件。

| 现象 | 先检查 |
| --- | --- |
| `senior-robot: command not found` | 已安装项目；使用 `.venv/bin/senior-robot` 或激活宿主机 `.venv` |
| `Could not read a valid vehicle profile` | `config.json` 是否存在且 JSON 合法；project_dir、云地址、挂载、凭据与权限是否满足条件。当前 CLI 会把多类部署失败折叠成这条通用错误 |
| `device.json` 不存在 | 先配对；克隆仓库不包含设备凭据 |
| 容器内 Python 不可用 | 容器运行状态、容器内 project_dir、`.ros-venv` 与本项目安装是否正确 |
| `Process: activating`、`ROS IPC: unavailable`，日志 `adapter_unreachable` 且约每 5 秒重启 | 先核对实际安装版本及 unit 是否仍是旧 Agent-only 入口；再检查适配器启动日志、ROS overlay、容器内项目路径、`.ros-venv`、目录权限、robot ID 与配置 ROS 域。每 5 秒是失败重启间隔，不是已经上线 |
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

现代实车 unit 应指向联合入口。`doctor` 只证明可检查的 Python、目录和挂载条件；ROS 就绪在 start 时核验。`Cloud: last event connected` 只是该次服务运行最近的连接事件，不是持续在线证明。缺失/无效急停状态保持锁定，不要清空安全意图、删除急停状态或用复位急停解决上线问题。

## 9. 升级已部署服务

仅更新代码不会自动改写 unit。尤其旧 native unit 直接执行 Agent 时，必须重新生成 unit 才能采用联合入口：

1. 记录当前提交、实际 unit 名、控制目录及运行环境映射；停止该服务，确认已无本服务写入，再备份整个控制目录（含 SQLite WAL/SHM、凭据与急停状态），备份也须私有保存。
2. 在宿主机更新 CLI 的 `.venv`，在实际运行环境更新 `.ros-venv` 的项目依赖并重建 ROS 包；按第 5 节加载正确 underlay。Docker 内外分别操作，不共用虚拟环境。
3. 对照 `config.json` 和 dry-run 输出审阅计划。如果需要重新生成，管理员先核对旧服务身份和链接目标；如曾启用则停用它，移除 `/etc/systemd/system` 中指向本目录旧 unit 的确切链接，并将本目录旧生成 unit 和 `agent.env` 归档到私有备份目录。不得操作其他同名前缀服务，不得移除 `device.json`、`agent.db`、急停状态或 socket 来绕过检查。
4. 执行宿主机 `systemctl daemon-reload`，再按第 7 节正式 deploy（它会重新 link 和 reload），审阅生成文件后显式 start。若需要开机自启，由管理员对已核对的实际 unit 另行 enable；deploy 自身不 enable。
5. 检查日志、Process、IPC、Cloud 与现场完整安全链。回退前核对数据兼容性，恢复已记录的代码/环境和生成文件；不要用旧凭据覆盖新配对身份。

老部署可继续显式使用 `--profile /path/to/old.json` 的兼容入口（//Compatible）。`deploy/senior-agent.service` 仅供这条旧路径使用；它的 `/opt`、`/etc`、`/var/lib`、`/run` 布局不是新 `--control-dir` 模式的默认值，不要混用两套说明。

## 工作台只读地图

如需配置同车导出的只读地图包，可在生成的 `agent.env` 中由运维人员补充 `SENIOR_WORKSPACE_MAP_BUNDLE` 与 `SENIOR_WORKSPACE_MAP_SHA256`。文件路径必须是 Agent 运行环境内可读的绝对路径；Docker 模式还需已有挂载覆盖该文件。

Agent 会校验完整小写 SHA-256、地图包内容与来源车辆。只读包可用于显示长期占据层；任务编辑、定位和导航仍依赖实时来源。更新文件后须核对摘要并按服务流程重启 Agent。

## 源码依据与验收边界

入口与统一目录解析见 [cli.py](../../src/senior_control/cli.py)，生成 unit / 挂载推导见 [vehicle_deploy.py](../../src/senior_control/vehicle_deploy.py)，联合生命周期、身份和 socket 回收见 [docker_start_helper.py](../../src/senior_control/docker_start_helper.py)。本文是部署操作说明；目标 Linux、厂商 underlay、权限映射、systemd 启停、云连接和整车安全/运动链仍须现场分项验收，不能从静态源码或成功配对推导通过。
