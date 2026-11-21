# Humble / Fortress simulation

Current startup and the task roadmap are in the repository root `README.md`. The sensor contract and the watchdog negative control passed on Ubuntu 22.04 aarch64; evidence is `docs/testing/2026-10-27-linux-simulation.md`. That record does not cover the safety gate or the command chain.

| OS | ROS | Gazebo | integration |
| --- | --- | --- | --- |
| Ubuntu 22.04 | Humble | Fortress | `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` Humble binaries; sensor contract verified on the aarch64 VM |
| macOS | none | none | portable tests only; not a simulation validation |

After installing ROS Humble and Fortress from their official repositories on Ubuntu 22.04, start in the repository root:

```sh
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
rosdep install --from-paths ros2/src --ignore-src -r -y --rosdistro humble
cd ros2
colcon build --packages-select senior_sim senior_robot
source install/setup.bash
ros2 launch senior_sim simulation.launch.py headless:=true use_sim_time:=true
```

`world:=/absolute/path/to/world.sdf` and `headless:=false` select another world or GUI. Run the following in another sourced terminal at the repository root, with the isolated simulation already running:

```sh
cd /path/to/senior-robot-control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
/usr/bin/python3.10 -m pytest tests/ros/test_sim_topics.py -q
```

The tests move the simulated robot and temporarily relocate it through the Fortress UserCommands service, restoring its pose afterward; run them only in an isolated baseline simulation. They require the `ign` executable and its `/world/baseline/set_pose` service. A skipped test, a successful static test, or a build alone does not establish the ROS runtime contract. Install the core Python wheel separately into the ROS interpreter when using it; `rclpy` is supplied by ROS, not PyPI.

`tests/ros/test_scan_adapter.py::test_simulated_self_and_wall_evidence` is a separate ambient-world check and is skipped unless `SENIOR_AMBIENT_SCAN_TEST=1` is set. It requires an already running isolated `baseline.sdf` world with the default rear attachment and a running `scan_adapter` configured with both `simulation_motion.yaml` and `simulation_motion_rear_mask.yaml`; it does not start or stop that world. The fixture must provide `/scan`, `/perception/scan_filtered`, `/perception/scan_status`, `/ground_truth/odom`, and the laser transform. The wall assertion is specific to the baseline obstacle at x=1.9 m. The owned motion qualification tests use their own fixture and are separate evidence.

`/motion/scan` identifies the scan matcher's continuous source epoch. `/motion/evidence` uses a separate opaque epoch made from the current MotionFusion process identity and that scan epoch, so restarting either producer breaks continuity. Valid and invalid evidence from the same combined epoch share its identity and carry their source measurement stamp; neither a newer failure nor a producer change can be undone by a delayed older observation. After a failure, a qualifying interval must start at or after its failure stamp; an undated failure requires a new interval starting at or after receipt. ROS clock rollback retires the current input epoch and clears the Fusion alignment buffers. `startup_reference_valid` remains false: neither a matcher origin nor a restarted process proves the power-on pose. These identities and unique ROS endpoint checks detect lifecycle changes and misconfiguration in a controlled localhost domain; they do not authenticate publishers in a hostile DDS domain.

The chassis uses vendor senior_4wd_dl wheel positions and 0.076 m wheel radius. The conservative simulation body collision box is 0.674 × 0.54 × 0.15425 m; its safety footprint is ±0.337 × ±0.276 m, including wheel collision projections. A rear attachment makes lidar occlusion physical. The collision envelope retains the full conservative footprint; visual boxes include a front camera aperture so the forward ray is unobstructed. Camera position is left at the supplied vendor reference, whose parent frame still requires verification. A static wall gives sensors an observable target. Vendor sensor transform values are approximations requiring validation against actual frame parentage. Original vendor assets are not copied. The truth publisher reads simulator pose independently; its TF topic is deliberately not bridged. Do not use truth for production localization or present simulated data as hardware data.

The official `diff_drive_controller` uses feedback (`open_loop: false`) and `cmd_vel_timeout: 0.3` independent of the later safety process. Its clock follows simulation time: a pause can prevent timeout and permit stale commands on resume. A monotonic safety gate must prevent this before motion is relied upon. Controller loading, ROS frame IDs, RGB-D topic names, graphics support in headless mode, geometry contact, stop timing, and installed binary compatibility require the Linux runtime gate. Hardware profile is illustrative only and must be verified on the robot; it grants no motion permission.

Sources: [Humble/Fortress control compatibility](https://github.com/ros-controls/gz_ros2_control/tree/humble), [controller timeout and feedback](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html), [Fortress ROS bridge](https://gazebosim.org/docs/fortress/ros2_integration/), [simulator odometry publisher](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1OdometryPublisher.html).

`python3 tools/detect_environment.py` reports sourced ament package manifests and versions, missing prerequisites, and passive topic discovery. Its matrix does not certify runtime readiness or the Fortress version; check the installed `ignition-gazebo6` packages on the Linux host. The Humble plugin `<ros><remapping>` forwards controller topic remaps as documented in [the Humble integration documentation](https://github.com/ros-controls/gz_ros2_control/blob/humble/doc/index.rst). Runtime tests must confirm the actual `/drive` publisher and subscriber mappings.

### Watchdog negative control

Use the baseline world in an isolated ROS domain. Run the normal launch and test first, then stop that entire launch before the negative control. The test moves the model to x=-3 m with a clear forward corridor, checks its world position and forward lidar during the 2 s post-command window, and restores the model pose in `finally`. Both runs require working `/world/baseline/set_pose` and a running clock; a setup failure is not a successful negative control.

Normal launch, from the repository root in the first terminal:

```sh
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
export ROS_DOMAIN_ID=71
ros2 launch senior_sim simulation.launch.py headless:=true
```

In a second terminal from the repository root, with the same sourced environments and domain:

```sh
export ROS_DOMAIN_ID=71
/usr/bin/python3.10 -m pytest tests/ros/test_sim_topics.py::test_watchdog_stops -q
```

It must pass. Stop the launch, then start a fresh simulation with a **temporary 10-second timeout** in the first terminal. The cleanup after the launch and the exit trap remove the temporary configuration; the installed and source controller YAML are never edited.

```sh
export ROS_DOMAIN_ID=71
watchdog_dir=$(mktemp -d)
trap 'rm -rf "$watchdog_dir"' EXIT
/usr/bin/python3.10 - "$watchdog_dir/controllers.yaml" <<'PY'
import sys
import yaml
from pathlib import Path
source = Path('ros2/src/senior_sim/config/controllers.yaml')
config = yaml.safe_load(source.read_text())
config['drive']['ros__parameters']['cmd_vel_timeout'] = 10.0
Path(sys.argv[1]).write_text(yaml.safe_dump(config))
PY
ros2 launch senior_sim simulation.launch.py headless:=true controllers_file:="$watchdog_dir/controllers.yaml"
rm -rf "$watchdog_dir"
trap - EXIT
```

Run the identical pytest command in the second terminal. It must **fail specifically on the assertion with `watchdog failed to sustain low velocity after timeout`**, after movement and clearance checks pass; an import, launch, service, timing or clearance failure is inconclusive. Stop the negative-control launch, which removes its temporary file. This deliberately failing run proves the stop assertion detects the long timeout only when observed on the actual Linux simulator; it does not test pause/resume safety.

### Linux VM rendering

On the tested Ubuntu 22.04 aarch64 Parallels VM, a displayless server-only launch aborted during Ogre2/EGL texture initialization. With `xvfb`, `xauth`, and `mesa-utils` installed, `glxinfo` under Xvfb reported Mesa llvmpipe / OpenGL 4.5; the following launch ran the RGB-D, scan, IMU and odometry sensor contract successfully on that VM:

```sh
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1280x720x24' \
  ros2 launch senior_sim simulation.launch.py headless:=true
```

Keep the same sourced ROS environments and isolated domain described above in both terminals. This establishes the sensor test only; the complete ROS suite and watchdog negative control remain separate gates. Fortress also offers `--headless-rendering` for EGL with OGRE2, but that option has not been tested in this VM. `-s` alone means server-only and does not request that option. See the [Fortress headless rendering documentation](https://gazebosim.org/api/gazebo/6/headless_rendering.html).

## 原生控制输出包迁移（批次 1）

两个 guard 和 FinalCommand 帧实现已归 [senior_motion_control](src/senior_motion_control/README.md)。senior_robot 的 SafetyGate 实际使用新包索引路径，实车/仿真仍共用该入口；先构建新包再启动现有组合。native_receipt 已在 B1 迁入 senior_sensor_ingress；senior_motion_msgs 仅保留消息与 C++ 时序契约，其余功能迁移仍按实施计划推进。ABI、原截止和停止语义不变。


## B1：原生接收归传感器接入包

基线 `51b25e6`。本批共 16 个逻辑文件（迁移单计）：产品 4、测试 3、配置构建 5、文档 4；
新增 2、修改 11、迁移 3、单独删除 0。文档包含同步更新已迁测试目录的职责说明。
本批仅迁已有接收程序与 Python 客户端，保留生产与仿真原入口，
不迁感知算法、进程监管、executor、诊断设施或时间类型，不新增顶层项目。

```text
ros2/src/senior_sensor_ingress/
  CMakeLists.txt, package.xml           # 接收目标及 ament Python 安装
  src/native_receipt.cpp               # 原消息包程序，逐字节迁移
  senior_sensor_ingress/
    __init__.py
    native_receipt.py                  # 唯一客户端实现、公开入口、安装查找
ros2/src/senior_motion_msgs/
  CMakeLists.txt, package.xml           # 消息及 C++ typesupport/时序导出
  include/, msg/, test/                # 既有接口与 installed_header_consumer 保留
ros2/src/senior_robot/senior_robot/
  native_receipt.py                    # //Compatible：四个公开入口转发同一实现
ros2/tests/
  test_native_motion_executor.py       # 从 tests/ros 迁入，原实车/仿真入口 DDS 用例
  test_receipt_process_shutdown.py     # 从 tests/ros 迁入，原生接收原关停用例
```

生产调用：`motion_observation_adapter`、`hardware_reference`、`optional_motion_inputs`
及 `command_node` 仍通过原 `senior_robot.native_receipt` 公开入口，显式转发
`senior_sensor_ingress.native_receipt` 的同一 `NativeReceiptGroup`、`subscribe`、
`received_sample`、`received_steady`。没有第二套接收逻辑、旧可执行路径兜底或额外 relay。
客户端通过 `ament_index_python` 找到
`<senior_sensor_ingress-prefix>/lib/senior_sensor_ingress/native_receipt`。
`senior_robot/package.xml` 原本已依赖 `senior_sensor_ingress`，本批保持该真实装配依赖。

客户端的 ROS 类型反序列化与稳态时钟仍按原位置延迟导入；Python 安装由功能包的
`ament_cmake_python` / `ament_python_install_package` 承担，不新增 SDK 发行包。
客户端继续使用现有 `vehicle_core.sample_time.SampleTime`、
`sensors.policy.default_policy` 与 `runtime.diagnostics` 的公开接口，
因此仍需既有根 Python 项目安装；这些 owner 和原安装/锁不迁移，后续管理项目迁移须重新核对边界。
本批根 pyproject 只补 pytest 的接入包源码发现路径，不改变生产依赖。

C++ 目标以 C++17 构建，直接依赖 rclcpp、nav_msgs、sensor_msgs、geometry_msgs、std_msgs
与 `senior_motion_msgs` 的公开安装导出。
消息包仍先生成并导出 rosidl C++ typesupport，再导出链接该 target 的手写 `sensor_timing`；
保留原 installed_header_consumer 对两个安装 target 的检查。
移走接收可执行目标后消息包不再依赖/导出 rclcpp，原消费者在各自功能包声明 rclcpp。
本机未执行该 ROS 构建，以上是实际 CMake/清单核对，不能作为安装验证通过的证据。

保持行为：SDR2 包布局、source/GID、顺序/代次和撤销屏障、整数 SampleTime、
原接收时钟/原帧龄阈值、初始 5 秒等待、20 MiB 单帧、64 样本/32 MiB 待处理上限、
原诊断观测、有界关闭和父死亡行为不变。控制 guard、40 字节 ABI、停止/截止协议无改动。
C++ 接收程序 SHA256 为 `c3339adb292735c7aa0ab0349358af16b37dbd78cd9d55f9f97a64aa6071e203`，
与 `51b25e6` 旧路径内容完全相同。Python 客户端只提取并更换 native 安装定位，其余实现沿用原代码。

### B1 验证与限制

环境：macOS 26.4 arm64、CPython 3.12.15、pytest 8.4.2、Pydantic 2.14.1、
uv 0.12.23、ruff 0.17.3。命令中的 Python 使用原主仓库 `.venv/bin/python`。

- `python -m pytest ros2/tests/test_native_motion_executor.py ros2/tests/test_receipt_process_shutdown.py tests/unit/test_native_motion_observation_adapter.py tests/unit/test_runtime_diagnostics.py tests/unit/test_control_process_resources.py tests/unit/test_motion_launch_profile.py tests/unit/test_sim_contract.py -q`：35 passed、10 skipped；通过部分包含原来源/GID拒绝、诊断有界、真实 spawn 快照/关闭及装配静态契约，10 个 ROS 接收/关停用例因 rclpy 缺失跳过。
- 以 `PYTHONPATH=src:contracts/src:ros2/src/senior_robot:ros2/src/senior_sensor_ingress` 导入两端公开入口，断言四个入口对象身份及 SampleTime 身份相同、未加载 rclpy：通过；这是源码导入验证，不是 ROS 安装验证。
- 实际调用 `native_executable()`：因 `ament_index_python` 缺失无法执行安装查找。colcon、CMake、rclpy 本机均不存在，原生接收编译、C++ typesupport 安装消费、实际 ROS 包安装查找、DDS 时序与子进程关停均未验证；没有以假 prefix 或模拟可执行文件替代。
- `python -m pytest tests/unit/test_laser_reference_publication.py -q`：6 个夹具错误，`control_process_test_support.py:74` 仍尝试替换不存在的 `motion_core_output.ProcessWorker`。在完整 `51b25e6` archive 的 `/tmp/receipt-narrow-baseline` 用同命令复现相同 6 错误，未改无关产品逻辑或旧夹具。本批只将该原用例的时间补丁路径改到唯一客户端实现。
- 定向 `ruff check`、`ruff format --check`、`git diff --check` 及 C++ 字节比对通过；它们不替代 ROS 实测。

需在 ROS 2 Humble 环境续验：`colcon build --base-paths ros2/src --packages-up-to senior_sensor_ingress senior_robot`，
`colcon test --packages-select senior_motion_msgs senior_sensor_ingress`，先确认安装前缀中接收程序及 Python 模块，
再以 `ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1` 执行已迁入的 native executor 用例；关停原用例还需
`SENIOR_RECEIPT_BINARY_DIR=<prefix>/lib/senior_sensor_ingress`。未启动开发服务、部署或推送。

### M4B composition entry locations

```text
src/
├── senior_robot/
│   ├── senior_robot/real_runtime.py       # canonical calibrated real preflight
│   ├── pyproject.toml                    # ROS-free Python wheel; no ROS assets/scripts
│   ├── setup.py                          # existing ament assets/node installation
│   ├── launch/safety_motion.launch.py    # real composition calls canonical preflight
│   ├── launch/simulation_motion.launch.py # Compatible Include only
│   ├── launch/simulation_rgbd.launch.py   # Compatible Include only
│   └── config/                           # original calibrated/shared config remains here
└── senior_sim/
    ├── launch/simulation_motion.launch.py # canonical simulation composition
    ├── launch/simulation_rgbd.launch.py   # canonical scoped RGBD diagnostic composition
    └── CMakeLists.txt                    # existing launch directory installation
```

The manager now requests `ros2 launch senior_sim simulation_motion.launch.py`.
The old senior_robot launch names forward every original declared argument and retain
inherited launch context. No executable shim or duplicate nodes are introduced.
Both new launch files still use original senior_robot config/BT assets and original
functional nodes. Dependencies remain sim → robot, without robot → sim manifest cycles;
using the optional legacy simulation Includes requires installing senior_sim as before.
The real provider uses the installed senior_robot Python preflight before sourcing ROS.
Its original hardware-profile dependencies require the existing SDK safety extra; this
wheel does not provide an independent ROS or manager deployment.

Portable reused tests and installed SDK checks pass with five reproduced legacy fixture
failures and one Linux-peer skip; full commands, counts and limitations are recorded in
[manager M4B](../senior_control/README.md#m4b-实车预检与仿真组合入口).
ROS/ament/colcon and cmake are unavailable locally. Real ament installation, launch
context evaluation, vendor hardware, DDS, simulation, cmd_vel ownership and shutdown
remain unverified; no simulation or real service was started.

### M5C camera and media boundary

`senior_robot/senior_robot/ros_video.py` now owns the original lazy ROS camera subscriber
thread and one latest frame slot. Its required frame constructor is supplied by the
original production binding as `senior_perception.vision.synchronizer.ImageFrame`;
`CapturedFrame` is only a read interface, without a replacement value class.
The module imports neither manager nor Root nor rclpy until the original runtime path
requests ROS. The legacy `senior_robot.camera_source.RosCameraSource` remains a
Compatible constructor over that same implementation.

The original sim node still owns publication. It now directly imports the sole RGB1
publisher from `vehicle_core.rgb1`, within the existing shared legacy owner,
so it does not pass through the manager consumer compatibility module. Header,
permissions, monotonic capture time, dimensions and publisher/GID checks are unchanged.
No image geometry, calibration or detection algorithms moved in this batch.

Management WebRTC/session/signaling/TURN and latest-file consumption are in
`vehicle_manager/media`. The actual shared ImageFrame and RGB1 implementation remain
unique in Root, pending a separate shared-owner decision. Diagnostic queues are still
provided by the original owner. This does not establish an independent ROS or manager
deployment. The installed execution camera imports no Root/manager/ROS, and the shared
publisher imports no manager media; real ROS subscription and simulator publication
remain unverified because ROS/ament are unavailable. Original real aiortc loopback and
installed-wheel RGB1/peer tests passed; commands and platform limits are in
[manager M5C](../senior_control/README.md#m5c-管理媒体与执行端图像接入).


## M5D1 real platform commands

`src/senior_robot/senior_robot/platform_runtime.py` owns the original adapter,
ROS launch, motion, camera command builders and optional video environment probe.
Its `RuntimeProfile` protocol exposes only seven read-only fields consumed by
these functions; it does not import `vehicle_manager` or import ROS on module load.
The old `senior_robot.service_runtime` signatures remain Compatible, with
`ServicePorts` still binding those wrappers. Each camera request injects the real
original `restart_prefix` and original launch hook; no recovery/peer/IPC/learning
implementation moved. The video warning retains `senior_robot.service_runtime`.

The existing Python package collection in setup.py and wheel declaration already
include this module; no build or launch configuration changed. The one-package
Robot wheel was installed outside the checkout and the provider imported without
Root, manager or ROS. Calling the real preflight and running the original ROS
commands still requires their existing owners/environment. This is not independent
vehicle execution or management deployment. Commands, unchanged behavior audits,
old fixture failures and native/Linux/ROS limits are in
[manager M5D1](../senior_control/README.md#m5d1-实车平台命令与视频探针).


## B7 existing navigation

Existing route execution, Nav2 adapter, DedicatedController and navigation launch
now belong to [senior_navigation](src/senior_navigation/README.md). Real safety and
sim motion Includes use the canonical launch; sim controller and AMCL assets read
its share. Historical Python/ROS entries remain explicit Compatible aliases or
Include wrappers. Robot's four historical share assets remain byte-identical frozen
compatibility resources; canonical production assets are in the navigation package.

Map/localization/control authority and the sole chassis cmd_vel owner are unchanged.
The package uses the original shared values/errors/recovery owners, with no manager
or robot implementation imports. Hardware_navigation already forwards localization
and did not move. SDK wheel/old identity checks are separate from unavailable real
ROS/ament/Nav2/Linux validation; see the package record for exact old failures and
commands. No exploration, new map/Home rules or new qualification thresholds landed.
