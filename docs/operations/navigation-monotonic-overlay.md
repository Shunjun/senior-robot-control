> Historical navigation-only instructions below are superseded by [the full clock overlay contract](control-clock-overlays.md): senior_motion_msgs 0.2.0, control-monotonic-v2, 111 selected original files and 11 patched files across controller/MPPI/AMCL. AMCL retains its upstream LGPL-2.1-or-later notices; controller/MPPI retain Apache-2.0. Installed vehicle versions and ABI remain unverified.

# Nav2 1.1.20 单调时间生产端 overlay

本次工程基线为官方 Nav2 Humble `1.1.20`，不可变 commit `a097086719c88f781aa59788eca29ac6ca5e56db`，不表示实车已安装此版本。源码来自官方仓库，保留原 Apache-2.0 文件头及 LICENSE；本仓仅维护最小补丁、111文件散列清单和准备脚本，不维护整套 Nav2。

## 构建与安装前提

在 Linux/Humble 的独立构建目录操作，先人工核实当前车用 Nav2/MPPI 与声明依赖的1.1.20版本和ABI。脚本及运行入口检查 controller、core、util、costmap、msgs、nav_2d_utils、nav_2d_msgs、mppi、bt_navigator、behavior_tree 的 package.xml 版本；版本不符明确报错，不自动升级、安装或部署任何依赖。这些声明版本检查不能替代实际链接/插件ABI验证。

先在项目ROS工作区构建并 source `senior_motion_msgs`，使新的 NavigationSample 及公开 timing header 可用。以下命令只编译，不启动节点；具体工作区路径由操作者填写：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2/src/senior_motion_msgs --packages-select senior_motion_msgs
source install/setup.bash
python3 tools/prepare_nav2_overlay.py \
  --source /tmp/nav2-1.1.20-cache \
  --destination /tmp/nav2-navigation-overlay/src \
  --download --build
source /tmp/nav2-navigation-overlay/install/setup.bash
```

The manifest selects 111 original files from nav2_controller, nav2_mppi_controller and nav2_amcl, with original/final SHA256 for 11 patched files and both patch hashes. Preparation requires a new destination, exact source checks and zero-fuzz patch context. Controller/MPPI retain Apache-2.0; AMCL retains upstream LGPL-2.1-or-later. Build and source senior_motion_msgs 0.2.0 first; the runtime marker is control-monotonic-v2. Package declarations do not prove target plugin ABI. No dependencies are upgraded or deployed automatically.

构建时应 source 实际 senior_robot 工作区与相关依赖，使 overlay 的 setup.bash 链接到正确的父工作区。生产配置 `ros_setup` 须人工指向含此overlay的setup.bash，确保 agent_adapter 与 DedicatedController 启动环境能找到同版 NavigationSample、controller 和插件；未 source overlay 时入口因 marker 缺失明确报错。不要只替换一个消息包而继续使用旧controller二进制。

## 生产与接收契约

非零目标在 `computeAndPublishVelocity()` 入口、读取姿态和调用控制插件前捕获 `CLOCK_MONOTONIC`。在最终发布边界保留该前沿，不因计算或排队耗时重新盖时间。独立零目标在 `publishZeroVelocity()` 产生时捕获单调时间，两条分支共用最终发布函数及序号。NavigationSample携带 Linux boot_id、私有run nonce、严格递增sequence、produced_steady_ns、vx/wz。

C++ CLOCK_MONOTONIC与Linux Python `time.monotonic()`属于同车同域；不能跨车、重启或不同monotonic time namespace比较。部署需明确核实容器未创建独立monotonic time namespace。CommandNode绑定boot/run与唯一controller图谱GID，按生产sequence和原始steady年龄校验，原始deadline一直传递到FinalCommand。Humble Python消息元数据可缺少逐样本GID，保留原有私有run-topic身份契约；有GID时必须匹配，图谱writer重启/替换即不接受旧绑定样本。此版本不新增旧Twist或wall时间兼容路径。

控制等待、失败容忍与SimpleProgressChecker超时改steady；TF/路径和日志ROS时间语义保留。统一限速在共用ConstraintEngine以当前RiskEnvelope同比缩放目标后继续执行组合扫掠与最终输出核验，导航既有degraded_cap策略保留。

## 验证边界

本地macOS仅验证源码锁定、补丁上下文、独立C++编译及发布函数、纯Python控制链。完整ROS/Humble构建、生成消息、ament安装导出、真实DDS/GID和实际插件ABI须在目标Linux同版环境完成；部署和实车验收均未执行。新增头文件也接入现有installed_header_consumer编译测试，但本机未运行该ament测试。
