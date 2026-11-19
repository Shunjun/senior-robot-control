# 批次 1 验证记录

分支 `feat/vehicle-architecture-refactor`，本批基线 `a445b1b`；记录对应本文件所在提交。确认稿位于 `docs/architecture/ROS2与车端架构重构确认稿.md`。本批只迁契约和原生控制输出，未实施后批算法、未启动开发/车辆服务、未部署或推送。

环境：macOS arm64；CPython 3.12.15；pytest 8.4.2；ruff 0.17.3；uv 0.12.23；Apple clang 21.0.0。无 colcon、rclpy 或 ament_index_python。下述 PY 为父仓库既有 `.venv/bin/python`（完整路径 `/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python`）；pytest 显式使用本工作区 src 和新包源码路径，不借父仓库旧算法代替。

## 已选择的原回归

```sh
"$PY" -m pytest -q tests/unit/test_protocol.py tests/unit/test_native_output_protocol.py ros2/src/senior_motion_control/test/test_final_command_frame.py tests/unit/test_startup_learning_chain.py tests/unit/test_settings.py tests/unit/test_motion_launch_profile.py tests/unit/test_sim_contract.py tests/ros/test_navigation.py tests/ros/test_mapping.py --disable-warnings --maxfail=1
```

结果 **115 passed, 9 skipped**，5.45s。包括 Python 40字节字段、原输出停止/截止确定时钟 harness、FinalCommand 真实C++头→Python roundtrip、完整学习控制 tick、驾驶配置、实车/仿真启动参数与既有导航静态/直接调用装配。6项 native SOCK_SEQPACKET 为Linux限定；其余ROS缺失项跳过，不能当作真实Nav2或SLAM运行通过。

```sh
"$PY" -m pytest -q tests/unit/test_launch_recovery.py --disable-warnings
```

结果 **6 passed, 1 skipped**，12.00s。测试需临时Unix域socket，沙箱中 `bind` 出现 EPERM，获测试权限重跑后通过；未启动应用服务。该组原来对 `motion_core_output.LearningClient` 的补丁地址在 a445b1b 已失效，本批只迁到真实 owner `vehicle_learning.LearningClient`，保留原恢复/停止断言。

## 真实 wheel 构建与非仓库安装

```sh
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-batch-one-wheels
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build contracts --offline --out-dir /tmp/vehicle-batch-one-wheels
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build ros2/src/senior_motion_control --offline --out-dir /tmp/vehicle-batch-one-wheels
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv venv /tmp/vehicle-batch-one-venv --python 3.12
UV_CACHE_DIR=/tmp/vehicle-uv-cache uv pip install --python /tmp/vehicle-batch-one-venv/bin/python /tmp/vehicle-batch-one-wheels/*.whl
cd /tmp
/tmp/vehicle-batch-one-venv/bin/python -I /Users/zhangshunjun/workspace/orca/senior-robot-control/vehicle-architecture-refactor/tools/verify_batch_one_install.py
/tmp/vehicle-batch-one-venv/bin/senior-robot --help
```

三个独立 wheel 构建/安装成功。干净前缀实际安装 senior-control / senior-contracts / senior-motion-control 0.1.0，Pydantic 2.14.3；无ROS，不借仓库sys.path导入。旧管理模块的 Velocity、DomainError、RoutePlan/Progress、DrivingSettings、序列化/解析函数与新契约为同一对象；新控制 Python 模块可无 ROS 导入；原 CLI help 正常。依赖锁只新增 senior-contracts，没有升级现有依赖；独立 wheel 安装解析的是其公开版本范围，与根锁文件解析结果分开记录。

原生定位工具明确输出 `UNVERIFIED (ament_index_python / ROS prefix absent)`；完整 ament 安装、包索引与 native 可执行存在性在此环境没有验证。Python wheel 不含 native，不得用于车辆运行替代 colcon 安装。CI增加同一离仓库 wheel smoke；真实ROS环境可用该工具核对安装程序位置。

## 静态与行为保留

对本批所有改动 Python 文件运行 ruff check / format --check；`git diff --check`；`uv lock --check --offline`。对三个迁移 native 文件与 `git show a445b1b:原路径` 逐字节比较相同。契约AST没有导入 senior_control/senior_robot/rcl；新包只含传输值及校验，没有 QualifiedRoute 执行。扫描旧 native 源/安装路径后，生产查找都使用 senior_motion_control；native_receipt 有意暂留消息包。

迁移验证沿用原场景，没有新增驾驶情景。安装检查工具只验证本批批准的真实安装/类型身份/定位边界。原两个守卫、继承FD、唯一cmd_vel、上游事件、原合并截止与停止屏障不变。

## 失败与未验证

- 已选回归迁移前的RED：更新 native 测试到新目标路径，包尚不存在时 Python FileNotFound 与 C++ include失败；实现迁移后同测试通过。
- 尝试过扩展组 `test_motion_guard_to_owner.py` 与 `test_terminal_native_channel.py`：均在进入场景前被已有 `control_process_test_support.local_resources` 对不存在 `MotionCoreOutput.ProcessWorker` 打补丁阻断；这是 a445b1b 已登记的旧架构夹具，不修改产品伪造属性、不删测试或弱化断言；仅更新其真实源码位置，留后续旧夹具迁移。
- 缺缓存的离线 uv lock / install 初次失败已通过获取正常构建/安装依赖解决，未改变版本上限来迁就环境。
- 未执行完整全量套件、Python3.10/Windows、Linux六项协议运行场景、真实ROS/ament/colcon构建安装、DDS、实际Nav2/SLAM/底盘、实车和仿真端到端；本地静态装配不能代替这些验收。
