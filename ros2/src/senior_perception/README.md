# 感知功能包

B2 首片以 `eb4a147` 为基线迁移已有雷达接入、解码、原始/补点观测与相关几何。
这部分已有实现；学习 owner、碰撞/速度算法、RGB-D 和完整感知职责仍待后批迁移。
实际 48 个逻辑文件（迁移单计）：产品 25、测试/夹具 12、配置构建 8、工具文档 3。
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
执行已迁 ROS 回归及原建图/导航入口回归，单独登记实测。没有新增测试场景、部署或推送。
