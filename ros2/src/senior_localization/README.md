> **2026-11-22 当前状态：** 调用方直接导入本功能包；senior_robot 的历史定位转发模块已删除。 下文 B 阶段记录中的旧包名、转发路径和保留承诺属于当时历史，不作为当前安装说明。当前清单及迁移步骤见 [ROS 包归属](../../PACKAGE_OWNERSHIP.md)。

# senior_localization

B5.1 基线 `7f1d603`：既有定位算法、导航地图/位姿资格入口与最小 ROS 定位适配器已迁入。
这是行为迁移；原未启动的定位壳没有接入生产 launch，未接新 GPS、全局纠偏或手动微调算法。

## 实际目录与生产入口

```text
ros2/src/senior_localization/
  CMakeLists.txt, package.xml, pyproject.toml
  scripts/localization_node                 # 原 main 的真实安装入口；本片不启用
  senior_localization/
    __init__.py, py.typed                  # 默认不加载 ROS，保留公开类型信息
    geo.py                                 # 原地理对齐/不确定性与导航姿态策略
    heading.py                             # 原被动航向观测
    relocalization.py                      # 原扫描匹配后才能 ready 的 owner
    navigation_reference.py                # 原固定 grid/SHA/来源和质量配置
    hardware_reference.py                  # 真实导航地图/pose 接入 owner
    localization_node.py                   # 原最小适配/ROS 工厂；不是已接通全局定位
  test/test_localization.py                 # 原同名纯定位用例迁入
ros2/tests/
  test_navigation_reference.py              # 原真实 pose 时间/来源/地图代次回归
  test_mapping.py, test_relocalization.py    # 原 ROS 工作区装配用例
```

geo/heading/relocalization 原在 `src/senior_control/localization/`；navigation_reference 原在
`src/senior_control/hardware_navigation.py`；hardware_reference/localization_node 原在
`senior_robot/senior_robot/`。旧模块及原 localization 公共导出均明确 Compatible，转发同一类型/函数。

生产 agent_adapter 在原 `hardware_navigation_reference` 参数非空时直接组装本包 HardwareReference。
原车端 Unix IPC state.get/pose.get、WorkspaceMap 的地图读取和既有导航 qualified 回调保持。
实车/仿真总装配、AMCL/map_server 与导航实现仍留原组合位置；本片不迁移仿真 launch。
原 `senior_robot` localization_node CLI 名称保留为 canonical main 别名，不能据此宣称定位壳已启用。

## 公开接口与 owner

| 入口 | 原输入 → 原输出 | 保护与可写状态归属 |
|---|---|---|
| NavigationReferenceConfig / load_navigation_reference | 原绝对路径 JSON、固定 grid SHA → 原不可变配置/WorkspaceGrid | 地图像素/来源与协方差门限沿用；display-only 地图不能资格化导航 |
| HardwareReference | 原 /map 与配置 PoseWithCovarianceStamped、原生 receipt → grid/pose/qualified/state/pose_reply | 唯一持有该接入的 source/GID、retired identity、序号、stamp、clock generation 与 map serial；按原时间截止，重复/迟到不续期，切图和来源/时钟切段撤旧 pose |
| GeoAlignment / HeadingObserver | 原 QualifiedFix/MotionSample → 原 PositionEstimate/HeadingObservation | 角度/单位/坐标、不确定性、无航向/无地理参考及质量语义不变；不向底盘输出 |
| Relocalizer.begin | 原 map_ref、PoseHint 与 scan_match 回调 → 原 OperationResult、ready/pose/reason | hint/GPS/last_pose 只作 seed，真实可信匹配后才 ready，失败清除旧 pose |
| LocalizationAdapter / create_localization_node | 原依赖注入算法 → 原观测/地理转换结果 | ROS 工厂仍 unreferenced/unlocalized、scan_match 返回 None且无真实订阅，明确是原未装配壳 |

硬件 pose 接入公开复用 senior_sensor_ingress 的 receipt/metadata 与 senior_motion_estimation 的来源识别；
不反向导入 senior_robot。沿用原单调/原始测量时间、map 帧、正且有界 covariance、来源/GID 资格，
不从 TF、Home、轮速或初值推断地图定位，不把读 state 的时间当新 pose。

## 依赖与未迁职责

ament_cmake_python 安装 Python；CMake 安装真实 localization_node CLI，并收集包内原算法用例。
package.xml 声明 rclpy、geometry_msgs、nav_msgs、senior_sensor_ingress、senior_motion_estimation。
SDK 不内置 ROS，默认导入不加载 ROS；实际 node/原生接入依赖仍需 ROS 安装。
两份 SDK 带 py.typed，迁移后的既有公开注解仍可供类型检查器读取；未宣称全仓 mypy 通过。
算法使用现有根项目公开 Clock/模型/错误与 mapping archive 的 Georeference，不复制值类型。
这些基础库及地图事务按后续 B5/B6 处理，本片不新建共享顶层项目或将车内类型塞进 WS contracts。

原 `localization/gps_course.py` 仍服务运动估计，未随地理定位整目录搬迁，也未启新 GPS。
每图 Home、新建/保存/切换/删除地图完整产品操作、可信定位后持久写图和手动初值微调留后续模块实现。
现有导航、权限、原生输出和停止保护不改。

## B5.1 范围与验证记录

实际 45 个逻辑文件：产品 22、测试及辅助 6、配置构建 13、工具文档 4；新增 15、修改 26、测试迁移 4，
无额外逻辑删除。旧源码仍为 Compatible 文件，因此新 canonical 文件与旧转发各按实际文件计数。
除四份迁移用例外，原 map_references 只改真实 adapter 源路径；原 navigation_collision_qualification
只把已有 ROS 边界替身放到实际 canonical HardwareReference 导入位置，没有添加属性或弱化断言。
两份生产装配文件保留原 CRLF，其余原字节分别只变一个 import/package；不夹带整文件换行改写。

环境：macOS 26.4 arm64，CPython 3.12.15，pytest 8.4.2，Pydantic 2.14.1，uv 0.12.23，ruff 0.17.3。
pytest 使用原主仓库 `.venv/bin/python`。

- 原基线：`python -m pytest tests/unit/test_localization.py tests/unit/test_map_references.py tests/unit/test_navigation_reference.py tests/ros/test_mapping.py tests/ros/test_relocalization.py -q`：64 passed、2 skipped。
- RED：先迁测试读取 canonical 模块，目录实现尚未迁入时因 ModuleNotFoundError 失败；迁入实现后原 console/launch 两项先因未接线失败，补实际安装和 launch 后通过。
- 最终定向：`python -m pytest ros2/src/senior_localization/test/test_localization.py tests/unit/test_map_references.py ros2/tests/test_navigation_reference.py ros2/tests/test_mapping.py ros2/tests/test_relocalization.py tests/unit/test_navigation_collision_qualification.py tests/unit/test_sim_contract.py -q`：77 passed、2 skipped；两项为缺 rclpy 的实际节点检查。覆盖原重复/迟到不续期、切图/时钟切段、hint/无匹配、反序列化空响应及 None/异常、anchor/单 odom owner、stock/native 分支和原扫描参数、现有导航质量与装配。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --offline`、`uv lock --check --offline`：67 包，新增两份既有骨架功能包 SDK 的本地依赖；root pyproject、Docker COPY 与 CI wheel/测试发现一起接线。
- 同缓存 `uv build ros2/src/senior_localization --offline --out-dir /tmp/b51-wheels`、`uv build ros2/src/senior_mapping --offline --out-dir /tmp/b51-wheels` 与根 `uv build --offline --out-dir /tmp/b51-wheels`：三个 sdist/wheel 通过。
- `uv pip install --offline --python /tmp/vehicle-batch-one-venv/bin/python --reinstall --no-deps /tmp/b51-wheels/*.whl`；从 /tmp 用该解释器运行本仓库 tools/verify_batch_one_install.py：已安装 SDK/根公开旧入口类型身份相同、共享契约仍隔离、未加载 rclpy，native 定位明确 UNVERIFIED；`uv pip check` 的 16 个已装包兼容。
- 七份原实现 AST 除导入和模块说明外一致，ROS 旧源码 wrapper 身份相同、无组装包反向导入、包依赖图无环；原生 SLAM、地图事务、Home、导航、仿真 launch 与 slam.yaml 字节未变。
- 定向 ruff check/format 通过；`git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check` 按原 CRLF 检查通过，保留真实尾空格检查。

未验证：本机无 colcon、CMake、rclpy、ament_index_python；实际 ROS/ament 安装/入口定位、DDS/TF、
SLAM 2.6.10、native receipt/graph/capture、AMCL、Linux及实车/仿真建图导航运行未执行。
无 ROS wheel、源码身份、AST 与 launch 替身检查不代替这些实测；SDK 中也不包含 ROS CLI 安装文件。
未运行全仓测试，不宣称解决 B4 已登记的旧控制夹具失败；原保护/断言保留。
未启动开发或实车服务、部署或推送，等待负责人亲审后继续后片。


## B5.2 地图值消费收尾

geo/relocalization/localization_node/navigation_reference 的地图只读类型与 grid 导入现直接来自 senior_mapping，
本包声明该实际运行依赖；原定位业务正文、时间/来源/地图代次/质量门、既有 hint/ready 与旧入口不变。
地图状态和事务由 mapping 持有，schema/数据库/Home 管理仍在原 owner，不将这些业务倒灌定位。
本片 55 文件、225 passed/2 ROS skipped、4 项合成地图持久回归及 SDK/实际 ROS 限制见
[地图包 B5.2 记录](../senior_mapping/README.md#命令版本与结果)。
