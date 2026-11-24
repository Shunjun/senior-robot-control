> **2026-11-22 当前状态：** ROS 消息和服务统一迁到 senior_interfaces；mapping.launch.py 和 slam.yaml 由本包安装。 下文 B 阶段记录中的旧包名、转发路径和保留承诺属于当时历史，不作为当前安装说明。当前清单及迁移步骤见 [ROS 包归属](../../PACKAGE_OWNERSHIP.md)。

# senior_mapping

当前已交付 B5.1–B5.3 的既有实现迁移；最新原生目录和验证见本文 B5.3 记录。下面保留 B5.2 批次记录，
其中“未迁原生”描述的是当时边界，现已由 B5.3 接续；新地图/Home/定位和后置导航功能仍未提前实现。

## B5.2 批次历史记录

B5.2 基线 `a34f485`：现有地图证据/参考、持久化、归档、事务和读取实现已迁入。
本片保留原数据库及外部行为；不是新地图操作、每图 Home、写入 IPC 或原生 SLAM 功能交付。

## 实际目录与来源

```text
ros2/src/senior_mapping/
  CMakeLists.txt, package.xml, pyproject.toml
  scripts/slam_adapter                         # B5.1 原安装入口
  senior_mapping/
    __init__.py, py.typed                       # 既有地图公开值与 store
    evidence.py, occupancy.py                  # 原证据与稳定层
    references.py, store.py                     # 原锚点/绑定/代次与持久快照
    archive.py                                 # 原一致捕获/验证与 bundle
    workspace.py, workspace_export.py           # 原地图读取/导出 grid
    operations.py                              # 唯一地图事务业务 owner
    persistence.py                             # 最小连接/写锁/schema/Home effect 接口
    schema.py                                  # 原地图 journal SQL 声明
    slam_adapter.py                            # B5.1 原 SLAM Python 适配
  test/test_map_archive.py, test_map_references.py
ros2/tests/test_map_operations.py
src/senior_control/
  mapping/*.py                                 # 原入口显式 Compatible
  storage/map_persistence.py                    # 原管理 schema/Home 回调组装
  storage/{database,migrations,core_schema}.py   # 原连接/迁移引擎，本片未改
  waypoints/{store,home}.py                     # 原 Home 管理，仅更新地图类型导入
```

8 份既有实现来自 `src/senior_control/mapping/` 同名文件。原 __init__ 和普通旧模块只转发同一类型/函数。
MapRef/Pose2D/错误等原本地公开基础值继续复用，不新建顶层包，不复制到 WS contracts。
16 份真实消费文件改 canonical 导入：storage/schema、waypoints/home/store、tasks/executor、
agent/dispatcher/workspace/composition、vision/depth/tracking、senior_robot/agent_adapter、
senior_sim/scripts/world_reference、senior_localization 的 geo/relocalization/localization_node/
navigation_reference、以及本包 slam_adapter；其业务正文不变。

## 公开存储边界与状态归属

| owner / 接口 | 实际职责 | 连接、锁与提交约束 |
|---|---|---|
| MapOperations | 原 map lock、graph work/guard/review、导入/导出/clear、幂等记录和恢复 | 仍持有调用者原 database 对象，以原 write_lock 和 BEGIN IMMEDIATE/rollback/commit 执行所有地图 SQL |
| MapDatabase | 只读公开 connection 与 write_lock | 结构化描述原对象；不建连接、不包新锁、不管理生命周期 |
| MapPersistence.prepare_schema | 原 domain migrate 后 core migrate | 在 store.guard_reference、stop_and_block_navigation 后调用；原 Database/path/备份/迁移版本引擎归管理 owner |
| HomeImport | 仅 adopted_home_blocks_foreign 与 replace_imported_home | 接受地图已开启事务的 connection；不加锁、不连接、不 commit，不持有 Home 业务状态 |
| storage.map_persistence | 每次构造明确组装上述回调 | 委托原 storage 与 waypoints 函数，延迟调用；无全局工厂、注册表或给 Database 增加地图业务方法 |
| schema.OPERATION_JOURNAL_STATEMENTS | 原地图四段 SQL 声明 | 根 storage/schema 直接引用并按原 version 1/2/3 组装 domain migrations，避免导入业务 owner |

canonical 不导入管理 storage/waypoints/agent 或组装包；原 Database、schema 迁移、任务与 Home 管理仍原位。
新 canonical MapOperations 构造必须显式提供 persistence；真实 SQLite 原夹具用管理回调连接原 database。

### 旧构造的精确兼容形式

`senior_control.mapping.operations.MapOperations` 是明确标注 Compatible 的薄构造子类，保留原路径和构造签名，
只组装存储回调并调用 canonical super 构造；所有业务方法和可写事务状态只有 canonical 一份实现。
**旧 owner 类与 canonical 类不是同一对象**，旧类是其子类；import_bundle/reconcile 等业务方法仍是同一个函数。
共享 CellEvidence/MapReferenceStore/HomeLink/OperationRecord 等值与异常保持同一类型身份，不使用类似构造壳。
E2E 保留旧构造路径，事务集成原夹具改用显式新端口；没有复制第二套业务实现或隐式默认注入。

## 原持久与安全行为

- 原 SQLite 路径、busy timeout、foreign_keys=ON、WAL、synchronous=FULL、原 .backup 路径不变；domain 版本 1/2/3、core 版本 1–4、Home owner_token 与所有原 SQL 保留。没有增加、重编号或重建 schema。
- MapOperations 仍先停机/封导航再等 DB，操作 id 绑定原 action/payload/完整 bundle 指纹；原锁顺序、事务、查询、取消和 NEEDS_REVIEW/FAILED/SUCCEEDED 语义不变。
- Home 导入检查、waypoint 写入、map_home_link、operation 成功与 guard 删除仍在同一 SQLite 写事务；外机 Home pending_confirmation/adopted=0，确认 Home 防覆盖和同 ref 新 generation 的旧位姿失效保持；不让导入成功等同 relocalized。
- 原 active/.active.previous/.staging/history、opaque map id 的 ~sha256 路径、JSON/bundle format_version=1、哈希、字节/成员界限、fsync/rename、symlink 防护、原稳定层与 removed 记录不变。
- 原 graph work shield、0.25 秒确认/等待、graph_guard 和 reconcile 的 quiesce/matches/一致捕获证明保持；迟到工作未完成或未 fence 时不能恢复，服务 ack/文件 copy 不当作 graph 身份确认。

## 真实装配与未交付能力

现有地图读取继续由 agent_adapter/WorkspaceMap 提供原 map.active/manifest/tile，定位和导航继续核对原地图与定位代次。
根管理 StoreMapPort 仍是既有直接适配；生产 composition 使用 IpcMapPort，但原 agent_adapter 并没有地图 export/import/clear 写 handler。
本片不新增生产 MapOperations 构造或写入 IPC，不声称未装配链已经可用。

B5.1 默认 stock slam_toolbox 加 Python adapter、capture/verified 分支的 senior_slam_bridge、原扫描参数和
仿真总 launch 不变。原生 SLAM/C++/typesupport/捕获工具留后片；新建保存切图删除、每图 Home、手动微调及可信定位写图协作另批实现。

## 构建与实际范围

原 ament_cmake_python 安装本包全部模块，新增包内原算法用例的 ament_cmake_pytest 注册；不增加程序。
SDK 显式声明已使用的 aiosqlite>=0.20,<1，uv.lock 更新同一已有依赖，仍解析 67 包；已有 Docker COPY 无需改变。
localization/perception 增加实际地图值依赖；ROS manifest/CI/pytest 发现同步，根 SDK 继续提供现有基础库和管理适配器。
不猜测未核验的 rosdep key；真实 ROS 环境仍需原根 SDK 安装闭包，不能把 SDK 构建当作 ament 实测。

实际 **55 个逻辑文件**：产品 37、测试及辅助 6、配置构建 8、工具文档 4；新增 11、修改 41、测试迁移 3，无额外逻辑删除。
三份原测试分别迁入包内 test/ 与 ros2/tests，原 E2E/conftest 和真实 Database 夹具保留；只更新真实模块补丁、
canonical 显式构造的 persistence 和原故障注入子类夹具，不增加无关场景或假属性，不弱化断言。

## 命令、版本与结果

环境：macOS 26.4 arm64，CPython 3.12.15，SQLite 3.53.1，aiosqlite 0.22.1，pytest 8.4.2，
Pydantic 2.14.1，uv 0.12.23，ruff 0.17.3；pytest 使用原主仓库 .venv/bin/python。

- 基线 `python -m pytest tests/integration/test_storage.py tests/integration/test_map_operations.py tests/unit/test_map_archive.py tests/unit/test_map_references.py tests/unit/test_home.py tests/integration/test_workspace_data.py ros2/src/senior_localization/test/test_localization.py -q`：207 passed。
- RED：原归档测试先改 canonical 导入，在实现未迁入时失败；迁入后 205 passed/2 failed 定位到原故障子类夹具缺显式 persistence，补真实构造接线后原 207 项通过，未改产品去绕过故障。
- 最终 `python -m pytest tests/integration/test_storage.py ros2/tests/test_map_operations.py ros2/src/senior_mapping/test tests/unit/test_home.py tests/integration/test_workspace_data.py ros2/src/senior_localization/test/test_localization.py ros2/tests/test_navigation_reference.py ros2/tests/test_mapping.py ros2/tests/test_relocalization.py -q`：225 passed、2 skipped；两项实际 ROS 节点检查因缺 rclpy 跳过。通过项含真实临时 SQLite 的初始化/迁移/备份、幂等、损坏/无确认、取消/迟到/恢复、同事务 Home、同 ref 新代次、clear、持久层约束、真实读取接口与原定位/launch 回归。
- `python -m pytest tests/e2e/test_maps.py -q`：首次沙箱内 3 passed/1 error，SimulationProfile 本地端口预留被拒；允许测试绑定后同命令 **4 passed**。这是合成 Graph 加真实 SQLite/文件持久回归，不是实车或 Linux ROS SLAM 验证；没有启动监听服务。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --offline` 和 `uv lock --check --offline`：67 包，SDK aiOSQLite 元数据与锁闭包一致。
- 同缓存 `uv build ros2/src/senior_mapping --offline --out-dir /tmp/b52-wheels`、对 senior_localization 以及根项目 uv build：三个 sdist/wheel 通过；`uv pip install --offline --python /tmp/vehicle-batch-one-venv/bin/python --reinstall --no-deps /tmp/b52-wheels/*.whl` 后，从 /tmp 运行 tools/verify_batch_one_install.py：共享值 identity、旧构造子类/单一方法实现、原 schema 声明同一引用、无 ROS 默认导入均通过，native 安装明确 UNVERIFIED；`uv pip check` 的 16 包兼容。
- 七份非事务地图模块与 16 份消费者业务正文 AST 不变；事务 AST 只改 schema 准备和两项 Home effect 的边界调用与构造注入，原 SQL 字符串、schema tuple 和旧构造签名逐项一致；管理/组装反向导入与 ROS 包循环检查通过。作者已单独审阅 diff，等待负责人亲审。
- 定向 ruff check/format 与保留 CRLF 的 `git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check` 通过；Database/migrations/core_schema、原生 SLAM、launch、导航与写入 IPC 实现未改。

未验证：缺 colcon、CMake、rclpy、ament_index_python；真实 ROS/ament 构建安装、DDS/TF、SLAM/AMCL、
native capture/receipt、Linux及实车/仿真运行仍待对应环境。SDK/AST/合成 Graph 不替代这些实测。
未运行全仓或全仓 mypy，不宣称解决 B4 旧控制夹具失败；没有启动开发/实车服务、部署或推送。


## B5.3 原生 SLAM 迁移

基线 `4ec3d12`，本片将原运行代码归本功能包，原 bridge 只保留 ROS 类型。
不修改16份实现、CLI和18份测试/探针内容；35个迁移文件与基线逐字节一致。

### 当前增量目录与安装

```text
senior_mapping/
  src/slam/
    slam_graph_node.cpp, acquire_scan_receipt.cpp
    native_capture.cpp/.hpp, staged_graph.cpp/.hpp
    bounded_scan_ingress.cpp/.hpp, static_lidar_tf_ingress.cpp/.hpp
    snapshot.hpp, graph_commit_history.hpp, laser_geometry.hpp
    static_lidar_tf.hpp, scan_receipt_admission.hpp, candidate_stationary_match.hpp
  scripts/candidate_preflight.py
  test/native/
    test_{snapshot,graph_commit_history,laser_geometry,static_lidar_tf}.cpp
    test_{stationary_match_quality,native_capture,staged_graph}.cpp
    test_candidate_preflight.py
  test/probes/
    probe_{read_graph,native_capture,bounded_scan_ingress,graph_commit_delivery}.py
    probe_{static_lidar_tf,candidate_live_scan,candidate_stationary_match}.py
    probe_{acquire_scan_receipt,candidate_match_ambiguity,candidate_load}.py
```

两个 native 程序与 candidate_preflight 共同安装 `lib/senior_mapping/`，原安装 basename 不变。
CLI用自己的绝对调用路径找两个兄弟程序，因此不能只搬脚本、只留一个旧路径symlink或依赖merge-install。
新命令统一为 `ros2 run senior_mapping {slam_graph_node,acquire_scan_receipt,candidate_preflight}`；
旧包三个运行命令取消，无 shim 或反向依赖。原消息/服务仍为 senior_slam_bridge，六份定义一字未改。

生产 mapping.launch 的 capture/verified 分支只改 native 节点 package 为 senior_mapping；默认stock+原Python adapter、
node名slam_toolbox、原扫描/TF/frame/receipt、参数文件、进程数及restart差异不变。
单live图owner保留，隔离candidate仍只诊断，不得应用位姿、抢live图或授予定位/写图/导航/运动权限。
原capture、graph cut/history、0.5秒接收准入、私有文件、单次匹配quota、两seed、取消/超时/确切子进程回收均不改。
原消息对历史receipt和authority的限制说明不变，不能把字段valid重新解释成当前上游或地图资格。
内部C++ namespace/header guards保留原名，以便本次只归包、不夹带ABI或序列化重命名；私有头不新增公共安装API。

### 构建与注册

默认完整构建所有原生目标，不加可选自动探测、静默漏目标或缺依赖fallback。
当前构建使用车端 ROS 2 Humble 容器中已有的 slam_toolbox 2.6.8。CMake 和 package.xml 均锁定 2.6.8，沿用其私有 async_slam_toolbox/toolbox_common/kartoSlamToolbox
库定位与Boost serialization/thread、OpenSSL::Crypto、TBB::tbb、Threads、rclcpp/tf2/sensor_msgs/motion_msgs依赖。
bridge只rosidl生成接口，mapping通过标准ament导出消费它，不猜外部imported target名称或复制同项目graph_typesupport变量。
真实Humble类型导出/私有符号链接仍须实际编译验证，源码核对不能替代。

全部原native CTest名称、target、探针模式参数、30–120秒timeout和配置文件COPY注册保留，只改源路径与运行包prefix。
原mapping_algorithms用pytest ENV明确忽略新增native子目录，preflight仍由原专用CTest执行，避免CTest重复收集；
根Python CI仍通过mapping/test收集该原unittest一次，pyproject提供迁入scripts导入路径。
ENV参数的源码依据为 [Humble ament_add_pytest_test](https://github.com/ament/ament_cmake/blob/humble/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake)，本机未实测该CMake注册。

### 实际范围

46个逻辑文件：产品17、测试19、配置构建6、工具文档4；新增1、修改10、迁移35，无额外逻辑删除。
产品是16份C++与实际runtime CLI；测试是18份原用例/探针迁移与现有launch断言改owner。
两包CMake/manifest、mapping.launch与根pytest路径同步，现有安装工具只读检查新prefix/原类型，无新场景。
写入IPC、B5.2事务/Home、原控制守卫、仿真总组合、导航算法与新地图/定位功能未动。

### 命令、版本与结果

环境：macOS 26.4 arm64、CPython 3.12.15、pytest 8.4.2、uv 0.12.23、ruff 0.17.3、
Apple clang 21.0.0、pkg-config提供libcrypto/OpenSSL 3.6.2；无colcon、CMake、rclcpp/rclpy、Karto/Humble安装。
pytest使用原主仓库.venv/bin/python；以下原生输出与数据均在/tmp。

- RED：先修改原launch断言为canonical运行包，原装配尚未修改时失败；补实际package接线后通过。
- `python -m pytest ros2/src/senior_mapping/test/native/test_candidate_preflight.py ros2/tests/test_mapping.py -q`：17 passed、1 skipped。
- `python -m pytest ros2/src/senior_mapping/test ros2/tests/test_mapping.py -q`：**72 passed、1 skipped**，包括原地图算法、preflight确切子进程关停与原stock/native参数分支；一项实际ROS节点检查缺rclpy跳过，没有启动ROS节点或服务。
- 对snapshot、graph_commit_history、laser_geometry、static_lidar_tf分别执行 `c++ -std=c++17 -Wall -Wextra -pedantic -I ros2/src/senior_mapping/src/slam ros2/src/senior_mapping/test/native/test_NAME.cpp -o /tmp/b53-native/NAME` 并运行产物：四个原C++程序通过。
- 对native_capture、staged_graph分别按同旗标增加对应src/slam/NAME.cpp与 `pkg-config --cflags --libs libcrypto`，运行时传入/tmp/b53-native/NAME-data私有目录：两个原C++程序通过；这是host POSIX验证，原test_native_capture的Linux条件分支未执行，不称为Linux捕获验证。
- `c++ -std=c++17 -fsyntax-only -I ros2/src/senior_mapping/src/slam ros2/src/senior_mapping/test/native/test_stationary_match_quality.cpp`：因缺karto_sdk/Mapper.h失败，未验证Karto质量计算或私有库链接。
- 对slam_graph_node用同语法检查并增加 `-I ros2/src/senior_motion_msgs/include`：因缺rclcpp/rclcpp.hpp失败，未验证ROS节点编译、typesupport或私有SLAM链接；不以纯单元代替。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build ros2/src/senior_mapping --offline --out-dir /tmp/b53-wheels` 与根uv build：sdist/wheel通过；wheel仍只有原Python SDK，不能代替native/CLI安装；`uv lock --check --offline`仍67包。
- 用uv pip向/tmp/vehicle-batch-one-venv安装上述两个wheel后，从/tmp运行现有tools/verify_batch_one_install.py：SDK兼容与共享边界通过，16个依赖包兼容，原guard与新SLAM安装/types检查都明确UNVERIFIED（无ament_index/ROS prefix）。工具首次误用未安装SDK的pytest解释器缺senior_contracts，改到实际wheel环境后通过，未把前置环境缺失算产品通过。
- 35个移动文件和6个接口逐字节核对，完整原生target/探针参数/timeout注册核对、包依赖无环、无bridge反向运行依赖、launch仅owner/说明变化：通过；只证明源码和声明。
- 定向ruff check/format、按原CRLF的 `git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check`：通过；作者已审diff，仍待负责人亲审。

未验证：真实Humble隔离prefix的colcon构建/安装、旧类型生成与typesupport导出、私有2.6.8链接、
Karto单元、10个DDS探针及其全部模式、native receipt/capture/TF/子进程链、Linux和实车/仿真运行。
推荐对应环境用全新独立prefix构建bridge→mapping，再执行原CMake/CTest及安装工具，不自动清理已有安装树，
以免旧lib/senior_slam_bridge残留程序被误判为兼容入口。
没有启动开发/实车或ROS服务，没有部署、推送或提前实现其他功能；未跑全仓，也不宣称解决旧控制夹具失败。
