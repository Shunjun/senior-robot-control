# senior_mapping

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
