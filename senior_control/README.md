# 车端管理

M1a 设置、M1b 车辆日志、M2a SQLite引擎已从原根 SDK 迁入；其他管理职责仍待各片迁移。本项目目录 `senior_control/`，
导入名 `vehicle_manager`，发行名 `vehicle-manager`。没有新增CLI/服务入口，也未迁导航或ROS执行。
下文各片记录按当时范围保留，当前SQLite边界与验证以末尾M2a记录为准。

## 实际目录与调用

```text
senior_control/
  pyproject.toml                       # contracts、Pydantic、aiosqlite
  src/vehicle_manager/
    __init__.py, py.typed              # 顶层无业务加载副作用
    settings/
      __init__.py                     # 原公开类名按需加载，不随models导入service
      models.py                       # 不可变设置值、偏好范围及原默认值
      ports.py                        # 只读connection/write_lock接口
      store.py                        # 原同连接SQLite事务/完整图像/receipt
      service.py                      # 唯一设置owner、runtime gate和DEBUG恢复timer
    vehicle_logs/
      __init__.py                     # 按需公开服务/Store，不随值导入加载owner
      models.py                       # 原车辆日志值、过滤/脱敏，复用共享查询DTO
      store.py                        # 原独立SQLite文件、MEMORY journal及锁
      service.py                      # 唯一日志业务实现，授权/队列/订阅/持久化
      diagnostics.py                  # 原诊断owner的显式技术端口，无诊断实现
      wire.py                         # 同一共享LogRequest/LOG_WIRE_BYTES
    storage/
      __init__.py                     # 只公开技术入口，不导入domain装配
      database.py                     # 唯一连接引擎，显式注入原写锁构造端口
      migrations.py                   # 原顺序迁移、备份、取消与回滚
      core_schema.py                  # 原core v1–v4 SQL及迁移实现
      agent_schema.py                 # 原journal两条SQL tuple，无任何实现导入
  tests/
    test_vehicle_settings.py          # 原62项设置用例，无新增场景
    test_vehicle_logs.py              # 原38项日志用例，无新增场景
    test_database_ownership.py        # 原2项连接所有权/重入用例
    README.md
```

原 `src/senior_control/settings/{models,service,store}.py` 的唯一实现迁入上述同名模块；
旧三模块和包公开入口仅显式 `//Compatible` 转发，值、函数、SettingsStore/Service/RuntimeGate均为同一对象，
没有第二份业务实现、薄构造子类或全局provider。

实际生产仍由原 `senior-robot` / `python -m senior_control.agent.main` → 原agent composition装配：
`SettingsService(SettingsStore(database), vehicle_settings_profile(), gate)`使用迁入的同一实现，
原Database、连接生命周期、诊断、任务/Home、日志及运行入口保持原位。
设置模型仍是产品偏好，不是物理能力/安全证明，原ROS command_node继续从旧路径读取同一DrivingSettings。

## 公开接口和所有权

`SettingsDatabase`只声明两个只读属性：`connection: aiosqlite.Connection` 与 `write_lock: asyncio.Lock`。
原Database结构满足接口；store直接保留注入对象，既不打开/关闭数据库，也不创建连接/锁或复制迁移引擎。
事务仍为同锁下rollback、BEGIN IMMEDIATE、写值+receipt、commit，BaseException回滚。
数据库路径、表/字段、schema版本、原五字段兼容恢复为六字段的唯一规则不变。

service的原锁顺序、runtime guard/generation和刷新、安全准入与不可变发布时序不变；
commit取消仍先settle真实提交/发布再传播，拒绝/幂等receipt和DEBUG恢复不重复执行。
DEBUG timer仍仅归SettingsService，日志只读published_snapshot，不新增恢复owner或自动恢复运动。

纯设置值的公开模块是 `vehicle_manager.settings.models`；包公开入口仍有RuntimeGate/SettingsService，
但仅在实际请求这些名字时加载service。旧包入口显式转发同一按需入口，不对全模块做动态导出。
新进程验证旧/新models导入都不加载manager service/store或原Database；请求公开服务类后两端身份仍相同。
这不代表已清除原ROS中所有agent/manual_control引用；那些后续边界不在M1a改动内。

依赖图为 `senior-control → vehicle-manager → Pydantic/aiosqlite`。
manager没有Root SDK、cloud、WS契约、ROS或它们业务实现的反向依赖；原连接以对象接口传入。
根uv锁只新增manager项目，原68包版本不变，总69包。
CI增加manager源码/用例发现与wheel构建，旧根wheel安装须同时带manager wheel。
本片不引入独立manager锁；云锁、镜像与WS contracts未改。

## 实际范围

基线 `0928aef`，实际20逻辑文件：产品11、测试1、配置4、工具文档4。
产品为4个canonical设置文件、4个旧转发、ports、py.typed及现有manager init；
原test_vehicle_settings唯一迁入，没有复制测试或修改其他夹具。
配置为manager/root pyproject、根uv.lock、Backend CI；工具文档为既有安装工具、本页、tests README和原实施计划。
保留旧路径的实现迁移在Git中可能显示新增canonical+修改旧文件，不把兼容文件当已删除。

## 命令、版本与结果

环境：macOS26.4 arm64，主回归CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23，
Pydantic2.14.1/aiosqlite0.22.1。所有用例均为原SQLite/ASGI内存/私有IPC测试，不启动开发或实车服务。

- RED：先把原设置用例导入改为vehicle_manager，迁移前collect报ModuleNotFoundError，随后实现接线。
- `python -m pytest senior_control/tests/test_vehicle_settings.py -q`：**62 passed**。
- `python -m pytest senior_control/tests/test_vehicle_settings.py tests/integration/test_settings_logs_wire.py tests/integration/test_settings_logs_review.py::test_settings_refused_enqueue_settles_owner_without_leaking_eight_slots tests/integration/test_settings_logs_review.py::test_settings_discard_settles_only_the_original_connection ros2/tests/test_manual_command_generation.py -q`：**75 passed**（3.14s），包含前述62项，不相加。
  覆盖真实同连接SQLite并发/幂等/取消、旧图像与损坏拒绝、DEBUG恢复、真实unsafe gate证据/原帧IPC、日志策略只读、设置wire和原手动命令装配。
- 初轮另加tests/unit/test_degraded_manual_speed.py：84 passed、50 failed；
  该文件单独在git archive的0928aef基线复跑为9 passed、50 failed，失败node ID逐条相同，
  退役夹具依赖已不存在的MotionCoreOutput._snapshot。没有补假属性、改控制实现或弱化断言来掩盖旧失败。
- `MYPYPATH=senior_control/src:src:contracts/src:ros2/src/senior_mapping:ros2/src/senior_localization:ros2/src/senior_motion_control:ros2/src/senior_motion_estimation:ros2/src/senior_perception:cloud/src mypy senior_control/src/vehicle_manager`：6份源码通过。
  同路径全Root+manager+cloud检查仍38项错误；与本轮0928aef归档全链检查逐条相同，未称全仓类型检查通过。
- 12份相关Python的ruff check/format --check与按原CRLF的git diff --check通过。
  models/service/store及原62项测试的业务AST除导入/公开DB类型注解外相同；
  原DB/日志/agent/ROS/云/contracts/deploy均无diff，公开manager源码无反向实现导入。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --check --offline`：69包，原版本集合只增加vehicle-manager0.1.0。
- `uv build senior_control --offline --out-dir /tmp/m1a-wheels`及根uv build：sdist/wheel成功。
  仓库外 `/tmp/m1a-manager-venv`只装manager wheel及其真实第三方7包，pip check通过；
  元数据仅aiosqlite/Pydantic，Root/cloud/ROS/contracts均不存在，公开值导入隔离及包服务入口身份验证通过。
  同一wheel在已安装CPython3.10.22的/tmp/m1a-manager310也完成7包安装/pip check与上述导入/公开身份检查；
  初次离线因缺对应pydantic-core wheel失败，补真实声明版本依赖后通过，未把空环境pip check当安装验证。
- `/tmp/vehicle-batch-one-venv`复用既有完整SDK依赖，重装本轮Root/manager wheel：24包pip check通过；
  从/tmp执行更新后的tools/verify_batch_one_install.py，旧新值/函数/owner同一身份、py.typed及新进程值导入隔离通过。
  原DrivingSettings的旧module断言改为精确canonical归属+旧新is断言，没有删除保护；其他Root值归属断言保留。
  native/SLAM安装仍明确UNVERIFIED，无ROS/ament前缀。

没有跑全仓测试或实际Linux/ROS/DDS、实车/仿真运行；原四项控制夹具失败、C3 Docker daemon/镜像实测缺口仍承接历史记录，
本片不宣称解决。日志迁移、通用诊断边界、数据库/任务/Home与管理启动闭包留后续各片，导航算法最后。
没有部署、推送或开始下一片；本轮作者自审后交负责人亲审。

## M1b 车辆日志owner迁移

基线`ec7c031`。原日志models/store/service/wire和公开入口归已有
`senior_control/src/vehicle_manager/vehicle_logs/`；原路径保留显式`//Compatible`。
实际19逻辑文件：产品12（其中1份仅更新既有manager包说明）、测试1迁移、配置2、工具文档4。
产品为6份canonical文件、5份旧兼容入口和既有manager init；配置为manager pyproject与根uv.lock；
工具文档为原安装检查、本页、tests README和实施计划。没有新增顶层项目或迁诊断、数据库、agent/ROS/云实现。
Git中业务迁移显示canonical新增和旧路径改为转发；测试是原文件迁移，不重复计增删。

### 生产调用与公开边界

原agent composition及logs_wire/logging继续调用
`senior_control.vehicle_logs.service.VehicleLogService`，旧签名不变；它是canonical服务的薄构造子类，
**两个Service类不是同一对象**，enqueue/start/query/follow/download/close等业务方法直接继承同一实现。
LogAuthorizer、LogSubscription、Store、值/常量/辅助函数及wire DTO均显式转发同一对象；
服务和Store的包公开入口按需加载，纯models导入不拉起业务服务。

canonical Service要求显式`LogDiagnostics`：只声明counter/lock/outbox工厂与bind_async操作，
`QueueCounter`和`SubscriptionQueue`只描述原实现实际使用的方法，不包含诊断状态或算法。
旧构造入口注入原`PendingCounter`、`DiagnosticLock`、`DiagnosticAsyncQueue`以及在start时调用的
原`process_diagnostics().bind_async()`；默认构造仍绑定原进程诊断owner，不新造诊断对象或全局provider。
writer容量1024、subscription outbox容量4、丢弃/overflow观测与日志公共模块标识均保留。

Store继续拥有原独立SQLite日志文件、MEMORY journal、PRAGMA、连接和写锁；
授权/撤销、筛选/分页/游标、脱敏、磁盘上界、故障隔离、关闭和取消行为未变。
日志只读原SettingsSnapshot，不接管DEBUG恢复或控制状态。
模型对共享LogQuery DTO的本地过滤扩展、LogRequest/LOG_WIRE_BYTES别名不变，不扩WS contracts。
依赖为`senior-control → vehicle-manager → senior-contracts / Pydantic / aiosqlite`；
manager无Root、cloud或ROS实现导入，原技术设施仅从根兼容装配显式传入。
根锁仍69包且版本集合逐项不变；CI原M1a已发现manager源码/用例并构建wheel，本轮不改CI或云镜像。

### 命令、版本、结果与限制

macOS26.4 arm64；CPython3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3、uv0.12.23；
Pydantic2.14.1、aiosqlite0.22.1。以下python/mypy/ruff使用原主仓库`.venv/bin/`，uv缓存为
`/tmp/vehicle-uv-cache`。均为既有SQLite/内存ASGI/私有IPC检查，没有监听或启动服务。

- 迁移前原日志用例改canonical导入，collect报ModuleNotFoundError；实现后执行原用例。
- 最终完整定向：
  `python -m pytest senior_control/tests/test_vehicle_logs.py tests/integration/test_settings_logs_wire.py tests/integration/test_settings_logs_review.py::test_settings_refused_enqueue_settles_owner_without_leaking_eight_slots tests/integration/test_settings_logs_review.py::test_settings_discard_settles_only_the_original_connection tests/unit/test_runtime_diagnostics.py -q`：
  **72 passed，3.76s**。覆盖原38项日志授权/筛选/重开/损坏/故障/溢出/控制隔离、真实wire及原诊断界限/故障隔离，不新增场景或改变断言。
- 先前本机慢速执行：日志单文件36 passed/2 failed（132.18s，两个原5秒flush时限）；
  相同两项在`git archive ec7c031`独立目录复现2 failed（32.72s）。
  其后选择集67 passed/3 failed/2 deselected（265.38s），三项为settings wire、logs wire和DEBUG恢复的原短时限；
  基线三项2 failed/1 passed，重现settings wire与DEBUG恢复，logs wire当次通过。
  当前两个wire重跑1 passed/1 failed，日志单跑仍于1秒接收时限失败；基线日志再次单跑通过（1.53s）。
  最终上述**全部72项原时限通过**；保留这些执行差异，不把早期失败全部宣称为基线失败，也未改超时/保护。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：12份源码通过。
  全Root/manager/cloud及原ROS依赖路径检查38项错误；同命令在ec7c031归档为38项，错误诊断逐条相同，
  当前检查204份、基线198份源码；不称全仓类型检查通过。
- ruff check与format --check：19份相关Python通过；models/store/wire业务AST一致，
  service仅已列明的显式诊断注入差异，原38项测试仅改导入及排序，69包版本集合一致。
  原Database/runtime诊断/settings/agent/ROS/cloud/contracts/deploy无diff；git diff --check通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --check --offline`：69包通过。
  `uv build senior_control --offline --out-dir /tmp/m1b-wheels`及根`uv build --offline --out-dir /tmp/m1b-wheels`：
  manager/根sdist和wheel成功。
- 仓库外`/tmp/m1b-manager-venv`安装本轮manager及原contracts wheel、原约束Pydantic2.14.1/typing-inspection0.4.4：
  `uv pip install --offline --python /tmp/m1b-manager-venv/bin/python --constraint /tmp/m1a-constraints.txt /tmp/m1b-wheels/vehicle_manager-0.1.0-py3-none-any.whl /tmp/c1-wheels/senior_contracts-0.1.0-py3-none-any.whl`，
  8包安装及`uv pip check`通过；从`/tmp`导入全部日志模块、公开服务/Store及值，确认Root/cloud/ROS未安装、
  无反向实现导入、纯值不加载service/store。没有把空环境的pip check当安装通过。
- 原完整SDK环境`/tmp/vehicle-batch-one-venv`离线重装本轮Root和manager wheel：24包及pip check通过；
  从`/tmp`运行`python <checkout>/tools/verify_batch_one_install.py`通过，核对旧新值/Store身份、
  旧Service非同类但继承相同方法、真实诊断类及进程owner、原设置导入隔离和其余既有安装保护。

本轮没有全仓测试、Python3.10安装重验、真实Linux/ROS/ament/DDS或实车/仿真运行；
原安装工具仍明确native/SLAM实际ROS安装UNVERIFIED，历史ROS编译、Docker镜像及退役控制夹具缺口未消除。
数据库/任务/Home、agent连接/分发、监管/CLI及诊断最终归属留后片；现有地图/导航保留，导航算法后置。
没有启动开发/实车服务、部署或推送；提交后等待负责人亲审。

## M2a SQLite引擎与core schema

基线`d06f18c`；实际13逻辑文件：产品7、测试2、配置0、工具文档4，
即新增4、修改8、测试迁移1、删除0。新增为canonical storage四文件，旧database/migrations/core_schema三文件修改；
原test_database_ownership迁入manager/tests，跨车云test_storage保留原位置，只接canonical车辆fixture；
工具文档为原安装检查、本页、tests README及实施计划。没有修改pyproject、锁、CI或添加依赖。

### 单一连接实现与原装配

`vehicle_manager.storage.Database(path, busy_timeout_ms=1000, *, lock_factory=...)`拥有连接引擎；
组合者必须显式提供写锁构造端口，原数值校验后才创建`storage.write_wait`锁。
旧`senior_control.storage.database.Database`是标明`//Compatible`的薄构造子类，原签名不变，
注入原DiagnosticLock，仍拥有原Home协调字典/方法及HomeExecutionCoordination。
**两个Database类不是同一对象**，`__aenter__/_open/__aexit__`与is_open直接继承唯一实现；
泛型self返回注解保留旧子类进入上下文后仍是原实例的类型契约。
Home的目标锁、执行锁、连接代次、cancel owner和原按robot共享的状态没有迁移或复制。

旧Migration、migrate、_backup、CORE_MIGRATIONS、migrate_core是canonical同一对象的显式转发。
原`storage/schema.py`仍组合地图v1、航点v2、任务v3，不进入manager安装闭包；
`storage/map_persistence.py`仍在原边界延迟准备schema及注入同事务Home效果，不提前执行或迁入地图实现。
core_schema原v1–v4是既有SQL与迁移历史，未引入任务/Home/地图算法或新schema。

生产仍走原agent.run_agent→旧Database→TaskStore/WaypointStore/AgentJournal/SettingsStore，
设置与地图使用同一个原实例的connection/write_lock；不会为了manager再开数据库或另建锁。
原map_persistence回调、domain/core顺序、初始停机再等DB、Home导入同事务且不另加锁/commit均保持。
原独立cloud仍使用云自己的设施；旧cloud bootstrap继续经根兼容入口。
manager依赖仍仅contracts/Pydantic/aiosqlite，无Root、ROS、senior_mapping或任务执行实现导入。

原路径、busy_timeout 1–60000ms、foreign_keys/WAL/FULL PRAGMA、重入拒绝、关闭/重开和初始化取消保持；
迁移仍同写锁、连续版本、SQLite `.backup`/128页/有界progress与shield等待、BEGIN IMMEDIATE及BaseException回滚。
user_version与core_schema_version仍各自独立，SQL内容/顺序未变，domain第一份statements仍与地图原tuple同一身份。

### 命令、版本与实测

macOS26.4 arm64；CPython3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3、uv0.12.23，
Pydantic2.14.1/aiosqlite0.22.1；以下python/mypy/ruff使用原主仓库`.venv/bin/`。

- RED：先改原test_storage车辆fixture至canonical入口，`python -m pytest tests/integration/test_storage.py --collect-only -q`
  因缺vehicle_manager.storage报ModuleNotFoundError，随后迁移实现。
- `python -m pytest tests/integration/test_storage.py senior_control/tests/test_database_ownership.py -q`：**22 passed，0.72s**，
  包含原车/云SQLite失败回滚、未知版本、真实备份、busy/WAL锁、初始化取消、disk full和两项旧连接所有权用例。
  车辆fixture仅显式注入原DiagnosticLock，原用例输入/断言/时限不变。
- `python -m pytest tests/integration/test_storage.py senior_control/tests/test_database_ownership.py senior_control/tests/test_vehicle_settings.py ros2/tests/test_map_operations.py -q`：
  **139 passed，3.59s**（含前述22项，不相加）；实际运行原SQLite设置事务与地图事务/取消/恢复、
  同ref新代次/旧Home位姿失效和同事务Home保护，地图后端为原合成夹具，不代表真实SLAM运行。
- `python -m pytest tests/unit/test_home.py::test_shared_home_guard_serializes_claim_and_ignores_old_terminal tests/unit/test_home.py::test_old_home_guard_owner_cannot_update_or_delete_new_claim tests/unit/test_home.py::test_legacy_home_guard_migration_preserves_safety_and_unknowns -q`：
  **6 passed，0.29s**，确认原共享协调、旧token隔离及历史guard迁移未被引擎拆分改变。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：16份源码通过；
  原全Root/manager/cloud及ROS依赖路径命令仍38项错误、检查208份源码，与M1b的d06f18c源码记录中38项诊断逐条相同，
  不称全仓类型通过。本轮没有重跑与该片无关的历史退役控制夹具。
- 22份相关Python的ruff check/format --check和git diff --check通过；
  连接开关方法业务AST、Home类/方法、原迁移器/core SQL及全部原测试体核对一致，差异仅锁注入、self类型与必要导入/fixture。
  原schema/map_persistence、设置、任务、Home、诊断、agent、ROS、云、contracts、依赖配置和锁无diff。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m2a-wheels`及根同out-dir构建：
  manager/根sdist和wheel成功；从仓库外安装实际wheel并运行现有安装工具，没有启动服务。
- 新`/tmp/m2a-manager-venv`：`uv pip install --offline --python /tmp/m2a-manager-venv/bin/python --constraint /tmp/m1a-constraints.txt /tmp/m2a-wheels/vehicle_manager-0.1.0-py3-none-any.whl /tmp/c1-wheels/senior_contracts-0.1.0-py3-none-any.whl`：
  8包及`uv pip check`通过；从`/tmp`执行`python <checkout>/tools/verify_batch_one_install.py --manager-storage-only`通过，
  确认Root/cloud/ROS/mapping未安装，实际SQLite升级、非法SQL回滚、升级前备份、core迁移、退出和重开成功；
  独立装配显式使用asyncio.Lock，不伪装成生产诊断实现。
- 旧完整SDK环境`/tmp/vehicle-batch-one-venv`离线重装本轮Root/manager wheel后24包pip check通过，
  从`/tmp`执行同安装工具默认模式通过：旧Database非同类但继承同一连接方法、Migration/core对象同一身份、
  实际原DiagnosticLock及进程owner、Home锁/协调对象共享、原地图SQL身份、其他既有安装保护均保留。

本轮所选回归无失败；历史全链类型错误和native/SLAM真实ROS安装缺口仍登记。
未运行全仓、Python3.10重验、真实Linux/ROS/ament/DDS、SLAM编译或车辆/仿真服务；
安装工具仍将native/SLAM明确标UNVERIFIED，宿主SQLite/合成地图不能替代这些检查。
domain装配、map_persistence、任务/Home owner及新功能不在本片迁移；后续继续先精核依赖与预报，导航算法最后。
没有启动开发或实车服务、部署或推送；提交后交负责人亲审。

## M2b journal schema数据

基线`3ffeaf7`；只将`src/senior_control/agent/journal.py`的原
`AGENT_JOURNAL_STATEMENTS`声明迁入`vehicle_manager/storage/agent_schema.py`。
旧导入显式`//Compatible`转发同一tuple；AgentJournal、JournalEntry、事务/恢复/去重和执行owner仍在原模块，
LifecycleStore仍经原入口得到同一对象。本轮不迁MapRef/Pose2D/OperationResult/DomainError，也不修改调用它们的代码。
实际6文件：产品2、验证1（原安装工具）、文档3；新增1、修改5、删除/测试迁移0，配置和依赖0。

两份原SQL声明源码逐字符相同，运行值UTF-8字节及顺序相同：

| 语句 | 字节数 | SHA256 |
|---|---:|---|
| agent_operation | 241 | `825c40edab222711ed10683ed4b9cde8256cdec8f8a7a97ec52db30cabf79fe1` |
| agent_deleted_task | 199 | `6e2f79cdb3732ae8145c664e542ae9f94b0eb29ff6ddae83558aef3786e4a2b1` |

新模块没有任何import，manager安装不加载agent执行实现；没有改SQL版本、表格式、路径、备份、锁或事务顺序。
业务AST除移除原tuple声明和增加显式导入外相同；原任务/地图/Home/设置/ROS/云/contracts及构建配置无diff。

### 本轮命令与结果

环境沿用macOS26.4 arm64、CPython3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3、uv0.12.23。
python/mypy/ruff使用原主仓库`.venv/bin/`；审计脚本也显式使用该Python（系统python3较旧，首次zip(strict=...)审计未执行完，改用3.12后完成）。

- `python -m pytest tests/integration/test_task_lifecycle_store.py tests/integration/test_agent.py::test_edit_ack_only_after_device_commit tests/integration/test_agent.py::test_failed_commit_is_never_reported_as_success tests/integration/test_agent.py::test_operation_id_deduplicates tests/integration/test_agent.py::test_same_operation_id_with_different_content_conflicts tests/integration/test_agent.py::test_task_delete_rolls_back_if_the_tombstone_fails -q`：
  **30 passed，1.37s**；复用原SQLite接受/receipt/rollback/并发/CAS/取消/重启及journal原去重/提交确认/删除回滚场景，不新增测试或改夹具。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：17份源码通过；
  三份变化Python的ruff check/format --check及git diff --check通过；基线源码声明、SQL UTF-8/hash和journal执行AST核对通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m2b-wheels`及根同out-dir构建：sdist/wheel成功。
  新`/tmp/m2b-manager-venv`离线安装本轮manager和原contracts wheel，原约束Pydantic2.14.1/typing-inspection0.4.4，8包pip check通过；
  从`/tmp`执行`python <checkout>/tools/verify_batch_one_install.py --manager-storage-only`通过，导入原SQL数据且无Root/cloud/ROS，保留M2a真实SQLite安装验证。
  `/tmp/vehicle-batch-one-venv`离线重装本轮Root/manager wheel后24包pip check通过，同工具默认模式通过旧/新journal tuple身份及全部原保护。
- 本轮没有重跑全仓/全链类型检查或实际Linux/ROS/ament/SLAM、Python3.10和车辆运行；历史38项类型错误及native/SLAM未验证记录仍保留。
  未启动开发或实车服务、部署或推送，安装工具继续明确ROS缺口，不把纯wheel检查当ROS实测。

### 四类值的只读消费者核对

以下为当前实现直接导入及字段/构造/异常分支的静态源码核对，非运行时ROS追踪。
`agent/`、`tasks/`、`waypoints/`、`storage/`均指原`src/senior_control/`下路径；
ROS项按`ros2/src/<包>/<Python包>/`下相对模块列出，world_reference为senior_sim/scripts。
canonical manager和cloud当前都没有直接消费这四种Root值，不能因此把跨执行域值默认归管理实现。

| 值 | 真实管理/本地边界消费者 | ROS功能实现消费者 |
|---|---|---|
| MapRef | agent/{direct_navigation,dispatcher,ports,snapshots,workspace}；storage/map_persistence；tasks/{models,store,executor}；waypoints/{store,home}；原navigation | senior_localization/{localization_node,navigation_reference,relocalization}；senior_mapping/{archive,evidence,occupancy,operations,persistence,references,store,workspace}；senior_robot/navigation_adapter；senior_sim/scripts/world_reference |
| Pose2D | agent/{direct_navigation,dispatcher,ports,workspace}；storage/map_persistence；tasks/{executor,lifecycle_store,lifecycle_service,navigation_port}；waypoints/{store,home}；原navigation | senior_localization/{hardware_reference,navigation_reference,relocalization}；senior_mapping/{archive,evidence,occupancy,persistence,references,slam_adapter,store,workspace,workspace_export}；senior_motion_estimation/{estimator,motion_observation_adapter,motion_sensor_runtime,observations,optional_motion_inputs,pose_estimator,sample_types,temporal}；senior_perception/{learning_compute,scan_adapter,scan_decode,self_filter}；senior_robot/navigation_adapter；senior_sim/scripts/world_reference |
| OperationResult | agent/{dispatcher,main}操作确认/查询；storage/database的Home cancel回调；waypoints/home；当前tasks各文件没有直接消费 | senior_localization/relocalization；senior_mapping/{operations,slam_adapter}；senior_robot/navigation_adapter |
| DomainError | agent/{composition,connection,direct_navigation,dispatcher,main,ports,settings_wire,workspace}；tasks/{store,executor,runtime}；waypoints/{store,home}；原navigation | senior_localization/{geo,relocalization}；senior_mapping/{archive,operations,references,slam_adapter,store,workspace_export} |

另有仍在Root的motion/{constraints,kinematics}、safety/{space_estimator,startup_visibility}、vision/rgbd_body_motion消费Pose2D；
不是把这些执行模块打入manager的理由。MapRef在TaskDefinition/ExecutionContext中保存/比较，TaskStore._task唯一直接重建MapRef；
任务路径的Pose2D只作类型与x/y/yaw读取/序列化，未找到Pose2D构造调用。
DomainError则真实参与构造、except以及runtime的exc.code分支，不能仅改为TYPE_CHECKING或协议就认为解耦。

### 下一片建议及尚未决定的边界

优先只迁**schedule与lifecycle纯核心**：原tasks/{schedule,lifecycle}.py→已有vehicle_manager/tasks/同名模块，
新增有界公开tasks/__init__；旧两个模块显式同一对象转发，旧tasks/__init__按原名字按需加载以免导入纯核心拉起executor。
两个源码模块只依赖标准库，既不需要上述四类值，也不需要ROS、地图、schema或导航owner。
候选12逻辑文件：产品6（新增3、旧修改3）、测试2、配置0、工具文档4；新增3/修改8/测试迁移1/删除0。
原test_task_lifecycle整份随实现迁入manager/tests；原test_schedule混有TaskRunner/models/MapRef，
因此留原位置，只改Schedule导入，不把混合用例说成无Root的独立测试。
这取代旧只读盘点中先搬local_values/errors的候选，尚待负责人审核预报，不在本次实施。

下一片候选复用：test_task_lifecycle原重复命令/离线完成/接管与迟到ack/重启/终端结算，
输入重复request、旧task epoch、未确认revoke和新参考→保持原receipt、未知只查询、不复活旧权限；
test_schedule原interval/cron/DST/墙钟跳变与扫描窗口，输入UTC/时区重复或缺失墙钟段→原发生次数和skip决策；
test_task_lifecycle_service原已提交后dispatch、锁外IO与迟到ack，输入阻塞或未知导航→仍先持久化、不重发；
安装复用原工具核对旧新Schedule/core值及函数同一对象、无Root独立导入，不新造运动或业务场景。

后续models/runner可在消费侧定义只读MapRef形状，沿用原实例和相等性，不复制值或把ROS模块导入manager；
TaskStore读取重建需显式注入原MapRef构造入口，不能换个新dataclass导致跨边界相等性/身份改变。
Pose2D可经现有TaskNavigation/NavigationEffects本地端口传递原对象并只读字段，避免为任务搬动执行几何值。
OperationResult不构成当前任务核心的迁移前置条件。
Store/Executor/runtime的DomainError需要真实的错误构造/捕获/码读取端口或另审更小归属；
原RouteRejected和参考准入也仍属待精核边界，不引入全局错误工厂，不在本轮替换错误语义。
MapPersistence/MapDatabase虽已有公开接口，但位于ROS功能包，不能让manager为取类型反向依赖它的发行包。
完整任务持久/驱动片待上述边界收敛再重估，不沿用原23–29范围直接实施；原导航/IPC/地图/Home链保持，导航算法最后。
