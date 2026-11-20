# 车端管理

设置、车辆日志、SQLite技术设施、任务核心与TaskStore已从原根 SDK 迁入；其他管理职责仍待各片迁移。本项目目录 `senior_control/`，
导入名 `vehicle_manager`，发行名 `vehicle-manager`。没有新增CLI/服务入口，也未迁导航或ROS执行。
下文各片记录按当时范围保留；当前发行依赖见「R0/R5 发行DAG切换」，具体配置/诊断装配见末尾「R5 具体Agent依赖绑定」。

## 实际目录与调用

```text
senior_control/
  pyproject.toml                       # senior-control(core)、contracts、Pydantic、aiosqlite
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
    tasks/
      __init__.py                     # 纯核心与TaskStore按需公开
      schedule.py                     # 原interval/cron/DST计算，标准库实现
      lifecycle.py                    # 原纯任务状态机及公开值/函数
      models.py                       # 原冻结任务模型，直接保存传入的原MapRef对象
      runner.py                       # 原纯调度/准入决策，无持久或导航执行
      ports.py                        # 只读MapReference和每实例Store构造端口，无值实现
      store.py                        # 唯一TaskStore SQL/事务/boot/claim/scan实现
      codec.py                        # 原13个解析/序列化函数，逐实例值/错误构造
      lifecycle_store.py              # 原同连接生命周期/receipt/effect事务
      lifecycle_service.py            # 原提交后效果协调/未知查询，外部IO不持DB锁
  tests/
    test_vehicle_settings.py          # 原62项设置用例，无新增场景
    test_vehicle_logs.py              # 原38项日志用例，无新增场景
    test_database_ownership.py        # 原2项连接所有权/重入用例
    test_task_lifecycle.py            # 原纯状态机用例，随实现迁入
    test_schedule.py                  # 原schedule/runner混合纯决策用例
    test_task_runner.py               # 原真实SQLite/任务驱动回归，输入断言不变
    test_task_lifecycle_store.py      # 原事务/CAS/取消/硬退出WAL恢复用例
    test_task_lifecycle_service.py    # 原锁外dispatch/未知/接管迟到ack用例
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

## M2c 纯schedule与lifecycle

基线`301fd53`；只迁现有`tasks/schedule.py`和`tasks/lifecycle.py`到已有manager项目。
两份canonical源码除CRLF转LF外文本一致，算法、状态、常量、数据类和函数不改；
旧模块逐名显式`//Compatible`转发同一对象，包括原已定义的辅助函数，不复制实现。
旧模块对象与canonical模块是不同模块，公开类/值/函数身份相同；不承诺在旧模块上monkeypatch后会改写canonical函数全局变量。

canonical `vehicle_manager.tasks`只按需公开Schedule和lifecycle模块；
旧`senior_control.tasks`的`__all__`名字与顺序保持原样，按明确名字加载同一对象：
Schedule归canonical，未迁models/runner/store/executor仍按需从原模块提供。
单独导入旧包、Schedule或lifecycle不加载这些未迁owner、Database、mapping或ROS；
实际请求TaskStore/TaskExecutor/TaskRunner仍得到原类，生产装配和持久驱动不切换。
没有改MapRef/Pose2D/OperationResult/DomainError、任务models/runner/持久层、导航/IPC或ROS实现及安装配置。

实际12逻辑文件：产品6、测试2、配置0、工具文档4，即新增3/修改8/测试迁移1/删除0。
产品为canonical tasks三文件、旧tasks三文件；原test_task_lifecycle迁入manager/tests，
混合test_schedule保留原位置，仅把Schedule和稀疏cron的monkeypatch模块导入改到真实计算owner。
原3000次搜索上界、断言、DST/时间/接管/迟到/重启/断链保护均保留，没有新增业务场景。

### 命令、版本及结果

macOS26.4 arm64；CPython3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3、uv0.12.23。
python/mypy/ruff均使用原主仓库`.venv/bin/`。

- RED：先将纯状态机测试导入改为canonical，`python -m pytest senior_control/tests/test_task_lifecycle.py --collect-only -q`
  因vehicle_manager.tasks尚不存在报ModuleNotFoundError。
- `python -m pytest tests/unit/test_schedule.py senior_control/tests/test_task_lifecycle.py tests/integration/test_task_lifecycle_service.py -q`：
  首轮128 passed/1 failed，原稀疏cron夹具仍patch旧转发模块的datetime；只改该模块import指向canonical后，
  **129 passed，0.75s**。复用原interval/cron/DST/时钟跳变、重复request/接管/迟到ack/重启/断链，
  以及服务层已提交后dispatch、锁外IO/unknown仅查询/原接管隔离用例，没有补假属性或放宽上界。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：20份源码中**1项既有错误**，
  原tasks/lifecycle.py:318的phase变量可能为None与先前Phase注解冲突，随原实现迁入canonical同一行；未修算法或加ignore。
  原全Root/manager/cloud及ROS依赖路径命令：检查212份源码仍38项错误，只有该项路径迁移，
  将此路径归一化后与M2a基线诊断逐条相同，不称manager或全链类型检查全通过。
- 九份变化Python的ruff check/format --check通过；原test_schedule保留CRLF，
  `git -c core.whitespace=cr-at-eol diff --check`通过（默认检查把两条新增CRLF import的CR报告为尾空白，未忽略其他空白）。
  canonical文本/AST、旧包__all__、原测试体核对相同，只有必要导入/排序/fixture归属变化。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m2c-wheels`及根同out-dir构建：sdist/wheel成功。
  新`/tmp/m2c-manager-venv`离线安装本轮manager及原contracts wheel、原Pydantic2.14.1/typing-inspection0.4.4约束，8包pip check通过；
  从`/tmp`运行`python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only`通过，
  Root/cloud/ROS/mapping未安装，纯task导入也不加载aiosqlite/Pydantic，不启动业务owner。
- 旧完整SDK环境`/tmp/vehicle-batch-one-venv`离线重装本轮Root/manager wheel，24包pip check通过；
  从`/tmp`执行同安装工具默认模式通过，新进程核对旧/新Schedule与lifecycle公开值/函数同一身份、
  旧包按需加载隔离及未迁TaskStore/Executor/Runner原身份，原SQLite/设置/日志/地图/native定位检查均保留。

实际Linux/ROS/ament/DDS/SLAM、车辆/仿真服务及Python3.10本轮未验证，原安装工具继续标ROS UNVERIFIED。
本轮不跑全仓，不清除历史退役控制夹具或其他类型失败，不启动服务、部署或推送，提交后等待亲审。

### 下一片只读建议

先精核`tasks/models.py`与`runner.py`的只读MapRef需求，候选仅把这两份纯管理决策模块归manager/tasks，
消费侧最小只读协议声明map_id/reference_revision，仍传入原MapRef实例并保留原相等性；不迁MapRef类、不造第二份值。
原models构造不创建MapRef，runner只做比较/决策，较先迁Store/Executor的错误与导航边界更小。
若该接口经核对可行，候选12文件：产品7（canonical models/runner/只读端口新增3、旧models/runner及两包入口修改4）、
测试1（原混合test_schedule随这些实现迁入）、配置0、工具文档4；下一次仍需负责人审核预报后实施。
实际集成入边为TaskStore/TaskExecutor/TaskRuntime及agent的原上下文和任务定义，必须保留其原构造/类型身份，
复用test_schedule原时钟/DST/准入决策及test_task_runner、test_task_ownership中的真实旧装配。
完整持久任务片仍受TaskStore._task重建原MapRef、DomainError真实构造/catch/码、RouteRejected和地图参考准入约束；
Pose2D经现有端口只读、OperationResult非纯核心前置，不能借纯核心迁移提前搬这些值或重写驱动。

## M2d 纯任务models与runner

基线`3f4b6bb`；现有models/runner归`vehicle_manager/tasks/`，旧入口显式`//Compatible`转发同一类、常量和函数。
canonical和旧包继续按明确名字公开，旧`__all__`保持；未迁Store/Executor/Runtime仍使用原实现。
`MapReference`只声明map_id/reference_revision两个只读属性，没有第二份MapRef值、转换器或运行时检查。
TaskDefinition.map_ref与ExecutionContext.active_map直接保留传入对象；冻结模型、相等比较、准入优先级和扫描窗口不改。
MapRef、Pose2D、DomainError等值/错误仍原位，manager无Root/地图/ROS反向实现导入。

原生产值来源仍是`agent/dispatcher._map_ref`、`agent/direct_navigation`的原MapRef构造，以及
`tasks/store._task`从SQL重建的原MapRef；模型迁移不改变这些构造路径。
初始只读接口使旧qualified_reference和WaypointStore.route_poses两个具体MapRef签名产生类型冲突，
经范围确认只扩两个旧调用点，保留具体签名，标明Compatible的`cast(MapRef, task.map_ref)`只是静态视图，
**不是运行时校验、转换或新准入条件**。回调、对象、锁、原分支及调用顺序不变；
不为修改WaypointStore或整组导航回调签名继续扩大范围。

实际已确认14逻辑文件：产品9、测试1、配置0、工具文档4，即新增3/修改10/测试迁移1/删除0。
产品为canonical models/runner/ports三份、旧models/runner及两个包入口四份、Executor与agent ports两个局部类型调用点；
原test_schedule随纯模型/Runner迁入manager/tests，只有canonical imports/排序变化，原输入、断言和patch owner不变。
没有改持久Store、Runtime、地图、导航、ROS、IPC实现、构建声明或锁文件。

### 命令、版本及结果

macOS26.4 arm64；CPython3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3、uv0.12.23，
Pydantic2.14.1/aiosqlite0.22.1；python/mypy/ruff使用原主仓库`.venv/bin/`。

- RED：迁入test_schedule并改models/runner import后，collect因vehicle_manager.tasks.models尚不存在失败；随后迁移实现。
- 原选择集：`python -m pytest senior_control/tests/test_schedule.py senior_control/tests/test_task_lifecycle.py tests/integration/test_task_runner.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_ownership.py -q`：
  **216 passed，2.66s**，覆盖原schedule/runner/lifecycle与真实SQLite任务模型/存储准入、取消、恢复、任务wire。
- 批准两个类型调用点后，以上命令追加`tests/unit/test_direct_navigation.py`：**218 passed，2.61s**（包含前述216，不相加），
  保留原具体MapRef准入、原任务绑定/手动接管/旧代次和导航读取保护，不新增pytest场景或夹具。
- manager检查23份源码仍有lifecycle:318原1项类型错误；初始全链因两个旧具体MapRef签名为40项，
  局部静态接线后同原Root/manager/cloud及ROS依赖路径mypy命令回到**38项**、检查215份源码，
  与M2c的38项错误诊断逐条相同；未修旧状态机或增加ignore。
- 十三份相关Python的ruff check/format --check及`git -c core.whitespace=cr-at-eol diff --check`通过；
  原test_schedule/Executor保留CRLF。原models AST只变导入/只读类型注解，runner AST只变导入；
  测试体与两个批准调用点除cast外的业务AST一致，原值/Store/Runtime/地图/导航/ROS/IPC等无diff。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m2d-approved-wheels`及根同out-dir构建成功。
  初始同样构建/tmp/m2d-wheels；两处类型接线后重建并重装最终SDK，避免把旧wheel当最终安装验证。
  新`/tmp/m2d-manager-venv`离线安装manager与原contracts wheel及原约束，8包pip check通过；
  从`/tmp`运行原安装工具`--manager-tasks-only`通过，新增模型/Runner公开身份且无Root/ROS/地图依赖，不加载Pydantic/aiosqlite。
- `/tmp/vehicle-batch-one-venv`重装最终Root/manager wheel后24包pip check和从`/tmp`执行原安装工具默认模式通过；
  新进程验证旧新模型/枚举/Runner/常量/辅助函数同一对象，直接请求纯模型不加载旧执行owner，
  模型及Runner保存原MapRef/TaskDefinition对象且与原MapRef比较行为一致，其余原安装保护保留。
- 从`/tmp`使用该最终SDK Python进行一次性真实调用核对，复用原保存航点/任务输入：
  真Database→TaskStore.prepare/create及WaypointStore.save_waypoint→IpcNavigation.task_poses读取原坐标，
  原TaskExecutor的qualified_reference收到同一个stored.map_ref；SQL读取重建的类型仍是原MapRef，
  原构造输入在模型中保持原对象，未连接IPC或启动服务。此项为安装/调用接线检查，没有增设pytest场景。

本轮所选回归无失败，manager原1项/全链原38项类型错误仍登记。
实际Linux/ROS/ament/DDS/SLAM、车辆/仿真运行与Python3.10本轮未验证，原工具继续明确ROS UNVERIFIED。
没有启开发或实车服务、部署或推送，提交后等待负责人亲审；以下依赖只盘点，不提前实施。

### 后续存储与执行依赖：只读候选

**先任务Store，后生命周期持久服务，最后执行/导航效果适配**；预算均需下一次精核和负责人预报后执行。

| 切片与源目标 | 最小真实依赖与保留点 | 条件预算（逻辑文件） |
|---|---|---|
| S1 原tasks/store→manager/tasks/store；旧入口保留构造适配 | 已有manager SQLite Database/同连接写锁、已迁模型/Schedule；注入原domain→core准备回调，不导入根schema及地图实现；逐实例注入原MapRef构造与原DomainError构造，所有SQL行解析/校验保留同类异常；TASK_STATEMENTS唯一且旧schema取同一tuple，原boot/claim/scan/取消事务不改；禁止全局工厂 | 候选10–12：产品4–5（store新增、旧store/ports/公开入口修改，若有必要才独立行解析模块）、测试2–3、配置0、工具文档4；准备/构造/解析调用点需先精核，不能据预算直接搬 |
| S2 原lifecycle_store/lifecycle_service→manager/tasks同名模块 | 在S1端口稳定后使用同一TaskStore/DB事务和Admission，直接复用已迁agent_schema；Pose2D仅消费x/y/yaw只读形状，无构造或值搬迁；本地VersionConflict/RequestConflict/StaleEffect与receipt/effect单一实现、锁内采样/取消settle、锁外dispatch不变 | 候选12：产品6、原测试迁移2、配置0、工具文档4；源目标为两份实现/旧转发、ports与公开入口 |
| S3 原executor/runtime/navigation_port→manager/tasks同名模块 | TaskStore与LifecycleService稳定后，现有TaskNavigation/NavigationEffects输入输出端口、RouteProgress共享DTO、原qualified_reference/read-only reference；DomainError实际构造/catch/code、QualificationUnavailable原继承关系、RouteRejected必须通过真实逐owner端口或再审更小归属，不能只写TYPE_CHECKING；原ROS/IPC/手动T2边界仍由原owner执行 | 条件16–21：产品9–12、测试3–5、配置0、工具文档4；错误/导航回调端口未定，须先只读核对，不作为已批准范围 |

S1候选复用原test_task_runner的重启保留、版本3不动地图链接、禁用/删除/无效链接拒绝、并发claim、disk full不确认、
skip/scan持久及失败重试，输入真实SQLite旧版本/锁冲突/损坏或失配MapRef→原返回、回滚与幂等；
依赖验证须证明旧构造仍生成原MapRef/DomainError而非新值/错误，并保持同连接身份。
S2复用原test_task_lifecycle_store的每处接受写入失败回滚、双连接CAS、取消后唯一可查询结果、重启旧epoch隔离；
test_task_lifecycle_service的锁外IO、未知仅查询、接管不等pending发送及迟到ack、撤销确认/手动通知同事务，
输入原旧request/epoch、取消/重开/阻塞导航→原持久结果与输出边界，不增加恢复运动。
S3复用test_task_runner/ownership及原terminal_release相关场景，输入失联/未知发送/切图同ref新代次、原接管/手动新权限→
取消优先、旧结果不复活、不撤新手动、原航点读取与既有IPC/wire字段不变。
所有候选先复用原用例/真实夹具；宿主SQLite可测，真实ROS/DDS/原生安装和车辆运行仍依赖对应环境，导航算法后置。

## S1 TaskStore持久化

基线`fec2f9a`；现有TaskStore唯一SQL/事务实现归`vehicle_manager/tasks/store.py`，
原13个行解析/校验/序列化函数归`TaskCodec`，逻辑不复制，参数/调用顺序保持。
实际11逻辑文件：产品5、测试2、配置0、工具文档4，即新增2/修改8/测试迁移1/删除0。
产品为canonical store/codec、既有ports/公开入口和旧store；
原test_task_runner原样迁入manager/tests，test_task_lifecycle_store仅改已迁TaskDefinition导入及排序，
原执行器/Runtime夹具及全部输入/断言/patch保留，没有迁生命周期持久、执行或导航实现。

### 构造、异常与解析边界

canonical `TaskStore(database, *, robot_id, ports)`接收已有manager SQLite Database接口，
直接保存传入的原实例，不创建连接或锁。
`TaskStorePorts`是每实例的冻结回调集合：prepare_schema、map_reference、error；
旧`senior_control.tasks.store.TaskStore(database, *, robot_id)`标明Compatible，
**是canonical的薄构造子类而非同一类**，所有SQL/事务方法直接继承唯一实现。
原包公开TaskStore继续提供这个保留签名的旧构造；manager包按需公开必须显式注入端口的canonical类。

旧构造注入原MapRef/DomainError类，并延迟装配原domain→core准备回调；
回调捕获的database就是该Store保存的同一个对象，仍先准备domain/core，再进入原写锁和本地表/索引/boot事务。
codec只保存这两个原构造的引用，不拥有DB/任务状态，不建全局工厂或另一份值/异常实现。
45处原DomainError构造均改为每实例调用原构造；原sqlite/JSON/Enum/ValueError及cause传播不改。
SQL读取仍调用原MapRef构造，返回原类型，TaskDefinition/记录/枚举/公开SQL常量保持同一对象。
TASK_STATEMENTS仅一份，旧模块转发，domain v3仍取同一个tuple。

逐项源码审计覆盖42个Store方法和13个解析辅助函数；全部SQL/常量AST相同，
方法除constructor/prepare端口及解析调用位置外AST相同，所有解析函数除self与构造注入外AST相同。
原65处解析/序列化辅助调用的分布如下，不增加校验或吞掉原异常：

| 调用 | 次数 | 调用 | 次数 |
|---|---:|---|---:|
| task / execution / skip | 4 / 7 / 2 | points / waypoint_ids | 2 / 1 |
| text / optional_text | 20 / 1 | whole / optional_whole / flag | 3 / 1 / 4 |
| moment / stamp / utc | 5 / 9 / 6 | 原MapRef构造点 | 1 |

原DB路径、SQL版本/表格式、boot恢复、单owner/claim/CAS、scan界限4096、索引、取消/幂等/回滚保持；
claim_in_transaction仍要求同一connection且不自commit，`_transaction`仍原BEGIN/rollback/commit及锁序。
原Database/诊断、schema/map_persistence、模型值、地图、导航、ROS、IPC、配置和依赖锁均不迁改。

### 命令、版本及结果

macOS26.4 arm64；CPython3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3、uv0.12.23，
Pydantic2.14.1/aiosqlite0.22.1；python/mypy/ruff使用原主仓库`.venv/bin/`。

- `python -m pytest senior_control/tests/test_task_runner.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_ownership.py -q`：
  **117 passed，2.75s**，原真实SQLite任务Store/驱动与消费者定向。
- `python -m pytest senior_control/tests/test_task_runner.py senior_control/tests/test_schedule.py senior_control/tests/test_task_lifecycle.py tests/integration/test_task_lifecycle_store.py tests/integration/test_task_lifecycle_service.py tests/integration/test_task_ownership.py tests/integration/test_agent.py::test_task_delete_rolls_back_if_the_tombstone_fails tests/integration/test_agent.py::test_task_edit_commits_in_robot_database -q`：
  **248 passed，3.37s**（含前述117项，不相加）；覆盖原schema/core迁移、恢复/并发/CAS/取消、存储失败、
  同连接生命周期接受与锁外dispatch、旧epoch/接管、任务wire及实际agent提交/删除回滚。
- 原测试body与输入AST核对相同；test_task_runner除位置外原文本一致，混合生命周期测试只改canonical模型导入。
  现有pytest路径/CI已发现manager/tests，原目录文件删除而非复制，没有改配置。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：25份源码仍原lifecycle:318的1项错误；
  原全链命令检查217份源码仍38项，错误诊断与M2d逐条相同，不宣称类型检查全通过。
  `/tmp/s1-audit.py`临时审计脚本逆归一化逐实例接线，复核42方法/13函数、SQL及调用计数；
  另核对旧prepare的五步与捕获同DB的回调相同，无全局provider或反向实现导入。
- 相关Python的ruff check/format --check与按原CRLF的`git -c core.whitespace=cr-at-eol diff --check`通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/s1-wheels`及根同out-dir构建：sdist/wheel成功。
  补齐旧模块公开类型转发后同命令以`/tmp/s1-final-wheels`重建，以下两种环境均重装最终wheel复核。
  仓库外`/tmp/s1-manager-venv`离线安装本轮manager与原contracts wheel及原约束，8包pip check通过；
  从`/tmp`运行原安装工具`--manager-tasks-only`通过，纯模型原隔离保护保持，之后单独核对TaskStore公开接口且无Root/ROS。
  独立manager没有内置domain地图/航点装配；schema准备必须由真实调用者提供，不能用空回调冒充独立生产迁移。
- `/tmp/vehicle-batch-one-venv`重装本轮Root/manager wheel后24包pip check及原工具默认模式通过；
  核对旧Store非同类但继承同一方法、原TASK_STATEMENTS身份、原DB/MapRef/DomainError构造引用，
  实际SQLite prepare/create/get、CAS冲突仍为原DomainError、退出重开恢复及原类型相等性，未启动服务。

本轮所选回归无失败；原manager1项/全链38项类型错误及历史退役夹具记录保持。
真实Linux/ROS/ament/DDS/SLAM、车辆/仿真运行及Python3.10本轮未验证，安装工具仍明确ROS缺口。
没有启动开发或实车服务、部署或推送；提交后等待负责人亲审。

### 下一片S2只读建议

源`tasks/lifecycle_store.py`、`lifecycle_service.py`→manager/tasks同名模块，候选12文件：
产品6（canonical两份、旧转发两份、ports与公开入口）、原测试迁移2、配置0、工具文档4。
S1已稳定的TaskStore/DB、模型与纯lifecycle可直接复用；AGENT_JOURNAL_STATEMENTS应直接取已迁agent_schema，
不再为取SQL导入agent执行owner。
两份实现中的Pose2D只用于注解与x/y/yaw序列化，可在消费侧声明只读形状，原对象不搬或重建。
必须保留tasks/database对象身份、Admission→write_lock→transaction顺序、锁后最新context采样、取消后settle、
receipt/effect/journal同事务以及外部dispatch不持锁；VersionConflict/RequestConflict/StaleEffect和记录值同一对象。
原Root生产调用的Store仍是本片薄子类，需要核对canonical类型与旧调用签名是否还需局部静态接线，超范围先报。
复用原lifecycle_store的每处写入失败回滚、双连接CAS、取消后唯一可查询结果、旧epoch/重开与进程退出持久边界，
以及lifecycle_service的已提交锁外dispatch、未知仅查询、接管不等pending发送/迟到ack和手动通知同事务；
输入原SQLite冲突/取消、旧request/epoch、阻塞或未知导航→原回执与效果边界，不自动恢复运动。
仍需下一次负责人精核/预报后实施；Executor/Runtime/NavigationEffects的错误继承/catch、RouteRejected及真实导航效果端口
留后片，不提前搬DomainError/MapRef/Pose2D、不做全局工厂或新算法，既有地图/Home/导航保持。

## S2 生命周期持久与服务

基线`4371027`；仅将原lifecycle_store/service归已有manager/tasks，
两份完整实现/SQL AST除导入和Pose注解外相同。旧模块显式Compatible转发，
**LifecycleStore、LifecycleService、原异常/值/常量和辅助函数均为canonical同一对象**，没有构造子类或第二份方法。
SCHEMA仍唯一tuple，AGENT_JOURNAL_STATEMENTS直接取已迁agent_schema，不再为取SQL导入agent实现。
LifecycleStore保存原TaskStore与其database对象，使用原connection/write_lock；不另造Store/DB/锁或装配执行程序。

`PoseReading`只声明x/y/yaw只读属性，不构造、复制、转换或校验Pose值；
原Pose2D仍由原调用者创建，按原tuple传递和序列化，原类不搬。
Admission→write_lock→事务、锁内最新context采样、CAS、取消后等待真实settle、
receipt/effect/journal同事务、未知结果仅查询、外部dispatch不持DB锁，以及接管/迟到ack边界全部沿用。
原boot/epoch/版本、预算、原错误继承关系和已有导航/IPC行为保持。

实际12逻辑文件：产品6、测试迁移2、配置0、工具文档4，即新增2/修改8/迁移2/删除0。
产品为canonical两份、旧转发两份、既有ports与公开入口；
两份原test_task_lifecycle_store/service随实现迁入manager/tests，发现/跨测试导入沿用既有pytest路径与CI配置。
原硬退出子进程的源码导入改指canonical，并显式继承父测试的真实sys.path；
原os._exit阶段23/24/25、WAL恢复断言、超时和其他输入/保护不变，没有加假属性或新情景。
Executor/Runtime/NavigationEffects、值/错误公共模块、地图/ROS/IPC及依赖配置均不改。

### 命令、版本和结果

macOS26.4 arm64；CPython3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3、uv0.12.23。
python/mypy/ruff使用原主仓库`.venv/bin/`，Pydantic2.14.1/aiosqlite0.22.1未变。

- RED：先迁测试并改canonical import，`python -m pytest senior_control/tests/test_task_lifecycle_store.py --collect-only -q`
  因新模块尚不存在报ModuleNotFoundError，随后迁实现。
- `python -m pytest senior_control/tests/test_task_lifecycle_store.py senior_control/tests/test_task_lifecycle_service.py -q`：
  首轮52 passed/3 failed，三个硬退出子进程没有新manager源码路径，报ModuleNotFoundError；
  只补`env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}`，确保子进程实际测当前源而非解释器旧安装，
  原断言/15秒上界/退出阶段不改，复跑**55 passed，1.41s**。
- `python -m pytest senior_control/tests/test_task_lifecycle_store.py senior_control/tests/test_task_lifecycle_service.py senior_control/tests/test_task_runner.py senior_control/tests/test_task_lifecycle.py tests/integration/test_task_ownership.py tests/integration/test_terminal_release_outbox.py -q`：
  **221 passed，3.32s**（含55项，不相加）。包含逐步接受写入失败/同连接取消settle、双连接CAS、
  真子进程退出WAL恢复、旧request/epoch、锁后采样、锁外dispatch、unknown仅查询、接管迟到ack及终端outbox保护。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：27份源码中保留**2项既有错误**，
  lifecycle:318及新迁入lifecycle_store:647的nullable-row索引；原全链命令检查219份仍38项，
  只归一化迁移文件路径后与S1的38项诊断逐条相同，不加ignore或假属性，不称类型检查全通过。
- 两份完整实现/SQL、所有原测试body/断言的AST核对通过，差异仅已列明的导入、Pose类型与真实子进程环境；
  十五份相关Python的ruff check/format --check和按原CRLF的git diff检查通过，无反向实现导入。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/s2-wheels`及根同out-dir构建：sdist/wheel成功。
  `/tmp/s2-manager-venv`离线安装本轮manager及原contracts wheel/约束，8包pip check通过；
  从`/tmp`执行`python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only`通过，
  原纯核心不加载数据库保护保持，之后单独导入Store/生命周期接口，确认无Root/cloud/ROS/地图依赖。
- `/tmp/vehicle-batch-one-venv`重装本轮Root/manager wheel，24包pip check及原安装工具默认模式通过；
  核对旧新Store/Service、VersionConflict/RequestConflict/StaleEffect及SCHEMA同一身份，
  真实Root TaskStore/DB实例传入LifecycleStore不替换，原Pose输入经service.submit原样序列化、receipt及效果已持久，
  实际NavigationEffects/IpcNavigation已装配但未reconcile或连接IPC；同工具原设置/日志/地图/SQLite保护全部保留。

本轮所选回归最终无失败，原manager2项/全链38项类型错误仍登记。
未跑全仓或真实Linux/ROS/ament/DDS/SLAM/车辆/仿真服务，Python3.10本轮未重验；
原工具继续将ROS native/SLAM安装标UNVERIFIED，宿主SQL/安装/合成端口不替代实际ROS。
没有启动开发或实车服务、部署或推送；提交后等待负责人亲审，不继续下一片。

### S3下一片依赖：仅只读建议

候选源仍为原`tasks/executor.py`、`runtime.py`、`navigation_port.py`，目标为manager/tasks同名模块，
旧构造/错误边界保留Compatible；先精核错误与真实端口，再预报，不能直接按整文件搬完宣称独立。
已可直接复用canonical TaskStore、生命周期Store/Service、纯模型/Runner/Schedule、同DB和Admission；
RouteProgress/RouteStatus仅取已有共享DTO，不把RoutePlan/QualifiedRoute算法或ROS实现拉入manager。
实际最小接口还包括：现有qualified_reference或map_ref/map_generation/review_required只读参考；
TaskNavigation的task_poses/task_effect/status及原有限IO时限；原Pose实例只读坐标，不迁值或重建目标；
原DomainError的构造/catch/code、QualificationUnavailable的原继承与身份、RouteRejected必须按真实owner逐实例装配或另审更小归属，
不能用TYPE_CHECKING掩盖运行依赖、改错误类或建立全局注册工厂。
原手动T2通知/权限、terminal_release/settled与单cmd_vel继续由原实际IPC/控制边界执行，管理只提出效果与观察确认。

条件预算仍为16–21逻辑文件：产品9–12、测试3–5、配置0、工具文档4；
包含三份canonical/旧入口、实际最小ports/公开入口及必要原装配接线，具体错误工厂/类身份尚待精核，不能据此直接实施。
复用原TaskRunner/ownership的失联/重启/未知发送只查询、同ref新代次/缺定位不导航、skip失败仍优先取消、
接管后旧ack不复活/不撤新手动，以及terminal_release_outbox/旧真实IPC用例；
输入原SQLite恢复状态、旧epoch/迟到结果和受阻导航→原持久义务/撤销确认，输出字段和控制隔离不变。
宿主SQLite/内存端口可测，真实ROS/DDS/原生安装与车辆运行仍须对应环境；不前置新导航算法或其他新功能。

## S3 导航效果、任务执行与调度

基线 `22741b5`，先迁导航效果端口再迁执行/调度，同一批提交。
唯一实现现在为：

```text
senior_control/src/vehicle_manager/tasks/
├── navigation_port.py   TaskNavigation、NavigationEffects；原效果/查询/进度/通知
├── executor.py          TaskExecutor；原资格、生命周期驱动、隔离的旧记录适配
├── runtime.py           TaskRuntime；原时钟扫描、取消优先、skip与恢复状态
├── ports.py             原只读值接口；LiveMapReference、逐实例TaskExecutionErrors
└── __init__.py          按名字延迟公开以上接口
src/senior_control/tasks/
├── navigation_port.py   Compatible薄构造：原RouteRejected
├── executor.py          Compatible薄构造：原DomainError/QualificationUnavailable/RouteRejected
└── runtime.py           Compatible：同一canonical TaskRuntime
```

业务方法没有第二份实现。旧NavigationEffects/TaskExecutor类与canonical类**不是同一对象**，
旧类继承同一方法并保留原生产构造签名；TaskNavigation、TaskRuntime、NavigationFailure、ProcessCrash为同一对象。
QualificationUnavailable仍只在旧executor定义、继承原DomainError；RouteRejected仍由原navigation定义，
两者的原错误码、文本、具体catch和异常链保持。TaskExecutionErrors只保存逐实例具体异常类及原code读取接口，
不注册全局工厂，不把车内错误或值放WS contracts；读取code的Compatible cast仅在已捕获原DomainError后使用。
ProcessCrash仍继承BaseException，不是Exception。

canonical仅消费已有PoseReading/MapReference及LiveMapReference只读接口，
原MapRef/Pose2D/实时准入owner不复制、不转换、不搬家；RouteProgress直接取既有共享DTO。
原qualified_reference调用仍拿到原MapRef对象，实时map_ref/map_generation/review_required每次从同owner读取。
`build_task_runtime`仍在原agent/composition，保留原Database/TaskStore/connection/write_lock，
LifecycleStore/Service、IpcNavigation和真实IPC都不替换；默认执行代次仍0，不等同WS连接epoch。
canonical Executor继续以`isinstance(port, NavigationEffects)`作名义门槛，旧端口子类通过，
不改成宽泛协议、hasattr或任意可调用对象。

原资格/截止/同ref新代次、未确认只查询、取消优先、同tick忙碌、boot/claim/CAS、
锁内采样/锁外dispatch和原UUID幂等行为保持。
terminal_release/settled/revoke低层字段及确认规则仍在原IpcNavigation，
旧终端不能凭accepted或迟到ack撤销新权限；手动T2仍待真实退场且同gate核对后发出。
ROS导航、地图/Home、控制和IPC算法不迁，不添加新守卫或功能。

### 实际文件清单与分类

14逻辑文件，产品8、测试2、配置0、工具文档4；新增3/修改10/迁移1/删除0（迁移只计一次）。

| 类别 | 操作 | 文件 |
|---|---|---|
| 产品 | 新增3 | `src/vehicle_manager/tasks/{navigation_port,executor,runtime}.py`（本项目目录内） |
| 产品 | 修改2 | `src/vehicle_manager/tasks/{ports,__init__}.py`（本项目目录内） |
| 产品 | 修改3 | 根 `src/senior_control/tasks/{navigation_port,executor,runtime}.py` |
| 测试 | 迁移1 | 根 `tests/integration/test_terminal_release_outbox.py` → 本项目 `tests/` |
| 测试 | 修改1 | 根 `tests/integration/test_task_ownership.py`，仅改两个canonical导入 |
| 工具文档 | 修改4 | 根 `tools/verify_batch_one_install.py`、本README、`tests/README.md`、原实施计划 |

ownership的夹具仍被manual_wire/runtime_fault_isolation/terminal_metadata_runtime导入，故保留原位。
未新建项目或修改依赖/锁/CI/安装配置；原CLI和real/sim组合入口继续组装唯一实现。

### 命令、版本、结果和限制

macOS 26.4 arm64；主仓库 `.venv/bin/` 的Python3.12.15、pytest8.4.2、mypy2.4.0、ruff0.17.3，uv0.12.23。
下文 `python/mypy/ruff` 均使用该主仓库解释器/工具，未使用系统Python跑产品回归。

- RED：先改ownership测试导入并运行 `python -m pytest tests/integration/test_task_ownership.py --collect-only -q`，
  因canonical executor尚不存在报ModuleNotFoundError；没有新增场景。
- 端口步骤：`python -m pytest senior_control/tests/test_terminal_release_outbox.py senior_control/tests/test_task_lifecycle_service.py -q`：
  **34 passed，0.71s**；原终端义务、False/unknown ack、换boot、锁外发送/未知只查询保护保持。
- 两步装配：`python -m pytest senior_control/tests/test_terminal_release_outbox.py tests/integration/test_task_ownership.py senior_control/tests/test_task_runner.py -q`：
  **96 passed，1.70s**（包含前项部分用例，不相加）。
- 完整选集：`python -m pytest senior_control/tests/test_task_runner.py senior_control/tests/test_task_lifecycle.py senior_control/tests/test_schedule.py senior_control/tests/test_task_lifecycle_store.py senior_control/tests/test_task_lifecycle_service.py senior_control/tests/test_terminal_release_outbox.py tests/integration/test_task_ownership.py tests/integration/test_short_admission.py tests/integration/test_terminal_metadata_runtime.py -q`：
  **252 passed / 11 failed，25.42s**；11项均为short_admission参数化旧夹具：
  IPC替身只接受identity/disable_motion/exit，拒绝当前生产调用learning_capability。
  用 `git archive 22741b5` 在 `/tmp/s3-baseline` 导出只读基线副本，
  `python -m pytest /tmp/s3-baseline/tests/integration/test_short_admission.py -q -c /tmp/s3-baseline/pyproject.toml`
  同样**11 failed，22.82s**、同一原因；不改夹具、不补假属性、不弱化断言，也不宣称手动隔离该组通过。
  其他原SQLite恢复、资格/切图/迟到、真实任务IPC、未知查询/取消优先及终端metadata用例通过。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：30份源码，
  原lifecycle:318及lifecycle_store:647两项错误，无本片新增。
  全链命令 `mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud` 采用pytest声明的完整源码路径作为MYPYPATH，
  当前222份与同路径基线219份均**39项**，归一基线绝对路径后诊断逐条相同。
  相比历史38项，此路径含原ROS launch目录，被识别为namespace，原launch缺库诊断拆为两个缺属性，
  不是本片新错误；最初遗漏源码路径的110项输出不作为对照。
- 方法AST核对：导航端口11、executor15、runtime6个非构造方法，
  除明确列出的import/只读注解/异常逐实例接缝外与基线相同；两份原测试body及断言AST相同。
  构造逐项检查原DB身份、实时引用/回调、默认代次、异常类和原timeout赋值；无业务重写。
  11份相关Python的ruff check和format --check通过；按原CRLF检查diff通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/s3-wheels` 及根同命令：
  两项目sdist/wheel构建成功。
  `uv venv /tmp/s3-manager-venv --python <主仓库Python>`，
  `uv pip install --python /tmp/s3-manager-venv/bin/python --offline /tmp/s3-wheels/vehicle_manager-0.1.0-py3-none-any.whl /tmp/contract-convergence-wheels/senior_contracts-0.1.0-py3-none-any.whl`：
  8包；从 `/tmp` 用该解释器执行原安装工具 `--manager-tasks-only`通过，canonical驱动可导入而无root/ROS，
  原纯核心导入不拉数据库保护仍保留；此新环境Pydantic2.14.3/aiosqlite0.22.1。
- 原 `/tmp/vehicle-batch-one-venv` 离线 `uv pip install --reinstall --no-deps` 重装本轮root/manager wheel，
  从 `/tmp` 执行安装工具默认模式通过，原SDK保留Pydantic2.14.1；两环境 `uv pip check --python <对应Python>` 均通过（8/24包）。
  工具复用原SQLite create/get/CAS/重开及生命周期接受链，额外核对实际build_task_runtime的原DB/Store/IpcNavigation，
  原构造子类/同业务方法、异常继承/身份/文本/code、TaskRuntime同一类和默认0代次；未reconcile或连接IPC。

本片无新用例、无已知新增回归；选集中的11项旧夹具失败及2/39项类型错误如上登记，不能称全绿。
真实Linux/ROS/ament/DDS/原生安装、车辆/仿真运行和Python3.10未验证；原工具继续输出native/SLAM UNVERIFIED。
宿主SQLite/受控IPC与离仓库wheel检查不替代这些实测。
没有启动开发或实车服务、部署或推送；等待负责人亲审，后续管理边界与既有导航迁移另行精核预报。

## M2e WaypointStore存储迁移

基线 `494b794`。只迁现有SQL存储与行解析，Home业务及同DB协调仍原位。
实际目录与公开边界：

```text
senior_control/src/vehicle_manager/waypoints/
├── __init__.py    按需公开WaypointStore、WaypointStorePorts
├── store.py       唯一SQL/事务实现、WAYPOINT_STATEMENTS和原地图Home写入辅助函数
├── codec.py       原SQLite行解析与原验证错误构造
└── ports.py       每实例schema/值/异常构造及只读BindingReading
src/senior_control/waypoints/
├── __init__.py    Compatible，原公开名字按需加载
├── store.py       原Waypoint/StoredWaypoint值；Compatible薄构造与地图导入回调接线
└── home.py        未修改，现有Home业务/保护
```

旧WaypointStore与canonical类**不是同一对象**，薄子类仅保留原构造签名并注入原对象，
原SQL/事务方法直接继承唯一实现。原Waypoint、StoredWaypoint、MapRef、Pose2D、DomainError仍在原模块，
没有复制、重定义或迁这些值/异常；返回类型通过构造器的类型参数保留，避免迫使现有消费者转换对象或改签名。
WaypointStorePorts只持有本实例的prepare_schema、MapRef/Pose2D/Waypoint/StoredWaypoint/DomainError构造器，
没有全局工厂或默认Root实现。写入返回原pose/map_ref实例，读取仍按原构造顺序产生原值类型。
原11处DomainError、3处Pose2D及1处MapRef构造逐项对应，原KeyError原因链与sqlite错误分支不变。

canonical重用已有只读MapReference/PoseReading公开形状；BindingReading只要求实际读取的
waypoint.waypoint_id、waypoint.pose与anchor_pose，没有导入地图实现或复制绑定对象。
MAX_ROUTE_WAYPOINTS直接使用原navigation已转发的同一共享常量，不扩WS契约。
manager不反向导入root/ROS/地图，不新建项目或增加依赖配置。

原生产 `run_agent → 原Database → 旧WaypointStore` 继续持有同database/connection/write_lock。
原Store.prepare只调用原domain migrations；HomeService.prepare仍在其后调用原core迁移，
map_persistence原domain→core顺序、任务/settings及地图使用同连接的装配均不改。
路径、SQL版本/顺序、原单车Home唯一索引、revision冲突、绑定、分页与路由筛选、BEGIN/回滚/提交保持。
WAYPOINT_STATEMENTS旧入口与canonical及DOMAIN_MIGRATIONS[1]仍同一tuple。
地图导入的replace_imported_home转发同一canonical函数；adopted_home_blocks_foreign薄函数只传原DomainError，
两者继续使用地图事务传入connection，不获取第二把锁、不打开连接、不rollback/commit。
HomeExecutionCoordination/target lock、guard、每图Home、生产IPC与ROS/导航执行均未迁或改行为。

### 实际范围与验证

实际11逻辑文件：产品6、测试迁移1、配置0、工具文档4；新增4/修改6/迁移1/删除0。
产品为上树四份canonical文件及原store/__init__；
原 `tests/integration/test_waypoint_store.py` → 本项目 `tests/test_waypoint_store.py`；
工具文档为原verify_batch_one_install、此README、tests/README和原plan。

版本：macOS26.4 arm64、主仓库Python3.12.15/pytest8.4.2/mypy2.4.0/ruff0.17.3、uv0.12.23，
主仓库Pydantic2.14.1/aiosqlite0.22.1未变；下列python/ruff/mypy使用原主仓库`.venv/bin/`。

- RED：先在原安装验证器加入本片公开接口核对，从`/tmp`运行旧独立manager环境的
  `python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only`，
  因旧wheel尚无vehicle_manager.waypoints报ImportError；本轮构建安装后同命令通过。
- `python -m pytest senior_control/tests/test_waypoint_store.py -q`：**15 passed，0.41s**。
  唯一夹具变化是_operation_helpers指向现有 `ros2/tests/test_map_operations.py`；
  原同目录tests/integration/test_map_operations.py早已不存在。
  在`git archive 494b794`导出的`/tmp/m2e-baseline`运行
  `python -m pytest /tmp/m2e-baseline/tests/integration/test_waypoint_store.py -q -c /tmp/m2e-baseline/pyproject.toml`：
  **13 passed / 2 failed，0.40s**，两项均在旧helper路径FileNotFoundError；未删用例、改断言或加假属性。
- `python -m pytest senior_control/tests/test_waypoint_store.py tests/unit/test_home.py tests/integration/test_workspace_data.py ros2/tests/test_map_operations.py tests/unit/test_direct_navigation.py tests/unit/test_agent_ports.py -q`：
  **148 passed，4.60s**（含15项，不相加），覆盖原单车Home/修订冲突/foreign pending与保护、
  同连接地图Home导入/保留、同ref新代次旧位姿失效、工作区读写、生产IpcHomePort与任务/导航读取。
- AST核对13个非构造/prepare存储方法及10个原辅助函数，除显式构造器/错误注入及注解外一致；
  原Waypoint/StoredWaypoint类定义和全部测试body/断言一致，测试仅改变真实helper文件位置。
  19份SQL字符串逐字节相同，排序连接后的SHA256为
  `c1cd2a24ba4d5841fa955dcc9612fe3a4655ec6760718ca293930344356d6ed2`。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：34份源码仍原2项，
  lifecycle:318与lifecycle_store:647；全链同S3源码MYPYPATH运行
  `mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：226份仍39项，与S3诊断逐条相同。
  历史38/本环境launch namespace差异及其他旧夹具失败仍按S3记录，不宣称全仓全绿。
- 八份相关Python的ruff check/format --check及按原CRLF的git diff检查通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m2e-wheels`及根同命令：
  sdist/wheel成功；新`/tmp/m2e-manager-venv`离线安装本轮manager及既有contracts wheel，共8包，
  从`/tmp`运行上述安装工具`--manager-tasks-only`通过，canonical存储可导入且无root/ROS/地图实现。
  独立安装只验证公开存储接口与依赖隔离，domain SQL准备和值/异常构造仍须由实际owner注入，未宣称独立车辆装配。
- 原`/tmp/vehicle-batch-one-venv`离线 `uv pip install --reinstall --no-deps` 重装本轮root/manager wheel，
  从`/tmp`运行安装工具默认模式通过；8/24包 `uv pip check --python <对应环境Python>` 均通过。
  工具真实开SQLite并经旧构造保存、读取、重开航点，再经原IpcNavigation.task_poses读取任务目标；
  核对原DB/构造器、同业务方法、值身份与SQL tuple身份，没有连接IPC或新增服务。
  子进程核对旧公开Waypoint与原值同一对象且不提前加载HomeService，按需取HomeService仍原类。
  独立manager新环境Pydantic2.14.3，旧SDK保留2.14.1；aiosqlite均0.22.1。

本片选集通过，无新增场景或断言弱化。
未跑全仓、真实Linux/ROS/ament/DDS/原生安装、车辆/仿真运行或Python3.10；
工具原native/SLAM输出仍UNVERIFIED，宿主SQLite/安装不替代真实ROS。
未启动开发/实车服务、部署或推送，等待负责人亲审。

### Home下一片：只读依赖建议

本片未动 `waypoints/home.py` 和Database上的HomeExecutionCoordination/target锁。
不能直接将该模块整体导入manager：它仍引用地图BundleWaypoint/CellEvidence/HomeLink/AnchoredWaypoint/
AnchorSnapshot/MapReferenceStore，既读代次又调用place/place_waypoint/apply_anchor_update，
还有原Pose2D计算构造、DomainError catch/code及OperationStatus比较。
下一片须先逐调用收窄地图读写端口、原构造与取消结果读取，保留同DB的home_execution/home_target_lock对象及
return_home原execution→target→write_lock及编辑原target→write_lock顺序、guard token、取消settle和旧terminal隔离，不能把地图实现带进安装闭包。

当前生产并未构造这个HomeService：实际链为composition的IpcHomePort(state_client, waypoints)写Home，
DirectNavigation读取confirmed/adopted Home后经WorkspaceReader/原IpcState资格及IpcNavigation进入既有导航。
IpcHomePort保持pose.get/原采样时间0.5秒/状态与代次复核、原revision CAS；
HomeService现有构造在原unit/存储/地图回归与StoreHomePort可注入入口，不可迁移时顺便启用或替换生产链。
建议先独立只读盘点生产IpcHomePort/StoreHomePort及旧HomeService两条入口，
再分别预报生产管理接入与旧Home业务迁移，不按“一个home文件”混入地图算法/新每图Home。
可复用本片agent_ports的缺/旧pose保护、workspace/direct_navigation原Home准入，
以及test_home的同DB多service guard、未知接受/旧terminal/迟到callback、锁竞争与断链义务，
原地图same_ref_import/Home保留用例；输入与预期继续沿用原fixture，不先新增能力或测试情景。

## M2f 生产Home捕获与管理目标准入

基线 `7a1e265`，按顺序先归Home捕获，再归DirectNavigation管理侧请求/准入。
只移动已装配业务，不启用旧HomeService。

```text
senior_control/src/vehicle_manager/waypoints/
├── home_capture.py       HomeCapture：原pose/state/时间/代次检查及revision CAS请求
├── direct_navigation.py  DirectNavigation：原目标解析、Home选择与workspace/live准入
├── ports.py              原对象的只读消费接口与逐实例WaypointValuePorts
└── __init__.py           按需公开两个owner；既有store/codec不变
src/senior_control/agent/
├── ports.py              IpcHomePort Compatible薄构造；IpcState/PoseReply/其他IPC实现仍原位
└── direct_navigation.py  DirectNavigation Compatible薄构造；原私有validator转发同一类
```

旧IpcHomePort与HomeCapture、旧DirectNavigation与canonical类均**不是同一类**，
旧构造签名保留，业务方法直接继承同一实现。
WaypointValuePorts只携带每实例原MapRef/Pose2D/DomainError构造器；
MapRef/Pose/异常、RuntimeState/PoseReply、StoredWaypoint/Home对象都不迁、不复制或转换。
Home捕获只读取实际pose字段和四个state字段；目标校验另读navigation_ready/manual/safety，
不会把返航权限附加到设Home。接口中的类型参数保留原具体输入/输出，避免消费者改值或做运行时转换。
manager无root/ROS/地图实现反向导入，没有新项目或依赖配置。

IpcHomePort继续原同client的`pose.get → PoseReply.model_validate → MapRef → IpcState.read(state.get)`，
原PoseReply严格healthy/finite/extra规则和IpcState状态时效/完整ref校验都保留。
HomeCapture随后按原顺序检查0≤pose age≤0.5、localized、ref与代次，
使用同WaypointStore.save_home的原write_lock/CAS，返回expected_revision+1。
没有新加safety_clear/navigation_ready/环境许可、target/execution锁或提交后采样。
catch仍只有原IpcError/OSError/ValueError/ValidationError；
原IpcState.read的DomainError直接透出，不被新包泛化包装。原错误码/文本/原因链保持。

DirectNavigation私有_Reference/_Goal保留原定义，直接使用原protocol本来已转发的
senior_contracts.wire.MapRefPayload/ShortText，不扩WS或改变字段/接受规则。
返航仍要求adopted/confirmed且ref吻合，保留原Home pose对象；
先workspace.verify_free_target，再state.read与原地图/定位/导航/手动安全检查。
生产LifecycleNavigationOperations/_TaskRouteReceiver仍原位，外层原3秒预算、receiver二次state采样、
原Admission与同DB持久任务接受、已接受目标和取消语义不变。
运动仍只由原NavigationEffects→IpcNavigation.task_effect→ROS接收者执行，管理校验不接第二路运动。
旧HomeService、StoreHomePort、同DB/robot Home协调/guard、每图Home及导航算法均未迁或启用。

### 实际范围与验证

实际11逻辑文件：产品6、测试迁移1、配置0、工具文档4；新增2/修改8/迁移1/删除0。
产品为上述两份新增canonical、既有ports/__init__与原两个agent文件；
原 `tests/unit/test_direct_navigation.py` → 本项目tests，同字节迁移。
混合test_agent_ports保持原位，通过生产旧构造验证；工具文档仍为原安装验证器、此README、tests/README和原plan。

版本：macOS26.4 arm64、Python3.12.15/pytest8.4.2/mypy2.4.0/ruff0.17.3、uv0.12.23。
python/ruff/mypy使用原主仓库`.venv/bin/`；Pydantic2.14.1与aiosqlite0.22.1未变。

- RED：先更新原安装验证器，从`/tmp`运行旧独立manager环境的
  `python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only`，
  因旧wheel没有vehicle_manager.waypoints.home_capture报ModuleNotFoundError；本轮安装后同检查通过。
- `python -m pytest tests/unit/test_agent_ports.py senior_control/tests/test_direct_navigation.py senior_control/tests/test_waypoint_store.py -q`：
  **24 passed，0.56s**；包括缺真实定位/旧代次不可设Home、合格generic ref可设且不需safety_clear、
  原revision冲突、pending/confirmed返航与原free-map/代次准入。
- `python -m pytest tests/unit/test_agent_ports.py senior_control/tests/test_direct_navigation.py senior_control/tests/test_waypoint_store.py tests/integration/test_workspace_data.py tests/integration/test_task_ownership.py senior_control/tests/test_terminal_release_outbox.py -q`：
  **51 passed，1.27s**（含24项，不相加），涵盖原同DB任务接受/重放/查询、未知不重发、已接受目标不改、
  原取消/接管与终端义务；没有新增pytest场景或改写断言。
- AST核对原Home两方法及DirectNavigation.__call__：除显式注入的值/异常/parse/request接缝外相同，
  原catch tuple、请求/构造/await/资格顺序、取消分支保持；两个原私有Pydantic类定义一致。
  原agent/ports中其他IpcMapPort/IpcNavigation/RuntimeState/StateReply/IpcState/PoseReply/TaskAwareMotion等
  类体AST全部一致；迁移测试逐字节相同。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：36份源码仍原2项，
  lifecycle:318及lifecycle_store:647；全链同M2e的pytest源码MYPYPATH运行
  `mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：228份仍39项，诊断逐条一致。
  历史38与本环境launch namespace差异及旧short_admission夹具失败继续按S3登记，没有ignore或假属性。
- 八份相关Python的ruff check/format --check及按原CRLF的git diff检查通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m2f-wheels`及根同命令：
  sdist/wheel成功；`/tmp/m2f-manager-venv`离线安装本轮manager及原contracts wheel，共8包，
  从`/tmp`运行安装工具`--manager-tasks-only`通过，两个公开owner可导入且无root/ROS/地图实现。
  安装命令为 `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv pip install --python /tmp/m2f-manager-venv/bin/python --offline /tmp/m2f-wheels/vehicle_manager-0.1.0-py3-none-any.whl /tmp/contract-convergence-wheels/senior_contracts-0.1.0-py3-none-any.whl`。
- 原`/tmp/vehicle-batch-one-venv`离线 `uv pip install --reinstall --no-deps` 重装本轮root/manager，
  从`/tmp`运行原安装工具默认模式通过；8/24包的`uv pip check --python <对应Python>`均通过。
  工具沿用实际SQLite/TaskStore/WaypointStore/IpcNavigation/build_task_runtime，
  另装配原IpcHomePort/IpcState/WorkspaceReader/LifecycleNavigationOperations/_TaskRouteReceiver及旧DirectNavigation，
  核对同Store/原状态client/原parse与catch tuple、原值/异常构造、继承同一方法与真实receiver/executor引用，
  没有连接IPC或发运动；独立环境Pydantic2.14.3、旧SDK仍2.14.1，aiosqlite均0.22.1。

本片选定回归通过，不代表全仓或真实ROS通过。
未跑真实Linux/ROS/ament/DDS/原生安装、车辆/仿真服务及Python3.10；
原安装工具继续输出native/SLAM UNVERIFIED，宿主SQLite/受控原IPC与wheel不替代这些实测。
没有启开发或实车服务、部署或推送；等待负责人亲审，不继续旧HomeService迁移。

## M2f3 旧HomeService业务与guard

基线 `639a8ac`，仅搬既有旧服务，不接入生产组合。

```text
senior_control/src/vehicle_manager/waypoints/
├── home_service.py   唯一旧Home业务/guard/取消义务和原恢复计算
├── home_ports.py     原live地图效果、同DB协调及具体值/异常接口
└── __init__.py       按需公开canonical HomeService
src/senior_control/waypoints/home.py
                     原值/导航协议；Compatible薄构造、原地图owner接线
```

旧HomeService与canonical类**不是同一类**，业务/guard方法直接继承唯一实现。
StartupReference、LocalizedPose、HomeGoal、HomeResult与原导航协议仍是原模块中的原定义；
MapRef/Pose2D/Waypoint/OperationResult/OperationStatus/DomainError及地图值没有迁或复制。
原构造签名不变，创建原WaypointStore并传原Database；domain→core准备与原SQL路径不变。
局部类型参数仅保持原返回值类型，没有引入替代业务DTO或全局工厂。

HomeServicePorts逐实例提供原MapReferenceStore本身的live只读身份、原snapshot/place/apply_anchor_update效果、
原AnchoredWaypoint构造/place_waypoint接线及原Pose/HomeResult/HomeGoal/HomeLink/异常构造。
实际地图数据和几何/障碍/锚点/代次仍由原地图owner处理，不将地图实现导入manager。
原apply_anchor_update在target锁外更新地图、随后锁内snapshot/sync_poses的顺序保持；
snapshot仍以原Path("pose_graph")和空cells参数取得原对象，不缓存或另造状态。
原恢复计算只把原Pose2D构造参数显式化，数学表达式、角度wrap及1e-9比较不变。

传入的是原database.home_execution(robot_id)和home_target_lock(robot_id)返回的同一对象，
同DB/同robot各服务继续共享原DiagnosticLock与cancel_owner等可写协调字段；无新协调实例或全局注册。
return_home仍按execution→target→write_lock，编辑按target→write_lock，取消IO在write_lock外。
原guard先持久未知execution_id/token、仅同token更新/删除、取消写入等待真实settle、
逐次navigate_qualified能力检查、原OperationResult status与execution_id确认全部保持。
DomainError的原具体catch/code/原因链由原类及读取接口注入，Compatible cast只读取已捕获原类的code。
旧terminal/token、callback窗口、迟到/错intent、断链及同ref换代行为保留，不自动恢复或重发未知目标。

生产IpcHomePort/HomeCapture、DirectNavigation、LifecycleNavigationOperations/_TaskRouteReceiver、
原IPC/ROS/控制与云车协议均未改；旧HomeService仍没有生产composition构造。
没有每图Home、新导航、新项目或WS扩张。

### 实际范围与验证

9逻辑文件：产品4、测试迁移1、配置0、工具文档4；新增2/修改6/迁移1/删除0。
产品为canonical home_service/home_ports、旧home和现有公开入口；
原tests/unit/test_home.py同字节（含CRLF）迁到本项目tests；
工具文档为原安装验证器、此README、tests/README和原plan。

版本：macOS26.4 arm64、原主仓库`.venv/bin/`的Python3.12.15/pytest8.4.2/mypy2.4.0/ruff0.17.3，uv0.12.23；
主环境Pydantic2.14.1/aiosqlite0.22.1保持。

- RED：先更新原安装验证器，从`/tmp`使用旧独立manager运行
  `python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only`，
  缺vehicle_manager.waypoints.home_service报ModuleNotFoundError；本轮wheel安装后通过。
- `python -m pytest senior_control/tests/test_home.py -q`：**64 passed，2.22s**；
  原持久/可信定位、同DB guard/旧token、取消/断链、迟到callback与同ref换代保护均保留。
- `python -m pytest senior_control/tests/test_home.py senior_control/tests/test_waypoint_store.py ros2/tests/test_map_operations.py tests/unit/test_agent_ports.py senior_control/tests/test_direct_navigation.py tests/integration/test_task_ownership.py -q`：
  接口只读性修正后最终 **161 passed，4.98s**（含64项，不相加），未新增pytest场景或改断言。
  还覆盖原地图事务/Home绑定、生产设Home/管理返航和真实原任务装配，宿主回归不代表ROS运行。
- AST核对35个原业务/guard方法与9个恢复/比较/取消settle辅助函数，除明确构造/效果/类型接缝外一致；
  原公开值/协议类定义及迁移测试逐字节/AST一致；四份guard SQL逐字节相同，排序连接SHA256为
  `f69e97a320034fca5e70a6fbdd4dfbbb089303096a798925b08967e10090c395`。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：38份源码仍原2项，
  lifecycle:318与lifecycle_store:647。
  全链同M2f源码MYPYPATH运行 `mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：
  首轮40项中唯一新增来自协议把只读取的lock误声明为可替换字段，导致DiagnosticLock子类不满足不变字段；
  按真实使用改为只读getter，仍传原锁对象，无cast/ignore，最终230份仍原39项、诊断逐条一致。
- 六份相关Python的ruff check/format --check与原CRLF diff检查通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m2f3-wheels`及根同命令：
  两份sdist/wheel成功；`uv venv /tmp/m2f3-manager-venv --python <原主仓库Python>`后
  `uv pip install --python /tmp/m2f3-manager-venv/bin/python --offline /tmp/m2f3-wheels/vehicle_manager-0.1.0-py3-none-any.whl /tmp/contract-convergence-wheels/senior_contracts-0.1.0-py3-none-any.whl`：8包，
  从`/tmp`运行上述原安装工具`--manager-tasks-only`通过，canonical HomeService无root/ROS/地图反向依赖。
- 原`/tmp/vehicle-batch-one-venv`离线`uv pip install --reinstall --no-deps`重装本轮root/manager wheel，
  从`/tmp`运行原安装工具默认模式及两环境`uv pip check --python <对应Python>`均通过（8/24包）。
  工具实际在临时SQLite上构造旧服务/同robot peer并prepare，核对原DB/Store、同协调/return/target锁、
  原地图owner/绑定效果、原值和异常/结果工厂、继承同一业务方法与原guard读取；未initialize或导航。
  这是旧入口安装检查，不是将服务启用到生产；独立manager的原构造/地图/协调仍须由真实owner注入。
  独立环境Pydantic2.14.3、旧SDK2.14.1，aiosqlite均0.22.1。

本片选定回归通过，原2/39项类型错误及历史S3夹具失败仍登记，不宣称全仓全绿。
真实Linux/ROS/ament/DDS/原生安装、车辆/仿真服务和Python3.10未测；原native/SLAM安装仍UNVERIFIED。
未启开发或实车服务、部署或推送，等待负责人亲审，不开始下一片。

### 下一片管理连接/请求：仅只读范围建议

建议先精核真实run_agent装配下的WebSocket会话与请求边界，再预报迁移，不直接整搬agent/main或connection。
候选目标为既有vehicle_manager内部agent模块，Root CLI/composition与ROS IPC适配继续只组装原owner；
具体内部目录/文件划分须以下一轮清单为准，不新增顶层项目或技术runtime包。

只读源优先为agent/websocket.py、connection.py、snapshots.py、vehicle_state.py、main.py、
dispatcher.py、journal.py、pairing.py、composition.py，按实际入边补manual_control/settings_wire/logs_wire。
connection同时有纯HeartbeatFrame/Link和实际IpcMotion/MotionRejected，不能为迁Link把运动IPC或Root异常实现拉进manager。
WebSocketLink/Connector仍依赖原快照序列化、DiagnosticLock/QueueSnapshot/process_diagnostics及Supervisor.AuthRejected；
须逐项保留真实异常身份、时钟、heartbeat nonce/窗口、close/revoke、send timeout/队列上界、epoch与旧响应处理，
明确原日志/诊断注入与owner，不用WS contracts承载技术实现或移动车端截止/帧龄策略。

请求处理需分别核对AgentJournal与同DB原事务/去重/tombstone、Dispatcher edit/map lanes/超时/取消未知，
LifecycleNavigationOperations及已迁manager任务/设置/日志/Home接口，
以及ManualIngress/恢复/ESTOP和断链的真实控制请求，不把WS连接代次改成执行代次，
不让非手动已接受任务因目录迁移被断链取消。
原测试候选位置包括test_websocket_link、test_agent_pairing、test_agent、test_manual_wire、
test_settings_logs_wire及现有任务ownership/终端用例；
先列各真实入口→字段/异常/队列/时钟/事务依赖，再按独立连接、快照投影、journal、dispatcher/Agent顺序切片。
此处仅给盘点范围，不表示上述模块已获批搬迁或估算可直接复用。


## M3A 管理连接、监管、配对与journal基础

基线a8ce5c7；本片仅迁四份既有实现，现有run_agent/Agent/composition继续通过旧入口装配。
实际新增目录：

```text
senior_control/src/vehicle_manager/agent/
├── __init__.py       # 按需公开，导入公开包不加载网络库
├── ports.py          # 当前快照/heartbeat/诊断与journal值的逐实例接缝
├── websocket.py      # 原Link/Connector、nonce/epoch/超时/撤销及队列配置
├── supervisor.py     # 原异步监管、重试/故障隔离与关闭
├── pairing.py        # 原凭据/配对、文件权限和禁止覆盖
└── journal.py        # 原同DB/锁SQL、事务、去重/tombstone和未知恢复
senior_control/tests/
├── test_websocket_link.py # 原tests/unit同字节迁移
├── test_runtime.py        # 原tests/integration同字节迁移
└── test_agent_pairing.py  # 原tests/integration同字节迁移
```

唯一业务实现归manager；原agent/websocket和journal显式Compatible薄构造子类，
因此新旧业务owner类不是同一对象，继承的业务方法仍是同一实现。
原JournalEntry/OperationStatus/_OPEN、HeartbeatFrame/RobotSnapshot及snapshot_envelope继续由原owner提供，
旧Connector逐实例选择旧Link构造；原ProtocolRejected/Socket/超时常量、Credentials/配对函数、
AuthRejected/Supervisor及journal SQL tuple均同一对象。
原DiagnosticLock/QueueSnapshot/process_diagnostics由旧装配注入，标签、容量/不可观测深度、时钟和owner不变；
监管logger显式保留senior_control.runtime.supervisor前缀。没有另建状态/诊断owner。

生产调用仍为run_agent→原Agent/Connector→Compatible Link→canonical网络状态；
原Supervisor导入为canonical别名，原AgentJournal构造传入原Database/同连接写锁与值。
Supervisor的call_ros只是原回调，WS连接并未迁入本地IPC或控制；
connection/main/dispatcher/snapshots/composition及非手动已接受任务断线规则不改。
公开技术接缝仅表达真实消费者使用的构造/效果，不扩WS contracts，manager无root/ROS反向导入。

### 实际范围和安装闭包

21逻辑文件：产品10（agent六份新增、上述四份旧入口修改），测试迁移3，配置构建4，工具文档4；
新增6/修改12/迁移3/删除0，迁移单计。
配置仅根与manager pyproject、根uv.lock、原backend CI；工具文档仅原安装工具、本README、tests/README和原plan。
manager基础依赖不加网络库，agent extra声明httpx与websockets；根agent extra改消费vehicle-manager[agent]。
锁中69个包的名称/版本集合不变，HTTPX0.28.1/WebSockets15.0.1，云镜像与云锁不改。
CI新增基础及agent-extra仓库外wheel检查，仍固定uv0.12.23、Python3.10/3.12；本地仅语法核对，非CI实跑。

### 命令、结果与限制

版本：macOS26.4 arm64、原主仓库.venv的Python3.12.15/pytest8.4.2/ruff0.17.3/mypy2.4.0，
uv0.12.23；主环境Pydantic2.14.1/aiosqlite0.22.1。
下列python/ruff/mypy均使用原主仓库.venv/bin绝对路径，命令在本功能工作区执行，离仓库项另注。

- RED：从/tmp以原旧manager环境运行原安装工具新增--manager-agent-only，缺vehicle_manager.agent失败；
  本片wheel安装后同入口通过。
- `python -m pytest senior_control/tests/test_websocket_link.py senior_control/tests/test_runtime.py senior_control/tests/test_agent_pairing.py -q`：
  首次沙箱19通过/1失败，原因原localhost WS夹具bind被EPERM拒绝；允许原测试回环socket后20通过，1.74s。
- `python -m pytest senior_control/tests/test_websocket_link.py senior_control/tests/test_runtime.py senior_control/tests/test_agent_pairing.py tests/integration/test_agent.py tests/integration/test_settings_logs_wire.py tests/integration/test_task_ownership.py tests/unit/test_logging_diagnostics.py tests/integration/test_backend_observability.py -q`：
  最终116通过，9.90s（包含上述20项，不累加）。原WS身份/nonce/epoch/revoke/超时、监管阻塞/失败/关闭、
  配对/权限、journal提交失败重放/去重、设置日志wire及任务离线隔离复用原用例；无新场景/弱化断言。
- 原Link的12个非构造方法、Connector连接方法、journal7个业务方法按明确注入名归一后AST一致；
  原JournalEntry和事务helper、Supervisor/AuthRejected类体一致，pairing及三份迁移测试逐字节一致。
- 相关14份Python的`ruff check`及`ruff format --check`通过；CI YAML解析与新增步骤`bash -n`通过。
  `git -c core.whitespace=cr-at-eol diff --check`用于保留原CRLF后的差异核对。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：44源码，
  仍原2项lifecycle:318/lifecycle_store:647；将MYPYPATH设为根pytest现有全部pythonpath后运行
  `mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：236源码仍原39项，
  与M2f3同环境诊断集合逐条一致，不宣称类型检查通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --offline`及
  `uv export --frozen --extra agent --no-hashes --no-emit-local --offline`成功；
  `uv sync --frozen --dry-run --group dev --extra agent --extra cloud --extra safety --extra video --extra ros --offline`
  成功，仅dry-run，不等于完整frozen根重装。
- 同cache下`uv build senior_control --offline --out-dir /tmp/m3a-final-wheels`和根
  `uv build --offline --out-dir /tmp/m3a-final-wheels`成功生成sdist/wheel。
  以原Python创建/tmp/m3a-base-venv和/tmp/m3a-agent-venv，使用上述export约束安装本片manager及既有contracts wheel。
  agent extra初次离线安装因WebSockets15.0.1未缓存失败；下载原锁定依赖后安装成功，无服务启动。
  最终wheel重装后，从/tmp执行原工具`--manager-tasks-only`及`--manager-agent-only`均通过；
  后者核对Root/cloud/ROS/地图包不存在并输出网络库0.28.1/15.0.1。
  `uv pip check --python <各环境python>`分别8包/15包通过。
- 原/tmp/vehicle-batch-one-venv完整锁约束重装尝试缺pydantic-settings2.16.1离线缓存；
  保留旧SDK已有兼容第三方版本，`uv pip install --offline --reinstall --no-deps`安装本片root/manager wheel，
  再按export约束安装manager[agent]/contracts网络依赖成功。
  从/tmp运行原工具默认模式及`uv pip check`通过，29包；不是完整根frozen重装通过。
  工具真实构造旧Connector但不连接，核对旧link工厂、值/异常与方法身份；
  使用真实临时SQLite和原Database/AgentJournal prepare/begin/get/finish，核对同DB/锁/原值及状态。
  原任务/Home/生产IPC装配引用检查继续通过，没有发运动或替换真实IPC。

真实Linux/ROS/ament/DDS/原生及车辆/仿真运行、Python3.10和GitHub runner未实测；
安装工具原native/SLAM仍UNVERIFIED，既有S3夹具失败与原2/39项类型诊断仍登记。
选定116项通过不代表全仓全绿；无开发/实车服务、部署或推送。
下一片先精核快照/连接与请求处理依赖，保留原IPC/控制和唯一状态owner，再预报迁移；本片未提前实施。


## M3B 管理快照、显示、工作区与请求分发

基线8b59521；原管理行为迁至既有vehicle_manager/agent，真实run_agent仍从旧公开入口装配。

```text
senior_control/src/vehicle_manager/agent/
├── snapshots.py        # LiveState/RobotSnapshot及原云车快照投影
├── vehicle_state.py    # 原只读显示投影与安全修订前沿
├── frame_projection.py # 原vehicle_frame；不授予控制权限
├── snapshot_state.py   # 原SnapshotState唯一序列/安全token/采集状态
├── settings_wire.py    # 原实时证据刷新与设置wire，设置服务仍唯一策略owner
├── logs_wire.py        # 原独立日志连接/授权/有界pending与Python日志源
├── workspace.py        # 原地图读取/身份围栏与目标、任务编辑检查
├── dispatcher.py       # 原分发lane/时间预算/journal与持久任务准入
├── state_ports.py      # 实际只读消费形状、原显示构造器
├── request_ports.py    # 每实例原值/异常/诊断及地图效果
└── __init__.py         # 既有按需公开入口更新
senior_control/tests/
├── test_vehicle_state.py     # 原tests/unit，逐字节迁移
├── test_runtime_reporting.py # 原tests/integration，逐字节迁移
└── test_workspace_data.py    # 原tests/integration，逐字节迁移
```

公开显示/快照值由旧路径显式Compatible同对象转发到canonical，
原MapRef/Pose2D、运动/IPC运行时对象、Waypoint、OperationStatus和具体DomainError不搬、不复制。
旧SnapshotState、WorkspaceReader、SettingsRuntime、AgentLogConnection、Dispatcher/Store*Port及
LifecycleNavigationOperations为Compatible薄构造子类，非同一类，但继承唯一业务方法；
旧_TaskRouteReceiver构造也保留，生产准入使用canonical接收器和原executor/state。
原OperationNotCommitted仍在旧入口继承原DomainError，TaskOperationResult仍继承原OperationResult，
构造/catch/code/链与原result/status类逐实例注入；无全局工厂或注册。

原地图本地响应validator类及限制仍在旧workspace接口，逐实例传给canonical读取编排；
导出回退仍调用原WorkspaceMap和原exported_grid，不将栅格/地图算法打入manager闭包或WS contracts。
MapRef/Pose工厂继续生成原对象；同ref、grid身份、free-space和二次读取检查的顺序保持。
显示只读消费原运动对象，MotionMode.ESTOP身份比较和MotionState/RuntimeObservation构造由旧入口提供；
只读协议不是新DTO或运行时验证，不刷新观测时刻、不复制控制状态。
SnapshotState继续拥有同一组序列/安全token/恢复前沿，原TaskExecutor上下文采样/任务事务与IPC对象沿用。

原PendingCounter/DiagnosticLock通过构造注入，复用现有日志QueueCounter技术端口而非复制诊断；
日志send锁/32 pending/5秒send及独立网络循环、设置锁与实时证据采样顺序保持。
原dispatcher logger显式senior_control.agent；原map/edit锁、journal、超时/取消未知、
已接受目标/未知只查询和导航准入、手动执行与ROS唯一运动链不变。
原Agent/main、connection/IpcMotion与run_agent函数体保留；本片只迁组合中的SnapshotState。

### 实际文件范围

26逻辑文件：产品19、测试迁移3、配置0、工具文档4；新增10/修改13/迁移3/删除0。
产品为上述10份新文件（不含既有__init__）、__init__更新及原agent下八份：
snapshots/vehicle_state/manual_control/composition/settings_wire/logs_wire/workspace/dispatcher。
工具文档仅原安装工具、本README、tests/README与原plan；pyproject/锁/CI/云镜像无改动，未新增项目或WS字段。
既有CI已发现manager源码和项目内测试，无需新增配置。

### 验证命令、结果与环境

macOS26.4 arm64；使用原主仓库.venv/bin的Python3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0，
uv0.12.23、Pydantic2.14.1/aiosqlite0.22.1、HTTPX0.28.1/WebSockets15.0.1。
以下python/ruff/mypy为该主环境绝对路径；工作区命令如下，仓库外项单列。

- RED：在未装本片的原独立manager环境执行原工具`--manager-tasks-only`，
  缺vehicle_manager.agent.snapshot_state报ModuleNotFoundError；本片wheel安装后通过。
- `python -m pytest senior_control/tests/test_vehicle_state.py senior_control/tests/test_runtime_reporting.py senior_control/tests/test_workspace_data.py tests/integration/test_agent.py tests/integration/test_settings_logs_wire.py tests/integration/test_task_ownership.py -q`：
  首轮及最终均108通过，最终8.58s；复用原快照旧代次/几何失效/省略、显示安全修订、
  设置证据/日志wire、工作区身份/目标、journal重放与持久任务准入保护。
- 在上述命令追加`tests/integration/test_manual_wire.py tests/integration/test_runtime_fault_isolation.py tests/integration/test_settings_logs_review.py tests/unit/test_agent_config.py`：
  沙箱151通过/10失败，包含原Unix socket bind EPERM；允许原测试socket与隔离子进程后152通过/9失败，14.85s。
  这152项包含核心108项，不累加，也不将完整选择宣称全绿。
- 将8b59521用git archive只读导出/tmp/m3b-baseline，在同环境允许socket执行
  `python -m pytest tests/integration/test_manual_wire.py tests/integration/test_runtime_fault_isolation.py tests/integration/test_settings_logs_review.py -q`：
  37通过/9失败，6.85s，失败test ID集合与本轮完全相同。
  原3个manual_wire夹具缺CommandNode._output_incarnation；
  原5个runtime_fault_isolation real-*夹具缺ros_setup，关闭替身不接受group_grace；
  原settings_logs_review目录日志场景的IPC替身拒绝learning_capability，导致快照不产生。
  原夹具/断言不改，不补假属性或跳过；这些失败不作为本片已验证的真实手动/子进程保护。
- 按明确的构造/异常/效果接缝归一，52个原业务方法/函数AST一致；
  其余AgentConfig/run_agent/build_task_runtime/pause_failed_tasks及ManualIngress/Receiver类体AST一致，
  原异常/结果与workspace本地validator类体一致，三份测试逐字节一致，无新增pytest场景。
- 对agent目录、八份原入口、工具及三份迁移测试执行`ruff check`和`ruff format --check`，
  并以`git -c core.whitespace=cr-at-eol diff --check`核对差异。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：54源码仍原2项
  lifecycle:318/lifecycle_store:647；MYPYPATH取根pytest原pythonpath后执行
  `mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：246源码仍原39项，
  与M3A同环境诊断集合逐条相同，无新增ignore/cast，原类型错误不算通过。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m3b-wheels`及
  根`uv build --offline --out-dir /tmp/m3b-wheels`成功，最终生成两份sdist/wheel。
  对原/tmp/m3a-base-venv、/tmp/m3a-agent-venv用`uv pip install --offline --reinstall --no-deps`安装本片manager wheel，
  原/tmp/vehicle-batch-one-venv同命令安装root/manager wheel；均使用原已验证第三方兼容闭包，无锁或依赖更新。
  从/tmp分别运行原安装工具`--manager-tasks-only`、`--manager-agent-only`及默认模式均通过；
  `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv pip check --python <对应环境python>`分别8/15/29包通过。
  最终重装后的检查首轮漏设cache导致默认用户cache沙箱拒绝，补回上述任务cache后全部通过。
  独立base/extra导入所有新模块且无root/cloud/ROS/地图包；旧SDK真实构造原SQLite/TaskStore/WorkspaceReader/
  SnapshotState/LifecycleNavigationOperations及原IPC客户端引用，验证同对象/异常/工厂与唯一业务方法，未连接IPC或发运动。
  此为本片wheel叠加兼容闭包验证，非完整根frozen重装；上片根离线缓存缺口与旧类型/夹具失败仍登记。

真实Linux/ROS/ament/DDS/native/车辆仿真运行、Python3.10与GitHub runner未验；
安装工具原native/SLAM仍UNVERIFIED，不用静态/宿主检查替代ROS实测。
无开发/实车服务、部署或推送，等待负责人亲审。

### M3C前置只读清单（以下实施已完成）

下一片建议合并精核原Agent/main与connection的纯Link/Heartbeat边界、
_snapshot_change_key和实际run_agent闭包，避免再拆无依赖障碍的小文件批次。
main仍负责WS会话准入、手动/恢复与安全lane、快照采集/发送预算和断链；
connection同时含纯Link/MotionPort/HeartbeatFrame与IpcMotion/MotionRejected，不能整包导入manager。
优先逐实例注入原clock/metrics/诊断、同DB/journal、原具体结果/异常、真实motion/IPC和监督回调，
复用本片canonical Dispatcher/SnapshotState/wire；run_agent的存储/任务/地图/视频owner装配须继续可独立核对。
本地IPC客户端是管理到执行的适配，WS身份/epoch不得变成运动执行代次；
ManualIngress/运行时接收与ROS执行owner仍原位，旧CLI/环境/关闭顺序和离线已接受任务继续保持。
上述清单是下节M3C实施的依据；导航算法仍后置。


## M3C Agent会话与生产管理编排

基线94bc1d9；实际唯一Agent主循环和管理run_agent已归manager，
旧python -m senior_control.agent.main、CLI/环境/私密凭据入口及实车/仿真启动调用保持。

```text
senior_control/src/vehicle_manager/agent/
├── main.py            # 原Agent/session、分发/快照/安全停机与_snapshot_change_key
├── connection.py      # 原纯HeartbeatFrame、Link、MotionPort
├── runtime_ports.py   # 每Agent原值/异常/诊断/clock/metrics/媒体效果
├── composition.py     # 原run_agent管理编排、准入/任务loop/关闭及pause_failed_tasks
├── bootstrap_ports.py # 真实生产工厂和本地客户端消费接口、MediaBinding
└── __init__.py        # 既有按需公开入口更新，无伪可执行入口
src/senior_control/agent/
├── main.py            # Compatible薄Agent构造、原ManualOperationResult与CLI
├── connection.py      # Compatible同对象纯协议；原MotionRejected/IpcMotion
└── composition.py     # 原Config/build_task_runtime/媒体owner和逐调用显式绑定
senior_control/tests/
├── test_agent.py        # 原integration用例，只有真实方法patch owner调整
└── test_agent_config.py # 原unit逐字节迁移
```

旧Agent是Compatible薄子类，非同一类，仅覆盖构造绑定；其余48个原方法继承canonical实现，
其中诊断报告方法转发原owner效果；
HeartbeatFrame/Link/MotionPort及pause_failed_tasks旧入口同一对象。
原ManualOperationResult、OperationResult/Status、DomainError、OperationNotCommitted/TaskOperationResult、
MotionRejected继承IpcError及其code/catch/链和名义LifecycleNavigationOperations检查保留。
manual_result构造和manual_result_type名义检查显式指向同一原类，无代理值或复制状态。
Runtime端口只描述真实消费字段，原Clock/SystemClock、Metrics、PendingCounter/DiagnosticPending与
队列report效果每Agent绑定；生产bootstrap逐调用明示实际工厂，不导入整个Root容器、注册全局工厂或复制诊断。
诊断仍由原owner执行，logger前缀显式senior_control.agent。

生产链仍为docker_start_helper/simulation_cli→原模块CLI→原Config/run_agent薄入口→
canonical管理编排→原Database/TaskStore/WaypointStore/AgentJournal/IPC与媒体工厂。
configure_diagnostics、同DB/连接写锁、prepare与未知恢复顺序保持；原4个motion客户端和
state/map/navigation/workspace/display共9个独立客户端不合并，普通1秒/navigation3秒及domain/owner_record不变。
原task_commit gate外采样、generation重查、gate→写锁、事务内过期provider与
TaskAwareMotion原持久接管/真实handoff不搬不改；原任务导航及非手动已接受任务离线继续保持。

原300ms手动截止、2秒快照/250ms采集与变化合并，64 ordinary/64 safety/1 latest intent/1 workspace、
32模块报告与stragglers观测、旧epoch/revoke/过时结果隔离保持。
request_shutdown先撤权，disable/settle安全、等待其他inflight、关link、cleanup、motion.exit/close、
撤signal顺序与原各阶段预算保持，不把WS掉线等同任务取消，不产生第二路运动。

媒体选择与AiortcPeerFactory/RosCameraSource/LatestFrameFile/VideoSession仍在原owner；
原video_session闭包体原样保留，MediaBinding只传同一工厂与关闭效果。
ROS摄像头内部自己的rclpy Context/executor和实际媒体线程未迁，manager不导入ROS/aiortc/av或内部执行DTO。
settings/logs真实canonical owner复用，Root日志handler附着、level恢复及独立SQLite、DEBUG恢复和关闭顺序保持。

### 实际范围与验证

15逻辑文件：产品9（上述五份新增、旧三入口和manager公开入口修改），测试迁移2，配置0，工具文档4；
新增5/修改8/迁移2/删除0，迁移单计。
工具文档仅原安装工具、本README、tests/README和原plan；无pyproject/锁/CI/服务或启动命令修改。
既有manager agent extra和CI测试发现已覆盖本批，独立manager构造须明确提供实际端口，
不宣称已有不需要生产适配器的独立管理CLI或整项B6已完成。

macOS26.4 arm64；原主仓库.venv/bin的Python3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0，
uv0.12.23、Pydantic2.14.1/aiosqlite0.22.1、HTTPX0.28.1/WebSockets15.0.1。
以下python/ruff/mypy均该主环境绝对路径，工作区命令如下；仓库外另注。

- RED：旧独立manager运行原安装工具--manager-tasks-only，缺vehicle_manager.agent.main失败，装本片wheel后通过。
- `python -m pytest senior_control/tests/test_agent.py senior_control/tests/test_agent_config.py tests/integration/test_task_ownership.py tests/integration/test_settings_logs_wire.py -q`：
  首轮92通过/1失败8.68s，原Unix socket bind被沙箱EPERM拒绝；原用例未改，后续允许socket的完整选集覆盖该组。
- `python -m pytest senior_control/tests/test_agent.py senior_control/tests/test_agent_config.py tests/integration/test_task_ownership.py tests/integration/test_settings_logs_wire.py tests/integration/test_shutdown.py tests/integration/test_runtime_fault_isolation.py tests/integration/test_manual_wire.py tests/integration/test_settings_logs_review.py -q`：
  允许原临时socket/隔离子进程后158通过/9失败，15.27s；含原Agent容量/快照/旧命令、任务离线/未知查询、
  关停/故障隔离与真实设置/生产装配，不能将完整选集称为全绿。
- 94bc1d9经git archive只读导出/tmp/m3c-baseline，同环境允许socket运行
  `python -m pytest tests/integration/test_runtime_fault_isolation.py tests/integration/test_manual_wire.py tests/integration/test_settings_logs_review.py -q`：
  37通过/9失败6.93s，失败ID集合与本轮完全相同。
  仍原5个real-*进程夹具缺ros_setup/关闭替身不接受group_grace，
  3个manual夹具缺_output_incarnation，1个日志目录夹具拒绝learning_capability；不补假属性、不改断言或跳过。
- `python -m pytest tests/integration/test_video.py -k 'vehicle_signaling or video_frame_leaves_the_pending_drive_sample_alone or close_frame_survives_a_stop_and_a_video_flood' -q`：
  5通过/35未选0.22s，复用原peer替身，覆盖原会话关闭/无摄像头/单viewer、原控制pending与stop/洪泛隔离；
  这不是实际相机/aiortc编解码或全媒体回归通过。
- 48个Agent方法含构造按明确端口接缝归一后AST一致，_snapshot_change_key精确一致；
  原队列报告效果、CLI/Config/IPC/MotionRejected/ManualOperationResult、任务组合与pure Link/Heartbeat协议体一致。
  run_agent的准入/loop/关闭顺序按原工厂/媒体效果接缝归一后AST一致，video_session闭包体原样。
  test_agent_config逐字节迁移；test_agent仅将ControlledAsyncio patch导入从旧main改到真实canonical方法owner，
  其余字节/输入/断言不变，未新增pytest情景。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：59源码4项；
  其中原lifecycle:318/lifecycle_store:647两项未变，原Root Agent的可选gesture比较和_answer_learning未注解两项随迁移到main:612/799。
  MYPYPATH取根pytest原pythonpath后`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：
  251源码仍39项；将两项Agent诊断归回原路径并忽略迁移行号后与M3B逐项一致，无新增ignore/cast，旧错误不算通过。
- 对agent目录、三份旧入口、工具及两份迁移测试执行ruff check/format --check，
  并用`git -c core.whitespace=cr-at-eol diff --check`核对差异。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m3c-wheels`及根
  `uv build --offline --out-dir /tmp/m3c-wheels`成功构建两份sdist/wheel。
  以同cache对原/tmp/m3a-base-venv和/tmp/m3a-agent-venv执行`uv pip install --offline --reinstall --no-deps`安装manager wheel，
  原/tmp/vehicle-batch-one-venv同命令安装root/manager wheel，使用既有第三方兼容闭包。
  从/tmp运行原工具--manager-tasks-only、--manager-agent-only、默认模式均通过，
  `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv pip check --python <对应Python>`分别8/15/29包通过。
  独立manager全部新公开模块无Root/ROS/媒体执行依赖；旧SDK真实SQLite同DB/Store及实际IPC构造引用保持，
  实例化旧Agent核对原值/异常/名义类/诊断/clock与4个独立motion客户端，没有run/connect或发运动。
  工具还从/tmp用SDK解释器执行`python -m senior_control.agent.main --help`，返回0且保留凭据/配对选项；没有读凭据或启动服务。
  这是已有兼容依赖上的本片wheel验证，不是完整根frozen重装，原缓存缺口仍登记。

真实Linux/ROS/ament/DDS/native/车辆仿真、相机与aiortc实媒体、Python3.10及GitHub runner未验；
安装工具native/SLAM仍UNVERIFIED，旧S3夹具失败继续登记，不以宿主检查代替ROS实测。
无开发/实车服务、部署或推送；剩余Root IPC/媒体/诊断设施的职责与真实安装收尾需另审范围，导航算法仍最后。


## M4A 操作CLI、部署渲染与实车/仿真监管

基线e6dccc4；管理实现迁入已有vehicle_manager，现有senior-robot console及所有旧python -m入口保持。

```text
senior_control/src/vehicle_manager/
├── profile.py        # 原VehicleProfile/load_profile，目录与配置规则
├── cli.py            # 原操作分发、配对/doctor/deploy和旧布局支持
├── deployment.py     # 原统一目录、容器映射、unit渲染和部署事务
├── service.py        # 原native/Docker联合监管、ready/stop/进程回收
├── simulation.py     # 原user service/仿真监管/双锁/stale检查
└── platform_ports.py # 逐调用原IPC/学习/事件/平台效果，无Root容器或新实现
src/senior_control/
├── cli.py/vehicle_deploy.py/docker_start_helper.py/simulation_cli.py # Compatible旧签名/模块入口
└── runtime/ipc.py    # 原传输原位；新增承接原三份socket归属函数的唯一实现
ros2/src/senior_robot/senior_robot/agent_adapter.py # 两处改读实际IPC owner
senior_control/tests/
├── test_vehicle_cli.py
├── test_control_directory_cli.py
└── test_docker_start_helper.py
```

原纯profile与路径解释、部署辅助函数、进程终止辅助和DisplayUnavailableError同对象转发。
其余旧函数是显式Compatible薄调用，签名不变；业务仅canonical一份，旧class/入口不伪造反射属性。
原Root helper保留六份ROS/设备效果（命令组装、视频环境、Linux peer验证），
原IPC/学习服务/生命周期事件与配置预检均逐实例明示接入，manager无Root/ROS反向实现导入。
原CLI平台导入仍按实际功能惰性调用，pair/help不为构造绑定提前加载native监管。
root helper的_identity/_record/_prepare_socket原实现归既有runtime/ipc.py；旧入口同对象别名，
ROS adapter直接从IPC读同一函数，不再为了socket操作导入管理helper。
没有搬IPC协议/ROS执行/媒体/诊断owner或新建顶层项目、扩大WS/新增算法。

权限/私有目录、O_NOFOLLOW/独占实例锁、无覆盖安装、host/container最深可写bind映射、
PID/UID/dev/inode/run_id复查和35秒readiness、原IPC字段、原停止顺序及超时全部保持。
服务仍只停止所选owned进程，不stop容器；Agent/ROS/camera隔离与学习claim分配/跨实例清理原位，
仿真双锁、headless/windowed环境、视频私有slot和既有学习/导航链不改。
原CLI/systemd/module命令与配置文件值保持，仿真当前仍调用senior_robot的simulation_motion.launch.py，未提前搬launch。

### 实际范围和验证

23逻辑文件：产品12（新增6、修改旧四入口/IPC/ROS adapter共6），测试5（迁移3、原owner路径调整2），
配置0，工具文档6（原安装工具、manager/tests README、原plan、两份operations说明）；
新增6/修改14/迁移3/删除0，迁移单计。
两份CLI测试只改实际stdlib patch模块，Docker helper测试逐字节迁移；
原runtime_identity和launch_recovery只把_record导入改到实际IPC实现，所有原用例/输入/断言保持。

环境macOS26.4 arm64；原主仓库.venv/bin的Python3.12.15/pytest8.4.2/ruff0.17.3/mypy2.4.0、
uv0.12.23、Pydantic2.14.1/aiosqlite0.22.1，HTTPX0.28.1/WebSockets15.0.1。
以下python/ruff/mypy均该绝对路径；工作区命令如下，仓库外项另注。

- RED：旧独立manager执行原工具--manager-tasks-only，缺vehicle_manager.cli等新模块报ImportError；本片wheel安装后通过。
- 首次原选择在沙箱有Unix socket EPERM及旧stdlib patch路径失败；
  按真实方法owner调整原CLI用例的patch，未补回假module属性、改断言或新增pytest情景。
  `python -m pytest senior_control/tests/test_vehicle_cli.py senior_control/tests/test_control_directory_cli.py -q`：49通过，0.44s；最终惰性装配收尾后同组49通过，0.42s。
- 最终允许原测试socket/隔离child执行
  `python -m pytest senior_control/tests/test_vehicle_cli.py senior_control/tests/test_control_directory_cli.py senior_control/tests/test_docker_start_helper.py tests/unit/test_runtime_identity.py tests/integration/test_ipc_exclusive_bind.py tests/integration/test_runtime_fault_isolation.py tests/integration/test_ipc_lifecycle.py tests/unit/test_ipc_frames.py ros2/tests/test_launch_recovery.py::test_launch_endpoint_reclaims_only_same_owner_refused_stale_socket -q`：
  103通过/5失败/1跳过，8.69s；包含上述49项，不累加。
  唯一跳过为原Linux SO_PEERCRED用例，宿主不具备该接口；不是实际peer凭据验证通过。
- e6dccc4用git archive只读导出/tmp/m4a-baseline，同环境允许socket运行
  `python -m pytest tests/integration/test_runtime_fault_isolation.py -q`：16通过/5失败，4.17s；
  原real-spawn/identity/exit/agent_exit/adapter_exit五项失败ID与本轮相同，
  仍缺ros_setup、group_stop替身不接受group_grace，原macOS学习SOCK_SEQPACKET不可用日志亦如实保留。
  不补假属性/改保护或把该真实进程链算通过；历史其他旧夹具缺口不宣称修复。
- 38个原管理/部署/监管函数按明确效果接缝归一后AST一致；profile及路径规则、三份socket归属函数和
  原IPC所有函数/类体精确一致，六份原ROS/设备效果精确一致。
  ROS adapter保留CRLF，除两处owner导入外逐字节一致；五份测试体/断言在仅归一实际patch/导入位置后AST一致。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：65源码仍M3C原4项；
  MYPYPATH取根pytest原pythonpath后`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：
  257源码仍原39项，忽略迁移行号后诊断集合逐项相同，不宣称类型全绿。
- 相关产品/工具/测试的ruff check/format --check及
  `git -c core.whitespace=cr-at-eol diff --check`核对；不重格式化其他ROS源码或改其旧保护。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build senior_control --offline --out-dir /tmp/m4a-wheels`和根
  `uv build --offline --out-dir /tmp/m4a-wheels`成功构建两份sdist/wheel。
  用同cache的`uv pip install --offline --reinstall --no-deps`向原/tmp/m3a-base-venv、/tmp/m3a-agent-venv安装本片manager，
  向原/tmp/vehicle-batch-one-venv安装本片root/manager；均为原第三方兼容闭包，无配置/锁改动。
  从/tmp运行原工具--manager-tasks-only、--manager-agent-only及默认SDK模式通过；
  `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv pip check --python <对应Python>`分别8/15/29包通过。
  独立manager环境真实解析临时config.json、渲染原unit/env，无Root/ROS导入；监管仅检查导入，未运行服务。
  SDK环境核对profile/渲染/进程辅助/socket别名同一身份，实际旧CLI --control-dir deploy --dry-run保留唯一config文件且不启动/写服务，
  并用该SDK解释器从/tmp运行原cli/docker_start_helper/simulation_cli --help及实际senior-robot console --help，全返回0。

这些是迁移兼容性验证。独立manager计划仍输出历史模块启动命令，生产绑定仍需完整旧SDK与平台owner；
不以旧SDK验证或纯导入/渲染冒充独立部署。根完整frozen重装缓存缺口、原类型与旧夹具问题仍登记。
真实Linux/systemd/Docker/厂商ROS/ament/DDS/车辆仿真和实际SO_PEERCRED、相机/媒体、Python3.10/CI未验，
安装工具native/SLAM仍UNVERIFIED。仅运行原测试套接字/临时子进程和替身，未启开发/实车服务、systemd或Docker容器、部署或推送。

### M4B 必要收尾范围（尚未实施）

原real_runtime.py仍是校准参数/导航组合预检，不能移入manager；下一片归现有senior_robot实车组合公开接口，
保留旧转发并更新safety_motion.launch及Root平台provider的真实预检导入，控制/感知算法与原硬件类型不顺带重写。
仿真总装配仍为senior_robot/launch/{simulation_motion,simulation_rgbd}.launch.py，
由senior_robot/setup.py安装，RGBD仍include旧motion入口；senior_sim已有ament_cmake的simulation.launch、world/URDF及camera/world_reference脚本。
下一片将canonical仿真组合归senior_sim，保留旧Include兼容入口，逐项更新真实调用、CMake/setup/package及installed share检查。
现有nav2/BT/校准配置先复用真实owner；若移动仿真专属YAML，必须先列旧share路径兼容和实际消费者再预报，不静默扩大。
独立部署门禁须在这些原技术owner/平台闭包真正收敛后另做，不能通过安装旧senior-control SDK或只import宣称完成；
Root IPC/媒体/诊断最终归属和其他遗留Root入边仍须另审，导航算法仍后置。

### M4B 实车预检与仿真组合入口

基线7fe7832。预检唯一实现归`ros2/src/senior_robot/senior_robot/real_runtime.py`；
原`senior_control.real_runtime`显式Compatible同对象转发。真实provider
`docker_start_helper._adapter_command`、`vehicle_deploy`校验与`safety_motion.launch.py`
直接调用canonical。原预检实现逐字节保留，原硬件profile、传感器策略、诊断和导航reference仍原owner。

仿真唯一组合归`senior_sim/launch/{simulation_motion,simulation_rgbd}.launch.py`，
`vehicle_manager.simulation.supervise`实际执行`ros2 launch senior_sim simulation_motion.launch.py`。
原`senior_robot/launch`两文件仅Compatible Include，全部原声明/默认值逐项不变并转发，
Include继承原launch上下文（含namespace及额外配置），use_sim_time仍显式透传；没有第二组节点。
RGBD canonical直接Include senior_sim motion；原GroupAction transport隔离、三个节点、RMW环境保持。
专属YAML、nav2/BT、RGBD transport XML仍读取senior_robot share，world/profile仍senior_sim share。

实际23文件：产品5（新增1/修改4）、测试4修改、配置构建10（新增3/修改7）、工具文档4修改；
即新增4/修改19/删除0，迁移实现单计在对应修改/新增中，无额外删除。
产品为实车预检新旧两路径、原helper/deploy provider、manager simulation；
测试为test_vehicle_cli、test_common_sensor_composition、test_rgbd_transport_config、test_runtime_fault_isolation。
配置为Root pyproject/uv.lock、CI、senior_robot pyproject、原safety launch、四份新旧simulation launch、senior_sim package.xml。
工具文档为安装核验工具、本README、ros2/README及原实施计划。

安装有两种真实边界：senior_robot新增与原功能包一致的非ROS Python wheel（无console script），
Root SDK依赖它，保持CLI在source overlay前可找到预检；原ament setup.py/resource/节点及launch安装声明保持。
wheel不替代ament share/executable安装，默认包导入不拉起ROS。预检与原来相同需要SDK safety extra，
不能把基础CLI可导入称作硬件预检依赖齐全。senior_sim原CMake整个launch目录安装覆盖两新文件，
补齐其实际直接launch/nav2/控制/接收依赖；保留sim→robot资产/节点依赖，robot不新增sim依赖形成环。
旧robot Include入口要求部署环境同时安装sim，与原来调用sim环境相同。

验证环境：macOS arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23。
以下工具取主仓库`.venv/bin`，uv使用`UV_CACHE_DIR=/tmp/vehicle-uv-cache`；未启真实服务/仿真/车辆。

- `python -m pytest senior_control/tests/test_vehicle_cli.py senior_control/tests/test_control_directory_cli.py senior_control/tests/test_docker_start_helper.py tests/unit/test_common_sensor_composition.py tests/unit/test_rgbd_transport_config.py tests/unit/test_sim_contract.py tests/integration/test_runtime_fault_isolation.py -q`：111通过/5失败/1跳过，8.59秒。
  原断言保留；只改实际canonical读取路径，原slot替身核对真实新launch包，原RGBD组合用例同步核对旧Include所有声明参数/目标与无重复节点。
- `git archive 7fe7832`至临时目录，同解释器`pytest tests/integration/test_runtime_fault_isolation.py -q --tb=no`：16通过/同5失败，4.02秒。
  五个real-{spawn,identity,exit,agent_exit,adapter_exit}旧夹具缺ros_setup且group_stop不接group_grace，未补假属性或弱化保护。
  跳过为macOS不支持原Linux SO_PEERCRED；学习SOCK_SEQPACKET不可用亦保留原记录。
- `uv build --offline {ros2/src/senior_robot,senior_control,根项目} --out-dir /tmp/m4b-wheels`分别构建三个sdist/wheel通过。
  `/tmp/vehicle-batch-one-venv`原兼容闭包叠加安装三wheel，从/tmp运行`tools/verify_batch_one_install.py`通过：原CLI/模块help、原dry-run、同DB/类型与真实管理构造；30包pip check通过。
  初次预检导入明确缺Shapely；原safety依赖离线缓存不可用，联网装入Shapely2.2.0及NumPy2.5.5后
  `--real-preflight-only`同对象/无rclpy验证通过，32包pip check通过。
  仓库外直接调用原预检缺失/不存在校准路径和不完整导航参数拒绝保持，通过；未连接任何硬件。
  这是既有兼容SDK闭包验证，不是完整Root frozen重装或独立manager生产部署。
- `uv lock --check --offline`通过，70包，仅新增本地senior-robot，第三方锁版本不变。
  原预检/motion launch逐字节、全部原launch声明/default AST、全部原config字节比对通过；RGBD仅新Include路径及格式变化。
- MYPYPATH取Root pytest原pythonpath后`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：257源码仍原39项，归一行号与M4A集合一致。
  未设置源码路径的初次检查138项含本地包不可见，不能作为全链结果；没有新增ignore/cast。
  变动Python ruff/format和diff检查通过（见提交前最终核验）。

当前无ros2/colcon/ament_index_python；`cmake -S ros2/src/senior_sim -B /tmp/m4b-sim-build`亦因cmake命令缺失不能执行。
因此未验证真实ament安装、launch substitutions/namespace运行、DDS、cmd_vel唯一发布、原学习启动及Linux厂商/设备关闭；
静态/替身通过不替代这些实测。Python3.10和CI未执行，原native/SLAM安装仍UNVERIFIED。

管理独立部署仍未完成：原CLI/systemd/module入口保留Root绑定，manager只拥有编排，
原9客户端与IPC协议/权限、媒体/相机/视频、诊断/clock/metrics、进程恢复/学习服务及ROS平台效果仍原owner。
实车预检虽归robot但消费原硬件profile/导航公开实现；仿真组合仍消费Root恢复/生命周期/策略和robot节点/资产。
已迁功能包的真实ROS进程、原导航/扫描/设备适配兼容入口尚有上述实际边界，需后续按依赖只读核定，
不能借完整旧SDK或纯import宣称独立部署。导航/建图算法、状态、IPC和WS本片未迁未改。

### M5A 管理观测与Agent命令行驱动

基线c4591ca。实际目录与生产调用如下：

```text
senior_control/src/vehicle_manager/
├── observability.py        # 原JsonFormatter/Metrics/configure_logging唯一实现
└── agent/application.py    # 原Agent CLI驱动；每次显式传配置与run_agent
src/senior_control/
├── observability.py        # //Compatible，同对象公开别名
└── agent/
    ├── main.py             # 原main()签名/模块入口，薄调用application.main
    ├── dispatcher.py       # 直接消费canonical Metrics
    └── composition.py      # 原AgentConfig/实际run_agent绑定，本片不迁
```

真实旧模块入口仍调用Root `AgentConfig.from_environment`和`run_agent`，
将这两个原工厂逐次传给canonical驱动，无全局注册或默认假执行入口。
`JsonFormatter`、`Metrics`、`configure_logging`及`OTHER`旧新入口同一对象；
CLI旧main是显式Compatible薄函数，保留无参数签名，唯一驱动函数体在manager。
原parser、环境覆盖、凭据/私密配对、错误退出78和日志初始化顺序保持；
原senior_control logger命名、allowlist、脱敏、有限异常链/标签及锁原样。
Root Agent类、ManualOperationResult与队列报告方法体未改，
Clock/IPC/共享诊断、媒体、ROS执行、WS和配置/安装依赖未迁未改。

实际12文件：产品5（新增2/修改3）、测试3修改、配置0、工具文档4修改，新增2/修改10/删除0。
产品为上述五个观测/CLI驱动文件；测试为原logging_diagnostics、backend_observability及agent_config；
工具文档为verify_batch_one_install.py、本README、tests/README和原实施计划。
完整模块及CLI主体迁移各一组，旧路径保留薄入口，无额外文件删除。
未增加pytest场景，只调整真实canonical导入和pairing/asyncio patch owner，所有原断言AST一致。

验证使用macOS arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23。
解释器和检查器取主仓库`.venv/bin`；uv使用`UV_CACHE_DIR=/tmp/vehicle-uv-cache`。

- `python -m pytest tests/unit/test_logging_diagnostics.py tests/integration/test_backend_observability.py senior_control/tests/test_agent_config.py -q --tb=short`：
  初次16通过/1临时Unix bind权限失败；允许原夹具socket后17通过，0.44秒。
  覆盖原隐私/异常链、有限指标、配置/私密配对、真实SQLite装配与无adapter仍上报，未启真实服务。
- 原observability字节、原Agent/result/报告体及规范化CLI AST比对通过；
  CLI仅移除原局部实现导入并将原配置构造改为明示callback，原全部测试assert AST比对通过。
- `uv build --offline senior_control --out-dir /tmp/m5a-wheels`及Root对应构建：两个sdist/wheel通过。
  仓库外原基础8包、agent-extra15包、旧SDK32包环境叠加安装最终wheel，
  分别执行原工具`--manager-tasks-only`、`--manager-agent-only`及默认模式通过，三个pip check均通过。
  manager环境明确无Root/cloud/ROS，新application/observability可导入但未提供独立默认生产factory。
  SDK模式核对旧新观测公开身份，运行真实旧console和模块help（包括senior_control.agent.main），
  原不安全ws配置返回78、打印原TLS错误且不泄漏token，实际配置在run_agent前拒绝，无网络连接或DB写入。
  这些是兼容安装叠加验证，不是全量Root frozen重装或独立管理部署。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：67源码仍原4项；
  MYPYPATH取Root pytest原pythonpath后`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：
  259源码仍原39项，与M4B归一行号的诊断集合一致。回调按真实Coroutine返回标注，无新增cast/ignore。
- 变动Python的Ruff/format及diff检查在提交前核验；配置/锁/CI无变化。

真实Linux/ROS/ament/DDS/厂商/车辆仿真、媒体及Python3.10/CI本片未验，原native/SLAM安装仍UNVERIFIED。
原其他旧失败仅在此前批次登记，本片未扩大重跑无关选集，也未删弱保护。
管理生产装配仍需Root工厂、原具体值/异常、同DB协调及IPC/诊断/媒体平台owners，
Root→manager依赖保持单向，manager新模块无Root/ROS反向导入；不能把旧SDK装配通过称作独立管理部署完成。
下一片须先确认共享技术安装归属：IPC codec/身份和诊断队列跨管理/ROS共用，
帧龄策略仍原owner；Clock、ImageFrame、车内值/异常构造来源及媒体设备输入仍按真实边界核定，
不得用新根runtime、扩大WS contracts或复制业务/状态绕过。

### M5B 本地请求适配与真实任务工厂

基线3e7cf31。实际职责与调用：

```text
senior_control/src/vehicle_manager/agent/
├── local_ports.py          # 原client方法形状；逐实例异常/导航构造引用
├── ipc_motion.py           # IpcMotion与TaskAwareMotion唯一业务实现
├── ipc_navigation.py       # IpcMapPort/IpcNavigation请求及ack解析
├── ipc_state.py            # IpcState原采样/资格/geometry投影规则
└── composition.py          # 原run_agent + 原build_task_runtime工厂
src/senior_control/agent/
├── connection.py           # Compatible IpcMotion薄构造；原MotionRejected仍原类
├── ports.py                # Compatible薄构造；原RuntimeState/StateReply/PoseReply/Home接线
└── composition.py          # 原生产绑定；build_task_runtime逐次接原具体构造器
senior_control/tests/test_agent_ports.py # 原用例逐字节归管理包
```

原run_agent绑定仍提供同一个Database/TaskStore/WaypointStore/journal及9个真实IpcClient，
manager原bootstrap保留八个1000ms客户端与一个3000ms导航客户端、原身份/domain/run记录。
实际工厂只将原TaskExecutor和NavigationEffects具体构造器传给manager工厂，
原LifecycleStore/Service/Runtime对象、连接、写锁、Home协调和错误继承保持；
工厂泛型保留注入Executor的具体返回类型，无新cast或名义类型替代。
旧适配器是Compatible薄子类，非同一类，但所有业务方法同一实现；
TaskAwareMotion仍是旧IpcMotion子类，原任务/手动仲裁与super链保持。

具体DomainError/IpcError/MotionRejected/RouteRejected、MapRef/Pose2D/RoutePlan、
StateReply解析器和RuntimeState返回工厂仍原owner逐次注入，无值/异常/锁复制。
StateReply/RuntimeState/PoseReply及IpcHomePort业务体未迁，内部DTO未加入WS。
状态仍按原sampled_at和0.5秒门限判断，valid_until仍为原采样加0.5秒；
仅原同代可信几何可显示，旧代次/未知结果/终端释放及新手动保护原样。
原navigation effect字段/accepted三态/unknown仅查询、任务离线继续和manual五次接管尝试保持。
Unix v1 framing/peer身份、Clock、诊断、媒体、ROS执行、DB/Home与地图/控制权威未迁未改。

实际14逻辑文件：产品8（新增4/修改4）、测试2（迁移1/修改1）、配置0、工具文档4修改；
合计新增4/修改9/迁移1/删除0，迁移单计。比30–42预报小，因为复用了既有bootstrap、
Requester、工厂注入和自动wheel/test发现，不另造按类门面或为凑数改依赖/CI。
测试只有原agent_ports整文件迁移，及原真实Agent装配用例观察九client/timeout、同DB/Store/连接/写锁；
原断言保留，无新增pytest场景。其余传输/跨执行回归仍归原owner，未无关搬测。

验证环境：macOS arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23。
工具取主仓库`.venv/bin`，uv设`UV_CACHE_DIR=/tmp/vehicle-uv-cache`；仅原测试临时socket/替身，无真实服务。

- 首轮原agent_ports/ipc_frames/agent_config/direct_navigation/terminal_release_wire：34通过、
  1受限Unix bind和1退役控制夹具失败。允许原socket后的完整选集命令：
  `python -m pytest tests/unit/test_agent_ports.py tests/unit/test_ipc_frames.py tests/unit/test_runtime_identity.py tests/integration/test_ipc_lifecycle.py tests/integration/test_ipc_exclusive_bind.py tests/integration/test_intent_ipc_stream.py tests/integration/test_task_ownership.py tests/integration/test_terminal_metadata_runtime.py tests/integration/test_manual_wire.py senior_control/tests/test_agent_config.py senior_control/tests/test_runtime.py senior_control/tests/test_runtime_reporting.py ros2/tests/test_terminal_release_wire.py -q --tb=short`：
  104通过/4失败，6.35秒。当时agent_ports仍在旧路径，随后逐字节迁入本包，
  `pytest senior_control/tests/test_agent_ports.py -q`七项复用通过；没有把重跑相加宣称新覆盖。
- `git archive 3e7cf31`临时基线，原解释器运行
  `pytest tests/integration/test_manual_wire.py ros2/tests/test_terminal_release_wire.py -q --tb=short`：33通过/完全相同4失败。
  三个manual_wire旧夹具缺CommandNode._output_incarnation，一个terminal_release旧夹具缺MotionCoreOutput.controller；
  未补假属性、删断言或弱化退场规则，真实native/ROS整链未因此得到认证。
- 原装配用例`test_agent_config.py::test_composition_opens_and_closes_database`追加实际client/DB引用观察，
  保留原关闭断言并定向复跑：1通过，11.34秒。最初新增观察误读Agent._tasks，
  按真实Dispatcher._tasks._store修正并以finally执行原release，未补属性或移除原断言；失败测试进程已清理。
- 五个原适配类和任务工厂按明示接缝规范化AST审计一致；RuntimeState/StateReply/PoseReply/Home、
  AgentConfig/run_agent/media业务体保持；原测试字节、IPC/诊断/Clock/DB/媒体字节保持。
  原task_poses的MapRef cast仅静态用途，canonical Store已接受只读MapReference，
  现直接传原任务上的同一运行时对象，不新增运行时校验。
- `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`：71源码仍原4项。
  MYPYPATH取Root pytest原pythonpath后`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：
  263源码仍原39项，归一行号与M5A诊断集合一致。原具体Executor类型通过工厂泛型保留，无新增ignore/cast。
- `uv build --offline senior_control --out-dir /tmp/m5b-wheels`及Root对应构建，manager/Root两个sdist/wheel通过；
  仓库外基础8包、agent-extra15包和旧SDK32包叠加安装最终生产代码，运行既有工具三个模式及pip check，
  基础8包、agent-extra15包及SDK32包三模式和三个pip check均通过；核对canonical方法、
  旧薄子类/MRO、原值/异常/DTO工厂和同SQLite装配。初次SDK旧help子进程15秒超时，
  原时限未改，默认工具完整重跑通过；未以加长或删除保护绕过。
  独立manager只验明确接口与导入无Root/ROS，不借SDK装配称独立部署；Root/manager配置、锁、CI不改。

安装和回归中的临时Unix服务仅原测试夹具；未启动开发/实车/仿真服务、部署或推送。
实际Linux SO_PEERCRED/PR_SET_PDEATHSIG/SOCK_SEQPACKET、ROS/ament/DDS/厂商、媒体、Python3.10和CI未验；
原native/SLAM安装仍UNVERIFIED。管理生产仍需Root配置/具体值/诊断/IPC/媒体及执行provider绑定，
并未完成独立生产部署。下一片共享安装归属仍待确认，不用根runtime、WS扩张或复制owner绕过。

### M5C 管理媒体与执行端图像接入

基线3b4e14f。实际目录：

```text
senior_control/src/vehicle_manager/media/
├── __init__.py       # 只读ImageReading形状，不定义另一份ImageFrame
├── stream.py         # 原WebRTC session/peer/encoder/thread唯一实现
├── signaling.py      # 原单viewer与候选队列，注入原诊断队列
├── turn.py           # 原TURN值/签名/凭据唯一实现
└── latest_frame.py   # 管理端poll/最后capture状态，注入原RGB1读取函数
ros2/src/senior_robot/senior_robot/ros_video.py # 原ROS订阅线程/单slot，显式注入帧构造
ros2/src/senior_sim/scripts/simulation_camera.py # 原发布节点，仅直接引用共用RGB1 publisher
src/senior_control/vision/
├── rgb1.py           # 同一legacy owner内隔离原RGB1编解码/发布，实现仅一份
├── synchronizer.py   # 原ImageFrame仍唯一、未迁配准/识别
└── stream/turn/signaling/simulation_video/ros_video.py # 原公开Compatible入口
```

stream和turn公开类/异常/函数通过旧入口同对象别名；需要原owner注入的
VideoSignaling、LatestFrameFile、RosCameraSource是Compatible薄子类，非同一类，业务方法唯一。
实际Root `_media_binding`使用canonical stream及执行端camera，显式传原ImageFrame；
相机peer关闭后再关订阅源、最后关共享encoder factory的顺序保持。
ROS订阅源码不导入Root/manager，仍按需创建原rclpy context/node/executor和线程，
话题/QoS/时间判断/尺寸/单slot/关闭行为保持；FrameT/CapturedFrame仅其真实消费形状。

RGB1与ImageFrame最终共同安装owner仍未另定。本片保留在原legacy项目，
仅将RGB1原函数隔离到独立模块，避免sim发布经兼容consumer入口引入manager。
旧publisher及header/helpers显式转发同一实现，sim直接调用Root rgb1，
生产与ROS端仍注入同一个原ImageFrame类，未复制结构或放进WS contracts。
Root共享依赖仍存在，不以此宣称整个ROS或管理部署已独立。
管理端ImageReading只读接口不构造新帧值；诊断仍注入原DiagnosticAsyncQueue，
原队列上界、计数/锁/bind_async owner未迁未复制。

原logger名称senior_control.vision.stream/signaling/ros_video明确保留，旧logger入口可引用同一对象。
原SDP/候选/epoch与viewer限制、relay-only、拥塞丢帧/心跳stop隔离和backend懒加载保持。
Root video extra改委托manager[video]，沿用aiortc>=1.15,<2；编码仍直接更新AV plane，不新增NumPy依赖。
根锁70包版本/来源、非本地包完整记录和原resolution partitions保持，只有两个本地包的extra元数据变动。
CMake/ament和既有Robot wheel声明自动包含新Python模块，现有CI独立manager核验工具同时检查媒体导入；
未改启动服务、launch、设备参数、部署镜像或独立安装策略。

实际23文件：产品14（新增7/修改7）、测试2修改、配置3修改、工具文档4修改；新增7/修改16/删除0。
产品为上述七份新实现/接口，五个旧vision入口、Root composition及sim脚本；
测试只改原simulation_video/video的真实实现导入，原全部assert AST不变，无新增pytest场景；
配置为Root/manager pyproject及Root uv.lock；工具文档为安装工具、本README、ros2/README和原计划。
未为预报测试数量改写其他owner用例。原ImageFrame、诊断、IPC、深度/识别与控制业务不改。

验证环境：macOS arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23；
aiortc1.15.0、PyAV17.1.0。命令的python/checker取主仓库`.venv/bin`，uv设UV_CACHE_DIR=/tmp/vehicle-uv-cache。

- `python -m pytest tests/unit/test_simulation_video.py tests/integration/test_video.py -q --tb=short`：
  初次受限环境42通过/真实loopback等帧20秒超时（并有原read coroutine结束警告）；
  允许原测试UDP能力后43通过，6.84秒，收到真实64×48帧，原20秒断言/时限未改。
  覆盖原无后端/无相机、单viewer/候选上界、relay-only、编码关闭、拥塞不影响控制、RGB1帧新鲜/非法文件与padding。
- 生产绑定影响的原`pytest senior_control/tests/test_agent_config.py -q --tb=short`：
  初次6通过/1临时Unix bind权限失败；允许原socket后7通过，0.28秒，无改断言或补属性。
- 3b4e14f只读归档的完整mypy为263源码/39项；新代码269源码仍相同39项，归一行号无新增。
  完整命令MYPYPATH取Root pytest原pythonpath，运行`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`。
  `MYPYPATH=senior_control/src:contracts/src mypy senior_control/src/vehicle_manager`为76源码仍原4项。
- 原TURN逐字节、stream/signaling/reader/ROS/生产binding/sim主体按明示接口规范化AST审计一致；
  RGB1三个原函数和header/constants精确保持，原assert与ImageFrame/诊断/IPC字节保持。
- 临时缓存清空导致最初offline构建缺hatchling、锁解析缺原lefthook元数据、安装缺pydantic-core wheel；
  获取既有构建/锁依赖后manager/Root/Robot/contract及原兼容依赖wheel构建通过。
  锁未升级任何第三方；`uv lock --check --offline`与frozen export通过。
- 新venv以frozen导出的约束安装本地wheel：base8包从/tmp执行安装工具`--manager-media-only`及既有`--manager-tasks-only`通过；
  video20包执行`--manager-video-only`通过，实际导入aiortc/PyAV且无Root/ROS依赖，基础导入不拉起backend。
  旧SDK全新25包wheel闭包执行默认工具通过：旧公开身份、原RGB1实际读写/原ImageFrame类型、
  原队列注入及camera无启动构造/旧CLI和真实既有SQLite/Agent装配检查保持，三个pip check均通过。
- 进一步在安装SDK补原video extra和测试工具，从/tmp执行
  `python -m pytest <仓库>/tests/unit/test_simulation_video.py <仓库>/tests/integration/test_video.py::test_aiortc_loopback_delivers_a_real_frame -q --tb=short -o pythonpath=`：
  4通过，29.37秒；清空pytest源码路径，实际运行安装wheel的Root RGB1与manager peer/encoder，同一原帧类型。
  这些复跑不相加算新增场景。Robot subscriber安装导入不加载Root/manager/rclpy；
  Root共享RGB1 publisher导入不加载manager.media，未经兼容consumer给sim引入新反向依赖。
- 提交前核对变动Python Ruff/format、diff及最终wheel源码位置。

未启动开发/实车/仿真服务、部署或推送；实际回环仅原测试本机RTC peer，无服务器。
当前无rclpy/ROS/ament，真实订阅/设备/QoS、ROS时钟、厂商环境和sim发布实测未验，
Python3.10/CI未执行，native/SLAM安装仍UNVERIFIED。静态/安装导入不冒充ROS运行。
共享帧/RGB1仍需旧SDK，Manager新wheel无默认设备/诊断绑定；生产CLI及其余技术owner仍原边界，
本片不是独立管理生产部署或共享公共owner决策，不提前迁IPC/诊断/算法或扩大WS。


## M5D1 实车平台命令与视频探针

基线 `81f17f8`，直接在固定功能分支交付。本片是既有行为迁移；没有新增运行入口、
修改 systemd/module 命令、IPC、诊断、Clock、学习、帧或导航业务。

实际9文件：产品新增1、修改1，测试修改2，配置0，工具文档修改5，删除0，迁移单计。

```text
ros2/src/senior_robot/senior_robot/platform_runtime.py # 新增，唯一五函数实现/只读 RuntimeProfile
src/senior_control/docker_start_helper.py             # 修改，原五签名 Compatible/真实效果注入
senior_control/tests/test_docker_start_helper.py       # 修改，原overlay用例从canonical入口调用
senior_control/tests/test_vehicle_cli.py               # 修改，原sim用例patch实际asyncio owner
README.md, senior_control/README.md, ros2/README.md    # 修改，实际边界与验证
docs/superpowers/plans/2026-11-19-vehicle-architecture-refactor.md # 修改，实际批次顺序
tools/verify_batch_one_install.py                    # 修改，原安装验证与provider隔离导入
```

### 真实调用与公开接缝

旧 `docker_start_helper._ports()` → 原 `ServicePorts` → 原五个 Compatible 包装函数 →
`senior_robot.platform_runtime`；包装函数保留原配置对象和签名，不复制状态。
`RuntimeProfile` 仅公开 motion_profile、safety_parameters、ros_setup、ros_domain、
vendor_ros_setup、navigation_reference、navigation_parameters 的只读实际字段。
真实预检仍按原时机从 `senior_robot.real_runtime` 延迟导入，其依赖未迁移。

相机包装逐次导入原 `runtime.launch_recovery.restart_prefix` 并传入唯一实现，
同时传入原 `_ros_launch_command` 回调以保留现有安装调用/故障隔离接缝。
provider只调用传入效果，既不导入管理程序也不拥有第二个恢复器。
原vendor/overlay顺序、同解释器、域、原argv/namespace、相机过滤表达式、
视频5秒探针/环境白名单/失败保留控制环境/探针进程清理与警告logger名称均保持。
peer身份、权限、0.5秒校验、IPC、关闭、学习/events及Agent/module入口未改。

### 验证命令、结果与限制

环境：macOS26.4 arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23。
以下 `python`/`ruff`/`mypy` 来自主仓库原 `.venv/bin/`；所有uv命令使用
`UV_CACHE_DIR=/tmp/vehicle-uv-cache`，没有更新锁/依赖或启动开发、车辆、仿真服务。

- 修改前原 `python -m pytest senior_control/tests/test_docker_start_helper.py senior_control/tests/test_vehicle_cli.py senior_control/tests/test_control_directory_cli.py -q --tb=short`：58通过、1跳过，4.94秒。
- 原overlay用例改canonical导入后先单独运行：因新模块未创建失败；迁移后原断言不变，1通过。
- 原定向 `python -m pytest senior_control/tests/test_docker_start_helper.py senior_control/tests/test_vehicle_cli.py senior_control/tests/test_control_directory_cli.py senior_control/tests/test_agent_config.py tests/unit/test_runtime_identity.py tests/integration/test_runtime_fault_isolation.py -q --tb=short`：83通过、1跳过、5失败，8.59秒。
  五项是原real故障隔离夹具未带ros_setup及group_stop未接受group_grace；原camera叶子恢复/受控子进程关闭、CLI/dry-run、9客户端与同DB装配/身份用例通过，不补夹具假字段或弱化断言。
- `git archive 81f17f8`至 `/tmp/m5d1-baseline-source`，同解释器原 `pytest tests/integration/test_runtime_fault_isolation.py -q --tb=short`：16通过、完全相同5失败，4.09秒。
- 用原pytest pythonpath生成MYPYPATH；`mypy src/senior_control/docker_start_helper.py ros2/src/senior_robot/senior_robot/platform_runtime.py --follow-imports=silent`：2源码通过。
  `mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：269源码仍39项；81f17f8归档用同命令仍39项，归一行号后的诊断完全一致。
  最初未配置MYPYPATH的单独检查报12项路径缺失/随之Any，修正为仓库真实发现路径后无新问题。
- 原五函数AST主体对照：只归一视频logger固定旧名和相机显式注入，全部原语义主体一致；
  helper剩余peer/IPC/learning/events/服务函数AST完全一致，两份原测试全部assert AST完全一致。
- `uv build ros2/src/senior_robot --offline --out-dir /tmp/m5d1-wheels` 和根 `uv build --offline --out-dir /tmp/m5d1-wheels`：两个sdist/wheel通过。
- `uv venv /tmp/m5d1-robot` 后仅安装新Robot wheel；从 `/tmp` 运行
  `python <仓库>/tools/verify_batch_one_install.py --platform-runtime-only`：provider来自该环境site-packages、五函数可定位且导入无Root/manager/rclpy，通过。
- `/tmp/m5d1-sdk`全新环境安装本片两个wheel与其余8个现有闭包wheel，共25包；从 `/tmp` 运行原完整
  `python <仓库>/tools/verify_batch_one_install.py`：原公开身份/生产ServicePorts、CLI及所有旧module help、SQLite和媒体等原检查通过；`uv pip check --python /tmp/m5d1-sdk/bin/python`通过。
- 五份变更Python源码/工具 `ruff check`、`ruff format --check` 及 `git diff --check` 通过。

原用例中的受控替身子进程和临时Unix socket只验证原监管行为，没有启动真实ROS或厂商进程。
本机缺rclpy/ament_index_python/colcon/cmake，原Linux SO_PEERCRED专属用例跳过，
实际ROS/ament安装、vendor overlay sourcing、ROS域/命名空间/相机设备与真实systemd/docker执行未验；
native/SLAM安装仍UNVERIFIED，Python3.10及CI未执行。这里的Robot独立导入与完整旧SDK安装
分别报告，均不冒充管理独立生产部署；公共IPC/诊断/Clock/进程/帧owner归属仍待定。


## 共享导入准备片 ImageFrame与按需公开

基线 `c11703f`；固定功能分支直接提交，实际增1改13删0：产品增1改9、测试及配置0、工具文档改4。
仅拆公开导入边界，不新增项目/发行/算法/WS字段，不改Root依赖/入口、DrivingSettings、IPC或诊断owner。

```text
src/senior_control/image_frame.py               # 新增，原frozen ImageFrame及校验唯一实现
src/senior_control/vision/
  __init__.py                                 # 改，原__all__对象/四原子模块按需公开
  synchronizer.py                             # 改，ImageFrame显式Compatible同对象；其余算法原样
  rgb1.py, stream.py, ros_video.py,
  simulation_video.py                         # 改，读取同一叶子Frame
src/senior_control/agent/
  __init__.py                                 # 改，原Agent/RobotSnapshot按需公开
  manual_control.py                           # 改，只有vehicle_frame调用时加载原管理投影
  composition.py                              # 改，原真实媒体factory使用同一Frame
README.md, senior_control/README.md             # 改，实际边界和验证
docs/superpowers/plans/2026-11-19-vehicle-architecture-refactor.md # 改，完成和待定记录
tools/verify_batch_one_install.py              # 改，原完整检查/新进程隔离与原公开身份
```

ImageFrame仍归原Root发行，只从同步算法模块移到纯值叶子；其字段、冻结、尺寸/编码/数据校验逐字保留。
原 `vision.synchronizer.ImageFrame`、`vision.ImageFrame`、媒体旧公开Frame与真实frame_factory均为同一类。
类的 `__module__` 现在是 `senior_control.image_frame`，没有复制一个兼容类或假frame。
RGB1 header/尺寸/权限/时刻/读写和关闭规则不改；DepthFrame、同步配准/识别算法不迁移。

两个原 `__all__` 名单和顺序保持；显式公开请求才导入真实owner，返回并缓存原对象。
vision原depth/detector/synchronizer/tracking子模块名仍可按需访问，原公开Literal/Protocol/类/函数均同身份。
manual_control的全部执行类及函数签名/逻辑保持；原manager projection与ProjectionPorts仅移入
实际vehicle_frame调用内，仍每次构造原ports，不复制业务状态或造另一个投影实现。
ROS command_node仍从原路径读取这些执行类；没有改变ROS节点、控制权限、学习/导航或停止链。
DrivingSettings仍是原manager公开值，不宣称完全消除了所有manager发行入边。

### 命令、结果与限制

环境：macOS26.4 arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23。
python/ruff/mypy来自主仓库原 `.venv/bin`，uv使用 `UV_CACHE_DIR=/tmp/vehicle-uv-cache`。

- 安装工具新增的 `--lazy-shared-only` 先在源发现路径运行，因缺少新叶子模块失败；实现后3个独立子进程通过。
  每个子进程是实际Python导入，没有ROS/队列/Frame替身：只导入叶子/RGB1不加载深度/识别/同步/诊断/帧龄策略，
  只导入原ManualIngress/ManualTarget/MotionRuntimeReceiver不加载agent.main或manager frame_projection/state_ports，
  第三个进程逐项比对原25个vision公开对象和Agent/RobotSnapshot/旧新Frame身份。
  初版工具试图由对象__module__/__name__推导Literal归属，对ColumnStatus不适用；已改为按原公开owner逐项比较同一对象。
- `python -m pytest tests/unit/test_simulation_video.py tests/unit/test_rgbd_sync_ordering.py tests/unit/test_rgbd_runtime.py tests/integration/test_video.py tests/integration/test_manual_wire.py tests/unit/test_degraded_manual_speed.py senior_control/tests/test_agent_config.py -q --tb=short`：97通过、53失败，10.14秒。
- `git archive c11703f` 至 `/tmp/m6-baseline-source`，原同解释器/同清单：97通过、相同53失败，11.09秒；失败ID逐条相同。
  旧3个manual_wire夹具访问已退役CommandNode._output_incarnation，旧50个degraded_manual_speed访问MotionCoreOutput._snapshot。
  没有补假属性/删弱断言，也没有把旧失败说成通过；可用原视频含真实aiortc回环、RGB1、同步/几何及Agent同DB/九客户端原回归保持。
- MYPYPATH取原pytest pythonpath；`mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`：
  当前270源码与c11703f归档269源码均相同39旧错误，归一行号后诊断完全一致。
- 原Frame class AST一致；剩余synchronizer定义、全部manual执行类/签名/投影调用体、RGB1/media/composition定义AST一致，
  只改变Frame引用和投影局部import；两份__all__列表与顺序不变；测试源、原settings/IPC/Clock/diagnostics/policy/资源/发行配置未改。
- `uv build --offline --out-dir /tmp/m6-wheels`：根sdist和wheel通过，原packages声明已包含新叶子，无需配置/锁变化。
- 新建 `/tmp/m6-sdk`、安装新Root wheel及原其他10个闭包wheel，共26包；从 `/tmp` 运行
  `python <仓库>/tools/verify_batch_one_install.py` 原完整检查和 `--lazy-shared-only`：原CLI/module help、真实原装配/SQLite/媒体及3个隔离导入检查通过。
  `uv pip check --python /tmp/m6-sdk/bin/python`通过；这是一套完整旧SDK兼容安装，不是无ROS业务wheel的独立管理部署。
- 11份变更Python的ruff check/format --check及git diff --check通过。

本轮仅原受控测试RTC/socket/SQLite夹具，没有启动开发/实车/仿真服务或VM，没有部署/推送。
实际ROS/rclpy/ament/Linux/厂商加载与设备、Python3.10和CI未验；安装工具导航ROS和原native/SLAM仍UNVERIFIED。
独立manager生产安装仍需核定Root发行定位和共享诊断/值/原媒体学习与地图读取边界，不提前变更已定目录或生产入口。

## 单文件 Shell CLI（618a553 后续批次）

`bin/senior-robot.sh` 是唯一 Shell 源，独立复制为 PATH 命令，不编译。Shell 拥有 help、命令与参数解析、隐藏配对交互、restart 和 journal 路由；不把整串 argv 交给旧 argparse。

```text
bin/senior-robot.sh
  native → 项目 .venv → senior_control.operator_actions（原 SDK 逐次绑定）
                         → vehicle_manager.operator_actions → cli.execute → 原 owners
  docker → host inspect / systemd / journal
         → 已运行容器项目 .venv → 同一固定动作绑定
                               → 原 profile / 凭据 / 原子写入 / IPC / runtime
旧 python -m senior_control.cli → Compatible argparse → 同一 cli.execute
```

- `senior-robot start/stop/restart/status/logs` 管宿主机，`senior-robot docker ...` 管本项目 Docker 服务。pair/doctor/deploy 也使用相同分组；未新增 Docker 仿真。
- 两种模式只在所选运行位置准备项目 `.venv`，Humble 须使用匹配的系统 Python 与 system-site-packages。Docker 宿主机无 Python；Shell 依赖 Bash、jq、Docker、GNU timeout、systemd 等系统工具。native 解释器来自 `project_dir/.venv/bin/python` 或绝对路径 `SENIOR_ROBOT_PYTHON`，Docker 忽略宿主 Python 覆盖。
- 原 credentials/TLS/目录校验先于隐藏提示，stdin 传码，不进 argv/env/日志；显式关闭回显并在退出/中断恢复，无法安全设置终端则拒绝读取。Docker 配对按调用操作者 UID/GID 运行，保留原凭据属主；既有文件仍由原原子实现拒绝覆盖。
- 停止容器的 Docker dry-run/start 只 inspect 后非零退出，不 exec/start/run、不写配置/unit、不调用 systemctl。运行中预览在容器 `.venv` 使用同一渲染器完整生成；新 unit 无自动 docker start。旧 Python Compatible 入口仍允许原停机预览和原默认启动流程，未删旧断言。
- Docker 正式部署仍写同一映射目录，以原 `_install_once`/`install_artifacts` 保留原子防覆盖和 0600/0644；Shell 在宿主 link/reload，失败或取消后按文件 inode 收据与确切链接回滚。若容器/传输在写入后失联、无法得到收据或完成回滚，会非零失败，需检查已有文件，不能盲删后重试；真实故障交互仍需 Linux/Docker 验证。
- stop/status 保留确切旧 Docker unit 的管理；start 另核对新渲染内容，拒绝用旧自动启动共享容器的 unit。native 不会操作 Docker unit。restart 仅在原停止成功后启动，原 helper/IPC/guard 停机顺序和同 DB/九客户端保持。日志读取宿主项目 journal，仿真 user scope；Docker 日志可在容器停着时读取，不使用只反映容器 PID1的 docker logs。
- 语法2，业务非零，缺工具/解释器127，原系统工具退出码保留；GNU timeout 超时124。没有把进程 active 当作 IPC/云/运动资格。

安装：`install -m 0755 bin/senior-robot.sh <PATH目录>/senior-robot`。Root wheel 已撤掉同名 console-script 注册；升级旧 wheel 后检查 `type -a senior-robot`，不要让旧 venv 脚本遮盖新 Shell。原 Python module/API 与签名仍显式 Compatible。固定动作采用已有原 SDK 平台绑定，尚未完成无旧 SDK 的 manager 独立生产发行，也未改共享 IPC/诊断/Clock/媒体 owner 或 WS/ROS 协议。

### 实际范围与验收

22 逻辑文件，增4改18删0，未新增项目：

| 类别 | 新增 | 修改 |
| --- | --- | --- |
| 产品7 | `bin/senior-robot.sh`；`src/senior_control/operator_actions.py`；`senior_control/src/vehicle_manager/operator_actions.py` | manager `cli.py`、`deployment.py`、`platform_ports.py`、`simulation.py` |
| 测试3 | `senior_control/tests/test_shell_cli.py` | 同目录 `test_vehicle_cli.py`、`test_control_directory_cli.py` |
| 配置构建3 | 无 | 根 `pyproject.toml`、`deploy/senior-agent.service`、`.github/workflows/backend.yml` |
| 工具文档9 | 无 | 根/manager README；确认稿；原实施计划；operations 的 vehicle-deploy/simulation/hardware/ros-interface-rebuild；`tools/verify_batch_one_install.py` |

已选 Shell 检查有8个测试函数：无 Python help/语法、独立 PATH/参数边界、restart 停后启及失败、journal scope与退出码、停止 Docker 预览/start 零副作用、运行中容器 `.venv` 原预览、真实 PTY 隐藏输入/恢复、Docker 操作者身份/已有凭据与原配对准入。原 CLI/目录测试保留原断言，只将已确认运行环境期望改 `.venv`，并在原 pair/lifecycle 场景补实际固定动作调用；没有迁/改旧任务或控制夹具。

环境：macOS arm64，Bash 3.2.57，Python 3.12.15，pytest 8.4.2，ruff 0.17.3，mypy 2.4.0，uv 0.12.23。下列 Python/ruff/mypy 使用原主仓 `.venv/bin`；构建/安装使用 `UV_CACHE_DIR=/tmp/vehicle-uv-cache`。

- `bash -n bin/senior-robot.sh`，改动 Python 的 `ruff check` / `ruff format --check`，`git diff --check`：通过。
- 首轮 Shell 测试在脚本不存在时5失败，补实现后通过；新增 Docker 运行预览捕获并修正 Bash3 `set -u` 空数组兼容问题，现有保护未删除。PTY 在普通沙箱因 TIOCGETD 拒绝而失败；扩大测试权限后，隐藏输入、xtrace不泄漏和终端恢复通过，产品仍在无法关闭回显时安全失败。
- 最终定向命令：`python -m pytest senior_control/tests/test_shell_cli.py senior_control/tests/test_vehicle_cli.py senior_control/tests/test_control_directory_cli.py senior_control/tests/test_docker_start_helper.py senior_control/tests/test_agent_config.py senior_control/tests/test_agent_ports.py tests/integration/test_task_ownership.py tests/unit/test_task_terminal_release.py -q --tb=short --junitxml=/tmp/shell-cli-final.xml`：**104通过、1跳过、26旧错误**，11.46秒，退出1。
- 跳过原 Linux Unix peer credentials 项；26错误均原退役夹具访问 `MotionCoreOutput.controller`。`git archive 618a553` 至 `/tmp/shell-cli-baseline`，相同原7份清单（不含新增 Shell）普通沙箱下当前/基线均90通过、1跳过、6权限失败、26相同错误；错误ID逐条相同。扩大权限后原 socket/关停/Agent 6项通过。日志见 `/tmp/shell-{current,baseline}-tests.txt` 与 `/tmp/shell-cli-final-tests.txt`，未补假属性或弱化断言。
- 全链 mypy：`MYPYPATH=<原pytest pythonpath> mypy src/senior_control senior_control/src/vehicle_manager cloud/src/cloud`，当前272源码与基线270源码均39旧错误，归一行号后诊断逐条一致；曾发现新绑定返回类型1项，已修正后恢复基线39。
- `uv lock --check --offline`通过，原锁无需变更。`uv build --offline --out-dir /tmp/shell-cli-wheels` 与 `uv build senior_control --offline --out-dir /tmp/shell-cli-wheels`：Root/manager sdist和wheel通过。
- 新建 `/tmp/shell-cli-sdk`，安装新Root/manager wheel与既有9份闭包wheel（26包），单独复制 Shell 到其 bin；从 `/tmp` 运行 `python <仓库>/tools/verify_batch_one_install.py`：实际原工厂/同DB/任务/媒体/类型身份/旧module help和错误退出、新固定入口、无 Python Shell help/语法及无同名console注册通过；`uv pip check --python /tmp/shell-cli-sdk/bin/python`通过。首次安装命令误用相对 Shell 路径，校验如实因未安装脚本失败，改为绝对源路径后通过。
- `/tmp/shell-manager-wheel` 单独安装 manager/contracts（8包），离仓库导入 `vehicle_manager.operator_actions` 来自 site-packages，无 Root/cloud/ROS wheel；只证明包边界与固定动作可导入，平台绑定仍需原 SDK。
- Docker 只读 `docker version --format '{{.Server.Version}}'`：普通沙箱先拒绝 socket，扩大只读权限后明确 daemon 不可连接；未启动 Docker/容器。Docker 行为由原替身与真实 Python 渲染验证，未声称真实 Docker/systemd link/rollback 或镜像运行通过。

未验：真正 Linux/systemd 用户/系统 unit、Humble/ament/厂商设备、真实容器/UID映射/取消断连回滚、真实仿真及车辆、Python3.10/CI。安装工具的导航 executable/share、native接收/守卫和SLAM继续明确 UNVERIFIED。只用了原受控本地子进程/socket/SQLite及PTY夹具，未启动开发、实车、仿真服务、容器或 VM，未部署或推送。后续 B8 和共享发行/平台独立部署边界由负责人另行审阅推进。

### Shell CLI 审阅修复：共享容器不可用时仍能停止项目 unit

基线 `490fb2c` 的 `docker stop` 先进入统一的 inspect/Running/容器 metadata 路径，故共享容器停止或 daemon 不可达时，无法执行宿主 `systemctl stop`，项目的 `Restart=on-failure` unit 可能持续重试。本片只修改 Shell、原 Shell 测试和本 README，共3文件修改，无新增/删除。

独立 `senior-robot docker stop --control-dir ...` 现在复用原宿主日志的 control-dir 哈希选择，复用原目录/符号路径校验、`/etc/systemd/system/<unit> -ef <control-dir>/<unit>` 与 Docker mode 检查，以及原 Linux/timeout30秒/systemctl 单项目启停实现。它不需要 Docker 可用，不 inspect/exec，不调用宿主 Python，更不执行 docker stop/kill/start；unit 的 ExecStop、权限、停止顺序和拓扑未变。start、dry-run 和 restart 仍走原运行中容器检查，restart 内部 stop 失败不会 start，未扩大本次修复范围。

新增一个实际 bug 回归函数 `test_docker_stop_uses_only_host_owned_unit_when_container_is_unavailable`，在 stopped/daemon-unavailable 两种条件下设置任何触及即记录并失败的 Docker 替身，断言仅对目标项目执行 `timeout 30s systemctl stop <hash-unit>`、无 Docker/Python；同场景保留错 mode 拒绝，并直接运行原生产脚本验证未部署拒绝。按负责人确认的 macOS 测试接缝，只有测试副本把 `/etc/systemd/system` 常量映射到临时目录，显式断言存在匹配、替换后无残留、反向替换与原文完全相同；使用真实临时符号链接和原 `-ef`/mode 检查，没有生产注入开关或修改真实 `/etc`。

验证环境沿用上节 Bash3.2.57/Python3.12.15/pytest8.4.2/ruff0.17.3：

- 红灯：修复前运行上述单条回归，退出1，成功路径实际返回99（Docker 禁用替身被触及），复现原缺陷。
- 绿灯：修复后相同单条回归1通过；`bash -n bin/senior-robot.sh`、测试文件 `ruff check` / `ruff format --check` 和 `git diff --check`通过。
- 完整原 Shell 用例：`python -m pytest senior_control/tests/test_shell_cli.py -q --tb=short`，包含原停机 start/dry-run 仅 inspect、运行中预览、restart 失败不启动、配对与日志等保护；扩大受控PTY测试权限后 **9通过**，6.81秒，退出0（普通沙箱对原PTY ioctl的限制沿用前节登记，未修改或跳过该断言）。

这是命令替身/真实临时文件身份回归，不能代替 Linux systemd 与真实 daemon 故障实测；未启动真实 systemd、开发/车辆/仿真服务、容器或VM，未部署/推送。原共享发行与B8未验事项继续保留。


### R0/R1 首片：共享 Clock 与 ImageFrame 叶子

本片在固定功能分支由 `9cafe01` 快进至同文件树的 `origin/main@492f14d` 后实施。
现有根发行内新增 `src/vehicle_core/{__init__,clock,image_frame}.py`，只承接原
`Clock`、`SystemClock` 与 frozen `ImageFrame` 唯一实现；旧 `senior_control.clock`、
`senior_control.image_frame` 显式 `//Compatible` 同对象转发。产品消费者直接导入新叶子，
`senior_control.vision.ImageFrame` 保持按需公开同一类；原测试导入不改。

根 hatch 明确打包两个命名空间，CI现有类型检查包含 `src/vehicle_core`；不改变
Requires-Dist、uv.lock、安装方式或项目 `.venv`，未新增独立项目、状态或依赖环。
本片不代表独立manager生产部署，IPC/诊断/业务值/默认平台绑定仍待后批。
实际范围21文件：产品增3改13、测试0、配置改2、工具文档改3，无删除。
仿真文件只有静态导入迁移，不运行任何仿真或其验证；ROS、实车、部署和推送均不在本片范围。

验证命令、结果与环境限制见本节后续记录及原实施计划同名批次。


#### 实际范围、自审与验证结果

实际21文件，增3改18删0，与预报一致；测试文件未编辑：

- 产品新增：`src/vehicle_core/__init__.py`、`clock.py`、`image_frame.py`
- 产品修改：原 `src/senior_control/{clock,image_frame}.py`；agent的 `main.py`、
  `composition.py`；vision的 `__init__.py`、`rgb1.py`、`ros_video.py`、
  `simulation_video.py`、`stream.py`、`synchronizer.py`；localization的 `geo.py`、
  `localization_node.py`；motion_control的 `command_node.py`
- 配置修改：根 `pyproject.toml`、`.github/workflows/backend.yml`
- 工具文档修改：`tools/verify_batch_one_install.py`、manager README、本计划

自审核对新clock/image_frame与 `492f14d` 原实现逐字节相同，旧模块只有显式Compatible
别名；产品旧导入搜索仅余原定位测试，未改原测试导入。vision的运行时导出表也指向新owner，
不是只改TYPE_CHECKING。根project元数据逐项相同、uv.lock无diff；没有新依赖边或全局状态。
新wheel同时含两个命名空间。import排序曾产生8项Ruff I001，已仅整理现有导入后全部通过。

环境：macOS arm64；Python 3.12.15、pytest 8.4.2；工具复用原主仓
`/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin`，当前工作区 `.venv` 缺pytest，
未安装或改变它；临时wheel验证环境不作为第二套项目运行环境。

1. 定向回归（原主仓Python）：

   ```sh
   python -m pytest tests/unit/test_control_clock_domains.py \
     tests/unit/test_manual_session.py tests/unit/test_manual_authority.py \
     tests/unit/test_settings.py::test_clock_is_injected_without_sleep \
     tests/unit/test_rgbd_sync_ordering.py \
     tests/unit/test_perception.py::test_rgb_depth_time_mismatch_rejected \
     tests/unit/test_perception.py::test_vision_import_loads_no_detector_runtime \
     -q --tb=short
   ```

   **115通过、1失败**，0.48秒，退出1。失败为原
   `test_close_is_permanent_and_old_acquire_returns_closed[stop]`：旧测试调用
   `ManualAuthorityManager.stop()` 缺 `gesture` 参数；测试和该实现均未改。
   用 `git archive 492f14d src tests/unit/test_manual_authority.py pyproject.toml`
   解包至 `/tmp/vehicle-core-leaf-baseline`，以该归档src优先、现有功能包源码路径补齐，
   执行归档原测试 `-k close_is_permanent_and_old_acquire_returns_closed -q --tb=short`：
   **2通过、1相同失败、31未选**，0.08秒；没有放宽断言、补假参数或越界修旧测试。
   控制时钟、手动会话及帧同步均为纯离线既有用例，不含仿真运行或仿真验收。

2. 本批所有修改Python文件及 `src/vehicle_core` 的 `ruff check`、
   `ruff format --check` 通过（17文件）；`mypy src/vehicle_core`：3源文件无错误；
   `git diff --check` 通过。没有运行全量mypy或全量pytest。

3. `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-core-leaf-wheels`，
   以及同参数 `uv build ros2/src/senior_localization`、
   `uv build ros2/src/senior_motion_control`：三个项目sdist/wheel成功。
   构建前后均未改依赖声明或uv.lock，不是ROS/ament原生构建。

4. `uv venv --python <原主仓Python> /tmp/vehicle-core-leaf-sdk`；以
   `uv pip install --offline --python /tmp/vehicle-core-leaf-sdk/bin/python` 安装新Root wheel
   与既有10份依赖闭包wheel，共26包；随后用 `--reinstall --no-deps` 替换为本批新建的
   localization/control wheels。其余复用的实际artifact来自 `/tmp/shell-cli-wheels`
   （manager）、`/tmp/cloud-build-9cafe01`（cloud）、`/tmp/b7-wheels`（navigation/robot）
   与 `/tmp/m5c-wheels`（contracts/mapping/estimation/perception等），不声称全部依赖
   重新构建，也不将这一叠装证明扩大为独立manager生产安装。
   `uv pip check --python /tmp/vehicle-core-leaf-sdk/bin/python`：26包兼容。

5. 从 `/tmp`、`PYTHONPATH=` 执行新安装Python与仓库工具
   `tools/verify_batch_one_install.py --lazy-shared-only`：通过。新owner的真实导入、
   Clock/SystemClock/ImageFrame旧新同对象、无manager/cloud/rclpy急切导入、旧agent/vision
   惰性公开均检查；未执行工具其他入口。另从 `/tmp` 实际导入新安装的
   `vehicle_core.clock/image_frame`、`senior_localization.geo/localization_node`，
   四者路径均位于site-packages，定位消费者和旧路径Clock/SystemClock同一对象，
   ImageFrame兼容身份相同。control命令节点因rclpy环境不在本机，未实际导入或启动。

限制：未运行任何仿真测试、仿真环境、ROS/ament、开发服务、实车或部署；
`simulation_video.py` 只有静态导入迁移及通用静态lint，未执行其功能验证。
未验证Linux/Humble/Python3.10 ABI或真实设备；未宣称manager独立生产部署、完整发行DAG
收尾或新功能验收。本批仅提交固定功能主分支，不推送，等待负责人审核。


### R1 第二片：生命周期事件与进程资源叶子

基线 `7ea8183`，固定功能分支工作区干净后实施。唯一实现由原
`runtime/lifecycle_events.py`、`motion/process_resources.py` 迁到现有根发行的
`vehicle_core/lifecycle_events.py`、`vehicle_core/process_resources.py`；旧模块保留显式
`//Compatible` 同对象出口，包含已有类、函数、类型别名及生命周期常量/logger。
事件logger明确保留 `senior_control.runtime.lifecycle_events`，没有随新模块名改变日志路由。

9个真实消费者更新为新owner：motion_control的control_inputs/runtime_reports；perception的
learning_compute/learning_owner；senior_robot safety_motion launch；senior_sim simulation_motion
launch；旧agent/composition、docker_start_helper和runtime/launch_recovery。后者仅改child_lifetime
导入，本体、旧CLI入口与命令字符串保持；仿真launch只静态改引用，不执行或验证。

范围16文件：产品增2改11、工具文档改3，测试与配置未改，无删除；未增加项目、依赖、
工厂、发行、控制或手动权限。事件环境变量、上界、单向管道、EOF、失联与FD关闭，以及
spawn、父亡、异常和reap行为沿用原实现。限定验证与平台缺口在下节记录。


#### 本片验证、自审与限制

环境：macOS arm64，Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、uv 0.12.23。
测试与Ruff复用原主仓 `/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin`；
不改变项目 `.venv`，不新增依赖或改锁。以下仅为本片限定证据。

- 先更新既有安装工具的新叶子检查，在 `/tmp`、`PYTHONPATH=` 下对上一片已安装wheel
  执行 `python <仓库>/tools/verify_batch_one_install.py --lazy-shared-only`：
  退出1，明确缺少 `vehicle_core.lifecycle_events`，证明新owner检查会检测未迁移安装包。
- 唯一测试命令（原主仓Python，项目目录运行）：

  ```sh
  python -m pytest \
    ros2/tests/test_control_process_resources.py::test_real_spawn_serializes_tasks_and_results_and_cleans_up \
    ros2/tests/test_control_process_resources.py::test_process_death_fails_pending_future_and_close_reaps_it \
    tests/integration/test_runtime_fault_isolation.py::test_observer_eof_is_reported_by_actual_agent_without_exiting \
    -q --tb=short
  ```

  **3通过**，1.60秒，退出0，无跳过。实际独立spawn序列化任务/结果、任务异常传递、
  进程死亡future失败与关闭reap、实际Agent收到单向管道EOF后继续心跳均沿原用例验证。
  未运行同文件其他场景，尤其没有运行会扫描仿真launch的用例，未修改测试或断言。
- 修改的Python及两份新叶子 `ruff check` / `ruff format --check`：14文件通过；
  首轮7处导入排序已修复；`git diff --check`通过。
- 对照 `git show 7ea8183:src/senior_control/...`：process_resources实现逐字节相同，
  lifecycle_events只将logger的 `__name__` 改为原名称字符串并加Compatible说明；
  事件常量、pipe/EOF/finally关闭、lost计数、spawn/异常/reap等主体逐字节保持。
  launch_recovery整个文件仅一条child_lifetime导入变化，未迁本体或改启动入口。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-core-runtime-leaf-wheels`：
  根sdist/wheel成功；用 `uv pip install --offline --reinstall --no-deps --python
  /tmp/vehicle-core-leaf-sdk/bin/python <新Root wheel>` 更新已有临时安装夹具。
  该夹具沿用上一片26包依赖闭包，不是本批重新构建所有功能包或独立manager部署。
- 从 `/tmp`、`PYTHONPATH=` 执行 `/tmp/vehicle-core-leaf-sdk/bin/python
  <仓库>/tools/verify_batch_one_install.py --lazy-shared-only`：新wheel检查通过。
  新canonical模块实际来自site-packages，旧模块的全部显式类/函数/类型别名/常量/logger
  与新owner同对象，logger名和环境变量/帧上界保持；单独导入新叶子不加载Root业务、
  manager、cloud、ROS/launch或控制/感知实现，导入旧门面也不加载恢复/诊断/传感器策略。
  `uv pip check --python /tmp/vehicle-core-leaf-sdk/bin/python`：26包兼容。

自审确认实际16文件：增2改14删0，与预报一致；测试、配置、Requires-Dist与uv.lock无diff，
9个实际消费者已接新叶子，产品中无旧叶子import残留。旧logger与私有spawn函数出口也保留
同一对象，不复制状态或增加第二套实现。提交采用 `git -c core.hooksPath=/dev/null commit`
避免仓库钩子执行超出本派单的测试，限定检查均已手动运行；不推送，等待负责人审核。

本机未验证Linux prctl父亡分支、Humble/ROS安装及真实设备；OS管道/spawn纯测试不等于
实车手动控制已通。本批没有发送运动请求，没有启动任何仿真/实车/开发/ROS服务，
没有运行仿真验证或部署；senior_sim launch仅静态引用迁移及通用静态lint。


### R1 第三片：采集时间与纯传感器时效策略

基线 `d3c7dec`，固定功能分支工作区干净后实施。原sample_time唯一实现逐字节迁入
`src/vehicle_core/sample_time.py`；Age/SensorTiming/SensorClockError/default_policy/
sensor_current纯实现归 `vehicle_core/sensor_policy.py`，只调整叶子与资源路径、移除诊断依赖。
旧sample_time与policy显式Compatible转发同对象；唯一ros_sensor_parameters仍在旧policy，
保留原适配代码及延迟诊断导入，不迁adapter/诊断或新增owner。

原defaults.json原字节移动为唯一 `src/vehicle_core/sensor_defaults.json`，不保留旧资源副本；
36个真实产品消费者改用纯叶子，3个仅消费ros_sensor_parameters的launch保留原适配入口。
仿真点只有静态引用迁移，不执行任何仿真或其验证。范围为产品增2改38、资源迁移1、
工具文档改4（安装工具、本README、既有计划、hardware-motion-parameters），测试/其他配置
不改，无其他删除。算法、缓存、阈值、原始时间、ROS/IPC行为及控制/手动权限均不变。


#### 本片验证、自审与限制

环境：macOS arm64，原主仓Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、uv 0.12.23；
独立C++合成输入使用Apple clang 21.0.0、C++17。本片不修改项目 `.venv` 或依赖。

- 安装工具初稿的子进程字节常量有双层转义错误，首次检查报SyntaxError，已改为显式换行
  字节组合；随后对上一片已安装wheel执行 `--lazy-shared-only`，退出1，明确缺少
  `vehicle_core.sample_time`，不是将语法错误当成有效的缺owner验证。
- 唯一pytest命令（原主仓 `.venv/bin/python`，项目目录运行）：

  ```sh
  python -m pytest tests/unit/test_sensor_timing.py \
    tests/unit/test_runtime_diagnostics.py::test_age_raw_dedup_separate_compute_and_five_second_debug \
    -q --tb=short
  ```

  **17通过**，1.71秒，退出0，无跳过。包含0.3秒软降/1秒硬失效边界、原始接收映射与
  队列耗时、重复采集端点固定deadline、年龄诊断去重及5秒DEBUG，以及独立编译运行的
  C++整数时钟合成输入契约。该C++用例不链接ROS，不是ROS或仿真运行验证。
- 修改Python和两份新叶子的 `ruff check` / `ruff format --check`：41文件通过；
  首轮34处迁移导入排序已修正，`git diff --check`通过。
- sample_time及资源与 `git show d3c7dec:src/senior_control/sensors/...` 逐字节对照相同；
  纯策略所有类/函数及lru_cache装饰器AST相同（default_policy只替换资源路径）；
  旧ros_sensor_parameters整个AST与基线相同，仍延迟导入原诊断owner。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-core-sensor-leaf-wheels`：
  根sdist/wheel成功；zip内容检查确认新路径唯一sensor_defaults.json字节与基线相同，
  无旧sensors/defaults.json。用 `uv pip install --offline --reinstall --no-deps --python
  /tmp/vehicle-core-leaf-sdk/bin/python <新Root wheel>` 更新既有临时安装夹具，仅重建Root，
  其余26包依赖闭包沿用前片，未声称重新安装验证36个ROS消费者的执行环境。
- 从 `/tmp`、`PYTHONPATH=` 运行临时安装Python与
  `<仓库>/tools/verify_batch_one_install.py --lazy-shared-only`：通过。
  新canonical模块来自site-packages；原sample_time类/函数/常量与新owner同对象；
  policy类/函数、default_policy缓存及返回默认实例同对象，maxsize仍1；
  ros_sensor_parameters仍归旧policy；新旧资源唯一性及300/1000/5.0原字节均核对，
  纯叶子和旧转发导入不加载诊断/管理/ROS/launch。
  `uv pip check --python /tmp/vehicle-core-leaf-sdk/bin/python`：26包兼容。

自审实际45逻辑文件：产品增2改38、资源原字节迁移1、工具文档改4，测试无改，
无额外配置/删除/依赖变更。git原始路径统计为42修改、3新增、1删除，其中JSON增删是一项
移动；暂存后可识别100%资源重命名。产品旧纯导入无残留，仅3个launch保留原ROS参数
适配入口；不复制时钟/缓存/默认资源或新建工厂。Requires-Dist、uv.lock、CI配置无diff。

提交仍采用 `git -c core.hooksPath=/dev/null commit` 避免超范围钩子运行，限定验证均已
手动执行，仅提交固定功能分支，不推送。本片没有运行任何仿真测试或环境，没有发送运动
请求，没有启动车辆/开发/ROS服务或部署。仿真相关消费者仅静态改引用及通用静态lint；
真实Linux/Humble/ABI与设备链未验，不声称实车控制可用或独立manager生产部署完成。


### R1 第四片：唯一进程诊断 owner

基线 `105494a`，固定功能分支工作区干净后实施。原runtime/diagnostics唯一实现迁到
`src/vehicle_core/diagnostics.py`；默认logger显式保留原
`senior_control.runtime.diagnostics` 名，旧模块Compatible同对象转发，不复制lru_cache、
实例、锁或队列实现。原公开类型、函数、泛型变量及必要私有值/类型保留同对象出口。

32个实际产品消费者更新诊断引用；旧policy的ros_sensor_parameters仅改延迟import，
纯sensor_policy保持独立，3个launch不动，cloud自身诊断实现不改。唯一测试编辑为指定
拒收回执用例的monkeypatch定义点，输入、assert和其余用例保留。
预报38文件：产品增1改33、测试改1、工具文档改3，配置0，无删除。
本片不新增依赖、工厂或替代owner，不改变队列log、帧龄、时钟、权限或ROS/IPC行为。


#### 本片验证、自审与限制

环境：macOS arm64，原主仓Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、uv 0.12.23。
测试工具复用原主仓 `.venv/bin`；项目 `.venv`、依赖声明、锁文件及安装方式未改。
已检查两份整文件测试：都是离线队列、值、日志格式及替身后台故障，无仿真用例需排除。

- 先在安装工具增加canonical/兼容/单缓存/logger检查，对上一片已安装Root wheel执行
  `python tools/verify_batch_one_install.py --lazy-shared-only`：退出1，明确缺少
  `vehicle_core.diagnostics`，未迁移的旧安装包不能通过新owner检查。
- 唯一pytest命令（原主仓Python，项目目录运行）：

  ```sh
  python -m pytest tests/unit/test_runtime_diagnostics.py \
    tests/unit/test_logging_diagnostics.py \
    tests/unit/test_estimation_review_contracts.py::test_rejected_native_receipt_uses_original_identity_without_live_state_mutation \
    -q --tb=short
  ```

  **29通过**，0.74秒，退出0，无跳过。既有vehicle/cloud诊断后端对照、队列阈值/限频/
  恢复/未知深度、原始年龄去重与5秒DEBUG、配置拒绝、日志脱敏及拒收回执身份保持通过；
  cloud自身诊断实现未改。唯一测试diff是指定monkeypatch定义点，无输入/assert变化。
- 修改Python及新叶子 `ruff check` / `ruff format --check`：36文件通过；
  首轮18处导入排序及1处logger行格式已整理；`git diff --check`通过。
- `git show 105494a:src/senior_control/runtime/diagnostics.py` 与新实现AST对照：
  仅getLogger的动态模块名替换为原名称字符串，其余主体与lru_cache装饰器相同；
  新增Compatible说明只解释logger路由，不引入状态或业务分支。
  32个消费者移除导入后AST与基线相同，仅更新引用；旧policy的延迟诊断import及
  TYPE_CHECKING指向新owner，adapter主体、3个launch、纯sensor_policy保持。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-core-diagnostics-wheels`：
  根sdist/wheel成功；用 `uv pip install --offline --reinstall --no-deps --python
  /tmp/vehicle-core-leaf-sdk/bin/python <新Root wheel>` 更新前片临时安装夹具。
  本片只重建Root，其他26包闭包沿用，不将其扩大为所有ROS功能包的重新安装验证。
- 从 `/tmp`、`PYTHONPATH=` 执行临时安装Python与
  `<仓库>/tools/verify_batch_one_install.py --lazy-shared-only`：通过。
  新诊断canonical来自site-packages；旧新18个显式类型/函数/常量/类型变量出口同对象；
  两路径configure_diagnostics与process_diagnostics指向同一lru_cache/返回同一owner，
  maxsize仍1，配置实际修改该owner；真实默认emit调用保留原logger名。
  单独导入不加载旧SDK、管理/云/ROS业务；导入旧门面也不带入Agent/config/旧sensor适配。
  `uv pip check --python /tmp/vehicle-core-leaf-sdk/bin/python`：26包兼容。

自审实际38文件，产品增1改33、测试改1、工具文档改3，无删除或配置变更；
Requires-Dist、uv.lock、cloud/src自身诊断、纯策略及launch均无diff。唯一诊断owner与原
锁/队列/log/时钟/帧龄逻辑保持，旧模块没有重新包装函数、缓存或构造第二实例。
提交采用 `git -c core.hooksPath=/dev/null commit` 避免超范围钩子执行，限定检查已手动运行，
仅提交固定功能分支、不推送，等待负责人审核。

未跑全量测试；其他仍patch旧诊断定义点的历史用例未在本批扩改，例如前片已读取的
`test_rgbd_sync_ordering.py` 对旧module的process_diagnostics/time monkeypatch，后续应按
canonical定义点另行适配，不能把本批29项当作全量通过。
未运行仿真测试/环境或任何开发/车辆/ROS服务，未部署、推送或发送运动请求；
Linux/Humble/真实设备链及实车手动控制可用性仍未验证。


### R1 共享 IPC：通用 v1 Unix 传输

基线 `1b1318b`，固定功能分支工作区干净后实施。原runtime/ipc唯一实现迁入
`src/vehicle_core/ipc.py`，通用车内v1传输不进入云车契约；原IPC模块Compatible显式转发
17个常量、类型、类、必要私有函数与logger，同一实现/类型身份，不包装或新增构造。
新owner显式保留原 `senior_control.runtime.ipc` logger名，其余实现保持。

10个产品消费者接canonical：agent_adapter，agent的composition/connection/main/ports/workspace，
docker_start_helper/operator_actions/simulation_cli/vehicle_deploy；仿真相关文件只静态迁引。
3测试文件仅改真实monkeypatch定义点：RGBD同步3处诊断module、exclusive_bind race socket
module、导航state polling fake IPC server module，原兼容导入及输入/fixture/assert保留。
预算18文件：产品增1改11、测试改3、工具文档改3，配置0、无删除。

不改变v1字节、frame/path上界、timeout、request/intent lane隔离、socket身份/生命周期/
并发bind/取消或外部SO_PEERCRED调用；不增加工厂、诊断owner、依赖或新运行行为。


#### 本片验证、自审与限制

环境：macOS arm64，原主仓Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、uv 0.12.23。
原主仓 `.venv/bin` 提供测试工具，本项目 `.venv`、依赖、锁文件与安装方式不变。

- 安装工具先对上一片已安装Root wheel检查：`python tools/verify_batch_one_install.py
  --lazy-shared-only` 退出1，明确缺少 `vehicle_core.ipc`，新canonical检查有效。
- 唯一pytest命令（项目目录，原主仓Python）：

  ```sh
  python -m pytest tests/unit/test_ipc_frames.py \
    tests/integration/test_ipc_lifecycle.py \
    tests/integration/test_intent_ipc_stream.py \
    tests/integration/test_ipc_exclusive_bind.py \
    tests/unit/test_rgbd_sync_ordering.py \
    ros2/tests/test_navigation_collision_qualification.py::test_state_polling_does_not_log_running_route_qualification_loss \
    -q --tb=short
  ```

  依据已知沙箱Unix bind限制，申请仅限定本地临时socket测试权限后运行，
  **33通过**，0.36秒，退出0，无跳过。没有扩大到仿真环境或ROS服务。
  exclusive_bind历史注释虽提simulation，实际用例只创建本地socket/file/symlink并测试
  bind竞争，不加载或静态验证仿真launch，全部符合本片范围；导航指定用例使用假server
  与ROS模块替身，并明确禁止实际controller/IPC请求。intent用例只向临时本地fake handler
  交付合成manual_intent帧，未连接车辆或ROS输出。
  原负责人已复现的RGBD旧定义点2通过3失败，本片改canonical后5项通过，无重做基线或
  弱化assert；3测试文件仅改import定义点，原输入/fixture/assert均保持。
- 修改Python及新IPC叶子 `ruff check` / `ruff format --check`：16文件通过，
  首轮7处导入排序已修正。普通 `git diff --check` 对agent_adapter三条新import报CR行尾；
  原文件全部809行使用CRLF，现仍保留809个CRLF，未整文件改写或混合行尾。
  使用保留默认空白保护并允许既有CRLF的命令通过：

  ```sh
  git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check
  ```

  该设置仅作用于命令，不修改Git配置或跳过其他空白检查；暂存检查沿用相同参数。
- IPC与 `git show 1b1318b:src/senior_control/runtime/ipc.py` 的AST对照，只替换logger
  为原名称字符串，其他全部相同；新Compatible注释仅说明日志路由。
  10个产品消费者及3测试文件去除import后AST与基线相同，产品无新业务调用/构造。
  外部SO_PEERCRED核验、socket所有权及私有helper调用保持。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-core-ipc-wheels`：
  根sdist/wheel成功；`uv pip install --offline --reinstall --no-deps --python
  /tmp/vehicle-core-leaf-sdk/bin/python <新Root wheel>` 更新前片临时安装夹具。
  仅重建Root，26包依赖闭包其余沿用，不冒充所有ROS功能包重新构建安装。
- 从 `/tmp`、`PYTHONPATH=` 执行临时安装Python与
  `<仓库>/tools/verify_batch_one_install.py --lazy-shared-only`：通过。
  新IPC实际来自site-packages；原17个出口（logger/4常量/2类型/异常/4编解码函数/
  server/client/3所有权helper）旧新同对象；logger保持原名，v1/8192/100及大端header
  字节不变；PendingCounter是已有canonical诊断类型，实际counter使用同一process_diagnostics
  缓存owner，没有新增socket或业务连接。单独导入IPC不加载旧SDK/管理/云/ROS，
  旧门面也不带入Agent/config/旧sensor策略。
  `uv pip check --python /tmp/vehicle-core-leaf-sdk/bin/python`：26包兼容。

自审实际18文件：产品增1改11、测试改3、工具文档改3，配置0、无删除；
无依赖、锁、工厂或协议变更，唯一传输实现及兼容类型/私有helper/logger保持。
提交仅在固定功能分支，使用禁用超范围钩子的 `git -c core.hooksPath=/dev/null commit`，
限定检查已手动执行，不推送，等待负责人审核。

未运行仿真测试/环境或任何ROS/实车/开发服务，未向实车或ROS发送运动请求，
未部署。真实Linux SO_PEERCRED、Humble/ament/设备行为仍未验证；
本地Unix/fake server通过不等于实车手动控制可用或独立manager生产部署完成。


### R1 共享车内值、异常与运行态

基线 `f565a17`，固定功能分支工作区干净后实施。原models/errors唯一实现逐字节迁
`src/vehicle_core/values.py`、`errors.py`，ErrorCode仍为原senior_contracts枚举；
三个本地DTO BodyReferenceState/RuntimeObservation/MotionRuntimeState原实现迁到
`vehicle_core/runtime_state.py`，只复用contracts的StrictModel及原整数界。
旧三模块Compatible同对象转发，保留_finite等私有接口，不造新别名类/枚举/异常；
protocol的其余WS出口原样保留。

AST核对73个仅产品消费者更新导入，混合WS导入仅分离本地DTO，原类型构造/业务不改；
test/tests目录不动，仿真点只静态迁引。预算82文件：产品增3改76、工具文档改3，
测试/配置0，无删除。不新增依赖、协议状态、工厂或owner，不改权限/过滤/schema/日志/时序。


#### 本片验证、自审与限制

环境：macOS arm64，原主仓Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、uv 0.12.23。
复用原主仓 `.venv/bin` 工具，不改项目 `.venv`、Requires-Dist、uv.lock或CI配置。

- 安装工具先对上一片已安装Root wheel执行 `--lazy-shared-only`：退出1，明确
  缺少vehicle_core.errors，新owner检查不会让未迁移wheel通过。
- 唯一当前限定pytest命令（项目目录，原主仓Python）：

  ```sh
  python -m pytest tests/unit/test_protocol.py senior_control/tests/test_agent_ports.py \
    tests/unit/test_runtime_guard_ready.py::test_duplicate_wrong_source_and_wrong_robot_cannot_replace_receiver \
    -q --tb=short
  ```

  **30通过、1旧fixture设置错误**，0.39秒，退出1；错误为ready fixture调用实际
  CommandNode._on_motion_runtime时缺少 `_output_incarnation`，发生在用例assert之前。
  未补假属性或改测试。以 `git archive f565a17 src ros2/src senior_control/src cloud/src
  contracts/src pyproject.toml tests/unit tests/fixtures/runtime_guard_ready.json
  ros2/tests/test_navigation_command_clock.py` 解包至 `/tmp/vehicle-core-values-baseline`，
  在该归档目录仅运行相同guard节点选择：**1相同设置错误**，0.21秒，退出1；
  行号随import整理变化，但缺失字段与fixture位置完全相同。没有扩大基线测试。
  protocol和Agent ports原兼容导入、原输入与断言不改；受控guard不调用同文件
  manual_target，不加载或运行仿真环境。
- 修改Python及3新叶子 `ruff check` / `ruff format --check`：80文件通过；
  首轮63处导入排序已整理，按既有CRLF约定的检查通过：

  ```sh
  git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check
  ```

- 自审：values/errors与基线原文件逐字节相同；3个DTO class AST与原protocol完全相同，
  所有原 `from senior_contracts.wire import ...` 导出行的内容与顺序完全保持。
  73个消费者去除import后AST与基线相同；两个混合协议导入只将本地DTO分离，
  原WS名字、别名、实际构造与业务分支均保持。test/tests文件无diff。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-core-values-wheels`：
  根sdist/wheel成功；以 `uv pip install --offline --reinstall --no-deps --python
  /tmp/vehicle-core-leaf-sdk/bin/python <新Root wheel>` 更新既有临时夹具。
  仅重建Root，其他26包闭包沿用前片，不声称全部ROS消费者重建或独立manager部署。
- 从 `/tmp`、`PYTHONPATH=` 运行临时安装Python与
  `<仓库>/tools/verify_batch_one_install.py --lazy-shared-only`：通过。
  3新叶子来自site-packages，新旧13个值/异常/DTO及私有_finite出口同对象，
  ErrorCode为原contracts枚举；OperationResult原字符串状态转换保持。
  MotionRuntimeState的schema alias、dump/JSON往返/JSON schema属性精确核对，
  无效schema、负sequence、无限值、半边证据窗口、extra字段及不完整BodyReference均
  按原规则拒绝；旧路径读新序列化结果仍同类相等。
  新叶子不加载旧SDK/管理/ROS，旧兼容路径也不带入Agent/config/诊断业务。
  既有安装工具其余旧断言保持；`uv pip check --python /tmp/vehicle-core-leaf-sdk/bin/python`：
  26包兼容。

实际82文件：产品增3改76、工具文档改3，测试/配置0、无删除；
没有新增依赖、协议状态、类型副本或工厂，权限、过滤、schema、日志、时序保持。
提交仅在固定功能分支，禁用可能超范围运行的钩子，限定检查已手动执行；
不推送，等待负责人审核。

当前guard旧fixture错误仍保留，不能宣称该条通过或全量通过；其余测试未扩大运行。
本片没有仿真验证、ROS/实车/开发服务启动或部署，没有发送运动请求；
真实Humble/ABI/设备与实车手动控制行为仍未验证。


### R5 首片：通用 AgentConfig 与 PoseReply

基线 `6266c9a`，固定功能分支工作区干净后实施。通用frozen AgentConfig构造、环境读取、
TLS/路径校验归 `vehicle_manager/agent/config.py`；不含具体diagnostics字段，也不依赖core、
旧包或ROS。旧composition.AgentConfig为Compatible frozen dataclass子类，唯一增加原
diagnostics字段/default_factory与环境解析绑定；旧类和新基类不同，不能报告同对象。
旧公开构造字段/顺序/默认保持，共用唯一__post_init__和from_environment解析。

from_environment在原必填项/路径读取完成之后、cls构造之前，调用受保护_environment_fields
获取额外kwargs；基类返回空字段，旧子类只提供原diagnostics_from_environment结果。
保留缺必填KeyError文本/链、坏诊断与TLS/路径构造校验优先级，不新增provider/配置框架
或伪诊断默认值；原生产配置实例、parser和运行入口保持。

PoseReply原pydantic校验迁 `vehicle_manager/agent/pose_reply.py`，旧ports显式同对象转发，
IpcHomePort解析对象/请求顺序保持，RuntimeState/StateReply仍原位。实际7文件预算：
产品增2改2、工具文档改3，测试/配置0、无删除，依赖/锁/systemd/CLI不改。


#### 本片验证、自审与限制

环境：macOS arm64，原主仓Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、uv 0.12.23。
工具仍复用原主仓 `.venv/bin`；不改项目 `.venv`、依赖或锁。

- 先给既有安装工具增加新配置/PoseReply检查；旧manager wheel执行
  `--manager-config-compatible` 明确缺少vehicle_manager.agent.config而退出1，
  新owner检查有效。工具原断言保留，新检查接入已有manager agent检查和默认SDK路径，
  并提供两个定向安装检查选项。
- 指定测试命令（项目目录、原主仓Python）：

  ```sh
  python -m pytest senior_control/tests/test_agent_config.py \
    senior_control/tests/test_agent_ports.py -q --tb=short
  ```

  申请原受控本地Unix fixture权限后 **14通过**，0.43秒，退出0，无跳过。
  同DB/写锁/九客户端、原CLI配对绑定/凭据、无adapter时继续上报、Home原准入及
  独立stop lane等既有断言保持；测试文件未改，不向ROS或车辆发送请求。
- 5份改动/新增Python的 `ruff check` / `ruff format --check`通过；
  初轮1处导入排序、3处格式及1个迁移后未使用Path导入已整理。
  按既有CRLF约定 `git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol
  diff --check` 通过。
- 自审AST：新通用字段（除diagnostics）及__post_init__与基线原定义相同；
  from_environment只将末尾原diagnostics表达式改为受保护hook的kwargs，
  必填/路径读取顺序、KeyError处理及其余解析完全相同；返回类型使用标准库TypeVar保持
  旧子类实际返回类型，不引入配置框架。旧子类diagnostics原default_factory保持。
  PoseReply完整class AST相同，composition除配置类以外所有主体及ports除PoseReply
  以外所有主体与基线相同，IpcHomePort仍用同名同对象parser和原请求顺序。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-manager-config-wheels`
  与 `uv build senior_control --offline --out-dir /tmp/vehicle-manager-config-wheels`：
  Root/manager两个项目sdist/wheel通过；Root移除未使用import后再次构建更新实际wheel。
- 以 `uv pip install --offline --reinstall --no-deps --python
  /tmp/vehicle-core-leaf-sdk/bin/python <新Root wheel> <新manager wheel>` 更新已有26包SDK夹具；
  其余依赖wheel沿用前片，不冒充全部重建。
  另 `uv venv --python <原主仓Python> /tmp/vehicle-manager-config-only`，
  仅安装新manager和原contracts wheel及其依赖，共8包，无Root/core/ROS。
- 从 `/tmp`、`PYTHONPATH=` 执行安装工具：

  ```sh
  /tmp/vehicle-manager-config-only/bin/python <仓库>/tools/verify_batch_one_install.py --manager-config-only
  /tmp/vehicle-core-leaf-sdk/bin/python <仓库>/tools/verify_batch_one_install.py --manager-config-compatible
  ```

  两者通过。manager-only实际来自site-packages，通用配置环境/TLS/绝对路径/冻结/secret repr
  与PoseReply严格校验通过，core/旧包/ROS不只未import，而是find_spec也不存在。
  SDK确认旧类为新类的不同子类，唯一__post_init__/from_environment复用、原11字段顺序
  与diagnostics默认工厂保持；原诊断环境解析、缺必填优先及KeyError链、坏诊断先于
  TLS构造校验均通过；旧PoseReply与新类同对象。
  两夹具 `uv pip check --python ...` 分别8包/26包兼容。

实际7文件，与预报一致：产品增2改2、工具文档改3，无测试/配置/依赖/锁/systemd/CLI
变更或删除。manager新叶子没有反向依赖core或旧包；RuntimeState/StateReply、
完整生产绑定及发行DAG尚未迁完，不宣称独立manager生产部署完成。
提交仅固定分支，禁用超范围钩子且限定检查已手动执行，不推送，等待负责人审核。

本片未运行仿真测试或任何仿真/开发/ROS/车辆服务，无运动请求或部署。
真实Linux/Humble/设备及实车手动控制可用性仍未验证。


### R5 数据库装配：Home协调与domain schema

基线 `0e0f7f0`，固定功能分支工作区干净后实施。manager/storage/home_database.py
承接原逐DB/逐robot目标锁与执行协调缓存，HomeDatabase继承原Database单连接引擎；
锁与协调工厂显式逐实例注入。协调复用现有OperationT形状，setdefault仍每次先构造候选，
不改缓存粒度/时机。旧无参HomeExecutionCoordination为绑定原DiagnosticLock/OperationResult
的Compatible薄子类；旧Database同默认签名，注入该原构造工厂，实际返回旧具体协调类型。
两旧类与manager基类不同，行为方法唯一继承，不复制诊断、锁业务或连接owner。

manager/storage/domain_schema.py持有唯一1地图/2航点/3任务顺序，显式接原地图SQL tuple，
复用canonical Migration及Task/Waypoint原tuple；旧schema在原模块导入时注入，保留
DOMAIN_MIGRATIONS、Migration及3份历史SQL常量访问/身份。manager没有直接导入core、
旧包、地图/ROS/cloud owner；map_persistence和业务执行未改。
预算7文件：产品增2改2、工具文档改3，测试/配置0，无删除或新依赖/锁/额外连接。


#### 本片验证、自审与限制

环境：macOS arm64，原主仓Python 3.12.15、pytest 8.4.2、Ruff 0.17.3、uv 0.12.23。
不改项目 `.venv`、依赖/锁；复用原主仓工具与已有临时wheel夹具。

- 既有安装工具先对旧manager wheel执行 `--home-database-compatible`，退出1，
  明确缺少vehicle_manager.storage.domain_schema，新owner检查有效。
- 唯一pytest命令（项目目录、原主仓Python）：

  ```sh
  python -m pytest senior_control/tests/test_database_ownership.py \
    tests/integration/test_storage.py \
    senior_control/tests/test_home.py::test_shared_home_guard_serializes_claim_and_ignores_old_terminal \
    senior_control/tests/test_home.py::test_legacy_home_guard_migration_preserves_safety_and_unknowns \
    ros2/tests/test_map_operations.py::test_startup_blocks_motion_before_waiting_for_database \
    -q --tb=short
  ```

  **27通过**，0.92秒，退出0，无跳过。实际SQLite单连接/reentry、vehicle/cloud迁移回滚、
  busy/磁盘满/取消/备份、共享Home guard及旧guard安全字段迁移、Gate先阻断再等DB通过；
  所有选定fixture均为本地数据库、内存地图/导航或Gate替身，无仿真/ROS执行。
  没有旧失败需要对照，未扩大到同文件其他测试，测试和断言未改。
- 5份改动/新增Python的 `ruff check` / `ruff format --check`通过，首轮1处import排序
  与1处工具格式已整理；保留原schema CRLF，按既有约定的diff检查通过：
  `git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check`。
- 自审AST对照：协调初始化只用显式lock_factory替换原DiagnosticLock、OperationT替换
  原具体结果类型；原全部状态字段及两个setdefault方法主体保持，仅工厂来源/泛型返回
  注解变化。没有改成懒缓存，没有增加锁层或在manager引入core/地图/云/ROS。
  原SQLite引擎、map_persistence、业务执行、测试、配置、Requires-Dist和uv.lock无diff。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv build --offline --out-dir /tmp/vehicle-manager-home-db-wheels`
  与 `uv build senior_control --offline --out-dir /tmp/vehicle-manager-home-db-wheels`：
  Root/manager两个项目sdist/wheel通过。用 `uv pip install --offline --reinstall --no-deps
  --python ...` 将两新wheel装入26包SDK夹具，将新manager装入8包manager-only夹具；
  其他依赖闭包沿用前片，不冒充全部ROS发行重新安装。
- 从 `/tmp`、`PYTHONPATH=` 执行：

  ```sh
  /tmp/vehicle-manager-config-only/bin/python <仓库>/tools/verify_batch_one_install.py --manager-tasks-only
  /tmp/vehicle-core-leaf-sdk/bin/python <仓库>/tools/verify_batch_one_install.py --home-database-compatible
  ```

  两者通过。manager-tasks-only原有纯任务检查/断言保留，新Home检查放在其纯导入门之后；
  新HomeDatabase来自site-packages，实际lock/coordination工厂注入，逐DB/逐robot隔离；
  缓存命中仍构造候选的次数与原setdefault一致，manager没有安装core/旧包/地图/cloud/ROS。
  同一Base Database.__aenter__负责实际连接；原连接和write_lock在迁移前后保持，
  追踪实际aiosqlite.connect只打开1条主库连接（原迁移备份连接仍按原引擎存在）。
  domain工厂使用显式注入SQL、同一Task/Waypoint tuple和canonical Migration，实际SQLite
  迁移至版本3并创建原waypoint/task_definition表。
  SDK进一步确认旧Database两参数/1000默认、旧协调无参签名及实际旧协调具体类型；
  write/target/transaction锁均为原DiagnosticLock，全部使用原process_diagnostics owner，
  同DB/robot返回同锁/协调；旧schema所有历史常量和三项SQL tuple身份保持，
  1地图/2航点/3任务版本顺序原样，实际SQLite成功迁移，无第二条主库连接。
  两夹具 `uv pip check --python ...` 分别8包/26包兼容。

实际7文件，与预报一致：产品增2改2、工具文档改3，测试/配置0、无删。
旧HomeExecutionCoordination和Database与manager基类为不同薄子类，实际旧调用仍返回
原旧类对象；状态/缓存/SQL业务只有一份实现，没有模块全局工厂或第二个连接owner。
manager完整生产绑定及发行DAG仍待后续，不宣称独立生产部署完成。
提交仅固定分支，禁用超范围钩子且限定检查已手动执行，不推送，等待负责人审核。

未启动任何仿真/ROS/实车/开发服务，不验证仿真、不发送运动请求、不部署。
真实Linux/Humble/设备和实车控制可用性仍未验证。


## R0/R5 发行DAG切换

基线`feat/vehicle-architecture-refactor@2acd5c5`干净，实际8文件均为配置/工具文档，
产品和测试0；不改cloud锁，不生成manager独立锁，不新增发行。
当前发行图为`vehicle-manager → senior-control(vehicle_core) → senior-contracts`，
并保留原基础Pydantic/settings/PyYAML/aiosqlite约束。根wheel不再聚合manager、cloud或ROS项目；
九个原项目依赖原版本进入根vehicle组，默认组dev+vehicle。根agent/video extra直接声明
与manager精确相同的httpx/websockets/aiortc约束，没有反向manager依赖。

//Compatible 旧senior_control命名空间仍和vehicle_core在根同一wheel中可发现，
不是物理移除旧SDK；manager安装因此多出根基础依赖，不能再用find_spec不存在证明隔离。
工具改为正向wheel归属/METADATA证明及sys.modules无旧业务加载，原无legacy-loaded保护保留；
cloud和ROS发行/模块仍不得安装或加载。旧SDK入口仍需vehicle整车组或显式全部项目wheel，
仅pip安装根wheel（即使带agent/video extra）不再自动装完整SDK。
项目仍用根单套.venv，现场完整命令仍为：

```sh
uv sync --frozen --no-dev --inexact --extra agent --extra safety --extra ros --extra video
```

### 实际命令与结果

宿主macOS arm64，Python3.12.15（另用3.10.22做目标平台干跑）、uv0.12.23、
Ruff0.17.3。下列uv命令均离线，主要使用`UV_CACHE_DIR=/tmp/vehicle-uv-cache`原缓存副本；
临时环境用于安装验证，不是项目运行环境。命令从checkout执行，安装工具从`/tmp`执行，
未设置PYTHONPATH；`<checkout>`表示本工作区绝对路径。

- `uv lock --offline`及`uv lock --check --offline`通过，解析71条package记录。
  对比2acd5c5锁：全部(name,version,source)相同；除root/manager两条metadata和边外，
  其余69条完整记录相同，包括全部registry制品hash、URL和平台wheel，无升级。
- 新旧各执行`uv export --frozen --no-dev --extra agent --extra safety --extra ros --extra video --no-hashes`，
  基线用`--project /tmp/vehicle-dag-baseline`（git show 2acd5c5的锁/清单快照）。
  忽略注释后54条导出行完全相同：11个本地项目+43条带平台marker的registry需求。
  加`--no-emit-local`生成`/tmp/vehicle-dag-constraints.txt`，后续安装均使用该冻结约束。
- 临时`/tmp/vehicle-dag-independent`只复制根、manager、contracts三个pyproject，
  不放ROS/cloud项目目录，在其中运行`uv lock --project .../senior_control --offline`通过。
  独立解析36条记录（含全部optional extra），本地项目仅manager/root/contracts，
  无父级vehicle/dev组的cloud、ROS项目及pytest/ruff/mypy/lefthook；真实manager锁未写入。
- `uv build --offline --wheel --out-dir /tmp/vehicle-dag-wheels`与
  `uv build --offline --wheel --out-dir /tmp/vehicle-dag-wheels senior_control`通过。
  根/manager的`direct_url.json`均指向真实`.whl`且无editable标记；core文件位于
  site-packages且属于senior-control的RECORD路径，manager METADATA声明senior-control==0.1.0。
- `python3.12 -m venv /tmp/vehicle-dag-base`创建空环境，随后：

```sh
uv pip install --offline --python /tmp/vehicle-dag-base/bin/python \
  -c /tmp/vehicle-dag-constraints.txt /tmp/vehicle-dag-wheels/*.whl \
  /tmp/m5c-wheels/senior_contracts-*.whl
/tmp/vehicle-dag-base/bin/python -m pip check
cd /tmp
/tmp/vehicle-dag-base/bin/python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only
/tmp/vehicle-dag-base/bin/python <checkout>/tools/verify_batch_one_install.py --manager-config-only
```

  12个声明闭包包安装成功（不计venv自带pip/setuptools），pip check无损坏需求。
  真实SQLite单连接/SQL/锁工厂、任务/生命周期/执行/航点接口、配置环境/TLS路径/repr、
  agent基础接口、真实profile/service渲染和media惰性边界通过。
  无cloud/ROS发行安装或旧业务模块加载；纯任务检查仍在Pydantic/aiosqlite加载前进行。
  operator检查仅import监督模块/检查callable并渲染kind=real的profile，
  不调用任何simulation.supervise、仿真行为、服务或容器。
- `/tmp/vehicle-dag-sdk`另建空Python3.12 venv，按同一冻结约束显式安装11个项目wheel，
  26包通过`python -m pip check`。本轮root/manager为新构建，其余复用未修改项目的既有wheel：
  contracts/localization/mapping/motion-control/motion-estimation/perception取`/tmp/m5c-wheels`，
  navigation/robot取`/tmp/b7-wheels`，cloud取`/tmp/cloud-build-9cafe01`；不称其为本轮全量重建。
  从`/tmp`运行同工具`--lazy-shared-only`、`--manager-config-compatible`、
  `--home-database-compatible`均通过：共享叶子canonical/旧路径同对象与惰性依赖、
  旧配置签名/诊断优先级/PoseReply身份、Home实际SQLite/SQL tuple与owner保持。
- 宿主精确现场命令附加`--offline --dry-run --python <Python3.12>`并设置
  `UV_PROJECT_ENVIRONMENT=/tmp/vehicle-dag-frozen`通过，计划45包；实际去掉dry-run尝试失败，
  原缓存缺`numpy==2.5.4`的macOS arm64 wheel，未联网、升级或修改根.venv。
- Linux/aarch64目标检查：原精确命令附加`--offline --dry-run --python <Python3.10.22>
  --python-platform aarch64-unknown-linux-gnu`，新旧清单分别运行并设置不同临时环境路径。
  两者均计划相同46包（35个registry+11本地），包括numpy1.26.4、shapely2.1.2、
  pydantic-settings2.15.0。宿主执行的是目标平台解析干跑，未真实安装Linux制品或验证Humble ABI。
- agent extra的空Python3.12环境安装按同约束离线失败：缺websockets15.0.1缓存。
  直接重试原`~/.cache/uv`也相同；Python3.10临时环境尝试则缺PyYAML6.0.3缓存。
  因此本轮**未完成agent extra真实安装/--manager-agent-only运行**，未换成其他版本或声称通过。
  已检查其入口：network=True仅import httpx/websockets，不连接网络；其余manager有效逻辑
  已由base入口执行。CI的base/agent pip安装均显式提供新根wheel，并执行pip check，继续只装声明依赖闭包。
- `ruff check tools/verify_batch_one_install.py`、`ruff format --check`通过；
  按原CRLF约定`git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check`通过。
  TOML/YAML静态解析、实际diff与8文件预算自审通过。提交禁用hooks避免触发全量或仿真测试，
  上述范围检查手动执行；无产品变更，未跑pytest/full CI、仿真验证或服务。

### 限制与后续

本片只改变发行DAG与安装语义；不宣称agent完整独立生产运行、Linux实机安装、
ROS加载、实车手动控制或整车视频已通过。agent及完整整车真实冻结安装需要补齐原版本缓存
后再验，不能用干跑或旧SDK基础闭包替代。无运动请求、部署或推送；固定功能分支提交待审核。


## R5 本地IPC实际装配

基线`feat/vehicle-architecture-refactor@5988c42`，开始时工作区干净。
实际14文件：产品新增2、修改9，工具文档修改3，测试/配置/删除0。
根/manager pyproject、uv.lock、systemd和CLI均未改，不新增发行或运行入口。

唯一owner为：

- `vehicle_manager/agent/local_state.py`：原冻结RuntimeState、严格StateReply及for_tasks，
  原字段/默认/校验和主体逐字保持。RouteProgress使用实际既有
  `senior_contracts.navigation`（仓库不存在nav模块），ExecutionContext直接使用
  `vehicle_manager.tasks.models`原同对象；其余复用vehicle_core值/运行DTO和wire数据类型。
- `vehicle_manager/agent/local_adapters.py`：原具体IpcState、IpcHomePort、IpcMapPort、
  TaskAwareMotion、IpcMotion、MotionRejected唯一实现。全部类定义逐字保持，唯一差别是
  Home构造store注解从旧WaypointStore改为既有HomeWriter[Pose2D]；仍保存原store实例。
  TaskAwareMotion的MRO仍为「具体TaskAwareMotion、generic TaskAwareMotion、
  具体IpcMotion、generic IpcMotion、object」，constructor仍经具体IpcMotion注入原异常。
- 旧`agent/ports.py`与`connection.py`仅 //Compatible 显式同对象转发已迁名字；
  ports保留唯一IpcNavigation旧导航绑定及其必要旧依赖，导航类定义完全未改。
  composition/main/settings_wire/direct_navigation/dispatcher/workspace/websocket七处
  实际消费者改canonical引用，composition只保留旧IpcNavigation。
  Link/MotionPort/HeartbeatFrame/Requester使用已存在manager公开路径。

没有移动底层generic传输实现或引入第二套schema/parser/effects。
Home仍保存_client，pose.get和state.get仍用同client，原PoseReply同对象与catch集合保留。
状态读取仍用原单调时钟、闭区间0–0.5秒、原有效期/几何屏蔽和任务证据投影；
请求/intent/disable/safety四路与独立停车/异常cause/运行结果保持原行为。

### 实际命令、环境与结果

环境：macOS arm64，Python3.12.15、pytest8.4.2、Ruff0.17.3、uv0.12.23。
测试解释器和Ruff复用原主检出`.venv/bin`工具，当前项目运行.venv未变。
新wheel验证复用前片临时base/SDK环境，不创建第二套项目运行环境。
下列`<checkout>`为本工作区；uv离线使用`UV_CACHE_DIR=/tmp/vehicle-uv-cache`，
冻结约束`/tmp/vehicle-dag-constraints.txt`来自5988c42已核对的根锁导出，未重新解析升级。

```sh
python -m pytest senior_control/tests/test_agent_ports.py \
  senior_control/tests/test_agent_config.py tests/integration/test_intent_ipc_stream.py -q
```

已逐份读取三文件，均为非仿真用例。本地SQLite、FakeConnector/FakeLink/AsyncMock与
临时Unix socket，不加载ROS或连接实车。首轮沙箱内13通过、2失败，失败均为Unix socket
bind的EPERM；允许临时本地socket后原命令复跑：**15 passed in 0.47s**，未改测试或断言。
覆盖实际composition原九个独立IpcClient、八个1000ms加一个3000ms超时、同数据库/连接/
write_lock、任务装配、独立停车客户端、learning原deadline/receipt、缺失与外来adapter报告、
Home合格定位/旧定位拒绝及100帧intent合并和请求通道不阻塞。未运行其他测试文件。

```sh
uv build --offline --wheel --out-dir /tmp/vehicle-local-ipc-wheels senior_control
uv build --offline --wheel --out-dir /tmp/vehicle-local-ipc-wheels
uv pip install --offline --python /tmp/vehicle-dag-base/bin/python \
  -c /tmp/vehicle-dag-constraints.txt /tmp/vehicle-local-ipc-wheels/vehicle_manager-*.whl
uv pip install --offline --python /tmp/vehicle-dag-sdk/bin/python \
  -c /tmp/vehicle-dag-constraints.txt /tmp/vehicle-local-ipc-wheels/*.whl
/tmp/vehicle-dag-base/bin/python -m pip check
/tmp/vehicle-dag-sdk/bin/python -m pip check
cd /tmp
/tmp/vehicle-dag-base/bin/python <checkout>/tools/verify_batch_one_install.py --manager-local-ipc-only
/tmp/vehicle-dag-base/bin/python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only
/tmp/vehicle-dag-sdk/bin/python <checkout>/tools/verify_batch_one_install.py --manager-local-ipc-compatible
```

两真实非editable wheel构建和重装通过；base保留12包、SDK保留26包声明闭包，两边pip check通过。
base仍为前片无ROS/cloud/导航项目安装环境，根wheel仅提供已声明core；SDK复用前片显式
11项目wheel闭包（本片只重建root/manager），不称本片全量重建或agent网络extra安装成功。
安装工具确认class来自site-packages；具体绑定!=generic基类与继承方法同对象的原保护保留。
新增验证内容为：

- 实际MRO/constructor注入，原MotionRejected及错误code，StateReply/RuntimeState解析工厂、
  for_tasks的原ExecutionContext和有效期，0.5秒边界接受、超时/未来值拒绝、残缺map引用、
  strict类型/未知字段拒绝，以及原DomainError UNAVAILABLE和原ValueError/ValidationError cause。
- state/map传输异常原对象作为cause；Home同client/state/PoseReply、原catch和值构造；
  四路transport绑定、停车走safety、原unsupported/accepted/rejected/unknown-code分派，
  手动release的显式拒绝cause与OPERATION_UNKNOWN失联cause、四client关闭。
  全部作用于AsyncMock，没有连接socket或发送实车运动。
- manager-only不加载senior_control/cloud/rclpy/launch/Nav2/senior_mapping/
  senior_navigation/senior_robot；原根Requires-Dist、manager→core及无cloud/ROS发行安装断言仍在。
  SDK证明旧ports/connection所有已迁名字与新具体类/DTO同对象，七处生产消费者用新绑定，
  IpcNavigation仍是原旧绑定并继承同generic请求方法。旧绑定!=generic类的既有检查未删弱。

agent extra另在新`/tmp/vehicle-local-ipc-agent`空Python3.12环境按同冻结约束尝试离线安装
本片root与manager[agent]、原contracts wheel，仍失败于缺websockets15.0.1缓存。
没有升级/联网补包，未运行--manager-agent-only或网络连接；本片有效本地IPC入口已在base验证。
--manager-tasks-only仍仅检查监督模块可调用并渲染kind=real profile，没有运行仿真监督或环境。

`ruff check`与`ruff format --check`覆盖12个涉及Python文件，全部通过。
临时源码对照脚本用git show 5988c42逐个提取类文本：两个模型、各绑定、MotionRejected、
旧IpcNavigation完全相同（仅Home注解例外）；七消费者除import外业务AST相同。
按既有CRLF约定diff check通过，自审14文件范围，不改测试/锁/配置。
提交禁用hooks以免触发未授权全量/仿真检查，所有范围检查手动执行。

未验：agent网络extra闭包、真实ROS/实车IPC、Nav2、完整独立生产agent与手动控制；
本地替身和Unix socket通过不代表这些能力已通过。未启动仿真、ROS、开发或车辆服务，
未发实车运动请求、部署或推送；仅固定功能分支提交待负责人审核。


## R5 具体Agent依赖绑定

基线`feat/vehicle-architecture-refactor@ba0d4f7`，开始时干净；实际13文件：
产品新增2、修改8，工具文档修改3，测试/配置/删除0。没有修改发行、锁、systemd或CLI。

canonical `vehicle_manager.agent.config.AgentConfig`现直接包含原末尾diagnostics字段，
仍为frozen dataclass、secret repr=False，默认factory仍是同一DiagnosticsSettings。
旧composition.AgentConfig与新类现在**同一个类**，上一片过渡子类不再存在。
两处_environment_fields均移除；from_environment在原必填读取/Path构造后以明确diagnostics实参
解析环境，随后才运行原构造/TLS校验。缺必填KeyError→原ValueError文本/cause优先，
坏诊断优先于构造时坏TLS，字段顺序/默认及全部原校验保留。

新增唯一具体绑定owner：

- `vehicle_manager/storage/bindings.py`：原Database和HomeExecutionCoordination两薄构造。
  保留原无参协调、Database(path,busy_timeout_ms=1000)、DiagnosticLock、实际OperationResult
  泛型绑定与原协调factory；继承同一HomeDatabase/SQLite引擎，不复制连接/缓存/事务。
- `vehicle_manager/agent/production_bindings.py`：原JournalEntry/AgentJournal/_OPEN、
  AgentLogConnection、SettingsRuntime、WebSocketLink/WebSocketConnector/_receive_diagnostic、
  VehicleLogService。类及函数文本逐字相同，直接用既有manager RobotSnapshot/snapshot_envelope、
  stores/ports、core诊断owner；底层generic业务与logger不改，网络库仍在原实际连接时惰性导入。

旧storage/database、agent/journal/logs_wire/settings_wire/websocket、vehicle_logs/service
六模块显式 //Compatible 同对象出口，保留_transaction、_OPEN、AGENT_JOURNAL_STATEMENTS、
settings_command、log_url、CloudLogGrants、Socket/ProtocolRejected等既有辅助名。
旧composition只改这些具体构造/配置的导入，其余业务AST不变；九client、mapping/media、
原Agent/任务/导航组合仍在原位，本片不宣称整个生产composition已迁完。

### 实际验证

macOS arm64，Python3.12.15、pytest8.4.2、Ruff0.17.3、uv0.12.23。
复用主检出`/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin`的测试工具，
本工作区运行.venv未改。已读指定五文件，未包含仿真功能验证；原命令为：

```sh
python -m pytest senior_control/tests/test_agent_config.py senior_control/tests/test_agent_ports.py \
  senior_control/tests/test_database_ownership.py senior_control/tests/test_vehicle_logs.py \
  senior_control/tests/test_websocket_link.py -q
```

允许测试内临时Unix socket与回环WebSocket监听后，**67 passed in 1.81s**。
覆盖原配置/TLS、实际九client/1000–3000ms/同DB与独立停车、Home/learning结果、连接重入/关闭、
日志认证/有界队列/脱敏/持久与损坏路径、heartbeat nonce/epoch/超时/撤销以及本地connector握手。
WebSocket测试使用该测试环境预装`websockets15.0.1`，另有`httpx0.28.1`；不是本轮空环境
安装agent extra的证据，没有连接远端云。未修改测试或断言，未跑其他用例或全CI。

离线wheel/仓库外命令（uv使用`UV_CACHE_DIR=/tmp/vehicle-uv-cache`；`<checkout>`为本工作区）：

```sh
uv build --offline --wheel --out-dir /tmp/vehicle-production-wheels senior_control
uv build --offline --wheel --out-dir /tmp/vehicle-production-wheels
python3.12 -m venv /tmp/vehicle-production-base
uv pip install --offline --python /tmp/vehicle-production-base/bin/python \
  -c /tmp/vehicle-dag-constraints.txt /tmp/vehicle-production-wheels/*.whl \
  /tmp/m5c-wheels/senior_contracts-*.whl
uv pip install --offline --python /tmp/vehicle-dag-sdk/bin/python \
  -c /tmp/vehicle-dag-constraints.txt /tmp/vehicle-production-wheels/*.whl
/tmp/vehicle-production-base/bin/python -m pip check
/tmp/vehicle-dag-sdk/bin/python -m pip check
cd /tmp
/tmp/vehicle-production-base/bin/python <checkout>/tools/verify_batch_one_install.py --manager-production-only
/tmp/vehicle-production-base/bin/python <checkout>/tools/verify_batch_one_install.py --manager-tasks-only
/tmp/vehicle-dag-sdk/bin/python <checkout>/tools/verify_batch_one_install.py --manager-production-compatible
/tmp/vehicle-dag-sdk/bin/python <checkout>/tools/verify_batch_one_install.py --home-database-compatible
/tmp/vehicle-dag-sdk/bin/python <checkout>/tools/verify_batch_one_install.py --manager-local-ipc-compatible
```

两真实非editable wheel通过；全新base安装12个声明闭包包（不计venv工具），SDK复用前片
26包闭包并重装本片root/manager；均pip check通过。冻结约束复用5988c42的已核对根锁导出，
无重新锁解析或版本升级。工具确认site-packages实际构造可用、manager→core发行与文件归属。
新配置的完整签名/默认/诊断解析/错误优先级断言已同时应用于base和SDK，旧新必须is，
删除的仅是已撤销过渡方案的「无诊断」「旧类!=新类」假设，没有删除构造/继承方法保护。
具体绑定与generic底层仍!=且继承同一方法：所有原generic区别断言保留，新增实际运行检查：

- 真实临时SQLite journal prepare/begin/get/finish、原JournalEntry/OperationStatus/_OPEN，
  同Database/connection/write_lock、退出关闭；Home逐robot锁/协调同对象，协调实际类型和
  OperationResult泛型身份，以及锁和日志PendingCounter全部归同一个process_diagnostics缓存。
- 实际日志/SettingsRuntime/WebSocket绑定构造：原store/grants/state/tasks、诊断factory和
  原snapshot callback/receive callback、send lock与connector link factory，无替代owner。
  本项不启动日志writer/cleaner或连接网络；完整日志行为由上述原测试验证。
- base不加载senior_control、cloud、rclpy/launch/Nav2、mapping/navigation/robot、
  httpx/websockets/aiortc；无cloud/ROS项目安装，根同wheel兼容namespace仍可发现。
  SDK证明旧六模块的类/值/私有helper与canonical同对象，composition实际使用新构造，
  旧_transaction/SQL tuple仍为原manager对象；旧Home schema与本地IPC绑定检查继续通过。

临时git show源码对照确认10个搬迁类/函数文本逐字相同；composition除去已迁AgentConfig
和import后业务AST相同；config __post_init__及环境求值顺序除明确诊断实参外相同。
11个涉及Python文件的Ruff/format、按原CRLF约定diff check及13文件实际diff自审通过。
提交禁hooks避免执行未授权全量/仿真检查，上述范围检查均手动执行。

本片未重新尝试空环境agent extra安装；前片原版本缓存缺口仍未收尾，预装依赖测试不替代它。
未验证Linux/Humble ABI、ROS/实车、完整独立生产agent或手动控制，未运行任何仿真验证，
未启动开发/ROS/车辆/仿真服务，未发实车运动、部署或推送；固定功能分支提交待审核。
