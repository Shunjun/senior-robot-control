# 车端管理

设置、车辆日志、SQLite技术设施、任务核心与TaskStore已从原根 SDK 迁入；其他管理职责仍待各片迁移。本项目目录 `senior_control/`，
导入名 `vehicle_manager`，发行名 `vehicle-manager`。没有新增CLI/服务入口，也未迁导航或ROS执行。
下文各片记录按当时范围保留，当前任务生命周期持久/服务边界与验证见末尾S2记录。

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
