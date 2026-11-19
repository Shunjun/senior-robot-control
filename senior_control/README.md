# 车端管理

M1a 设置、M1b 车辆日志已从原根 SDK 迁入；其他管理职责仍待各片迁移。本项目目录 `senior_control/`，
导入名 `vehicle_manager`，发行名 `vehicle-manager`。没有新增CLI/服务入口，也未迁导航或ROS执行。
下文M1a记录按当时范围保留，当前日志装配、依赖与验证以末尾M1b记录为准。

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
  tests/
    test_vehicle_settings.py          # 原62项设置用例，无新增场景
    test_vehicle_logs.py              # 原38项日志用例，无新增场景
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
