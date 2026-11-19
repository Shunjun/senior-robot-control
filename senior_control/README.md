# 车端管理

M1a 设置已从原根 SDK 迁入；其他管理职责仍待各片迁移。本项目目录 `senior_control/`，
导入名 `vehicle_manager`，发行名 `vehicle-manager`。本片没有新CLI/服务入口，也不迁日志、导航或ROS执行。

## 实际目录与调用

```text
senior_control/
  pyproject.toml                       # 只依赖 Pydantic、aiosqlite
  src/vehicle_manager/
    __init__.py, py.typed              # 顶层无业务加载副作用
    settings/
      __init__.py                     # 原公开类名按需加载，不随models导入service
      models.py                       # 不可变设置值、偏好范围及原默认值
      ports.py                        # 只读connection/write_lock接口
      store.py                        # 原同连接SQLite事务/完整图像/receipt
      service.py                      # 唯一设置owner、runtime gate和DEBUG恢复timer
  tests/
    test_vehicle_settings.py          # 原62项设置用例，无新增场景
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
