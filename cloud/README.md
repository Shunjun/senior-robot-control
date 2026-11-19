# 独立云端

当前 C3 已接独立生产bootstrap、Docker上下文和frozen锁；本机无可连接Docker daemon，镜像构建实测仍有缺口。
详见下方 [C3 记录](#c3-生产镜像与独立锁接线)；C1/C2段保留各批历史证据。

## C1 业务迁移历史记录（ee735b0）

基线 `859c954`，九份既有云业务归 `cloud/src/cloud`，发行名 `vehicle-cloud`。
云包只依赖 `senior-contracts`、FastAPI、Pydantic、aiosqlite；不导入车端管理、ROS 或其执行实现。
业务实现唯一，原入口显式注入原 owner；C2 的云数据库/schema/诊断独立设施及 C3 镜像闭包仍待处理。
没有新业务、服务入口、顶层技术包或全局 provider 注册；现有部署入口继续使用旧 bootstrap。

## 实际目录与职责

```text
cloud/
  pyproject.toml                      # wheel/真实第三方与契约依赖
  src/cloud/
    __init__.py, py.typed              # 无导入副作用；类型标记
    app.py                            # ASGI factory、认证路由、原寿命装配
    auth.py                           # scrypt、会话、CSRF、失败限流
    pairing.py, registry.py            # 一次性配对、设备摘要、撤销与持久 epoch
    db.py                             # 原同连接写事务与读取；不拥有连接/迁移引擎
    limits.py                         # 原云限额值
    relay.py                          # Hub、控制代次、有界出箱、安全优先与私有响应
    logs.py                           # 独立日志通道与有界队列
    snapshots.py                      # 云快照缓存、过期、字段清空与省略
    ports.py                          # 只声明实际注入的技术接口，不含实现/业务状态
  tests/
    test_cloud.py, test_cloud_epoch.py
    test_agent_snapshots.py, test_heartbeat_wire.py, test_drive_challenge.py
    heartbeat_test_support.py          # 原跨端回归夹具，只保留一份
    README.md
```

源为 `src/senior_control/cloud/{app,auth,db,limits,logs,pairing,registry,relay,snapshots}.py`。
原路径保留显式 `//Compatible` 转发或薄构造；旧包 init 更新为兼容说明。
没有把云车协议、车内 Unix IPC 和 ROS transport 混为一个接口。

### 实际生产调用与注入

```text
Jenkinsfile → deploy/jenkins-deploy.sh → deploy/Dockerfile.cloud
  → 原 python -c bootstrap
  → senior_control.cloud.app.create_app          # Compatible 薄装配
      → cloud.app.create_app                     # 唯一业务实现
          → Hub / LogRelay / SessionStore / PairingService / RobotRegistry
          → senior_contracts.wire/errors/logs    # 线上契约
```

旧 bootstrap 原样构造 `CloudSettings` 和 `Database`，旧 factory 接口不变。
它局部注入同一个数据库对象、同一个写锁、原 `migrate` 及版本 1–5 的迁移声明、
原 `configure_diagnostics(settings.diagnostics)` 和 `DiagnosticDeque` / `DiagnosticAsyncQueue`。
配置已在原 owner 校验；云侧 `Settings` 只读取 secret，配置校验逻辑没有复制。
云 factory 同步构造时不打开数据库，lifespan 才按原 owned 规则打开/迁移/关闭，
外部已打开连接不被关闭；事务的 BEGIN IMMEDIATE、commit/rollback、取消传播保持原实现。
数据库路径、SQL、user_version、备份、Home/任务 owner 均未迁或改动。
`ports.Database` 是只读连接/锁视图，不创建连接，不复制存储实现。

`Hub` 显式接收 deque_factory，`Outbox` 使用该 factory 保留 reject/discard_index 与原诊断计数。
`LogRelay` 显式接收 queue_factory，agent/viewer 队列仍分别为 32/4；无 stdlib 无诊断回退。
每个 Hub/relay/peer 仍各自持有原业务状态，诊断仍归原进程 owner。
保留原逻辑 logger 名 `senior_control.cloud.relay`，避免改目录影响原日志白名单归类。

### 兼容身份

- `CloudLimits`、`SnapshotCache`、会话/注册值、`Connection`、认证算法与共享契约经旧入口取得的对象仍是 canonical 同一对象。
- 旧 `Hub`、`Outbox`、`LogPeer`、`LogRelay` 是注入原默认设施的薄子类，**不是同一个类对象**；业务方法继承 canonical，没有第二份业务实现。
- Hub 内部产出的 outbox 是 `cloud.relay.Outbox`，不能用旧构造子类做 isinstance 判断；原 video relay 的实际夹具已改为 canonical 类型，断言保留。
- 原 `senior_control.cloud.app.create_app` 为薄函数装配，业务在 `cloud.app.create_app`；迁移对象版本/SQL保留，不使用全局工厂或可变注册表。
- 旧 time/asyncio 模块入口保留同一 stdlib 对象；时钟夹具显式对 canonical 模块生效。

## 范围与安装声明

实际 **42 个逻辑文件**：产品22、测试12、配置构建5、工具文档3。
业务9份、原测试/共享夹具6份迁移；九个旧兼容文件明确保留，不作为“已删除”隐藏。
其余产品是两个 init、技术 ports 和 py.typed；测试另改6份真实消费者/夹具。
配置为 cloud/root pyproject、根 uv.lock、Backend CI、Dockerfile.cloud；工具文档为本页、tests README 和原实施计划。
Git 可能把保留旧路径的业务迁移显示为新增 canonical + 修改旧文件，范围仍按真实路径计。

根 wheel 显式依赖 vehicle-cloud，uv path source 指向现有 cloud 项目；锁仅新增该本地项目，原第三方版本不变，共68包。
CI 同时构建云 wheel、发现 cloud/tests 并核对旧入口/值身份；云 py.typed 随 wheel 发布。
Dockerfile 仅补必需的 `COPY cloud ./cloud`，原 CMD/环境/安全保护不改。
**现存 Docker 上下文缺口仍未修复**：`.dockerignore` 默认全排除，尚无 cloud/contracts/ros2 本地依赖白名单；
Docker 中 uv 0.5.11 对现锁 revision 5 也未实测，C3 必须验证，不能因本片 SDK/wheel 通过宣称镜像可构建或可部署。
C2 通用诊断最终复用落点仍待定，不新增 runtime，不把技术设施加入 WS contracts。

## 验证证据

环境：macOS26.4 arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23。
定向测试使用主仓库 `.venv/bin/python`，FastAPI0.162.0、Starlette1.10.0、Pydantic2.14.1、aiosqlite0.22.1。
所有测试使用原 ASGI 内存客户端/SQLite/受管测试子进程；没有启动开发、实车或 ROS 服务。

1. RED：先把原 test_cloud 认证导入改为 cloud.auth，迁移前 collect 失败 `ModuleNotFoundError: cloud.auth`。
2. `python -m pytest cloud/tests tests/integration/test_agent_pairing.py tests/integration/test_settings_logs_wire.py tests/unit/test_runtime_diagnostics.py tests/unit/test_logging_diagnostics.py tests/integration/test_backend_observability.py tests/unit/test_live_map_geometry.py tests/unit/test_protocol.py -q`：**102 passed**。
3. 原队列/工作区/agent心跳组：运行 test_agent 的 cloud_cache_marks_offline_snapshot_stale、delayed_replies_preserve_send_cadence_and_connection，test_workspace_data 全部、test_settings_logs_review 的两组设置 settle，以及 test_manual_wire 除下述3个基线失败：**28 passed、3 deselected**，没有修改/跳过源断言。
4. `python -m pytest tests/integration/test_video.py -k 'hub_controller_selection or video_offer_on_control or video_answer_reaches or malformed_or_misdirected or video_frame_leaves or video_frames_remain or ending_the_viewer or close_frame_survives or close_is_only_sent or viewer_can_close or agent_close_for or full_controller_queue' -q`：**14 passed、26 deselected**，只选既有relay场景，无新视频算法场景。
5. 第一轮包含全部 manual/review 原用例的命令：83 passed、4 failed；四项见下，不隐藏为通过。上述三组通过用例互不重复，共144项；相同选择合并复跑为144 passed、3 deselected（7.01s）。
6. `MYPYPATH=cloud/src:contracts/src:src:ros2/src/senior_mapping:ros2/src/senior_localization:ros2/src/senior_motion_control:ros2/src/senior_motion_estimation:ros2/src/senior_perception mypy cloud/src/cloud`：11份源码通过；相同路径下全 root+cloud 检查仍有38个错误记录，与 git archive 的859c954基线逐条比较完全相同，不能称全仓类型检查通过。
7. `ruff check` / `ruff format --check` 覆盖全部34份迁移/改动Python：通过；按原CRLF处理的 git diff --check 通过；六份叶子业务 AST（auth/db/pairing/registry/limits/snapshots）除导入外完全相同，云包无车端/ROS实现导入。
8. `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --check --offline`：68包；同缓存下 `uv build cloud --offline --out-dir /tmp/c1-wheels`、根 uv build、contracts uv build 均成功产生 sdist/wheel。
9. 仓库外 `/tmp` 的纯云 `/tmp/c1-cloud-venv` 安装 vehicle-cloud+contracts wheel及真实第三方依赖：14包、pip check通过，所有云模块可导入、find_spec(senior_control/vehicle_manager/rclpy)均无，分发元数据只含4项真实依赖；此轮 wheel 解析到 Pydantic2.14.3，与测试环境2.14.1不同，均在声明范围内，未称 frozen 安装。
10. 原 `/tmp/vehicle-batch-one-venv` 安装根/云/契约wheel：23包、pip check通过；仓库外导入并构造旧 app factory，确认数据库未打开、原4/5迁移、同一值身份、薄类方法继承、原有界诊断队列；原 tools/verify_batch_one_install.py SDK检查通过，native/SLAM安装仍明确 UNVERIFIED（无ament/ROS）。

安装命令采用 `uv pip install --offline --python <venv>/bin/python <wheel...>`，缓存先从本机既有 uv cache 复制实际第三方 wheel 归档到 /tmp，未改全局缓存或联网。
初次缓存不全、默认缓存写权限和一次错误的旧 contracts wheel 路径均导致安装前置失败，补本地已有缓存/当前契约wheel后重跑成功；未把前置失败记录为产品通过。
SDK安装检查只构造 factory、不进入 lifespan，不是独立云服务或 Docker 运行验收。

### 已复现的旧失败与未验证项

从859c954用 git archive 取到 `/tmp/c1-baseline`，在该目录以相同解释器单独重跑，四项同样失败：

- test_settings_logs_review::test_directory_log_database_does_not_abort_composed_control：原组合夹具断言旧 IPC handler 集应含 `learning_capability`，与当前装配不符。
- test_manual_wire::test_real_authenticated_relay_agent_ipc_and_native_owner：原 CommandNode 夹具缺 `_output_incarnation`。
- test_manual_wire::test_estop_ack_does_not_recover_until_real_t5_state_then_new_input：同上。
- test_manual_wire::test_inflight_estop_retransmits_original_event_until_output_owner_confirms：同上。

未给旧夹具补假属性、未弱化断言、未迁后续控制职责。
未验证Docker镜像/uv0.5.11锁兼容、真实Uvicorn网络/TLS/反代、Linux/systemd、ROS/ament/DDS与实车仿真；
B5原生ROS实测缺口继续保留，未跑全仓测试，未部署/推送，C2/C3须另批审阅预报。


## C2 独立云设施与应用装配

基线 `ee735b0`，当前云端可以只安装 vehicle-cloud、senior-contracts 和第三方依赖，
构造真实 ASGI app 并运行 lifespan；这不代表生产 Docker 已切换或已部署。
新增/修改的实际位置如下，其余 C1 业务、旧兼容入口与车端/ROS实现不变：

```text
cloud/src/cloud/
  application.py             # 独立装配；只注入本云进程的设施
  config.py                  # CloudSettings；secret/role/diagnostics原约束
  storage.py                 # 同一SQLite连接、DiagnosticLock及原取消/关闭规则
  migrations.py              # 原备份、连续版本、BEGIN IMMEDIATE、rollback算法
  historical_schema.json     # v1–v5全部原SQL值和逐语句SHA256；数据，不是Store实现
  schema.py                  # 从安装资源读取上述历史数据
  diagnostics.py             # 原有界队列/等待诊断；云自己的进程owner
  ports.py                   # 接入说明同时覆盖云装配和旧显式注入
cloud/tests/
  test_cloud.py              # 原认证/配对/epoch/lifespan改走独立装配
  test_cloud_epoch.py        # 原重开epoch使用云Database
  test_schema.py             # 已选历史格式验收：0–5起点重开、凭据/epoch不变
```

实际19文件：产品8（新增7、改ports1）、测试6（新增schema1、修改5）、
配置2（cloud pyproject、Backend CI）、文档3（本页、tests README、实施计划）。
无删除/目录迁移；没有新顶层项目或修改WS contracts。

### 所有权与公开用法

```python
from pathlib import Path
from cloud.application import create_app
from cloud.config import CloudSettings
from cloud.storage import Database

# admin_password_hash沿用原scrypt格式，由调用方提供；这里不读取环境或启动服务。
app = create_app(
    CloudSettings(secret="explicit-secret"),
    database=Database(Path("/var/lib/senior/cloud.db")),
    admin_password_hash=admin_password_hash,
    allowed_origins=("https://cloud.example",),
)
```

app 构造不开库；原 cloud.app lifespan 按 is_open 判定连接所有权，打开/迁移/关闭行为不变。
传入已打开连接仍不被 lifespan 关闭；CLI/HTTP/WS业务仍是 C1 同一实现。
CloudSettings 保持 extra=forbid、非空secret、隐藏错误输入、有限数值、cloud角色和原诊断设置约束，
只接受显式数据，不新增隐式环境读取或把设备配置/车端通用加载器复制过来。
`senior_control.cloud.app.create_app` 继续原设置/DB/迁移/诊断注入路径，没有被偷偷换成新设施。
旧实例与新独立云设施分别归各进程owner，不要求跨项目类身份相同，也不互相导入。

### SQLite与历史格式

storage保留原 timeout=1..60000ms、PRAGMA busy_timeout/foreign_keys/WAL/synchronous FULL，
asyncio.shield初始化、取消等待连接后关闭、失败复位entered标记及原关闭算法。
写锁仍为原技术DiagnosticLock语义，连接/写锁唯一；不带HomeExecutionCoordination、Home锁或任务执行状态。
migrations保留备份到同路径`.backup`、原deadline/pages/progress、备份取消等待，
以及同锁下连续版本检查、未知版本拒绝、BEGIN IMMEDIATE与BaseException rollback。

historical_schema.json冻结的是原SQL**运行值UTF-8字节**（包括原首尾空白），不是重新排版SQL：
v1来自senior_mapping.schema的4条map journal声明，v2来自waypoints.store的2条声明，
v3来自tasks.store的4条声明，v4来自云认证/配对/设备的3条声明，v5为原epoch ALTER。
每条SQL附SHA256并与原源声明静态提取逐字节核对；全部5个版本和14条原语句保留。
历史数据包含旧表不代表云端运行Home/任务/导航，未复制那些Store或执行算法；后续变更只能追加新迁移，
不得编辑既有快照或重编号。数据库路径由原调用方显式提供，生产路径/格式不改。

### 最小云队列诊断

诊断文件提取QueueSnapshot/QueueWarningPolicy、有限统计、RuntimeDiagnostics的queue/wait/pump、
PendingCounter（锁等待所需）、DiagnosticDeque、DiagnosticAsyncQueue、DiagnosticLock及进程配置owner。
保留128流/owner上界、等待元数据4096上界、饱和计数、weakref退休、同名最严重owner、
队列容量/丢弃/覆写/等待字段、瞬时越线、抑制/恢复、递归日志隔离、100ms泵和原5s DEBUG周期。
真实cloud.app的控制出箱及日志32/4队列、数据库锁全部使用这个owner，绝非普通deque/Queue替代。
queue变更只记状态，pump在队列锁外输出并吞掉诊断sink故障；cloud拥有自己的logger命名空间。

没有复制Age、传感器时钟/帧龄阈值、age/sensor_attempt/raw去重、传感器summary状态、
DiagnosticPending业务字典、车端环境加载或ROS执行器；DEBUG仍保留原字段名，
summaries/sensor_results为空，等待/duration统计与告警原样。
云周期默认5.0s是原默认值的独立云配置值，不再读取车端传感器资源。

### 定向命令、版本与结果

环境同C1：macOS26.4 arm64、CPython3.12.15、pytest8.4.2、ruff0.17.3、mypy2.4.0、uv0.12.23；
测试FastAPI0.162.0/Starlette1.10.0/Pydantic2.14.1/aiosqlite0.22.1。
所有 SQLite 与 ASGI 内存回归均未启动开发/实车/网络服务。

- RED：原test_cloud改导入cloud.application，迁移前collect报ModuleNotFoundError；独立装配完成后14项认证/epoch原回归通过。
- `python -m pytest cloud/tests tests/integration/test_storage.py tests/unit/test_runtime_diagnostics.py tests/unit/test_settings.py -q`：**114 passed**（4.42s）。
  原8组SQLite用例在vehicle/cloud真实后端上复用原断言，覆盖未知版本、失败回滚、disk full、锁等待、PRAGMA/备份、取消初始化；
  原队列告警/等待/突发/多owner/递归隔离用例也在两个真实诊断后端上复用原断言，车端Age/环境用例仍只针对原owner。
  原cloud设置拒绝用例对旧加载器和新CloudSettings参数化，未改车端配置行为。
- C1同一选定跨端集（含新增6个历史起点与队列后端参数）复跑：**160 passed、3原基线失败deselected**（6.62s）；
  选择仍为C1列出的认证、快照/字段省略、设置/日志wire、队列、安全消息和私有视频relay，不增加无关算法情景。
  两组存在重复，不能相加宣称274项唯一通过。
- 四项C1既有控制夹具失败本轮单独复跑仍4 failed：settings_logs_review的learning_capability集合断言，
  manual_wire原三项缺_output_incarnation；未补假属性或弱化断言，名称和原证据见上方C1记录。
- `MYPYPATH=cloud/src:contracts/src mypy cloud/src/cloud`：17份源码通过；未跑全仓类型检查，C1的38项旧错误未宣称修复。
- 23份相关Python的ruff check/format --check通过，git CRLF差异检查通过。
  静态核对原SQLite生命周期/迁移/备份和原队列/锁/计数/聚合AST保留、全部SQL字节与14条SHA256匹配；
  原src/senior_control、ROS、contracts、deploy均无diff，云无反向执行实现导入。
- `UV_CACHE_DIR=/tmp/vehicle-uv-cache uv lock --check --offline`：68包不变；
  `uv build cloud --offline --out-dir /tmp/c2-wheels`：sdist/wheel成功，JSON与py.typed随wheel安装。
- `/tmp/c2-cloud-venv`从wheel仅安装云、契约与第三方14包，pip check通过；
  从`/tmp`实际执行CI新增的Python验证块：无senior_control/vehicle_manager/rclpy包，
  构造独立factory确认未开库，两次实际lifespan均查询PRAGMA user_version=5并在退出后关闭连接，exit=0。
  该环境Pydantic2.14.3，属于原声明范围，和测试环境2.14.1差异保留，不称frozen安装。
- 原SDK环境仅重装新云wheel，仓库外原bootstrap factory/原诊断队列及兼容身份检查继续通过；
  安装后不进入旧lifespan、不启动服务，旧入口实现本轮没有修改。

CI新增真实纯云wheel/lifespan检查，cloud pyproject补独立测试路径与asyncio配置；没有修改Docker接线。
本轮自审后交负责人亲审，未做全仓测试或部署；C3仍须处理既有.dockerignore、生产bootstrap切换、
旧uv0.5.11与锁revision5、真实Docker/Uvicorn/TLS/反代；Linux/ROS/实车仿真及B5原生实测缺口均未覆盖。


## C3 生产镜像与独立锁接线

基线 `05a2d8b`，实际14文件：产品1、测试4、配置构建6、工具文档3；新增2、修改12，无迁移/删除。
唯一产品新增为 `cloud/src/cloud/bootstrap.py`，其余C1/C2业务、WS契约及SQLite/诊断实现不改。

```text
cloud/src/cloud/bootstrap.py         # python -m cloud.bootstrap；显式环境factory与main
cloud/pyproject.toml                 # server extra及required uv==0.12.23
cloud/uv.lock                        # 独立20包跨平台闭包，server版本/轮子hash固定
uv.lock                              # 根68包不变，仅记录云server extra元数据
.dockerignore                        # 默认拒绝，仅cloud/contracts必要源码/声明与Dockerfile
deploy/Dockerfile.cloud              # uv0.12.23 + Python3.10.15，两个项目frozen安装
.github/workflows/backend.yml        # 同uv版本，离仓库frozen生产入口检查
cloud/tests/test_cloud.py            # 原epoch跨app重开用例改走生产bootstrap
tests/deploy/test_jenkins_config.py   # 原镜像依赖/保护断言随实际安装归属更新
tests/deploy/test_jenkins_deploy.sh   # 原保护改用私有构建输入，不要求现成web/dist
tests/integration/test_deployment_config.py # 原部署保护及bootstrap不打开DB检查
docs/operations/deploy.md            # 使用独立云环境生成原scrypt hash
docs/superpowers/plans/2026-11-19-vehicle-architecture-refactor.md
```

### 构建与运行图

Jenkins/Compose路径不改：`docker build -f deploy/Dockerfile.cloud ... .`仍从仓库根构建。
Docker只COPY cloud、contracts；上下文白名单只包含这两包pyproject/锁/src和原Dockerfile，
不引入根管理项目、根src、ROS、开发venv、测试、网页产物或密钥。
Hatch仅需这两包的声明与源码，历史SQL JSON/py.typed仍随wheel进入安装。

Docker执行 `uv sync --project cloud --frozen --no-dev --extra server --no-editable`，
环境路径为 `/app/cloud/.venv`，不再同步根项目或用第二条未锁pip命令装server。
固定uv0.12.23与本机/CI一致，Python基础镜像仍3.10.15-slim-bookworm。
Uvicorn0.32.1和WebSockets15.0.1固定在server extra与cloud锁；WebSocket实现导入检查保留。
初始化锁曾解析出较新的公共库版本，最终重新选回根锁已验证的FastAPI0.162.0、Pydantic2.14.1、
AnyIO4.15.1、annotated-doc0.0.5、typing-inspection0.4.4；公共包版本集合与根锁一致。
根锁只新增云extra的元数据，不升级原68包；云独立锁20包包含平台/解释器条件分支。

镜像CMD改为 `python -m cloud.bootstrap` → 独立 `cloud.application.create_app`。
factory只在调用时读取SENIOR_SECRET、DATABASE_PATH、ADMIN_PASSWORD_HASH、ALLOWED_ORIGIN，
main沿用TRUSTED_PROXY；无额外环境读入规则、端口/路径回退或全局app构造。
main才按server extra加载Uvicorn并保留 `host=0.0.0.0, port=8000, proxy_headers=True` 与原forwarded_allow_ips。
旧兼容bootstrap仍可显式调用，但生产镜像没有安装它或车端/ROS包。

原UID/GID10001、/var/lib/senior目录创建/chown、EXPOSE8000、
Compose中/var/lib/senior/cloud.db卷和环境、只读/no-new-privileges、
Caddy地址/TRUSTED_PROXY、DDS隔离、Jenkins本地daemon/发布锁/健康和current保护均保持。
Jenkins脚本和Compose本身未改，生产env字段、凭据格式、SQL与线上消息没有变化。

### 验证证据与缺口

环境：macOS26.4 arm64，宿主CPython3.12.15与已安装3.10.22；pytest8.4.2、ruff0.17.3、mypy2.4.0、
uv0.12.23；Docker CLI29.5.2、Compose v5.1.3。

- RED：先把原test_cloud_image_installs_websocket_protocol更新为真实独立安装断言，迁移前因缺server pin失败；改实际接线后通过。
- `python -m pytest tests/deploy/test_jenkins_config.py tests/integration/test_deployment_config.py cloud/tests/test_cloud.py cloud/tests/test_cloud_epoch.py cloud/tests/test_schema.py -q`：**35 passed**（5.85s）。
  复用原哈希/CSRF/Origin/epoch、真实SQLite重开、Compose凭据/代理/DDS与部署保护，新增bootstrap显式环境及不打开DB检查属于本批入口验收。
- `bash tests/deploy/test_jenkins_deploy.sh`：exit=0，原远程daemon拒绝、成功健康才切current、错误API响应不切current均保留。
  首次原夹具因仓库没有web/dist退出1；只将现有替身场景的脚本/发布输入搬到/tmp私有fixture并放最小静态输入后通过，没有生成生产网页、改部署脚本或删断言。
- `uv lock --check --offline`：根68包；`uv lock --project cloud --check --offline`：独立20包。
  初次离线因缓存无固定Uvicorn元数据失败，补实际包元数据/轮子后完成，未用静默降级或可选缺失代替。
- 将两包的pyproject、cloud锁和src复制到 `/tmp/c3-frozen/{cloud,contracts}`，
  执行 `uv sync --project /tmp/c3-frozen/cloud --frozen --no-dev --extra server --no-editable`：
  Python3.12.15真实构建/安装18包、pip check通过；最新源码重装后offline同命令再检查通过。
- 同一声明/锁复制到 `/tmp/c3-frozen310`，加 `--python 3.10`：Python3.10.22真实frozen安装19包、pip check通过。
  两个环境共同版本与根锁一致；Starlette按原Python条件分别为1.10.0/1.7.0，3.10还含exceptiongroup。
- 从 `/tmp`在上述两个解释器执行CI新增的同一个验证块：模块实际来自site-packages，
  senior_control/vehicle_manager/rclpy均不存在，Uvicorn/WebSockets版本和原websockets_impl可导入，
  生产bootstrap实际构造app且未建DB，随后两次真实lifespan完成SQLite v5迁移并重开；
  Uvicorn.run仅在测试中拦截，断言原host/port/proxy/forwarded参数，**没有监听或启动服务**。
  此为冻结安装与进程内入口验证，不称完整网络服务/镜像验证。
- `MYPYPATH=cloud/src:contracts/src mypy cloud/src/cloud`：18份源码通过；四份相关Python lint/format通过，
  CI YAML/Python块解析和CRLF差异检查通过；公共锁版本核对、旧业务/Compose/Jenkins生产脚本无改动。
- 实际执行 `docker build --progress=plain -f deploy/Dockerfile.cloud -t senior-cloud-c3:local .`：**exit=1**，
  原因是 `Cannot connect to the Docker daemon at unix:///Users/zhangshunjun/.docker/run/docker.sock`。
  首次受限socket读报permission denied，随后授权构建仍确认daemon不可连接；没有启动Docker或重配daemon。
  未产出本批镜像，因此没有运行镜像内一次性导入，也未验证镜像tag拉取、Linux Python3.10.15构建/安装或镜像UID行为。

所有测试只用ASGI内存客户端、SQLite或原Jenkins替身；没有启动开发、生产、ROS或实车服务，没有部署/推送。
未跑全仓或修复C2已登记的四项控制夹具失败/原38项全链类型错误；本片未触及那些实现。
Docker可用后仍须完成同一真实构建与不启服务的镜像一次性导入/入口检查；HTTPS/WS/TLS/反代和现场网络验收仍保留。
