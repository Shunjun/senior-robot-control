# 独立云端

## C1 已迁业务，独立生产部署尚未完成

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
