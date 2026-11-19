# 车端管理测试

M1a迁入原tests/integration/test_vehicle_settings.py，62项原用例及原断言不变，仅改canonical导入。
继续使用原Root Database真实连接和写锁，覆盖持久/并发/幂等/commit取消、旧配置、损坏不可用、runtime gate、DEBUG恢复。
没有复制测试或拆假数据库。

跨端test_settings_logs_wire、settings_logs_review和ROS手动命令相关用例保持原位置，
本片只在它们的真实旧兼容调用上做定向回归，不为改变目录增加无关场景。
独立manager wheel与完整旧SDK的安装/类型身份检查复用既有tools/verify_batch_one_install.py，
新进程确认单独导入公开值不加载service/store/Database，旧新包入口服务类仍同一对象。

[manager记录](../README.md#命令版本与结果) 给出实际命令/版本、75项定向通过、
基线同样复现的50项退役控制夹具失败及38项旧类型错误、真实ROS/安装环境限制。
其他管理/监管测试在各自实现迁移时再归入，不先搬空覆盖。

M1b将原`tests/integration/test_vehicle_logs.py`完整迁入本目录，38项原用例的输入、断言和时限不变。
models/store（含测试内局部prepare_entry）改从canonical导入；service仍通过生产实际使用的
旧Compatible构造入口装配原诊断owner，没有构造假诊断或假属性。
覆盖授权/撤销、过滤分页/游标重开、脱敏、损坏库拒绝、存储失败、队列溢出、订阅上界和控制隔离。
跨端wire/review及原runtime诊断用例仍归原位置，未为目录调整新增场景。

本轮完整选择集72项通过（3.76s），包含38项日志及原wire/review/诊断用例，不与子集相加。
之前的慢速执行出现flush/接收/恢复时限失败，原断言保留；基线对照和复跑详情见
[M1b记录](../README.md#m1b-车辆日志owner迁移)。独立manager与旧SDK安装检查通过，真实Linux/ROS未验证。

M2a将原`tests/unit/test_database_ownership.py`原样迁入本目录，继续用生产旧Database构造入口，
两项原断言确认重入拒绝不丢原连接、退出确实关闭以及同对象可重开。
跨车云`tests/integration/test_storage.py`仍在原位置，车辆fixture改为canonical Database并显式注入原DiagnosticLock；
云fixture、所有用例体与时限不变，未增加场景或复制SQLite用例。
SQLite/所有权22项、包含设置和地图原事务139项、原Home协调/guard迁移6项通过；子集不重复累加。
仓库外安装复用原安装工具新增的`--manager-storage-only`模式，实际开库/迁移/失败回滚/备份/core迁移/重开，
默认模式继续验证旧入口及诊断owner；详见[实际命令与限制](../README.md#m2a-sqlite引擎与core-schema)。
这些检查不代表Linux/ROS/真实SLAM或车辆运行通过。

M2b只归档journal SQL数据，没有迁测试或改原fixture。
原task_lifecycle_store与agent的提交确认、失败、去重、冲突和tombstone失败回滚定向30项通过；
安装工具验证旧journal常量与canonical agent_schema是同一tuple，独立manager无Root时可导入该数据。
完整命令、SQL字节hash及只读后续清单见[manager M2b记录](../README.md#m2b-journal-schema数据)。
下一片候选仅schedule/lifecycle；test_task_lifecycle可随实现迁入，混合TaskRunner的test_schedule先留原位置改必要导入，
仍须负责人预报后实施，不为纯核心迁移提前搬共享值或放宽原保护。

M2c已按预报迁入原纯`test_task_lifecycle.py`，只改canonical import；
原`tests/unit/test_schedule.py`混合TaskRunner/models仍在原位，Schedule类及稀疏cron的模块patch改指向真实canonical实现。
首轮旧patch路径失败后只修import，原断言、3000次上界和输入保持；最终原schedule/lifecycle/服务层选择集129通过。
服务层用例仍在原位置，覆盖已提交后dispatch、锁外IO、未知查询和迟到ack，不复制测试或增设业务情景。
纯manager与旧SDK安装分别复用原工具的`--manager-tasks-only`和默认模式，确认按需导入隔离与公开身份。
迁入lifecycle带来的1项旧类型错误、全链38项及真实ROS未验记录见[实际命令与限制](../README.md#m2c-纯schedule与lifecycle)。

M2d将原test_schedule随models/runner迁入本目录；原Root MapRef仍是实际输入，未造替代值，patch继续指向原canonical Schedule。
仅改模型/Runner imports与排序，所有时间、准入、比较、3000次稀疏搜索上界和断言保持。
原schedule/runner/lifecycle及任务模型/存储准入选择集216通过；两处已批准的静态类型接线后追加原direct_navigation回归218通过。
原集成/任务存储/ownership用例仍原位，安装工具确认原对象、旧公开身份和无反向依赖；最终SDK另核对真实TaskStore解码、
IpcNavigation航点读取和qualified_reference原MapRef对象，不连接IPC、不增pytest场景。
局部cast不是运行时校验；manager原1项、全链原38项及ROS未验记录见[manager M2d](../README.md#m2d-纯任务models与runner)。

S1将原test_task_runner原样迁入本目录，继续通过生产旧TaskStore构造测试canonical唯一SQL/事务实现，
原执行/Runtime合成夹具和保护不改；test_task_lifecycle_store留原位，只改已迁TaskDefinition类型导入。
原Store/消费入口首组117通过；包含纯决策、同连接接受/并发/CAS/取消、服务层与agent持久回滚的248项通过。
实际命令和13个解析函数/45处异常构造的同义审计见[manager S1](../README.md#s1-taskstore持久化)。
安装工具核对旧构造非同类但业务方法同一实现、TASK_STATEMENTS/原MapRef/DomainError身份，并真实开库create/get/CAS/重开。
不新增pytest场景或假数据库/错误类型；原1/38项类型错误及真实ROS未验保持。

S2两份原test_task_lifecycle_store/service已迁入本目录，canonical实现/旧入口使用同一类与异常对象。
原硬退出WAL用例只改子进程导入路径及继承父进程真实sys.path，避免落到旧安装或找不到manager，
os._exit的23/24/25阶段、15秒上界、原断言与其他输入不变，仍是实际子进程退出而非mock。
首轮缺manager路径的3项失败修正后原核心55通过，含原Store/runner/ownership/terminal outbox的完整221项通过。
安装工具验证真实TaskStore/DB对象、Pose序列化与receipt/effect持久化及旧类/异常/SCHEMA身份，不执行RPC。
manager原2项、全链原38项和实际ROS未验见[manager S2记录](../README.md#s2-生命周期持久与服务)。

S3迁入原test_terminal_release_outbox，仅改生命周期canonical imports，原用例体/断言AST保持。
原test_task_ownership因其他集成测试直接复用其夹具保留原位，仅将Executor/Runtime类型导入指向canonical；
仍通过真实build_task_runtime/原IpcNavigation装配，没有假属性、假DB或新增场景。
原端口步骤34通过、装配96通过，完整选集252通过/11失败，不重复累加。
11项short_admission失败在22741b5基线副本同样复现：旧IPC替身未接受learning_capability；
本片不修改其断言或静默跳过，不将该组算作手动隔离通过。
恢复/CAS/未知只查询、缺资格/切图、迟到及终端义务保护继续复用原用例。
命令、同环境基线、独立manager/旧SDK安装身份和真实ROS缺口见[manager S3记录](../README.md#s3-导航效果任务执行与调度)。

M2e将原test_waypoint_store迁入本目录；所有原用例体与断言保持，
仅把_operation_helpers从已不存在的同目录test_map_operations改指现有ros2/tests同名文件。
原494b794基线13通过/2项旧路径失败，本片15通过；包含原Home/地图事务/工作区/导航入口的选择148通过。
没有迁Home业务、补假属性或新设场景；原地图同连接事务与foreign Home保护继续使用真实SQLite和已有回调。
独立manager只验证存储接口无root/ROS依赖；旧SDK实际验证原值/构造器、保存读取重开及IpcNavigation任务目标读取。
命令、SQL/AST审计、原2/39项类型错误和真实ROS未验见[manager M2e记录](../README.md#m2e-waypointstore存储迁移)。

M2f将原test_direct_navigation逐字节迁入本目录，仍通过生产旧Compatible构造，
原goal/free-map/代次/cancel及pending/confirmed Home输入和断言不变。
混合test_agent_ports留原位，继续验证Home捕获的原定位、代次及不额外添加安全权限语义；
0.5秒表达式和采样顺序另以原方法AST核对，未新增时限边界情景。
首组选定24通过；含原workspace、真实任务装配及终端义务的完整选择51通过，不重复累加。
旧HomeService用例/业务与协调未迁或启用；没有新情景、假属性或断言弱化。
独立安装与实际Root接线、AST/类型对照及真实ROS缺口见[manager M2f记录](../README.md#m2f-生产home捕获与管理目标准入)。

M2f3将原test_home逐字节（含CRLF）迁入本目录，原fixture、输入、ROOT定位与断言不变。
原64项通过；含原地图/航点/生产Home捕获/管理返航和任务装配的最终选择161通过，不重复累加。
没有启用旧服务、加新情景或弱化旧token/未知取消/迟到callback/断链和同ref换代保护。
SQL/方法/原值类核对、同DB/robot协调锁和旧入口真实SQLite安装检查见[manager M2f3](../README.md#m2f3-旧homeservice业务与guard)。
Root生产IpcHomePort/DirectNavigation保持原装配，真实ROS/物理退场及车辆运行仍未验证。


M3A将原test_websocket_link、test_runtime和test_agent_pairing逐字节迁入本目录，
原fixture/导入/断言保持；pairing仍复用原test_cloud共享夹具，不新建替身业务或测试情景。
核心20项、完整选定116项通过（包含核心，不累加）；初次沙箱原localhost bind被拒的一项，允许回环测试后通过。
原WS nonce/epoch/revoke/超时与容量、监管阻塞/失败/关闭、配对文件权限/无覆盖、journal事务/失败重放、
设置日志wire和任务离线隔离复用原用例，不因迁移改变原网络或控制保护。
原安装工具在独立base/agent-extra与旧SDK环境从/tmp运行，实际核对原构造/值身份及临时SQLite journal提交；
未启生产连接或发运动，真实Linux/ROS与历史类型/夹具缺口见[manager M3A记录](../README.md#m3a-管理连接监管配对与journal基础)。


M3B将原test_vehicle_state、test_runtime_reporting、test_workspace_data逐字节迁入本目录，
共享integration夹具仍用原pytest pythonpath，不改输入/断言或新造场景。
原快照/显示/工作区、agent/设置日志wire与任务装配核心最终108通过；
追加原手动/故障隔离/日志review/配置用例后152通过、9失败（包含核心，不累加）。
8b59521只读归档同环境重跑复现完全相同9个失败ID，缺_output_incarnation/ros_setup、
group_grace签名和拒绝learning_capability的旧夹具保持，不能据此宣称真实手动/子进程整链通过。
原安全修订/快照失效/目标资格/未知只查询保护不改；52个业务方法/函数及其余组装/执行类体审计保持。
安装和同DB/类型/效果身份、完整命令/环境/已知失败见[manager M3B记录](../README.md#m3b-管理快照显示工作区与请求分发)。


M3C将原test_agent与test_agent_config迁入本目录；Config原字节不变，
Agent仅改原ControlledAsyncio的patch模块为vehicle_manager.agent.main，实际方法已迁，其他输入/断言不变。
原Root CLI/装配的class/Connector patch仍由每次调用显式构造的真实bindings保留，不补假属性。
核心首次92通过/1项原socket沙箱限制；允许测试socket的完整选择158通过/9失败，
94bc1d9只读基线相同9个失败ID；旧夹具问题和未验执行链不算通过。
媒体原选集5通过（其他35未选），无新测试情景、假owner或保护弱化。
48方法/组装/媒体/值身份审计、离仓库manager/SDK和原CLI help检查及完整命令见[manager M3C](../README.md#m3c-agent会话与生产管理编排)。
