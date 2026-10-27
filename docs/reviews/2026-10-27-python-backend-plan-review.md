# Python后端规划审查

日期：2026-10-27。对象：实施计划与统一设计，非产品代码审查；当前没有后端产品代码和运行测试证据。

结论：原规划的src布局、按领域拆分、依赖注入、车端权威存储、边界校验与失败场景测试方向正确。但还缺少运行与恢复约束，不能直接称为已符合完整Python工程最佳实践。下列8项已经修订进实施计划，不需要推翻架构，也不需要增加Redis/Celery/完整ORM。

优先级针对“若按缺少约束的原稿实现会产生的风险”，不是已经复现的代码缺陷。修订内容尚待实现和测试证明。

## 1. [P1] ROS、CPU计算与asyncio的执行边界不明确

原稿说“外部操作均异步”，同时提供同步导航/存储接口，未定义rclpy Future与网络循环的关系。仅添加async关键字无法避免spin、同步SQL或地图计算阻塞心跳；CPU繁忙也可能延迟停车。

修订：同步纯核心、异步I/O分开；独立ROS安全进程与代理通过受限Unix socket通信，rclpy回调不等待网络，CPU密集工作独立进程；安全门独立按单调时钟撤销运动资格。补充慢ROS、队列背压与后台失败停车测试。

落点：实施计划“同步核心、异步I/O与ROS边界”；任务4、5、6、13、14、15。

## 2. [P1] 持久化去重不等于运动副作用只发生一次

原稿已经考虑任务去重和结果未知，但未明确SQL提交与ROS动作之间的故障窗口，以及地图SQL回滚无法撤销文件/SLAM副作用。

修订：操作先落盘、内容hash冲突检查、单worker claim、阶段日志和execution_id对账；无法确认的动作保持needs_review，不重发。运动命令不自动重试。地图操作有文件切换日志和必要fsync，恢复不默认启动运动；长操作不依赖FastAPI BackgroundTasks的进程内存活性。

落点：“SQLite、操作日志与恢复”；任务9、12、14；新增崩溃窗口、取消结果未知、同ID不同内容测试。

## 3. [P1] 取消、异常和关停后的资源释放不够具体

原稿未定义create_task的拥有者与异常收集，也没有保证视频/DB/网络关闭前撤销运动资格。协程超时不证明线程或ROS目标已经停止。

修订：FastAPI lifespan、AsyncExitStack、受监督任务集合；先禁止运动再关闭资源；取消后await并传播CancelledError，关停有界。补充SIGTERM、任务异常、编码器卡顿与连接清理测试。

落点：“生命周期、错误与观测”；任务13、14、16。

## 4. [P1] 多ASGI worker会分裂内存中的设备连接与控制租约

原稿没有部署进程约束。在未引入共享协调的情况下，两个worker可能分别持有浏览器和小车连接，导致路由不可见或控制权判断不同。

修订：首版一个ASGI worker/服务实例，支持该进程内多车异步连接；入口拒绝多worker配置，持久化目录加进程锁。多实例水平扩容明确不在本期范围。

落点：“生命周期、错误与观测”；任务13、20。

## 5. [P2] SQLite并发、迁移与备份策略未定义

仅选SQLite不能保证异步不阻塞或多写者正确；WAL不是无限并发，复制运行中的单个db也不构成一致备份。

修订：aiosqlite、连接生命周期归属、单写入锁、短事务、foreign_keys/WAL/FULL/busy_timeout；事务内不等待外部调用；有序schema迁移与backup API；真实临时文件覆盖锁冲突、磁盘满与迁移失败。

落点：“SQLite、操作日志与恢复”；任务1、9、10、12、13。

## 6. [P2] Python解释器与ROS二进制兼容没有落到部署门槛

原稿只要求Python 3.10+和依赖锁，没有约束ROS解释器ABI，也没有防止在云端装入Torch/ROS等无关重依赖。本地当前存在Python3.12/3.14，不代表可运行目标Humble的rclpy。

修订：核心最低3.10，CI3.10/3.12；ROS使用系统匹配解释器和独立环境，云端/agent/vision extras分组，rclpy通过系统ROS安装；uv.lock与ROS包版本清单分别记录，补充干净wheel和可选依赖隔离测试。

落点：“运行环境、依赖与质量门槛”；任务1、2、20、21。

## 7. [P2] 配置、异常、DTO与类型检查仍偏原则性

原稿将全部领域模型集中到protocol.py，且签名含宽泛dict，容易形成大文件并让边界与业务耦合；配置优先级、导入副作用和标准错误映射未明确。

修订：共享值类型与领域DTO分离，协议为判别联合模型；配置显式加载并注入，秘密无默认值；导入不创建连接/任务；错误分型并统一映射；Ruff、mypy strict、Python3.10目标语法与CI建立为完成门槛。没有实际消费者前不搭建通用Repository框架。

落点：“共享契约与规划决定”“任务接口归属与补充验证”；任务1及各领域任务。

## 8. [P2] 缺少用于定位现场故障的统一观测规范

原稿有操作日志和连接质量，但未定义跨网络/ROS关联、日志秘密脱敏和周期/队列饱和指标，难以定位停车迟延、数据库锁、打滑或重连风暴。

修订：结构化日志携带robot/request/operation/execution ID；脱敏包括校验失败输入，指标使用有界标签；观测事件循环延迟、安全周期、DB等待和队列深度。暂不引入完整外部可观测平台。

落点：“生命周期、错误与观测”；任务1、14、20、21。

## 应用的Python Development技能

- [项目结构](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-project-structure/SKILL.md>)、[设计原则](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-design-patterns/SKILL.md>)、[反模式](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-anti-patterns/SKILL.md>)。
- [异步](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/async-python-patterns/SKILL.md>)、[后台任务](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-background-jobs/SKILL.md>)、[韧性](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-resilience/SKILL.md>)、[资源管理](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-resource-management/SKILL.md>)。
- [配置](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-configuration/SKILL.md>)、[错误处理](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-error-handling/SKILL.md>)、[类型安全](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-type-safety/SKILL.md>)、[代码规范](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-code-style/SKILL.md>)。
- [测试](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-testing-patterns/SKILL.md>)、[打包](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-packaging/SKILL.md>)、[uv](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/uv-package-manager/SKILL.md>)、[可观测性](</Users/zhangshunjun/.codex/plugins/cache/claude-code-workflows/python-development/1.2.3/skills/python-observability/SKILL.md>)。

技能示例不是可直接照抄的规范：本项目不采用模块导入时创建配置单例/退出进程的示例，不将所有CPU任务放入线程，不因后台任务示例使用Celery而添加消息队列，也不在Python3.10直接使用3.11+专有API。

## 核验依据与范围

- [Python3.10 asyncio文档](https://docs.python.org/3.10/library/asyncio-task.html)：任务、取消、超时与跨线程调度边界。
- [FastAPI lifespan](https://fastapi.tiangolo.com/advanced/events/)：启动/关闭资源统一管理。
- [FastAPI BackgroundTasks](https://fastapi.tiangolo.com/tutorial/background-tasks/)：请求后后台工作的使用范围；本项目持久化与恢复是额外设计要求，不将该机制当持久队列。
- [Python sqlite3文档](https://docs.python.org/3.10/library/sqlite3.html)：线程约束、锁等待与事务控制。
- [aiosqlite文档](https://aiosqlite.omnilib.dev/en/stable/)：每连接后台线程与请求队列；不等于取消调用可停止已提交SQL。

本次做了计划结构与接口一致性自查，未运行后端测试、ROS仿真、性能或安全验收。ROS官方Python包页面访问被拒绝，解释器兼容性要求保留为部署检测与Linux实测门槛，未声称已验证本机ROS。
