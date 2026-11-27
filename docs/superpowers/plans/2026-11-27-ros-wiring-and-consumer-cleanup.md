# ROS接线与消费判定统一实施计划

**基线：** `081b4ba`，功能主分支 `feat/frontier-navigation`；前批已提交，完整功能仍未验收。
**需求：** [已确认清单](../specs/2026-11-27-raw-data-filter-inventory.md)末尾讨论决定。使用writing-plans/executing-plans自行顺序实施；用户要求仅进度监督，不新增独立审核流程。

## 约束与范围

- 自定义话题/服务接线集中维护；官方话题沿官方名称，不改名。内部预期节点与namespace也来自同一配置，已有硬件profile的真实驱动来源继续由profile提供。
- 一个JSON定义源，Python直接读，C++由构建工具生成只读头，ROS YAML中的引用构建时展开；生成物不作为第二份手工维护源。
- 共用入口拓扑检查不比较实际消息GID，不在算法中重复graph查询。保留每个独立话题接收边界的来源唯一/name/ns/type检查，算法保留几何和质量职责。
- ScanLedger只按容量及来源维护账本；安全消费仍按原采集时间执行既有时效策略，不把旧观测重盖时间，不扩大原安全期限。
- 取消固定证据端点/格子的2048上限，对应持久化读取一致变更；不取消总地图/文件预算或其它未批准数量限制。
- 建图消费以原采集时间判重，不以原始sequence拒帧或因跳号清窗；图提交/CAS/身份/floor/参考切换保护保留。
- 取消preflight的map/odom额外128字符限制；非空和不同frame保留。
- SLAM队列8帧及原满队列策略不变；相机15FPS/latest、其它暂缓项不扩改。GetState不再调查，不启动整链/实车/环境suite。
- 预计产品45–60、测试6–10、配置构建6–10、工具文档3–5，总60–85文件；超出先更新预报。

## 顺序任务

- [x] 前批代码提交，排除node_modules和外部诊断资料；保留RED及未验说明。
- [x] 1. `vehicle_core/ros_wiring.json`、Python读取/入口检查、C++生成工具和原生导出头：统一内部话题及节点接线，迁移Python/C++/launch及必要YAML。官方名称不迁入自定义改名机制。
- [x] 2. ScanLedger去时效拒收/清理，保留容量；复用现有SafetyFreshness回归证明旧观测不能作当前安全依据。
- [x] 3. evidence/references移除已确认2048点/格子上限；preflight去frame长度限制；store/occupancy以采集时间消费判重，保留来源/基准变化重置。
- [x] 4. 相关单测和内部流转验证、必要原生目标编译；不重复全量/原完整case。记录实际结果、未验项、源码及提交；每阶段同步负责人后继续。

## 本次定向覆盖（最多8个新增/实质改写场景，复用有效回归另列）

1. 一个定义源生成Python/C++/YAML相同自定义名称，官方名称不改写（含编译消费生成头）。
2. 配置namespace及name/type/唯一性通过共用接收入口生效，GID不作拒收条件。
3. 账本在容量内保留超过原3秒的帧，安全消费者仍拒过期观测；容量淘汰独立有效。
4. 5000端点/格子证据与删除格子变更保存读取一致通过，不裁剪内容。
5. 长map/odom frame通过preflight参数解析，非空/不同frame旧保护复用。
6. 建图新采集时间且sequence倒退或跳跃仍累计；同帧重复不重复累计；原基准变化清窗保护复用。

已有相关pytest/原生target按实际改动选取，不为凑数量扩矩阵。纯接口格式迁移复用现有验证；新失败先找具体原因，不放宽原功能断言求绿。

## 执行记录

- 前批提交`081b4ba`共157文件；pre-commit缓存路径只读错误后改用私有UV_CACHE_DIR离线执行，hook通过并格式化7文件，无行为修改。缓存未入库。
- 最终本轮58文件：产品39、测试5、配置构建10、工具文档4（按路径去重）。需求相关新增/实质迁移7场景；另有两个已复现bug的定向回归（重复graph查询、重复采集清空累计），合计9场景。实际bug回归单列，不扩大其它假想测试矩阵。
- JSON是项目自定义接线及内部发布角色的定义源；Python/launch直接读取，C++配置头构建生成并导出，参数YAML的逻辑角色键在安装时展开为配置中的完整节点名。单节点旧self_filter参数文件改用标准`/**`匹配，避免节点改名后丢参数。官方话题及已有物理驱动profile的外部来源字段不改名、不伪造实车来源。
- 使用Python/C++共用拓扑谓词，保留各独立接收话题的入口检查；运动回调对NativeReceipt已经检查的来源不再查询graph。保留GID诊断，不增加GID认证或仿真分支。8帧队列、满时策略、相机15FPS/latest未改变。修改JSON后需重新构建并部署匹配的Python包、原生包和生成参数文件，本次不提供运行中热更新。
- ScanLedger不再按采集年龄拒绝/淘汰；容量和boot/generation边界保留。既有SafetyFreshness继续按原始测量时刻和既有配置执行安全过期，不把3秒改成其它算法的统一新期限。旧ledger年龄参数仅兼容接受，不再控制安全决策。
- `StableSupport`端点/格子及对应保存/恢复和格子移除delta取消2048上限。5000点/格子完整往返通过；保留有限坐标、内容对应、重复格检查及原整体文件/地图预算。不是5000束实车SLAM资格结论。
- 建图以采集时间消费判重，移除原始sequence大小和跳号清窗；来源/坐标基准变化仍重置。旧容量回归原“只拒绝一次”依赖跳号清窗，已改为逐个容量拒绝批次检查原子性及必要撤销fallback；原固定墙、撤销持久有效、后续帧及未退休保护断言保留。
- 本地首批26 passed；扩大相关组64 passed/1 failed/5 errors，其中5项Mac沙箱Unix socket EPERM，另1项上述旧容量次数断言。两个新增夹具错误（未经JSON编解码、误选命中格计空闲次数）先修正，未改产品求绿。错误路径选取导致一次pytest未收集测试也留在会话记录，不算产品执行。
- VM新私有`/tmp/nvwire01-20261127`：六包首编译因acquire_scan_receipt条件迁移误截回调失败（build01=2）；保留原源码/日志后恢复回调/原期限，只改谓词和话题引用，build02六包通过。Humble缺少所选参数读取API的工具smoke在启动单测前失败，改普通YAML及实际Node构造签名检查，不装依赖或改ROS库。
- 最终VM标准`python3 -m pytest -p pytest_asyncio.plugin -q`相关消费/来源/安全/建图装配组 **69 passed in 22.58s**，含Mac未能执行的Unix IPC夹具。原生CTest选定5目标：test_raw_scan_receipt、test_raw_receipt、test_rgbd_candidate_receipt、test_scan_ledger、test_scan_egress，**5/5通过**，覆盖真实DDS内部流转；candidate_preflight目标 **1/1通过**。没有全量pytest、131 CTest或整链重跑。
- configure/install私有资源、最终单测及build02均0；build01失败2单列，所有monitor无timeout/monitor_error，含两项补漏复验累计登记635个PID:starttime最终含Z残留均空。未将测试成功等同每个故障注入内部行为都成功。
- 证据：本机会话共享`navigation-wiring-cleanup-input01/`，source-manifest01/02/03、源码归档、单文件修正、实际ELF/build-id/ldd、运行maps/参数/进口源码hash及每任务PID记录齐全；最终原始包`output/wiring-1795770018003305415.tar.gz`，SHA与解析索引见`output/verification-summary.json`。外部诊断工具不混入产品提交。
- 本轮只完成今天已确认实现和单测/内部流转。前批整链5次RED/GetState未闭环、实车、最终完整保存恢复/退休/R4及性能仍未验；按用户决定不继续GetState调查，不签整导航验收，不自动推送/PR。
- 收尾发现运动回调调用`observation_origin`前仍先执行graph查询，且雷达来源默认值残留写死。真实回调回归先1失败（已经检查的NativeReceipt通道意外再次查询graph）后修正；仅跳过该已检查通道的重复查询，普通订阅仍走配置校验，来源默认值统一读取JSON。Mac和VM相关8项通过。
- 同一路径进一步复现：相同采集被放入新邮箱revision时，NativeSession会清空已累计的1299个格子状态。回归先1失败，修正为忽略重复/旧采集、不清窗后通过；生产装配夹具分开邮箱revision与原始扫描sequence，保留原后续运动撤销/固定墙断言。VM对整个受影响装配文件补验23 passed in 24.81s；这是单测/内部装配，不是原完整ROS case。
- 两项补漏均Python-only，未变更原生输入，不重复六包编译。最终源索引为source-manifest05；最终增量原始包`output/wiring-1795770907776103131.tar.gz`及SHA/PID索引`output/final-summary.json`，旧包不覆盖。前述69项与补验存在重叠，不相加当新场景总数。
