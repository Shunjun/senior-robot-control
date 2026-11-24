# Independent Task Names Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement your assigned tasks task-by-task. Steps use checkbox syntax for tracking. Orca worker分工及文件边界由负责人明确；不得自行派子Agent。

**Goal:** 任务名称独立持久化且可修改，内部身份稳定，列表/预览/当前执行一致显示名称。

**Architecture:** 扩展现有TaskDefinition/TaskStore与task.create/update/get及snapshot链路；名称不参与执行身份或路线语义。以既有vehicle_ui能力列表的task.names.v1门控页面写入，不另建名称服务。后端/前端在同一契约上并行，串行审阅后正常merge到feat/control-workspace，再做同版联调验收。

**Tech Stack:** Python dataclass/SQLite现有migration/Pydantic contracts；React19/TypeScript/CSS Modules/现有Vitest与Playwright。无新增依赖。

**Spec:** docs/superpowers/specs/2026-11-24-control-workspace-design.md，更新2026-11-25任务名称补充：用户“按此设计，但是都没有旧任务，没设置过”。批准设计正文随派发提供，先归档后实施。

## Global Constraints

- 固定功能主分支feat/control-workspace，父基线119dc4c578eaa73e0f1fbf3709ec9b8add602112，PR40保持Draft。后端子分支feat/control-workspace-names-backend，前端子分支feat/control-workspace-names-frontend；正常merge回父分支，不cherry-pick、不推main、不擅改主分支归属。
- 用户无已配置旧任务：不做历史任务回填/专项恢复，不清库、不改历史迁移；仍对空任务库正常升级字段，并保留真实内部直接导航任务的构造/执行路径。无需为了新字段批量重写无关测试fixture。
- name去首尾空白，内部空白保留，1–128个Unicode字符，允许重复。显式null/空白非法，省字段兼容逻辑明确标注//Compatible（Python用# //Compatible）。JS长度需与Python字符计数一致，可用Array.from(name).length。
- 新建草稿生成稳定UUID task_id，改名不换ID；名称是展示元数据，不影响定时、路线、enabled、实例ID/lifecycle version/导航效果。definition expected_revision并发保护保留，名称更新递增定义revision。
- 旧API创建省name以task_id兜底；更新省name保留现值。新UI遇旧车端无task.names.v1只读ID fallback，禁名称编辑/需要名称的新建，保留其他旧功能。共享contracts/cloud需要配套部署，不宣传任意版本双向兼容。
- 禁止启动开发或业务服务/实车/ROS/真实配对兑换/真实视频。调试仅可使用已批准的静态产物与全API/WS安全fixture。测试不触发真实车。
- 代码可读、职责清晰；相关props>3独立接口，组件样式CSS Modules。保持多点FAILED/unknown恢复、控制/切车/急停/地图身份、run-now/FIFO暂未实现等已交付语义。
- 产品17–20文件、测试8–10、文档3，总28–33；明显超出需先向负责人报范围，不静默扩大。

## Review Focus

1. task_id与execution_id不同：名称投影不能替换命令所需的实例ID；N8验证。
2. 纯改名与到期claim相遇：不能漏执行/多执行，也不能放宽地图/路线/禁用资格；N5及既存保护验证。
3. 新增dataclass字段与执行指纹/冻结定义耦合：不把名称带入执行身份，保留已接受路线；N3和既有lifecycle回归验证，不新增历史部署迁移专项。
4. 旧端零任务列表：不能靠第一项name探测能力或假报保存成功；N9验证。
5. 异步多点保存/任务冲突时名称草稿、内部ID保持，旧API省字段不能擦掉名称；N4/N7及既有F1回归验证。

## Fixed Contract

- TaskDefinition追加`name: str | None = None`兼容现有内部构造，post-init把省字段解析为task_id；显式API null在StoreTaskPort边界拒绝。存储读取对新用户任务返回实际非空name。
- domain schema增量v4增加task_definition.name，不做历史用户数据回填；列默认或内部构造fallback仅为现有内部路径与空库升级，代码注明用途。后续insert显式写入name。
- task.create/update body可带`name: string`；省字段规则见上；task.get.data.name与snapshot.tasks[].name为新服务输出，旧协议解码可省。修改沿原expected_revision，不新建rename请求。
- vehicle_ui.capabilities增加`task.names.v1`。vehicle_ui.task可选增加`name: string`和`definition_task_id: string`；原`task_id`仍是执行实例ID，不能换。当前执行名称来自同一次collect读取的定义列表；没有命名定义的隐藏直接导航任务省元数据并ID fallback。
- 新前端类型保留可选name语义，严格decoder只扩展明确的可选字段，不放开任意额外字段。支持能力与字段校验一致；名字只作文本节点渲染。
- 领取时若最新定义仅name和伴随revision有差异，可以在既有写事务内采用最新定义执行原occurrence；执行语义字段变化仍原conflict。使用显式命名的语义比较，不泛化“忽略所有revision”。如需要新execution revision或跨更多模块，应先报告架构取舍，不扩调度owner。

## Task 1: Storage, mutation and snapshot contract (backend owner)

**Modify:** senior_control/src/vehicle_manager/tasks/{models,codec,store,lifecycle_store}.py；storage/domain_schema.py；agent/{dispatcher,workspace,snapshot_state,snapshots}.py；contracts/src/senior_contracts/wire.py。
**Tests:** senior_control/tests/{test_task_runner,test_task_lifecycle_store,test_agent,test_workspace_data}.py；tests/unit/test_protocol.py，优先在最贴合的现有文件承载N1–N6，无需每个文件都新增场景。
**Docs owned here:** 更新现有control-workspace-design.md名称段；新增docs/superpowers/plans/2026-11-25-task-names.md（本计划）；验证文档由最终集成owner追加。
**Consumes:** 既有TaskStore版本/事务、StoreTaskPort与Dispatcher journal；固定契约。
**Produces:** 持久name、capability、列表/get/执行投影，供Task2按同契约消费。

- [ ] 先核对分支/基线/干净状态、读审计/tmp/control-workspace-task-name-audit-2026-11-24.md，按用户最新无旧任务约束替换旧报告的历史迁移建议；归档批准设计与本计划。
- [ ] 实现前用已选N1–N6编写/迁移必要测试并验证失败原因确为缺名称能力；不增加未选参数矩阵。
- [ ] 在TaskDefinition与现有codec/SQL添加name，空任务库v3→v4可重开，不动core_schema/agent_schema，不清库。检查SELECT列索引及snapshot删除标记位、insert/UPDATE和执行定义JSON写读，现有内部任务路径可用。
- [ ] StoreTaskPort创建/更新校验name；更新省字段从真实当前任务保留值，再以expected_revision写入；禁止覆盖路线/定时/created_at。set_enabled保留name。
- [ ] 纯改名与claim在既有事务中协调，只兼容展示元数据改变；保留实际字段变化/禁用/删除/map invalidation的拒绝与既有忙碌语义。
- [ ] 处理asdict与新增name的指纹投影，不让新展示字段改变执行身份；冻结执行路线仍来自原execution_definition/route。不要为用户不存在的历史任务新增重写工具或回填任务。
- [ ] 同步summary/get/collect/strict wire；执行投影使用同次读取的定义名称和definition_task_id，原实例task_id/version原样。capability仅在实现支持时发布。无需修改ROS/导航IPC。
- [ ] 运行实际相关pytest与现有lifecycle、schema、协议有效回归；按仓库现有环境/命令执行，不编造模块路径。先list/读pyproject确认命令，失败报告完整原因。
- [ ] 提交本子分支，提供准确HEAD/diff/测试命令及结果；不得合父分支或改PR状态。等待负责人审阅再集成。

## Task 2: Name editing and display (frontend owner, parallel Task1)

**Modify:** web/src/api/{protocol,workspace,vehicleState}.ts；features/tasks/{schedule.ts,ScheduleEditor.tsx,TaskList.tsx}；features/devices/VehicleStateView.tsx。必要局部样式Tasks.module.css/VehicleStateView.module.css。
**Tests:** web/src/features/tasks/tasks.test.tsx；web/src/features/devices/vehicleState.test.tsx；web/src/api/vehicleState.test.ts或client.test.ts选实际入口承载；e2e/control.spec.ts、e2e/fixtures/control-fixture.js由本owner维护名称真实流程。
**Consumes:** 固定契约（无需等待后端文件才能实现）；现有VehiclePort/任务草稿/版本保护。
**Produces:** 支持名称的页面、严格兼容解码和可安全复现的UI证据。

- [ ] 不修改后端/共享contracts/文档（由Task1及最终集成负责）；先核对基线与选定用例。
- [ ] 先在现有任务编辑位置呈现名称独立输入、内部编号次要只读、改名/重复名列表辨别的视觉及交互预览，可仅mock本地fixture；产物位于功能原位置。用既有安全静态预览验证，不启动开发/业务服务；正式接线后移除临时预览逻辑，留下正式fixture测试。
- [ ] 按N7–N10建立必要失败测试，名称采用“东区巡检”→“夜班巡检”的真实连续流程；不复制7种F1恢复分支做名称矩阵。
- [ ] TaskDraft拆name与task_id；新建时生成UUID一次并在重试保留，不从名字派生ID，不在每次render或submit重生ID。创建设置名称，编辑独立可改且保留原schedule/anchor/route/enabled与expected_revision。
- [ ] 严格解码可选name/definition_task_id及新capability；旧端无能力ID fallback、禁名称写入/新建，其他原编辑省name保留。新端能力声明不意味着可忽略字段校验，异常响应沿原错误路径。
- [ ] 列表/预览/route进度/当前执行按正确定义关联名称并弱化显示编号，重复名仍能辨识操作目标。VehiclePort.taskCommand仍用执行实例ID与版本，名称改动不替代React key或关联字段。
- [ ] 复用共享输入/现有控件与CSS Modules，相关复杂逻辑清晰多行，不扩大TaskList架构重写。日志/设置/控制/地图/急停及暂未实现行为保留。
- [ ] 运行相关Vitest、typecheck/build；安全fixture用既有1440/390/844流程加入一次创建/改名/展示检查，保留底部/急停/小图owner检查；查看截图，报告临时预览已移除。无需真实服务集成、实车或ROS。
- [ ] 提交本子分支并报告HEAD、测试和截图，不合父分支/不改PR。等待负责人审阅。

## Selected Verification Scenarios

以下为已确定要实现的测试行为；其他既存有效回归继续保留执行。用最贴合层次验证，不为每一层复制全场景。

- N1：真实空任务domain3库升级到v4，重开仍可创建带名称任务；core/agent版本和非任务数据不动。基于test_task_runner现有schema测试，无旧用户任务回填fixture。
- N2：经StoreTaskPort→真实TaskStore创建task_id≠“东区巡检”，重开/get/snapshot读回名称及原字段，验证可持久化。
- N3：已接受执行的任务改名为“夜班巡检”；原task_id/路线/schedule/timezone/enabled/created_at/实例ID/lifecycle版本/已接受执行路线不变，定义revision+1，无新导航效果。
- N4：旧API省name创建默认ID，随后新API改名，再旧API省name更新与启停，名称保留；复用既有operation_id和expected_revision保护，不另外扩大矩阵。
- N5：已读到期定义→提交纯改名→原occurrence领取一次；用确定性事务/事件控制并发点。复用原语义字段变化、禁用、重复领取保护，不能将纯改名记资格SKIP。
- N6：summary/envelope经现有cloud缓存或协议入口往返名称及revision更新，不丢中文名，缺省旧帧仍能读；选择真实现有最短链路，避免新增平行接口。
- N7：页面创建“东区巡检”并得到独立稳定ID，编辑仅改名为“夜班巡检”，预览/列表正确；复杂cron或interval anchor/路线顺序保持，已有多点部分确认恢复用例只迁移fixture和断言。
- N8：定义ID=route-01、实例ID=exec-88，当前执行名称同步更新后pause/resume仍发exec-88和真实expected_version；沿现有连续操作用例验证。
- N9：新UI遇无task.names.v1的旧端（包括切到零任务列表），ID fallback、名称不可写、明确升级提示；原非名称编辑仍可用，切回支持端才允许命名创建，不假报成功。
- N10：名称trim后保留内部空白且可重复（两个不同ID同名正确辨别）；全空白显式提交被前端阻止、API校验拒绝且不持久化。只添加必要独立场景，不做长度/字符参数矩阵；前后端字符计数一致由代码审阅核对。

## Task 3: Review, normal merge and final validation (sequential, later dispatch)

- [ ] 负责人亲阅两个子分支实际diff、测试证据，必要时原类别worker返修；确认接口/名称计数与最新用户无旧任务要求一致。
- [ ] 在原/home/toby/orca/workspaces/senior-robot-control/control-workspace的feat/control-workspace正常git merge两个审核通过子分支，不cherry-pick、不ours/theirs整文件覆盖；解决冲突核对两边功能均保留。
- [ ] 验证合并产品版本的后端真实SQLite/协议主路径及前端名称fixture必要检查，证据充分的未变代码测试不例行重跑；不以mock替代真实持久化。
- [ ] 更新docs/testing/2026-11-24-control-workspace-validation.md并区分新增/改写场景、实际bug回归、复用回归、版本和未验证项；提供负责人实际视觉截图。
- [ ] 推送原功能分支并更新同一PR40描述；名称与控制页均完成亲审后决定ready，不自动merge main。
- [ ] 核对独有提交、干净工作树和仍在使用会话后，按Orca机制清理已合子worktree/分支，保留用户正在使用会话。

## Self-review

用户名称持久化、可重复/可改、自动ID、运行中同步、无旧用户任务、旧端提示均覆盖。两子任务文件归属无交叉，接口固定；数据库/接口与页面可并行，合并与真实同版链路验证必须顺序。无额外调度/几何/服务需求，名称不进入身份，现有保护保留。文档仅归backend和集成owner，避免并行冲突。
