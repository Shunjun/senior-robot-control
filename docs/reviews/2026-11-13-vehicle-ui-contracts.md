# T9a 车辆工作台接口边界与静态预览（2026-11-13）

本轮只交可审阅接口映射和静态 HTML，不改产品网页、共享协议、client、composition 或控制输出。
依据已批准的 spec/plan Task 9，消费 T2/T5/T6 及 T7/T8 核心；本报告中的新 wire 名称均为待实现提案。
用户最新明确的首版验收顺序已单独同步 spec/plan：优先正常功能，专项实测后置，不撤销既有行为要求。

## 本地提交核对

获审完整 patch + 审核修复 patch 在临时目录重建，与全部 12 个授权源码/测试逐字节一致，未发现后来增量。
两次提交各自开始时 index=[]；提交前仅指定路径；提交后 index=[]，无 docs、plan 或其他 worker 文件。
索引/逐文件 SHA256 记录：`/private/tmp/task_e3b06640e031-commits.json`、
`/private/tmp/task_e3b06640e031-approved-hashes.json`。

| 阶段 | 完整 commit SHA | 信息 |
| --- | --- | --- |
| T7 | 4f3a7506f07c56df520f9b68c369103b37950c54 | feat: persist vehicle settings and project confirmed runtime state |
| T8 | 3473d5497eeee7b763276a6ed50b9a0b318d8508 | feat: add bounded vehicle log storage and streaming core |

T7 提交前 index 精确清单（7）：

```text
src/senior_control/agent/vehicle_state.py
src/senior_control/settings/__init__.py
src/senior_control/settings/models.py
src/senior_control/settings/service.py
src/senior_control/settings/store.py
tests/integration/test_vehicle_settings.py
tests/unit/test_vehicle_state.py
```

T8 提交前 index 精确清单（5）：

```text
src/senior_control/vehicle_logs/__init__.py
src/senior_control/vehicle_logs/models.py
src/senior_control/vehicle_logs/service.py
src/senior_control/vehicle_logs/store.py
tests/integration/test_vehicle_logs.py
```

没有 push。T7/T8 是通过定向验证并获审的阶段成果，不代表认证、生产链、网页、Linux 负载或实车全产品验收。
本轮没有再次扩展后端测试矩阵；负责人此前独立验证 151 passed，提交内容与该获审状态相同。

## 布局与现有视觉体系

预览：`docs/reviews/2026-11-13-vehicle-layout-preview.html`，用 file:// 在 Orca browser 打开，未启动服务。
沿用 web/src/theme.css 的 graphite/amber/teal/red、Chakra Petch/IBM Plex Mono 和现有工业仪表语言；
字体取自项目已安装包并内嵌，无外部请求，不新增品牌体系。所有数据都明确标为示例，按钮不调用任何 API。

默认先呈现在线手动的一个状态/原因、车体速度、两档方向力度与急停，设置和日志在下方。
窄屏按自然顺序堆叠，急停按钮固定底部，正常任务的暂停/取消仍可见；轮速只在折叠诊断内。
手动区提供示例授权下的申请/释放控制、任务中请求接管入口；接管必须等车端确认撤销旧任务输出后再授予手动会话，
按钮不自动把状态改成手动或假报已停。
设置仅五项驾驶字段与四项日志字段，没有磁盘上限、PI、标定或其他驾驶参数。
场景选择保留正常任务、断网、方向障碍、反馈停车、软件 pending/锁存、实体锁存、独立故障恢复、
暂停后明确继续、设置冲突/未知、日志 gap 等设计样本；这些是布局表达，不是专项实际场景验收。

浏览器实际检查：1440×1000 桌面、390×844 窄屏，页面 scrollWidth 分别 1425/390，均未超出视口。
窄屏急停按钮 top=780、bottom=834，在 844 高视口内；pending 场景保留手动状态且显示“急停请求待确认”，
硬件场景没有网页恢复按钮；断网保留“运行中·最后已知/任务可能继续”。控制台无错误。
Orca page ID：`b4a38bc4-2303-489b-9673-9e630e491f77`；当前停在桌面正常手动预览。

截图（均为 Orca browser 实际生成，已复制至稳定的本次临时目录）：

| 检查 | 路径 |
| --- | --- |
| 桌面正常手动 | /private/tmp/task_e3b06640e031-preview/desktop-manual.png |
| 桌面完整布局 | /private/tmp/task_e3b06640e031-preview/desktop-manual-full.png |
| 窄屏正常任务 | /private/tmp/task_e3b06640e031-preview/mobile-task.png |
| 窄屏完整布局 | /private/tmp/task_e3b06640e031-preview/mobile-task-full.png |
| 窄屏急停待确认 | /private/tmp/task_e3b06640e031-preview/mobile-pending.png |
| 实体急停 | /private/tmp/task_e3b06640e031-preview/desktop-hardware.png |
| 设置冲突完整布局 | /private/tmp/task_e3b06640e031-preview/desktop-settings-conflict-full.png |
| 窄屏断网最后已知 | /private/tmp/task_e3b06640e031-preview/mobile-offline.png |

原始浏览器检查记录：`/private/tmp/task_e3b06640e031-preview/checks.json`。
本轮无产品 React 改动，不把静态检查算作网页集成测试或控制功能通过。

## 当前产品接口与下一步映射

当前前端不能直接消费新核心；Python dataclass/service 是内部契约，不是已注册的 HTTP/WS schema。
检查 web/src/api/protocol.ts 和后端 protocol/dispatcher，尚无 drive.intent、settings.*、logs.* 或统一 vehicle.state 接线。
旧 RequestType/ResponsePayload、RobotSnapshot 和 client 仍在使用 velocity/challenge/timing 分支。

| 当前文件/行为 | 新核心来源/目标字段 | 后续明确修改与缺口 |
| --- | --- | --- |
| RobotPanel.tsx 的 Readout 显示 execution_status、stale/connected | T7 VehicleState mode/reason/detail_reason、body_*、task_*；连接独立 | 用 VehicleStateView 集中投影展示；保留连接诊断，不能把 execution_status 字符串推成新模式 |
| ControlSurface.tsx 场景/地图/视频编排、navigation.goal/resume/cancel | T5 允许动作、T6 当前执行/回执与地图引用 | 保留地图/视频工具，统一运行摘要；任务执行动作不得继续由各组件自行推导 |
| DriveView.tsx MAX_LINEAR=.5、MAX_ANGULAR=1、ASSIST_SCALE；DriveController 发 drive.velocity | T2 ManualIntent(authority_id,gesture,sequence,linear,turn,gear) | 改方向力度 [-1,1] 和 low/high；不把 UI 数字换算成目标速度；车端映射/轨迹/约束 |
| Joystick.tsx 指针/键盘及 ReleaseReason | 同一归一化意图与明确新手势边沿 | 复用输入与松开行为，配置/档位变化不复活旧 hold；页面失焦、卸载、连接变化清待发样本 |
| EmergencyStop.tsx safety.estop 的局部 acknowledged/inFlight | T7 confirmed_estop_status + 当前请求 pending；estop_revision/fault_revision | 请求超时不显示已停；复位与故障恢复分别走车端命令和各自 expected revision |
| TaskList.tsx 的“暂停”调用 task.enable(enabled=false) | T6 Lifecycle phase/stopped_for/completed/pending，Receipt | 这是停用计划，不是暂停当前执行；拆开命名/动作，不能把 task.enable 回执当已停止 |
| api/client.ts 的 RobotLink、RobotClient timing/challenge/drive queue | T2 authority 生命周期 + 新状态订阅；RTT 仅诊断 | 串行迁移 transport，最多一个待发最新意图/一个未确认意图；急停/取消/接管不进长任务 FIFO |
| api/protocol.ts SnapshotPayload 无 body speed/新模式/设置/日志字段 | 新 runtime DTO、settings DTO、log DTO | 严格 decoder/请求类型映射待实现，不能先把任意 response.data 当新协议 |
| api/demo.ts 与 drive/tasks/devices 测试 | 明确的 fixtures 和规范化 DTO | 与新协议同步，移除与新版冲突的断网暂停/浏览器时序资格假设，保留仍有效保护 |
| 尚无设置组件 | T7 SettingsSnapshot/SettingsCapabilities/SettingsResult | 新 VehicleSettings + module.css，草稿与生效值分开；分组版本/请求结果查询 |
| 尚无日志组件/独立日志连接 | T8 LogQuery/LogPage/LogDownload/LogSubscription | 新 VehicleLogs + module.css，独立认证连接，过滤/跟随/暂停/历史/分段下载及 gap |

## 最小新接口提案（尚未实现）

只规定需要的数据与所有者，下一批协议接线统一落实名称、版本和 decoder；本轮不修改共享文件。
建议一个 runtime 读接口、两个设置组接口及独立日志流，不新增通用任务框架。

### 1. Runtime 只读帧

拟定 `vehicle.state` 帧（新类型）：robot_id、可信 stream identity/sequence、完整 T7 VehicleState，
附 T6 TaskExecutionView 与当前 ManualSessionView、由车端生成的 ActionAffordances。
已有 `snapshot` 的地图/视频资源可以并行保留，不能用其旧 execution_status 替代新确认状态。

- MotionState/VehicleState 的 mode、reason 和确认状态是车端权威；网页只映射文案。
  T5 已处理 ESTOP > execution_fault > feedback_stop > obstacle_hold > 来源，不在 React 重写优先级。
  detail_reason 是末次事件诊断，不能覆盖主 reason。
- BodyTwist 当前为 vx/vy/wz 三个有限 SI 数值，整体 body_average 可以为 None；缺失显示“—”。
  主区显示平均值与来源/年龄/过期标签，轮速独立诊断，不用输出回执或 wheel_linear_mps 填充主速度。
  显示 km/h 可取 vx*3.6 并明确有方向的纵向区间平均；若改展示合速度应另定标签，不能悄悄改变含义。
  T3 仅轮速退化估计与独立 body reference 的 provenance 必须由车端适配明确，不能由网页猜 body_source 字符串。
  单有 qualified 不等于物理真值证明；待适配时缺独立 body reference 的主区留空，退化估计只放诊断。
- body_computed_steady/valid_until_steady 是车端时钟，不与浏览器 performance.now/Date.now 相减。
  transport 应发布车端已算年龄/过期，前端只叠加本地自接收后的显示耗时，不据此推导驾驶资格。
- RuntimeSnapshot.hardware_estop 与其 sequence 一同提交，是原子快照字段。
  不接受独立迟到的硬件布尔补丁；T7 同/旧 sequence 或安全版本回退会被拒绝。
  跨车端重启 sequence/estop_revision/fault_revision 的持久化或正规化 owner 尚须在车端 transport 落实，
  无法延续时显式做可信流重建，不能在前端看到新 epoch 就忽略版本检查。
- 当前请求 pending 为独立 UI/transport 状态，可在失败、撤销、查询确认完成后清除，无须新遥测；
  已确认 software_latched/hardware_latched 不能因此清除。复位成功回执后仍按后续车端确认状态展示。
- ConnectionDisplay 来自实际链路状态/RTT/最后接收时间，独立于车端任务和运动。
  断线保留最新确认快照并标 last_known；不能改 task.phase 为 canceled/paused 或将速度置零。
  网页断开只清网页手动会话/待发送意图；非手动任务按车端生命周期运行。

ActionAffordances 是待补的最小车端适配输出，**不是当前 VehicleState 已有的完整字段**：
已有 allowed_actions 可公开 acquire、resume_task、estop_reset、fault_recover；replan_task 是内部动作，不直接做按钮。
T6 的 pause/cancel 与 settings driving 可应用提示、已有手动会话能否发送意图，还需按实际 car owner/授权补充。
只根据车端授予的动作、会话和在线可发送性展示按钮；不从 mode==idle、RTT 或地图示意自行制造资格。
方向可退说明也由车端约束诊断提供，不在浏览器靠前方红框推断后方许可。
这些是读模型映射的接线要求，不添加首版专项资格门。

### 2. 手动与恢复命令

拟新增 drive.intent：直接对应 ManualIntent 的六项字段，linear/turn 为 [-1,1]，gear 为 low/high。
control.acquire/release 虽已有名称，authority_id 回执、车辆/认证连接代次绑定与不可预测 ID 仍需串行接线。
每个 hold 的新边沿才激活新 gesture；旧 authority 永不因更大的 sequence 或重连恢复。
client 保留一个未发送最新样本、最多一个未确认样本，覆盖而非追加；迟到 acquire 回执不复活已松开的手势。
现有 drive.challenge/timedMotion/校时资格路径应删除，RTT 仅作诊断；不静默回退至旧速度协议。

软件急停继续复用现有 safety.estop 入口的认证/优先通道，但新确认状态和请求查询语义需适配 T5。
软件复位拟映射 MotionEvent.ESTOP_RESET + estop_revision；fault 恢复拟独立映射 FAULT_RECOVER + fault_revision。
旧 safety.reset 不能一键清两种状态；硬件释放只有现场/可信硬件事件，绝不开放网页 hardware_release。
所有命令以车端拒绝/回执为准；按下按钮或 Promise 成功不能独立伪造新 VehicleState。

### 3. 任务执行视图

T6 Lifecycle 的 phase 实际为 empty/starting/running/revoking/paused/succeeded/canceled/failed，
不要套用旧 task.enable 或单纯 execution_status。TaskExecutionView 至少映射 execution/task identity、
version、epoch、phase、stopped_for、已完成/剩余动作与原请求回执；这些字段不由浏览器重造。

| T6 来源 | 视图语义 |
| --- | --- |
| starting | 启动/规划待确认，不显示任务已完成 |
| running | 正在执行；断线只能标最后已知，任务可能继续 |
| revoking / pending revoke | 正在暂停/取消，等待车端确认撤销旧输出；不先显示停稳 |
| paused + takeover/fault/estop/restart | 保留进度；明确继续，按现位姿/地图重规划，不重做 completed 动作 |
| succeeded/canceled/failed | 终态展示，不能“继续”旧执行 |

Receipt.accepted 表示该命令的稳定受理结果，不代表物理撤销或任务终态。
T6 store version 是 CAS，epoch 是执行 fence，不能当 T7 runtime.sequence 或浏览器连接代次。
拟新增 task.execution.command/query 路由承载 submit/pause/cancel/takeover/resume 与 expected_version；
真实路由尚未注册，命令上下文/地图位姿引用由车端适配读取，不接受浏览器自报 motion_allowed。

### 4. 车辆设置

拟新增 settings.get / settings.apply / settings.result；这是 wire 名称提案，现有 operation.query
尚未路由设置事务回执，不能直接复用其历史结果语义假装接线完成。

| UI 输入 | T7 字段 | 显示/传输 |
| --- | --- | --- |
| 低/高档线速度上限 | low_linear_mps / high_linear_mps | km/h 输入 /3.6 后传 SI；回读 *3.6 |
| 低/高档角速度请求上限 | low_angular_rps / high_angular_rps | rad/s；两档独立 |
| 统一倒车上限 | reverse_linear_mps | km/h / SI；实际 min(档位,reverse) 由车端执行 |
| 常规级别、保留时间 | level / retention_seconds | 级别枚举；1h–30d，默认7d |
| 临时 debug、自动恢复时长 | debug_enabled / debug_restore_seconds | 60–3600s，默认300s |

DTO 复用 SettingsRequest(request_id,expected_version,group,values,restore_defaults)，完整分组原子提交；
SettingsResult 的 reason/snapshot/capabilities 是本次稳定回执。驾驶、日志各自版本，不能用一个全局版本覆盖。
驾驶只在 RuntimeGate 的最新可信静止/无 executing 运动任务/手动释放检查下提交；暂停任务可改且不续跑。
网页提示来自车端适配，最终 submit 仍由 RuntimeGate 再校验到持久提交，不在浏览器复制判断。
日志运行中可改，不发运动事件。profile 范围/default 只回读，不将预览的 2/5kmh、1rad/s 或倒车示例当物理能力。

页面保留 draft/effective 两份，applied 后回读当前快照；历史幂等回执不是当前最新配置，不能使新版本回退。
version_conflict 展示车端版本、保留草稿供核对；超时为结果待确认，用原 request_id 查询或原样重试，
不得改内容后复用 ID，不自动排队稍后生效。恢复默认也用同一校验/请求回执路径。
SettingsService 是唯一 debug deadline/原级别/离线恢复 timer owner，T8/UI 只显示与消费，不另起恢复计时器。

### 5. 日志查询、跟随、下载

拟新增独立认证日志 WS/下载路由，实际端点未注册。沿用真实 session -> vehicle grant 注入 LogAuthorizer；
前端不发送 authorized=true、不以 robot_id 自报获得权限、不携带文件路径/shell/磁盘配额。
后端转发必须验证请求车辆、当下会话及 action；跟随/导出期间撤权仍拒绝。

复用 T8 LogQuery(after,limit,levels,module,request_id,task_id,since_utc,until_utc)、LogPage(entries,gaps,next_cursor,
high_water,has_more,scanned)。按 (robot_id,stream,sequence) 归并；排序用车端序号，源时间/接收时间分别显示。
fields_json 为已脱敏 JSON 文本，前端只作安全文本/结构展示，不用 innerHTML；令牌不入日志/下载内容。

- 每页≤200条/256KiB；空 entries 但 has_more=True 仍续读，不能把过滤扫描预算当历史完整。
- 首次 follow 回放最多一页；follow_backfill 与 slow_subscriber 需要独立查询回补，evicted 明示不可恢复的历史缺口。
- R1 修复后 live 批次按真实字节/limit 打包；不要假定每页只有一条，也不要把 has_more=False 当生产者都已 flush。
- durable=False 的 gap 是当前未落盘丢失/错误提示，不能伪装成精确完整历史；restart_unknown 的数量未知。
- 暂停跟随只影响日志接收/显示，不暂停车端任务；恢复时用游标补读。客户端最多保留1000条，裁剪也显示窗口缺口。
- 单次导出≤2MiB且扫描≤16页，has_more/next_cursor 表示分段续读；UI显示分段和 gap，不静默生成“完整日志”。
- 日志连接/outbox 与控制收发独立，UI批量更新与窗口裁剪，不令日志渲染挤占摇杆/急停处理。

## 后续精确分工与共享文件顺序

先由负责人审本映射/预览，再做正常主路径产品实现；无需重新询问已批准需求。
所有共享文件必须一个接线 owner 串行负责，不由多个组件 worker 同时修改。

| 批次/独占 owner | 精确文件 | 交付/依赖 |
| --- | --- | --- |
| A 后端契约接线 owner（串行） | src/senior_control/protocol.py、agent/{composition,dispatcher,snapshots,main,websocket}.py、cloud/{relay,app}.py、ROS agent_adapter.py | 实际 DTO/认证/urgent 控制、T2 authority、T7投影/RuntimeGate、T6路由、settings/log lifecycle；先固定最小正常链 |
| B 前端 transport owner（A 后串行） | web/src/api/{protocol,client,demo}.ts；新增 web/src/api/vehicleState.ts；相关协议/client测试 | 严格解码、单位/边界、最新意图槽、无浏览器 timing资格、独立日志连接；不让其他 owner 改这些文件 |
| C 状态与操纵 owner | 新 features/devices/{VehicleStateView.tsx,VehicleStateView.module.css}；features/drive/{DriveView,EmergencyStop,Joystick}.tsx、Drive.module.css、drive.test.tsx | 消费已固定 DTO/允许动作；主速度、两档、正常启停、急停与独立恢复 |
| D 设置 owner | 新 features/settings/{VehicleSettings.tsx,Settings.module.css,settings.test.tsx} | 五驾驶字段、四日志字段、回读/版本冲突/未知回执；只消费 B 端口 |
| E 日志 owner | 新 features/logs/{VehicleLogs.tsx,Logs.module.css,logs.test.tsx} | 正常查看/过滤/跟随/分页/分段下载/gap；只消费 B 的独立日志端口 |
| F 工作台装配 owner（C/D/E 后串行） | features/devices/{RobotPanel,ControlSurface}.tsx、OperatorWorkspace.module.css；features/tasks/TaskList.tsx、Tasks.module.css；devices/tasks测试及 web/e2e/control.spec.ts | 统一状态入口、计划启停与执行暂停分离、正常导航；避免 C 同时改装配文件 |

表中的 ROS/backend 接线范围不是本轮已授权修改，也不意味可在无适用配置下接真实输出；由负责人按依赖安排。
T6 当前修复中的独占文件保持只读，不因本报告分工改动或提交它们。
后续组件 props 超过三个属性必须命名 interface/type；局部样式用 CSS Module，不加 theme.css 全局覆盖。
React 参考已读 vercel-react-best-practices：运行状态/RTT/log window 分开订阅和批量更新、日志面板按需加载，
不为纯显示重复拉取完整大快照；不把高频摇杆样本推入整页 React state 或制造请求瀑布。
有不可避免的旧数据兼容时，按项目要求明确 `//Compatible`；不以兼容保留已被新版撤销的浏览器时序资格。

## 首版验收收敛与尚未接线限制

本轮收到负责人的用户最新指令后，只在权威 spec/plan 增加同一验收优先级小节，历史矩阵和失败证据未改写。
独立文档增量 patch：`/private/tmp/task_e3b06640e031-acceptance-priority.diff`，未与两个代码 commit 混入。
首版实际场景主路径为常规手动两档/平滑启停/常规方向避障、正常导航、急停按钮与恢复、五项设置回读、日志查看。
断网、极端时序、滑移、故障注入、洪峰负载专项实测后置到首版后用户环境，不要求全矩阵过关才交一版。
保留必要定向回归和明确缺陷修复，不新增安全资格门，不把预览样本算作实测通过。

目前 runtime/settings/log 新 wire、真实认证、前后端 lifecycle、实际页面/命令输出尚未接入；
预览没有网络请求或配置/任务写入，不代表已可通过网页驾驶，不构成 Linux 时延或硬件证据。
这两个新增 review 文件和 spec/plan 增量均未提交、未 push，交负责人审阅后再安排下一批。
