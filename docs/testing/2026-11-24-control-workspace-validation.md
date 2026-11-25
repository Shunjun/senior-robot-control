# WHEEL 控制工作区验证记录

日期：2026-11-24，Bot1；派发 task_4b41d49f0b5f。只在 `/home/toby/orca/workspaces/senior-robot-control/control-workspace` 开发。固定分支 `feat/control-workspace`，分叉基线 `00dc3be5727b203bd4bbe6b7ed72e83226a7be71`；未修改原 main 工作区、未 reset、未合并 main。

## 材料实收

- 参考目录：`/tmp/control-workspace-approved/`。已读 index.html、concrete.jpg、vehicle-marker.svg 与两份设计/计划；生产没有复制原型 HTML、模拟调度或模拟配对状态机。
- 从 `/tmp/control-workspace-materials.zip.b64` 使用 `base64.b64decode(text, validate=True)` 独立还原为 `/tmp/control-workspace-approved/control-workspace-approved-demo.zip`，149114 bytes。
- 原 ZIP SHA256：`fdfaa1b451853c4b57d536aa0fd3e702a7e2e6673d90b64a69686a34ef9892c6`。ZIP 内三个资源与实收目录逐字节一致。
- index.html SHA256：`d84d64de8b806224756f3d326ca620cbe29e0fe856eb8125f2cb4469de05b754`。
- concrete.jpg SHA256：`c49573d186bec563689bca18dd5b4a2dd3cad8c03c43fa1e6bdcfa793aec8e92`。
- vehicle-marker.svg SHA256：`9ad68362e5b61f01198d2b7959c801cf925a5d1e22ec6a9d527746bf34f236ff`。SVG 只保留作后续真实几何接入资源，本轮不以示意比例绘制实测轮廓。

## 实际架构与接线

App 保留认证、CSRF、路由、5秒刷新、解除配对竞态保护；RobotPanel 按 robot.id 拥有唯一会话。ControlSurface 继续拥有 useWorkspaceMap、地图身份、导航/校准资格与四模式状态；TaskList 通过 renderWorkspace 提供唯一 MapCanvas 和任务卡片，主/小图只改变容器布局，不新增第二个地图 hook。DriveView 始终保留原 ManualController；切模式/日志/设置 active=false、失焦、换车、断线和退出走原 stop/release/dispose 语义。VideoPanel 不重写 WebRTC，摄像头关闭或离开控制页释放媒体。实际请求仍通过原 VehiclePort/RobotLink，未新增车端调度或地图算法。

共享控件包含受控 Switch、Button/IconButton、SegmentedControl、FloatingCard。Lucide 图标及 ECharts core/LineChart/Grid/CanvasRenderer 按模块引入；最多保留60个已收到的速度样本，缺失/过期保留断点，不补零，不生成历史。速度展示按原relay的5000ms报告期限从实际received_at_ms计时；运行sequence不前进则显示不可用/曲线断点，同序号不能刷新期限，新序号才恢复，不影响独立手动连接与控制权。性能超时仍3500ms并放在不随浮层开关重建的 owner；相同 sequence 不因开关重获新鲜度。Yaw 仅有真实且已定位样本时显示，Roll/Pitch 不可用。真实速度与归一化操控意图分开；摇杆10%入轴/18%脱离、8%中心死区、5%输出步长。

设置草稿保存SI字符串，m/s↔km/h只换显示/编辑尺度，角速度rad/s不变；保留六驾驶+四日志字段、版本冲突、原请求查询/重试。任务名称使用创建时task_id，创建后只读；复杂cron/timezone、interval/weekly、未来启用开关不变。有序多点先逐个确认航点，再提交任务；第二点失败不删除第一点，保留草稿和稳定operation_id，重试跳过已确认点；版本冲突仍锁定并提示重载。执行实例 pause/resume/cancel/takeover 沿原 allowed_actions、执行ID与版本，移至任务操作卡片可达。

## 四类能力

| 类别 | 本轮结果 |
|---|---|
| 已真实接入 | 原认证/选车/刷新/解除配对；网页申请配对码及实际 expires_in/复制；单摇杆与原控制权/档位/急停；地图与媒体开关/交换；真实速度、Yaw、CPU/内存/模块状态；原地图v2校准、Home、目录、导入导出；有序航点、任务定时、未来启用及当前执行操作；逐行安全日志及完整十字段设置 |
| 暂未实现 | 对某已保存多点任务立即执行、FIFO等待队列、全状态同任务去重、独立待确认变化/多帧未知→空闲证据层、线段拟合/墙语义分类；可见入口只提示“暂未实现”，不发不存在的接口、不建前端队列 |
| 状态不可用 | 缺/过期/离线速度与性能；Roll/Pitch；缺显式校准footprint的实际轮廓尺寸；无权限、定位/地图/导航不就绪与版本冲突仍用真实原因，不能统称未实现 |
| 未验证 | 实车、ROS、真实配对兑换及凭据代理上线、真实WebRTC/TURN视频、车端运动、实车地图与校准；本轮没有启动或连接这些服务，不把fixture通过当作上述链路通过 |

几何已与负责人闭环：`platform_runtime._adapter_command` 复用 `validate_real_runtime` 和现有 safety_parameters；当前 `config/hardware-safety.yaml` 没有显式 footprint_xy，默认值来自 length/width 矩形。负责人确认本轮只留箭头+尺寸不可用，不另读 DeviceProfile/hardware.yaml、不新建几何owner、不铺无来源协议。对应物理轮廓验收为**数据未配置/未接入**，不是已完成；因此本轮不改Python/协议、不跑无关Python或全仓仿真。

原生导航栅格的 -1/0/100 分别绘制未知/暖灰空闲/深灰占据；中间概率值只呈其栅格置信度，不宣称独立候选证据层。stable-only fallback没有free值时绝不补成free。没有15米圈、刻度或标称半径。

## 失败证据和回归

1. 先将配对测试改为正式 `python -m vehicle_manager.agent --pair`，再加入“raster与reference代次不同但合法”任务用例；原实现失败2项/通过94项，记录 `/tmp/control-workspace-red.log`。修复后同两文件96项全过，记录 `/tmp/control-workspace-bug-green.log`。配对码仍在车端getpass输入，不进入argv；凭据代理上线才是在线。
2. 新公共控件/单摇杆缺失时先运行失败，再实现。原ManualController迟到 acquire/stop、背压、松手等待新按压、失焦和急停回归保留。
3. 多点保存“第二点失败”先因缺入口失败，再实现逐点确认/保留/重试；测试证明未发 task.create、第一次已确认点不重建。
4. 自审发现保存路线预览误带拖拽处理，`keeps the saved-route preview read-only` 先失败（`/tmp/control-review-red.log`），再移除只读预览处理并限制编辑列表拖拽。整体回归通过。
5. 浮层开关不能更新旧CPU样本的新鲜度；补充 null→新样本→同sequence超过3500ms、Yaw有值/Roll-Pitch缺失及关闭再打开的连续测试。
6. 手机驾驶滚动后急停需要始终可达，新增浏览器断言并以旧构建复现，再使用固定顶部操作栏。布局的手机纵向滚动是为44px触控与可读字号保留的有意调整。

## 需求场景与证据位置

| # | 场景 | 证据/边界 |
|---|---|---|
| 1 | 空页→A→迟到请求切B | devices.test.tsx 原认证/切车/断连/解除配对矩阵继续通过；浏览器空页不挂急停/目标数据 |
| 2 | 同次按压双轴→吸轴→脱离→中心→松手 | workspace.test.tsx 指针轨迹，drive.test.tsx 原控制权安全回归 |
| 3 | 同0.5m/s显示/草稿单位往返保存 | settings.test.tsx SI草稿往返及payload/角速；浏览器0.50↔1.80并保存0.6m/s |
| 4 | Yaw/缺Roll-Pitch、CPU空→有效→过期 | PerformancePanel.test.tsx 新连续流及旧sequence过期；浏览器姿态字段不可用 |
| 5 | 摄像头→交换→小图缩放→普通点击→关闭→日志返回 | e2e两尺寸；比较map/stick DOM身份且各1个owner，普通点击不交换；原video生命周期回归 |
| 6 | .05m旋转origin、缩放/resize | 原map.test.tsx坐标与ResizeObserver回归，fixture原生旋转栅格、主/小图缩放 |
| 7 | -1/0/100及缺候选/拟合层 | map.test.tsx检查三种实际RGBA；图层入口只提示，没有雷达圈 |
| 8 | 真实轮廓旋转/倍增 | **未满足物理轮廓测试：显式校准数据未配置/未接入，已获负责人确认采用fallback** |
| 9 | 缺几何仅箭头 | 浏览器截图/DOM实际箭头与尺寸不可用提示；无新协议占位 |
| 10 | A/B/C→拖C→键盘调整→有序保存 | tasks.test.tsx原有序路线+新增drag与替代按钮；浏览器C/A/B持久payload、cron/timezone不变 |
| 11 | 第二点失败保留第一点/草稿 | tasks.test.tsx新增具体流程；重试仅发未确认点，没有假整体成功 |
| 12 | 复杂cron和未来启用开关 | 原schedule/tasks回归；浏览器复杂cron/timezone保存无损；task.enable保留expected_revision，不取消实例 |
| 13 | 实例pause→resume→cancel | vehicleState.test.tsx新增同实例版本3→4→5的连续操作；原VehiclePort allowed_actions/回执保护保留 |
| 14 | 缺立即执行/队列 | tasks.test.tsx零请求；e2e立即执行前后请求数不变 |
| 15 | 多行/HTML/结构字段/警告日志 | 原logs/API分页/下载/gap回归；浏览器真实协议fixture恶意字符串安全逐行展示、无img执行 |
| 16 | 设置版本冲突/十字段/SI | 原settings冲突/不确定请求回归未删，另加单位往返草稿 |
| 17 | 桌面1440完整交互 | e2e含Space开关、底三组父容器bottom差≤1px、预览owner与所有主要页；Orca DOM交互交叉验证 |
| 18 | 手机390触控 | e2e context isMobile+hasTouch、tap开关、排序替代按钮、主/小图工具、无横向溢出、急停滚动可见；Orca两个Switch圆点中心偏移均0px |

原有效API/地图生命周期/Home/版本tile隔离/解除配对/认证/急停/回波校准/日志设置回归保留，没有新增skip/xfail。断言迁移依据：两杆名称→单杆，低档/高档→低速/高速，视频/地图页签→摄像头Switch/专用交换图标，启停按钮→受控Switch，任务编号→“任务名称（编号）”；独立状态默认单位改为m/s，设置旧km/h回归显式包裹单位context。旧业务服务e2e替换为生产客户端的隔离协议fixture；手机原“整页零滚动/两杆同屏”断言迁移为44px可触控、允许纵向滚动且急停固定可见。

## 检查命令与实际限制

在 `web/` 运行：

- `npm ci --no-audit --no-fund`，随后通过锁文件引入 lucide-react / echarts。
- `npm test -- --run`：最终 **16文件、195项通过**。
- `npm run typecheck`：通过。
- `npm run build`：通过；按需ECharts仍使主bundle约900kB（gzip约302kB），Vite有>500kB提示，暂未新增拆包配置。
- `PLAYWRIGHT_BASE_URL=http://127.0.0.1:4147 PLAYWRIGHT_CHANNEL=chrome npm run test:e2e`：最终 **1440桌面/390触控2项通过**。只启动Python静态产物预览，fixture替代全部API与WebSocket；不需要密码或业务服务。复现也可对 `npm run preview` 的静态地址执行同命令；e2e自己在加载前安装测试fixture。
- `git diff --check`：通过。没有lint脚本，未编造npm lint结果。

Vitest仍打印少量JSDOM未实现Canvas getContext诊断，未抑制错误或增加skip；实际Canvas像素由Chrome浏览器和原坐标/新增RGBA测试交叉验证。原样记录此限制。

按要求使用Orca浏览器：page `892aab74-02fa-4ffe-a08f-cdd3b3a2ebf4`，桌面/手机交互及布局DOM检查正常，主/小图切换map/stick节点均保持同一对象。Orca screenshot/full-screenshot多次只返回整幅灰色（已保存样本），不能据此判视觉通过；补充用已安装Chrome的Playwright截图，并实际查看桌面驾驶、手机驾驶和手机任务页。没有声称真实摄像头画面验证通过，fixture中视频不可用是预期。

交付保留隔离worktree供负责人亲审，不自动merge。自审替代辅助worker（派发明确禁止子代理）；负责人仍需亲阅实际diff。PR为Draft，等待负责人视觉与代码验收。

## 版本、精确范围与证据归档

- `235e5de`：公共控件、依赖、参考资产和批准设计。此前188项绿灯来自当时**含未提交接线**的工作树，不是把基础提交单独验证为最终产品。
- `fce8d18`：页面、驾驶、地图/任务、状态、日志、设置接线；随后回归补充见 `ff68080`。
- 最终产品/测试代码版本：`5cb89a303c22271fd1ab3f184ead49d82af08466`（3d2ceed）。最终195项Vitest、typecheck、build与2项浏览器流程都针对这一产品树；后续提交仅为文档，不将阶段188项证据算最终绿灯。
- 归档：`/tmp/bot1-control-workspace-delivery-2026-11-24/`，含各RED/最终GREEN命令输出、4张实际Chrome截图、执行ledger与Orca观测JSON。
- 静态安全预览：`http://127.0.0.1:4147/`，目录 `/tmp/control-workspace-static-preview`，Orca静态预览终端 `term_9d79be75-2b92-4f12-b9ef-820d946cf926`；没有真实API/socket。
- Orca灰色截图样本：`/tmp/orca-browser-screenshots/1c9b5686-3810-4b31-b226-517bf0e2b8ce-screenshot.png`，不计视觉验收证据。

自审的重要修复：新增`useMeasuredSpeed`显示适配，防止普通快照仍到达却重复旧runtime sequence时继续展示旧速度（新增同序号不可恢复/新序号恢复回归，`/tmp/control-speed-freshness-red.log`→最终GREEN）；只读预览误拖拽（RED→GREEN）；未变sequence的浮层再打开不刷新CPU；部分航点确认后的任务版本冲突仍锁定并提示重载；手机驾驶滚动急停不可见（旧静态构建RED，固定顶部GREEN）。不派辅助worker、保留唯一owner、缺几何fallback、触控允许纵向滚动均依派发/负责人决定执行；若这些决策不合用户实际期待，需要负责人在Draft PR视觉/代码验收时调整。暂缓小项：ECharts主bundle拆包；真实轮廓、车辆与视频全链路属于已明确的数据/验收限制，没有混作通过。

实际总范围 **55 路径**，新增20、修改35、删除0：{'文档': 4, '测试/fixture': 11, '依赖/构建': 3, '产品/资产': 37}。低于原63路径预算：复用了未修改的VideoPanel、MapManager、MapCatalogController、ScheduleFields等owner/接口与原测试；几何无真实生产者，按负责人决定不额外增加10–15路径。新增测试fixture与可选Chrome channel配置已在计划登记；额外修改useVehicleView.ts只增加共享的显示新鲜度适配，不添加连接/控制owner。

| 变更 | 精确路径 |
|---|---|
| M | `README.md` |
| A | `docs/superpowers/plans/2026-11-24-control-workspace-implementation.md` |
| A | `docs/superpowers/specs/2026-11-24-control-workspace-design.md` |
| A | `docs/testing/2026-11-24-control-workspace-validation.md` |
| M | `web/e2e/control.spec.ts` |
| A | `web/e2e/fixtures/control-fixture.js` |
| M | `web/package-lock.json` |
| M | `web/package.json` |
| M | `web/playwright.config.ts` |
| M | `web/src/App.module.css` |
| M | `web/src/App.tsx` |
| A | `web/src/assets/concrete.jpg` |
| A | `web/src/assets/vehicle-marker.svg` |
| A | `web/src/components/ui/Button.tsx` |
| A | `web/src/components/ui/FloatingCard.tsx` |
| A | `web/src/components/ui/IconButton.tsx` |
| A | `web/src/components/ui/SegmentedControl.tsx` |
| A | `web/src/components/ui/Switch.tsx` |
| A | `web/src/components/ui/Ui.module.css` |
| A | `web/src/components/ui/ui.test.tsx` |
| A | `web/src/display/speedUnits.ts` |
| M | `web/src/features/devices/ControlSurface.tsx` |
| M | `web/src/features/devices/Devices.module.css` |
| M | `web/src/features/devices/OperatorWorkspace.module.css` |
| M | `web/src/features/devices/PairingPanel.tsx` |
| M | `web/src/features/devices/PerformancePanel.test.tsx` |
| M | `web/src/features/devices/PerformancePanel.tsx` |
| M | `web/src/features/devices/RobotPanel.tsx` |
| A | `web/src/features/devices/StatusOverlay.module.css` |
| A | `web/src/features/devices/StatusOverlay.tsx` |
| M | `web/src/features/devices/VehicleStateView.tsx` |
| M | `web/src/features/devices/VehicleTools.tsx` |
| A | `web/src/features/devices/charts.ts` |
| M | `web/src/features/devices/devices.test.tsx` |
| M | `web/src/features/devices/useVehicleView.ts` |
| A | `web/src/features/devices/useWorkspaceView.ts` |
| M | `web/src/features/devices/vehicleState.test.tsx` |
| A | `web/src/features/devices/workspace.test.tsx` |
| M | `web/src/features/drive/Drive.module.css` |
| A | `web/src/features/drive/DriveJoystick.tsx` |
| M | `web/src/features/drive/DriveView.tsx` |
| M | `web/src/features/drive/drive.test.tsx` |
| M | `web/src/features/logs/Logs.module.css` |
| M | `web/src/features/logs/VehicleLogs.tsx` |
| M | `web/src/features/map/Map.module.css` |
| M | `web/src/features/map/MapCanvas.tsx` |
| M | `web/src/features/map/map.test.tsx` |
| M | `web/src/features/settings/VehicleSettings.tsx` |
| M | `web/src/features/settings/settings.test.tsx` |
| M | `web/src/features/tasks/ScheduleEditor.tsx` |
| M | `web/src/features/tasks/TaskList.tsx` |
| M | `web/src/features/tasks/Tasks.module.css` |
| M | `web/src/features/tasks/tasks.test.tsx` |
| M | `web/src/features/video/Video.module.css` |
| M | `web/src/theme.css` |

## 2026-11-24 独立审核返修（F1 / F2 / G1）

本节更正并取代前文有关“同 ID 可恢复所有多点失败”“手机已满足底部三组布局”“fixture 可替代原集成入口”的结论；前文 195 项及两尺寸截图是历史证据，不能当作返修版本通过。初始分支为干净的 `feat/control-workspace`、`547659278be83f0d3e8075cef7170afc64fd3130`；本轮产品/测试提交为 **`91a061746800a61336fbe16a0d3709fc0b4e4b4b`**，后续仅更新本文档。

用户已另行明确需要**独立、可编辑的任务名称字段**，涉及存储、接口及旧任务兼容，等待后续盘点与单独派发。当前界面的“任务名称（编号）”仍是历史 `task_id` 展示，本轮没有完成该新需求，不能把编号命名宣称为最终批准设计。

### 调用关系、依据与前后行为

- 已阅读独立报告 `/tmp/control-workspace-independent-review-2026-11-24.md`、批准设计/计划、原验证文档和 `/tmp/control-workspace-approved/control-demo/index.html`；核对独立复现脚本。工作区及逐级祖先没有可用 AGENTS.md，按派发中明确给出的项目约束执行；没有其他 worker 并行写入，没有派辅助 Agent。
- F1：`TaskList.submit → RobotLink.request → RobotClient`。client 将 `ok:false` 转为 `CommandError`，只保留 code，不能从 code 得出 journal 终态；车端 `Dispatcher._execute` 初次 action 错误也不保证响应中带状态。`run(operation.query)` 返回对应 journal 状态，`_replay` 只对 SUCCEEDED 回放成功，对 FAILED 永远拒绝，NEEDS_REVIEW/运行中也不会重新创建；因此本轮直接接已有 query，无需改 client、后端协议或存储。
- 返修前第二点 unavailable 后总复用已失败 ID；现在收到保存异常后查询**相同 operation_id**，只有 `ok:true`、ID 匹配、`status:failed` 才为失败点生成下次显式提交的新 ID，不自动再次创建。`succeeded` 当作已确认，继续后续保存；查询失败、身份不匹配、running、needs_review 均保留原 ID，提示结果未知/需核查。再次提交仅重放原 ID 并查询，绝不将 timeout 或 unavailable 本身当终态依据。成功点移入当前目录且从草稿移除，后续不再创建；草稿、已确认计数及版本冲突锁定继续保留。失效 editor 不再发后续查询或保存。
- F2：返修前手机地图工具、图例和 dock 分别处于不同高度；现在视图卡片拥有一个底部区，MapCanvas 只将现有工具节点 portal 到该区，不增加地图、连接或几何 owner。桌面三组底边对齐；手机在同一底部区分两行，工具/比例尺与图例入口一行、四模式一行，图例点开后能看见内容。卡片内边距桌面 16px、手机 12px；操作卡片、预览与底部区之间保持 12px 量级间距，手机保留纵向滚动与完整触控目标。小地图保留本地图工具/实际比例尺/适应窗口，交换仍仅由小图专用按钮触发，只有摄像头可关闭。
- 恢复横屏流程后实际发现 844×390 滚动驾驶时顶部急停离开视口，修正在原 App CSS 中的横屏 sticky header；没有缩小触控目标或恢复零滚动约束。
- G1：安全 fixture 套件继续独立存在，恢复 `control.integration.spec.ts`。集成套件不安装 fixture，从 `/login` 走真实认证 HTTP 与 WebSocket，验证当前单杆与控制权/松手/释放路径及滚动后急停可达，保留 1440、390、844×390 原入口；真实测试目标必须显式指定为模拟车辆，不自动启动任何服务器。
- 可读性：整理 ControlSurface、DriveJoystick、TaskList、StatusOverlay 与相关响应式 CSS 为正常多行；提取指针/键盘处理、排序与 journal 查询函数，保留原 owner。StatusOverlay 分离 ECharts 实例生命周期与样本更新，样本/单位变化用 setOption，ResizeObserver 用 resize，隐藏/切分类/卸载时 dispose；未测量性能收益，不声称修复了已证实的卡顿。

### 此次真实 bug 定向回归

原第二点抛普通 Error、第三次无条件成功的用例被替换为按 journal 状态拒绝重放的用例。首次用五分支测试在旧实现上得到 **5 失败 / 22 通过**（`tasks-red.log`，缺 operation.query / 无法恢复）；随后补齐 timeout 与 running 分支，最终覆盖 7 个具体分支：

| 触发 / query 依据 | 断言 |
|---|---|
| 第二点 unavailable，query FAILED | 第一次不创建任务；用户再次提交用新 ID 成功；第一点只创建一次，草稿保留到确认 |
| 第二点 timeout，query FAILED | 即使同为 timeout，也仅凭匹配终态才允许换 ID；恢复后成功 |
| 丢回执，query SUCCEEDED | 第二点不再创建，直接继续任务保存 |
| timeout，query NEEDS_REVIEW | 重试保留原 ID，journal 重放仍拒绝，不创建任务 |
| 客户端 timeout，query RUNNING | 重试原 ID，运行中 conflict 不被误当成目录版本冲突锁死；不创建任务 |
| timeout，query operation_unknown | 原 ID 与草稿保留，提示核查，不创建任务 |
| query 返回其他 ID 的 FAILED | 不能作为终态依据；原 ID 保留，不创建任务 |

F2 使用旧 5476592 静态产物时，新手机底部区断言失败（`layout-red.log`）。布局首轮截图发现比例尺与图例入口过紧，随后增加实际矩形不重叠断言，在中间版本复现失败（`spacing-check.log`），最后用手机工具内换行排布比例尺通过，保留按钮 44px 目标。恢复 844×390 场景后急停可见性先失败（`browser-initial.log`），补横屏顶部栏规则后通过。一次临时检查曾因预览产物缺少新增 aria-label 而失败（`spacing-red.log`），该运行是测试与产物不一致，不算独立产品缺陷证据。

### 复用保护、场景数量与未运行入口

- 单元：定向执行原 tasks、client、workspace、PerformancePanel、drive、map 六文件；复用真实 client 的认证 envelope、错误与 query 解析保护，以及地图、控制权/新按压/释放、过期读数、拖序、任务版本冲突与只读预览保护，没有删除或弱化这些用例。
- 数量：原多点恢复 **1 个场景实质改写为 7 个 journal 分支**（执行实例净增 6），保留原其余 22 个 tasks 用例。安全 fixture 原 1440/390 两个场景实质补强布局/缩放/截图检查，恢复 844×390 一个入口；复用完整主要流程，没有为横屏机械复制新矩阵。集成从原四个重叠入口整理为三个尺寸入口，单杆与允许滚动替代失效双杆/整页零滚动断言，保留急停和实际 HTTP/WS 路径。**三个集成入口本轮均未运行**，只静态/类型检查。
- fixture 三尺寸均检查三组处于同一底部容器、底边距、工具与图例不重叠；390 检查触控尺寸，1440 继续检查三组底边差 ≤1px。另复用摄像头开关/小图图标交换、普通地图点击不交换、地图与摇杆 DOM owner 不变、缩放真正改变 zoom、小图适应窗口/比例尺、单位、设置 SI 保存、日志文本安全、路线顺序/cron/timezone、未实现入口零请求与急停可达。

集成入口仅在同时设置下列变量后才会连接，默认全部 skip，Playwright 配置没有 webServer：

```sh
# 指向专门配置的模拟测试部署；本轮没有执行此命令。
PLAYWRIGHT_CONTROL_INTEGRATION=1 \
PLAYWRIGHT_INTEGRATION_BASE_URL="$TEST_DEPLOYMENT_URL" \
SENIOR_CONTROL_PASSWORD="$TEST_ADMIN_PASSWORD" \
PLAYWRIGHT_INTEGRATION_ROBOT_NAME="$SIMULATED_ROBOT_NAME" \
npm run test:e2e -- control.integration.spec.ts
```

### 最终提交上的命令、结果和证据

以下命令都在 `web/`、产品 HEAD `91a0617` 上执行；没有拿旧 5476592 或早期 195 项证据补本轮通过。证据根目录 **`/tmp/control-workspace-repair-2026-11-24/`**，`version.json` 记录产品 SHA，`build-sha256.json` 记录最终 dist 文件校验值。仅更新已批准的 `/tmp/control-workspace-static-preview/` 产物，复用既有 `127.0.0.1:4147` 静态预览，没有启动开发或业务服务。

| 命令 | 结果 / 日志 |
|---|---|
| `npm test -- --run src/features/tasks/tasks.test.tsx src/api/client.test.ts src/features/devices/workspace.test.tsx src/features/devices/PerformancePanel.test.tsx src/features/drive/drive.test.tsx src/features/map/map.test.tsx` | **6 文件 / 86 项通过**，`unit-final.log`；其中 tasks 29 项（22 原保护 + 7 journal 分支） |
| `npm run typecheck` | 通过，`typecheck-final.log` |
| `npx tsc --noEmit --target ES2022 --module ESNext --moduleResolution bundler --strict --skipLibCheck --types node e2e/control.spec.ts e2e/control.integration.spec.ts` | 通过，`e2e-typecheck-final.log`；集成入口仅为类型检查 |
| `npm run build` | 通过，`build-final.log`；原 Vite 大包提示仍在，未改构建配置 |
| `PLAYWRIGHT_BASE_URL=http://127.0.0.1:4147 PLAYWRIGHT_CHANNEL=chrome npm run test:e2e -- control.spec.ts --output=/tmp/control-workspace-repair-2026-11-24/browser-final` | **1440 / 390 / 844×390 安全 fixture 3 项通过**，`browser-final.log` |
| `git diff --check` | 通过；最终文档提交后再次检查 |

每个 `browser-final/control-<尺寸>...-chromium/` 下有 `drive.png`、`mini-map.png`、`estop-scrolled.png`、`tasks.png`（共 12 张）。已实际查看最终桌面/手机驾驶底部、手机任务底部、手机小地图工具、横屏滚动急停截图；同时以 DOM 矩形断言避免只靠截图判断。驾驶截图是在已手动适配过小地图并关闭摄像头之后拍摄，主地图保留同一 viewport 的缩放，因此图形比首次加载时小，不表示使用虚构地图或重新创建 owner。

不重复无关全仓/车端测试；前文 195 项只作为历史结果，本轮不宣称全 201 项运行通过。未连接真实车、ROS、配对兑换、真实 WebRTC/TURN 或运动服务；fixture 通过不证明这些链路。真实服务集成没有部署可用且禁止本轮连接，因此**未运行**，不能声明通过。显式 footprint 缺失仍为箭头 fallback + 尺寸不可用；run-now/FIFO/证据层仍提示暂未实现。独立可编辑任务名称待后续派发。

### 实际范围与交付状态

实际 **12 路径：产品 8、测试 3、文档 1、配置/构建/依赖 0；新增 1、修改 11、删除 0**，在预报 11–16 范围内。较最初内部 7 产品文件估计多出的 App CSS 仅处理恢复横屏入口实际发现的急停可达缺口；未扩大架构或改后端。

| 分类 | 路径 |
|---|---|
| 产品（8，修改） | `web/src/App.module.css`、`web/src/features/devices/ControlSurface.tsx`、`OperatorWorkspace.module.css`、`StatusOverlay.tsx`、`web/src/features/drive/DriveJoystick.tsx`、`web/src/features/map/MapCanvas.tsx`、`Map.module.css`、`web/src/features/tasks/TaskList.tsx` |
| 测试（3，新增1/修改2） | `web/src/features/tasks/tasks.test.tsx`、`web/e2e/control.spec.ts`、新增 `web/e2e/control.integration.spec.ts` |
| 文档（1，修改） | 本验证记录 |

产品提交后只修改本文档；文档提交与最终远端/PR/清洁状态另记在证据目录 `delivery-report.md`。保持原分支、原工作区、原 PR #40 Draft，不创建第二 PR、不合并 main。自审实际 diff 后等待负责人亲审，本 worker 自审不冒充独立复核。


## 2026-11-25 独立任务名称：正常集成与同版验证

本节取代前文“独立名称待后续派发”的状态；此前控制页/F1/F2/G1的历史证据仍按原版本保留。负责人已完成两子分支实际diff与视觉亲审后正式派发Task3，集成未再次询问设计、未派子Agent。用户没有配置旧任务，本轮不扩历史数据回填、调度或几何功能。

### 最终行为与产品版本

名称独立保存在车端SQLite，可重复、可单独改名；新建生成稳定UUID，失败/未知重试保留草稿身份。名称去显式约定集合的首尾空白，保留内部字符，规范化后1–128个Unicode code point；显式null/全空白拒绝。旧API创建省name使用ID，更新省name与启停保留名称。改名递增定义revision，但不改变计划、路线、created_at、已接受实例/版本/冻结路线；同帧列表与执行卡片关联当前名称，控制命令仍使用执行实例ID。旧端缺task.names.v1（包括零目录）只读ID fallback、禁名称写入/命名创建并提示升级，仍可编辑已有任务的非名称字段。

- 准确父工作区 `/home/toby/orca/workspaces/senior-robot-control/control-workspace`，固定 `feat/control-workspace`，起点干净 `119dc4c578eaa73e0f1fbf3709ec9b8add602112`。
- 子工作区开始均干净：backend `6eb3eb9370cd0feb2009cee7b6a13cdee7ffefed`，frontend `00deeebce4b6389341a924d38dedcc4d91fecbc4`。
- 按顺序正常 `git merge --no-ff`：后端合并 `b9b4bfc1b3e7e7cdfad6bb712e6e75f009b6a9d0`，前端合并 `24b95555b5a5e7b9cbd11dcf10642fe659634299`；两次均ort成功、**无冲突**，没有整文件ours/theirs、cherry-pick、reset或替换父分支。
- 最终产品/测试版本 **`c864d82f46b032cd665844a86bf474697eac011b`**。Task3仅整理指定4个测试文件，产品新增/修改/删除均0；19个产品blob与正常合并HEAD `24b9555`逐个相同。随后仅提交本文档，最终交付HEAD/推送/PR状态另记证据目录delivery-report与version.json。
- 名称整体相对119dc4c共**31路径：产品19、测试/fixture9、文档3**，新增2、修改29、删除0；本次集成自身编辑**5路径：测试4、本文档1**，在批准预算内。没有依赖/配置改动。

证据目录 **`/tmp/task-names-integration-2026-11-25/`**：version.json保存产品/测试提交与blob，build-sha256.json保存本版dist哈希，ancestry.log给出两个merge的双parent及三个审核提交的祖先/零独有提交证明，product-preservation.log确认集成无产品改动。

### 同版导入、持久化与生产领取链

原先单独editable安装可能指向子工作区/旧main，因此显式使用父树PYTHONPATH，并在195项pytest结束后枚举并断言**127个项目/测试模块全部来自父树**（import-paths.json）。代表性实际路径如下；后端解释器的依赖环境随后原地移至父工作区 `.venv` 并重新安装父树editable，最终路径再查无误（import-paths-final.log）：

| 模块 | 实际__file__（父工作区相对路径） |
|---|---|
| senior_contracts / wire | contracts/src/senior_contracts/__init__.py / wire.py |
| vehicle_manager / dispatcher | senior_control/src/vehicle_manager/__init__.py / agent/dispatcher.py |
| cloud / snapshots | cloud/src/cloud/__init__.py / snapshots.py |
| test_agent / test_task_lifecycle_store | senior_control/tests/test_agent.py / test_task_lifecycle_store.py |
| vehicle_core | vehicle_core/__init__.py |

复用独立审核 `/tmp/task-names-review-probe.py` 与返修frontend-probe，保留适配副本 review-probe.py/frontend-probe.cjs；只改证据输出地址、实际父树TS路径并补齐本轮必要创建/改名读取，不另建协议。临时SQLite均在/tmp，退出删除；无服务、网络或车辆动作。

1. StoreTaskPort创建“东区巡检”→关闭/重开SQLite→真实WorkspaceReader.task.get→真实SnapshotState.collect任务summary→shared wire→真实cloud SnapshotCache及其snapshot_envelope→父树实际TypeScript parseInbound。随后同一ID改成“夜班巡检”，重复该链；名称与revision 1→2都正确。create-rename-wire.json保存真实get与完整最终帧，两个probe日志均通过断言。未把agent uplink当作带last_synced_at的cloud下行。
2. 审核原probe的“读取到期定义→纯改名→旧定义提交”实际经过**生产LifecycleStore.submit/原claim写事务**：accepted=True、冻结名称夜班巡检/revision2、初次1个effect、同请求重放0个effect且receipt相同。没有只用兼容executor的N5用例来宣称生产领取已验证。N3既有新增回归另验证已接受实例、冻结路线/版本/effects不变及实时名称投影。
3. R1真实FEFF-only/NEL-only写入均拒绝；NEL/FEFF包围中文存储/wire规范为“巡检”，实际parseInbound接收；严格拒绝null/FEFF/NEL summary。R2非法省name update与119dc4c对照均为validation/FAILED，未变unknown边界。

### 本轮命令与结果

后端从父工作区执行。195项运行最初使用 `../control-workspace-names-backend/.venv/bin/python` 的依赖解释器，显式导入父树；该环境已移至父 `.venv`，以下为可继续复现的相同命令，非子树产品测试。Python 3.11.2；Node v24.13.0。

```sh
PYTHONPATH=contracts/src:senior_control/src:cloud/src:senior_control/tests:tests/integration:. \
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest \
  senior_control/tests/test_agent.py senior_control/tests/test_workspace_data.py \
  senior_control/tests/test_task_runner.py senior_control/tests/test_task_lifecycle_store.py \
  tests/unit/test_protocol.py -q
# 195 passed in 15.80s；backend-regression.log；同进程额外断言127个import均在父树。

PYTHONPATH=contracts/src:senior_control/src:cloud/src:senior_control/tests:tests/integration:ros2/src/senior_mapping:. \
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python /tmp/task-names-integration-2026-11-25/review-probe.py
node /tmp/task-names-integration-2026-11-25/frontend-probe.cjs
# probe-backend.log / probe-frontend.log，均成功。

PYTHONPATH=contracts/src:senior_control/src:cloud/src:. PYTHONDONTWRITEBYTECODE=1 \
.venv/bin/python -m pytest senior_control/tests/test_agent.py \
  -k 'test_operation_id_deduplicates or test_same_operation_id_with_different_content_conflicts' -q
# 最终环境2 passed，61 deselected；idempotency-final.log。
```

前端从 `web/` 执行：

| 命令 | 结果 / 日志 |
|---|---|
| `npm test -- --run src/api/vehicleState.test.ts src/features/tasks/tasks.test.tsx src/features/devices/vehicleState.test.tsx src/api/client.test.ts` | **4文件/63通过**；frontend-targeted.log，包含N7–N10、同实例命令、F1七分支与真实client保护 |
| `npm run typecheck` | 通过；typecheck.log |
| `npx tsc --noEmit --target ES2022 --module ESNext --moduleResolution bundler --strict --skipLibCheck --types node e2e/control.spec.ts e2e/control.integration.spec.ts` | 通过；e2e-typecheck.log |
| `npm run build` | 通过；build.log，保留原>500kB提示 |
| `PLAYWRIGHT_BASE_URL=http://127.0.0.1:4147 PLAYWRIGHT_CHANNEL=chrome npm run test:e2e -- control.spec.ts --output=/tmp/task-names-integration-2026-11-25/browser-final` | **三尺寸3通过**（12.9s）；browser-final.log |
| `git diff --check` | 通过；最终文档提交后再记录清洁状态 |

未变化的完整lifecycle service/其他前端模块沿用独立审核及原交付/返修证据，不无故全仓重复。本次没有新增skip/xfail或删除119dc4c基线保护。早期失败保留并明确不作产品bug证据：新复制的忽略目录.venv耗尽共享磁盘，首次pytest输出中断；只删除本轮新复制环境释放214MB后重跑195项通过，后续复用原环境并移至父目录。首次probe漏调用cloud下行投影导致last_synced_at=null，补用现有cloud.snapshot_envelope后通过；一次npm在父目录而非web目录运行的ENOENT也仅为命令目录错误。

### 独立输入场景计数、bug回归与测试整理（2026-11-25最终校正）

下表以真实独立输入或连续使用流程计数，不按it块或for循环合并独立参数。创建→同ID改名→预览、同请求重放可作为连续流程；互不依赖的创建支线、不同坏值分别计数。原标签迁移、旧安全保护不计为名称新增。下列覆盖属于名称功能整体，**本次证据校正新增测试输入0**。

核心覆盖共 **20条独立输入/连续流程：后端10、前端10**。缓存单元路径仅测单个命名快照“夜班巡检”revision2；本次确实删除“东区巡检”revision1的重复正向输入（原B6），不是改变独立输入计数口径。原两次独立快照构造没有执行真实改名；真正创建→改名→数据库重开→cache/decoder更新沿用已审生产probe及B2/B3/N7/browser链路证据。真实创建后重开与旧API省名创建支线仍分别列出，原行号保留以便追溯。路径均相对父工作区：

| # | 独立输入/连续流程 | 真实测试路径与定位 | 来源 |
|---|---|---|---|
| B1 | 空domain3目录升级到4并重开，创建后改名“东区巡检”，其他schema/probe保持 | `senior_control/tests/test_task_runner.py::test_empty_domain3_upgrades_task_names_and_reopens` | 名称新增，复用schema隔离保护 |
| B2 | route-01以`  东区巡检  `创建，重开后task.get/真实summary读取 | `senior_control/tests/test_workspace_data.py::test_task_names_persist_and_legacy_mutations_preserve_them` | 名称新增 |
| B3 | old-api省name创建→改名“夜班巡检”→省name更新→禁用保名，重开summary读取 | 同B2，独立old-api支线 | 名称新增 |
| B4 | 接纳执行后“东区巡检”改“夜班巡检”，冻结路线/实例/版本/effect不变并重放 | `senior_control/tests/test_task_lifecycle_store.py::test_rename_keeps_accepted_execution_and_fingerprint` | 名称新增 |
| B5 | 读到期定义→改名→领取/同请求重放，仅一次effect | `senior_control/tests/test_task_runner.py::test_due_claim_accepts_only_a_concurrent_name_change_once`；生产事务另见历史`/tmp/task-names-integration-2026-11-25/review-probe.py::main`的LifecycleStore.submit链 | 名称新增；不把兼容executor冒称生产链 |
| B7 | “夜班巡检”revision2，经完整envelope/cloud cache往返 | `senior_control/tests/test_agent.py::test_named_summaries_roundtrip_through_cloud_cache`，唯一命名快照输入 | 名称新增 |
| B8 | 旧summary省name，经完整envelope/cache投影往返且不补字段 | 同B7，删除raw.payload.tasks[0].name后的分支 | 名称兼容输入 |
| B9 | API显式`None`创建拒绝，最终目录无写入 | `senior_control/tests/test_workspace_data.py::test_task_names_reject_explicit_null_or_blank_without_writing`，invalid第一输入 | 名称新增 |
| B10 | API普通全空白`   `创建拒绝，最终目录无写入 | 同B9，invalid第二独立输入 | 名称新增 |
| B11 | one/two不同ID分别以`  夜班  巡检  `及`夜班  巡检`创建，同名且保留双内部空格 | 同B9，两ID共同构成重复名称流程 | 名称新增 |
| F1 | execution `exec-88`/definition `route-01`/“东区巡检”/version3正向解码 | `web/src/api/vehicleState.test.ts`，`accepts named execution metadata…` | 名称新增 |
| F2 | 正常中文“东区巡检”summary完整快照解码 | 同文件，`preserves Chinese task summary names…` | 名称新增 |
| F3 | N7独立UUID创建“东区巡检”→同ID改“夜班巡检”→命名预览 | `web/src/features/tasks/tasks.test.tsx`，`N7 creates…` | 名称新增连续流程 |
| F4 | N9无能力旧端已有任务/零目录→省name非名称编辑→切回新端 | 同文件，`N9 locks names…` | 名称新增连续流程 |
| F5 | N10 two/one同名“东区  巡检”，识别ID并删除two | 同文件，`N10 trims names…`，删除支线 | 名称新增 |
| F6 | N10普通全空白`   `禁止提交 | 同F5，value循环第一独立输入 | 名称新增 |
| F7 | N10以`  东区  巡检  `创建，提交规范化名且保留内部双空格 | 同F5，最后独立创建支线 | 名称新增 |
| F8 | 当前路线进度显示“夜班巡检”，地图换代/陈旧后隐藏 | 同文件，`shows only vehicle-reported progress on the current live map` | 既有用例增强名称，陈旧地图保护复用 |
| F9 | N8执行改名后pause→resume→cancel始终发送实例ID及实际version3→4→5 | `web/src/features/devices/vehicleState.test.tsx`，`runs one real execution…` | 既有执行流程增强 |
| F10 | 1440安全fixture命名创建→改名“夜班巡检”→预览 | `web/e2e/control.spec.ts`，1440条件分支 | 既有fixture增强，仅历史本版证据；此次未重跑 |

**R1/R2实际bug输入另列19条覆盖记录**，不是“4个it/专题=4个输入”。其中R1共18条，R2共1条连续请求流程。R1-15/16是F3同一连续流程中的两个实际输入，已与核心表重叠，不能相加宣称39个不重复场景；其余列明确具体值与独立数量。

| # | 实际输入与预期 | 测试定位 |
|---|---|---|
| R1-1 | API `\uFEFF`-only拒绝/VALIDATION且该ID无写入 | `test_agent.py::test_task_name_boundary_whitespace_store_to_wire` |
| R1-2 | API `\u0085巡 检\u0085`经真实SQLite→wire为“巡 检” | 同上，持久化成功支线 |
| R1-3 | wire `\uFEFF`-only拒绝 | 同上，invalid循环唯一输入 |
| R1-4 | wire `\u0085巡 检\u0085`规范为“巡 检” | 同上，直接wire输入 |
| R1-5 | execution name=`\uFEFF`拒绝 | `web/src/api/vehicleState.test.ts`，命名execution专题 |
| R1-6 | execution name=`\u0085`拒绝 | 同上，独立patch |
| R1-7 | summary name=`\uFEFF`拒绝 | 同文件，中文summary专题 |
| R1-8 | summary name=`\u0085`拒绝 | 同上，独立name |
| R1-9 | task.get name=`\uFEFF`拒绝 | `web/src/features/tasks/tasks.test.tsx`，`rejects FEFF-only and NEL-only names from task.get` |
| R1-10 | task.get name=`\u0085`拒绝 | 同上，独立name |
| R1-11 | normalize ` \u0085东区 \u0085\uFEFF 巡检\uFEFF `剥首尾、保内部NEL/FEFF/空格 | `web/src/api/vehicleState.test.ts`，`uses the agreed boundary whitespace…` |
| R1-12 | isTaskName(`\u0085`)为false | 同上，独立输入 |
| R1-13 | isTaskName(`\uFEFF`)为false | 同上，独立输入 |
| R1-14 | ` \u0085`+128个😀+`\uFEFF `为有效名（code point边界） | 同上，独立边缘长度输入 |
| R1-15 | N7创建输入` \u0085东区巡检\u0085 `，发送“东区巡检” | F3创建阶段，重叠覆盖 |
| R1-16 | N7改名输入` \u0085夜班巡检\uFEFF `，发送“夜班巡检” | F3改名阶段，重叠覆盖 |
| R1-17 | N10 UI `\uFEFF`-only禁止提交 | F5所在用例value循环，独立输入，不并入F6普通空白 |
| R1-18 | N10 UI `\u0085`-only禁止提交 | 同上，独立输入 |
| R2-1 | 省task_id且省name的update→VALIDATION/FAILED→目录无写入→原operation_id重放仍同FAILED且无写入 | `senior_control/tests/test_agent.py::test_missing_update_task_id_is_failed_validation`，完整原流程未改 |

复用保护单列：F1多点恢复七分支（unavailable/FAILED、timeout/FAILED、SUCCEEDED、NEEDS_REVIEW、RUNNING、operation_unknown、其他ID的FAILED）仍是**7个独立journal输入分支**，不计为名称新增；原认证/envelope/strict协议、schema/core/agent版本隔离、任务expected_revision冲突、operation_id去重/异内容冲突、执行实例控制与急停、地图身份/陈旧地图、设置与日志安全等原保护保留。原标签迁移不算名称新增。

本次严格只改`senior_control/tests/test_agent.py`、`web/src/api/vehicleState.test.ts`和本文档。Agent原`test_task_edit_commits_in_robot_database`的body.name与两条stored.name恢复119dc4c原样，created/enable/revision/conflict及全部原断言不动；真实名称持久化由B2/B3等原已批准用例承接。R1去掉重复wire None及末尾省字段检查，由B9及B8承接；保留四个实际边界输入。execution专题去重复null/普通空白，summary去重复null；strict产品不改，normalize四个实际输入断言原样保留。未改tasks.test.tsx、R2、F1或其他批准存储用例，未合并it或引入新参数。

三尺寸原布局/控制检查均为基线复用，名称派发没有实质改写这些流程；只有1440新增命名动作。390/844未执行命名创建/改名，不能写成小屏名称验证，也不能用视口参数合并独立输入来调减计数。

### 本次证据校正验证（task_d58414bcb315）

起点准确HEAD为`ac2bd1041f22d2ae21a93e41145f8bd0a6116b02`，分支`feat/control-workspace`，干净状态；测试在该父树加本次两测试文件小diff上执行。产品/配置新增、修改、删除均0，共3条修改路径，无新增测试输入。最终提交完整SHA、逐项产品blob比较、实际3文件diff、命令原始日志/退出码、清洁状态与origin/PR40同HEAD证据记录于`/tmp/task-names-final-evidence-correction-2026-11-25.md`及同名证据目录；提交SHA不自嵌入本提交。

实测依赖：Python 3.11.2、pytest 8.4.2、pytest-asyncio 0.26.0、pydantic 2.14.4、aiosqlite 0.22.1；Node v24.13.0、npm 11.6.2、Vitest 5.0.5（`versions.json`/`frontend.log`）。本次命令及结果：

```sh
# 父工作区；run_backend.py只调用下列pytest并在同进程结束后断言项目模块路径。
PYTHONPATH=contracts/src:senior_control/src:cloud/src:senior_control/tests:tests/integration:. \
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  /tmp/task-names-final-evidence-correction-2026-11-25/run_backend.py
# 内部：pytest.main(['senior_control/tests/test_agent.py',
#                      'senior_control/tests/test_workspace_data.py', '-q'])
# 70 passed in 8.07s，exit 0；121个项目/测试模块均来自准确父树。
# backend.log、import-paths.json；包含原dedup/conflict及完整R2 journal回归。

# web/：包含必要N9/N10、task.get R1及F1七分支原断言。
npm test -- --run src/api/vehicleState.test.ts src/features/tasks/tasks.test.tsx
# 2 files / 42 passed，2.63s，exit 0；frontend.log。

# 父工作区；比较既有19产品blob、原Agent测试、R2、normalize与旧前端测试体。
python3 /tmp/task-names-final-evidence-correction-2026-11-25/check_preservation.py
# exit 0；19/19等于ac2bd10，除指定三路径外整树无改动。
git diff --check
# exit 0；diff-check.log；提交后再次核对并记录干净状态。
```

本次没有运行全量、build、typecheck、三尺寸e2e、生产probe或实车；上表B1/B4/B5/F9/F10等未重跑入口明确复用前文已审同一产品树证据。机械比对确认原Agent SQLite用例整体等于119dc4c，其他Agent函数除指定R1整理外等于ac2bd10，R2原样，前端119dc4c全部测试体原样，normalize四断言原样；`tasks.test.tsx`（含F1七分支、N9/N10与task.get）逐字节未改。

### 正式交付：单个缓存快照收敛（task_9a73b3b42e9c）

负责人已亲阅309c707相对ac2bd10的全部小diff并完成产品、交互、R1/R2及同版链路/视觉验收。本次起点为干净`309c7074db48aed60aea87c9e6f6468d724db1fa`，仅修改`senior_control/tests/test_agent.py`和本文档，产品/其他测试/配置改动0，新增输入0，删除一个重复缓存正向输入。直接构造“夜班巡检”revision2的current并走原wire/model/cache/get/恢复/assert路径，不留单参数循环；末尾旧summary省name分支原样保留，其余测试、R1/R2/F1与119dc4c原保护不改。

核心实际输入数由21降为20（后端11降为10，前端10不变），删除原B6行并保留B7。真实BUG表仍为R1的18条及R2的1条覆盖记录，N7两输入与核心流程重叠；复用保护与三尺寸说明不变。缓存单测只验证单个命名快照的序列化往返，真实改名更新不以参数构造冒充，沿用前文生产probe和真实使用链路证据。

```sh
# 父工作区；harness只执行该pytest并核对本进程真实导入路径。
PYTHONPATH=contracts/src:senior_control/src:cloud/src:senior_control/tests:tests/integration:. \
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  /tmp/task-names-ready-final-2026-11-25/run_cache_test.py
# pytest.main(['senior_control/tests/test_agent.py::test_named_summaries_roundtrip_through_cloud_cache', '-q'])
# 1 passed in 0.25s；exit 0；91个项目/测试模块均来自准确父树。
git diff --check
# exit 0；19/19产品blob与309c707相同；其他测试函数及省字段分支逐字节未变。
```

依赖仍为Python3.11.2、pytest8.4.2、pytest-asyncio0.26.0、pydantic2.14.4、aiosqlite0.22.1；版本实录、单测日志、实际两文件diff、机械比较、准确最终HEAD/远端/PR状态与清洁状态见`/tmp/task-names-ready-final-2026-11-25.md`及同名证据目录。此次不重跑70/42、build、e2e或实车，不启动服务。总负责人已授权在本次范围、单测、产品未变及远端/PR同HEAD全部满足后将原PR40转Ready for review；实际转换结果以交付报告为准，不合main、不新建PR。

### 三尺寸视觉、安全边界与清理

只更新批准的 `/tmp/control-workspace-static-preview` 构建/fixture，复用原 **4147纯静态预览**，没有启动开发/业务服务器。三尺寸保留原控制、小地图/唯一owner、底部容器与间距/触控目标、急停滚动可达、日志安全、设置SI保存、任务编辑/路线重排/cron/timezone、未实现能力零请求检查。**命名创建→改名→命名预览仅1440执行**；390/844仍完成全部原任务编辑/布局流程，不把未运行名称操作写成小屏验证。

browser-final有**13张本版截图**：每尺寸drive/mini-map/estop-scrolled/tasks各4张，另1440一张names-preview。13张均实际调用view_image查看：名称与只读编号分离、桌面“夜班巡检”列表/路线标题；390底部两行与完整触控、844横屏sticky急停及小图工具、桌面底部三组。滚动时的fullPage截图可能把sticky header捕获在滚动位置，estop-scrolled是实际viewport截图并有toBeInViewport断言；不把该截图机制当新布局功能。真实视频在fixture中不可用是预期。

合并后再次确认两子树干净、两HEAD相对父HEAD独有提交均0，检查Orca全部子树会话均在空闲shell/已完成worker提示，没有另行运行的开发/返修任务或明确keep-open要求。使用Orca `worktree rm --delete-branch --json` 正常移除两子worktree（无force）；机制保留了本地分支，随后基于当前父HEAD已包含全部提交以普通 `git branch -d` 删除。前后worktree/sessions清单及两个cleanup收据保存在证据目录；父工作区、原main工作区和main分支均保留。

**未验证**：真实HTTP/WS服务集成、实车、ROS、真实视频/WebRTC/TURN、真实配对兑换。原control.integration.spec.ts保留且默认不连接服务器，此轮只类型检查。缺实际轮廓仍箭头fallback并提示尺寸不可用；FIFO/run-now缺失仍提示暂未实现。需要contracts/cloud与车端协同部署，不能宣传任意版本双向兼容或新数据库可直接降级。集成交付时PR40保持Draft；负责人已完成同版验收，本次授权在前置检查满足后转Ready，最终发布结果见本次交付报告，不能以本地验证代替已推送声明。

## 2026-11-25 批准 DEMO 返修（待负责人亲阅）

本轮从已合并 PR40 的同一 tree 继续，在 `feat/control-workspace` 正常 merge `origin/main=11a26906` 更新祖先。移除 RobotPanel 旧 article、同步/诊断外层；共享 viewport、锚定设备菜单与独立配对 modal；驾驶唯一底部权限动作与大速度；浮动预览、任务 list/pick/editor、地图目录/包操作、状态 CPU 真样本；完整日志及四类设置。已批准追加登录按独立圆角页面接入，单密码与原认证链不变。

新增实质场景（六条连续流程，非样式镜像）：

- V1/V2 空页无默认菜单、外点/Escape焦点、选离线车/取消选择。
- V4/V6 connecting → waiting → snapshot 保持同一 viewport，无旧 article / 同步行。
- V7 preparing → prepared → release 始终只有一个权限动作，不自动发送 intent。
- V15 list → 三点 → 撤销 → editor，同一草稿和橙色顺序。
- V24 校准开始结果未知 → 换分类/日志/设置 → 查询原 start_request_id，仅一次 start。
- 登录显示/隐藏不改密码、不提交；busy防重复；成功清密码并初始化设备。

V1/V2、V4/V6、V7 在 `/dev/shm` 只读 baseline 副本的当前回归入口上分别失败（3 failed），当前实现对应场景通过。原视觉复现、具体断言、失败日志与同版截图索引归档在 `/tmp/control-workspace-visual-repair-delivery-2026-11-25.md`；不能将旧历史验收当成本轮批准。

复用原有效回归：手动控制迟到/新按压/松手/切页/背压/急停；名称 N7/N9/N10 与 Unicode；任务 F1 七恢复输入与 R1/R2；v2/legacy 地图身份/迟到 tile；原生栅格/比例尺；性能同 sequence 时效；日志原文/分页/download；设置 SI、版本冲突/未知同 ID 重试；固定回波校准状态。未修改后端、API、Transport 或 ManualController。

验证：受影响 13 个 Vitest 入口 187 passed；最终地图/任务补验 38 passed；`npm run typecheck` 与 `npm run build` 通过，保留原 Vite chunk-size 提示。三尺寸生产客户端安全 fixture `control.spec.ts` 分别以1440×1000、390×844、844×390执行；最终命令与结果以交付报告为准，浏览器截图不是实车证明。登录/控制/配对/地图/视频均仅安全静态 fixture，非 API fetch 一律拒绝、WebSocket全替换、路由仅静态资源放行。

真实能力差异：摄像头默认关闭且与设置共用 owner；无真实视频帧、无实车认证/配对验证；无校准几何仍箭头/尺寸不可用；roll/pitch/缺样本明确不可用；立即执行/FIFO/缺图层/画质采样写入仍暂未实现且零请求。390为44px热区使用184px dock、图例入口和更高底部避让，小图仍浮于驾驶卡上方；844允许冻结稿 min-height 产生滚动。负责人仍须亲阅实际 diff 和全部视图，本文不替代最终视觉验收。

### PR41 负责人亲阅后五项定向收尾（2026-11-25）

8cf7de6 的视觉交付仍有五项遗漏，旧版本不视为最终验收：设备菜单关闭/刷新相交约6px且配对入口非主色、390任务编辑卡宽315而非340、复制码仍为原生按钮、设置仍称高档/低档、桌面更多图层入口被隐藏。本次只修上述内容：菜单 actions 使用共享 Button/IconButton 的真实槽位，配对用 primary；配对页交互统一共享控件；桌面315px规则限定在461px及以上；设置文案改低速/高速；更多图层恢复可点击并沿原本地提示。

范围为7个现有产品文件、2个现有测试文件及本文，共10路径，新增/删除0，全部在原44路径内。App.tsx 是菜单关闭 owner 和配对 primary variant 的必要依赖，实施前已向负责人报告。Compatible：DeviceList 的 onClose 可选，独立使用不强制出现关闭；code/command/expiry、请求链、协议 key/SI值、任务 owner/调度、原底部三组同底线保持。

本次复用：devices/settings/tasks 共120条原有效回归通过，包含配对取消/过期/认证、配置SI/冲突/未知同ID、原任务恢复与调度；文案定位迁移，没有新增重复业务场景或CSS镜像单测。定向浏览器检查复用三尺寸安全fixture，在原生产客户端流程增加菜单真实bounds无相交及中心命中、配对复制及焦点环、390编辑卡左右12px/宽340及滚动重排/保存命中/可见按钮无相交、三尺寸图层入口命中与原提示/零新增请求；只豁免独立GET /api/robots轮询，所有其他HTTP与socket调用完整比较。

`npm --prefix web run typecheck`、`npm --prefix web run build`通过（原chunk-size警告保留）；`TMPDIR=/dev/shm PLAYWRIGHT_BASE_URL=http://127.0.0.1:4148 PLAYWRIGHT_CHANNEL=chrome npm --prefix web run test:e2e -- control.spec.ts --output=/dev/shm/control-pr41-followup/e2e-results`三尺寸3 passed。新增的按钮相交探针曾把闭合details中的未绘制按钮算入，三尺寸失败；修正为真实checkVisibility后通过，失败日志单独保留，不冒称产品失败。后端不变未重跑，没有实车/ROS/真实配对/业务服务连接；4148仅本地dist静态资源，fetch/WS由安全fixture全部替换。

修复前bounds、最终新原图、命中/请求计数、精确命令日志和小diff见 `/tmp/control-workspace-pr41-followup-2026-11-25/`；旧 `/tmp/control-workspace-visual-repair-2026-11-25/` 原图保持归档，不替换。844保留原min-height滚动，另附实际viewport图避免fullPage的sticky合成位置误读。PR41继续Draft，最终视觉验收仍待负责人亲阅。

### PR41 独立审核追加 R6/R7（2026-11-25，待负责人亲阅）

起点干净 `bc89449c80a1aebcdff7bd1889c814f48be78f47`，只改 Map.module.css、Drive.module.css、原 control.spec.ts 和本文4个已有路径，新增/删除0；与前五项合计9产品/2测试/1文档共12路径。MapCanvas/EmergencyStop/协议/资格/pending/控制状态机均未改，前五项修复保留。

R6：删除手机20px硬上限；线段显式 border-box（标称长度包含两侧各1px边框）、flex-shrink:0、min-width:0，避免另一个最小宽度改写真实线长。手机现有两列工具中，比例尺在原跨列槽位内上下排线段/标注，不改变三组底边和44px工具。离线原失败探针同输入zoom50/1m：390实际20→50px，1440仍50px。安全fixture390连续缩放：1m×55.393342=55.393342，inline55.3933、rect55.390625；随后0.5m×69.241678=34.620839，inline34.6208、rect34.609375。两次工具组右界115、dock左界133，无覆盖；原比例尺算法、真实坐标/resolution未动。

R7：恢复实际消费者ghost的橙色空心/9px圆角/44px热区、disabled和focus-visible；主急停红色不变。恢复态390占页头独立行，紧凑横屏单列并避让原退出入口。离线1440按钮高25.5→44px、radius0→9px；实际1440/390/844两个恢复按钮均98.015625×44px，透明底/橙边、2px键盘焦点，急停64×44且中心命中，所有可见页头按钮无相交。许可→禁止复位连续状态下，disabled opacity0.45、高44px，恢复请求数0→0；没有操作真实急停/恢复。

Bug回归：原control.spec.ts增量比较米数×data-zoom/inline/rect、dock边界、恢复按钮样式/热区/键盘焦点、可见页头按钮相交和禁用零请求。bc89449生产bundle的390新增几何断言失败（inline55.3933实际20）；修复后通过。第一次CSS验证发现844恢复按钮与退出真实相交，已在同Drive CSS内修正，失败日志保留。早期4148连接拒绝和4147旧8cf菜单失败不算R6/R7红绿。

复用回归：`npm --prefix web test -- --run src/features/drive/drive.test.tsx src/features/map/map.test.tsx src/api/vehicleState.test.ts`，3 files/49 passed；原急停/手动控制phase、资格及地图身份等保护保留。`npm --prefix web run typecheck`、`npm --prefix web run build`通过（原chunk-size警告保留）；E2E文件另用临时tsconfig严格类型检查通过。`TMPDIR=/dev/shm PLAYWRIGHT_BASE_URL=http://127.0.0.1:4148 PLAYWRIGHT_CHANNEL=chrome npm --prefix web run test:e2e -- control.spec.ts --output=/dev/shm/control-pr41-r6r7-delivery`，三尺寸3 passed；跨尺寸CSS影响恢复态布局，因此复用这一既有矩阵，无新增矩阵或后端重跑。

4147旧归档和前五项26张图均不覆盖；4148仅web/dist静态预览，fetch/WS安全fixture截断，未连接业务/实车/ROS/真实认证配对视频。新增8张实际viewport截图（390比例尺缩放前后；三尺寸许可恢复/禁止复位），34张统一原图索引和每命令/版本/数值见 `/tmp/control-workspace-pr41-scale-recovery-delivery-2026-11-25.md` 及 `/tmp/control-workspace-pr41-scale-recovery-2026-11-25/evidence-index.html`。PR41保持Draft，不代行负责人最终验收。
