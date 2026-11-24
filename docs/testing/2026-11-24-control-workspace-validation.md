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
