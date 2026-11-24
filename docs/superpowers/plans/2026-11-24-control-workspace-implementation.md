# WHEEL 控制工作区实施计划

已批准实施；此前只读限制由本次派发替代。使用现有隔离worktree，固定分支 feat/control-workspace，基线00dc3be5727b203bd4bbe6b7ed72e83226a7be71。

Spec: docs/superpowers/specs/2026-11-24-control-workspace-design.md

## Global Constraints
单人顺序开发，不派发worker；RobotPanel连接、ControlSurface地图/操作、ManualController运动、VideoPanel媒体owner沿用。真实接口和资格不弱化；不启动车辆/ROS/业务服务，不发运动，只安全fixture静态预览。未知能力只提示暂未实现。图表无伪历史，单位通信SI。几何待负责人确认契约。

## Review Focus
切车/退出/失焦释放；预览交换不重建地图和驾驶owner；地图栅格内容版本与参考代次分离；定时开关不取消实例；部分航点确认保留；样本过期不填零；手机所有入口可触控。

## Task 1: 公共控件与页面壳
核验资产，复现canonical pairing CLI和grid identity bug；锁定lucide-react/echarts。实现少量受控公共控件、空页、设备/配对、顶部控制/日志/设置及单位状态入口。保留认证和解除配对竞态。验证：相关Vitest、typecheck、build；Expected:全部通过，原有保护不删除。提交批次1。

## Task 2: 驾驶、状态与预览
单杆双轴迟滞吸轴输入原ManualController；低速/高速、控制权实心/描边。速度单位共享上下文仅展示/编辑换算；草稿以SI保持。状态yaw有值才显示，roll/pitch不可用；ECharts只接真实新鲜样本，超时断点。唯一地图/媒体布局交换，日志/设置退场释放。验证：drive/state/settings/video相关测试、typecheck/build。提交批次2。

## Task 3: 地图与任务
原坐标变换与身份保护保留；free/unknown/occupied主题、真实比例尺与居中。底左工具/中四模式/右图例同底边，手机采用可触控布局。任务点草稿排序、有序保存和部分失败保留；复杂cron/timezone保留，task_id创建后不改；真实执行动作沿VehicleStateView；缺立即执行/队列只提示。几何确认后最小跨层贯通并跑对应Python回归。验证：map/tasks/mapping相关测试及类型构建。提交批次3。

## Task 4: 日志设置和交付
逐行安全日志、完整十字段设置、冲突恢复保留；浏览器1440/390安全fixture静态产物验证。运行受影响前端测试/typecheck/build，审实际diff，记录已接入/暂未实现/状态不可用/未验证和每项验收证据。推功能分支、创建PR，未闭环为Draft，不合main。

## 开发前路径清单
以下为精确预算，按实际复用收敛，删除0；不为凑数改路径，扩张另报负责人。

### 产品/资源：基准新增14、修改30、删除0，共44

新增14：
1–6 `web/src/components/ui/{Button,IconButton,Switch,SegmentedControl,FloatingCard}.tsx` 与 `Ui.module.css`；
7 `web/src/features/devices/useWorkspaceView.ts`；
8–9 `devices/StatusOverlay.tsx`、`StatusOverlay.module.css`；
10 `devices/charts.ts`（ECharts按需注册/主题，不放业务数据owner）；
11 `drive/DriveJoystick.tsx`（样式沿Drive.module.css）；
12 `web/src/display/speedUnits.ts`（小纯转换，仅确有跨设置/状态复用）；
13–14 `web/src/assets/concrete.jpg`、`vehicle-marker.svg`（接收资产，不新生成）。

修改30：
- 全局3：`App.tsx`、`App.module.css`、`theme.css`。
- devices11：`RobotPanel.tsx`、`DeviceList.tsx`、`PairingPanel.tsx`、`Devices.module.css`、`ControlSurface.tsx`、`OperatorWorkspace.module.css`、`VehicleTools.tsx`、`VehicleStateView.tsx`、`VehicleStateView.module.css`、`PerformancePanel.tsx`、`PerformancePanel.module.css`。
- drive3：`DriveView.tsx`、`Drive.module.css`、`EmergencyStop.tsx`。
- video2：`VideoPanel.tsx`、`Video.module.css`。
- map2：`MapCanvas.tsx`、`Map.module.css`。
- mapping2：`MapManager.tsx`、`Mapping.module.css`（保留MapCatalogController逻辑）。
- tasks3：`TaskList.tsx`、`Tasks.module.css`、`ScheduleEditor.tsx`（保留ScheduleFields/schedule算法）。
- logs2：`VehicleLogs.tsx`、`Logs.module.css`。
- settings2：`VehicleSettings.tsx`、`Settings.module.css`。

基准不修改 ManualController、useWorkspaceMap、PosePicker、HomeControls、ImportDialog、MapCatalogController/MapCatalog、api/client、demo、main.tsx；若接线必须改则逐项说明，而不是草案先列一串必改。地图重排通过MapCanvas布局插槽/props与ControlSurface即可保留原变换/数据owner，不为了三组工具单独造第二个MapCanvas。

**车辆几何接线附加范围：产品修改估7–10、配置0–1、测试3–4**，取决于后续确定的真实config owner读取点：候选明确为 `vehicle_core/config.py`（若无需改模型只读即可不算）、`ros2/src/senior_robot/senior_robot/agent_adapter.py`（state_provider输出；实际配置加载位置仍需在实施前确定，最多再1–2个assembly文件）、`senior_control/src/vehicle_manager/agent/{local_state.py,ipc_state.py,state_ports.py,snapshot_state.py,snapshots.py}`（已核对 state.get → IpcState.read → RuntimeState/StateReading → SnapshotState.collect → LiveState/RobotSnapshot，通常这5个文件）、`contracts/src/senior_contracts/wire.py`、`web/src/api/protocol.ts`，MapCanvas/RobotPanel已经算入基准不重复计。不能宣称这块完全前端可做；该跨层清单是依赖发现后的范围估算，尚未完整设计新元数据路由，不伪称精确到最终源点。总产品44→约51–54，新增仍14、修改约37–40、删除0。若没有后续允许改该链路，保持中心箭头并把物理外轮廓列未满足项。

### 测试：基准新增2、修改11、删除0，共13；按几何接线附加3–4

新增 `web/src/components/ui/ui.test.tsx`、`web/src/features/devices/workspace.test.tsx`（集中新布局/浮层/切页/单位，不重写原功能矩阵）。修改现有 `devices/devices.test.tsx`、`devices/vehicleState.test.tsx`、`devices/PerformancePanel.test.tsx`、`drive/drive.test.tsx`、`map/map.test.tsx`、`mapping/mapping.test.tsx`、`tasks/tasks.test.tsx`、`logs/logs.test.tsx`、`settings/settings.test.tsx`、`video/video.test.tsx`、`web/e2e/control.spec.ts`。几何额外候选 `web/src/api/client.test.ts`、`senior_control/tests/test_vehicle_state.py`、`tests/unit/test_protocol.py` 与 `tests/ros/test_agent_adapter_shutdown.py`（仅若影响适配器生命周期），实际接口确定后选受影响文件；不为了“测试文件数”创建无行为价值测试。schedule.test.ts没有算法修改时只复用，不必算改。

### 配置构建：新增0、修改2、删除0，共2

`web/package.json`、`web/package-lock.json` 引入 lucide-react/echarts；当前React19/Vite/TS已具备，无需新测试框架、无必要改vite/tsconfig/Playwright启动服务配置。无前端lint script，后续不可假称 npm lint 已通过；按项目现有类型与构建检查。

### 工具文档：新增3、修改1、删除0，共4

新增 `docs/superpowers/specs/2026-11-24-control-workspace-design.md`、`docs/superpowers/plans/2026-11-24-control-workspace-implementation.md`、`docs/testing/2026-11-24-control-workspace-validation.md`；修改 `README.md` 的使用/能力/缺项说明；不预设新增工具脚本。此诊断报告位于/tmp，不计入仓库变更。

**总基准63文件（新增19、修改44、删除0）**；车辆几何额外约10–15个后约73–78。草案36–59偏低，主要少算多个现有CSS Modules、真实性能/校准保留、受影响行为测试与跨层几何接线。若精简公共控件/视觉差异收敛，可降低，但应由实际复用证明，不承诺虚低数量。


## 几何最终决定

已批准可选只读vehicle_footprint语义，但当前没有显式校准生产者。负责人已否决新增geometry配置路径，并确认本轮箭头fallback，不添加无来源协议字段。以下记录实际同源加载核对证据；几何尺寸本轮为数据未配置/未接入。

### 几何源核对结果（按负责人答复纠正）
已核对 `platform_runtime._adapter_command` 先调用 `validate_real_runtime(profile.motion_profile, profile.safety_parameters)`，然后以现有 `--params-file profile.safety_parameters` 启动 adapter；`real_runtime.load_real_parameters` 合并 safety_gate.ros__parameters 与 `real_parameter_defaults(profile)`。仓库当前 `config/hardware-safety.yaml` safety_gate 显式 body_frame=base_link，但没有 footprint_xy；默认 footprint_xy 是从 HardwareMotionProfile.length/width 派生的矩形。故此版本没有可声称“包含外凸四轮”的真实显式轮廓，不贯通这个默认矩形、不新增几何owner，当前只画中心箭头并提示尺寸不可用。外部部署若有显式校准footprint，后续可从同一已验证safety_parameters值传给可选vehicle_footprint_xy启动参数再贯通已批准契约；当前不修改安全/导航/启动行为，几何验收第8项仍待实际配置证据。

### 安全浏览器fixture范围补充
增加 `web/e2e/fixtures/control-fixture.js`（测试专用，在生产构建外），以原HTTP/WebSocket协议向未修改的生产客户端提供安全fixture；无真实socket和运动能力。原e2e/control.spec.ts改用此fixture，不依赖旧DEMO服务。静态预览仅装载构建产物与测试fixture，截图不当实车证明。

浏览器配置补充：web/playwright.config.ts允许可选PLAYWRIGHT_CHANNEL选择已安装Chrome；默认保持原Chromium。Orca截图返回整幅灰色，保留Orca交互/DOM证据并用Playwright像素截图交叉验证；不将灰色截图视为视觉通过。

## 最终完成记录

产品/测试版本 `5cb89a303c22271fd1ab3f184ead49d82af08466`；195项前端回归、typecheck/build及1440/390触控静态fixture浏览器流程通过。实际55路径（产品37/测试fixture11/依赖构建3/文档4），无删除、无后端算法/协议改动。完整精确清单、功能四分类、断言迁移、RED→GREEN、环境限制和同版证据见 docs/testing/2026-11-24-control-workspace-validation.md；不再按早期估计凑文件数。保留分支/worktree，推送后创建Draft PR等待负责人亲阅，不自动合main。

最终新鲜度补充：修改现有useVehicleView.ts，新增共享useMeasuredSpeed显示适配，以原relay5000ms期限/原received_at_ms隔离重复runtime sequence；驾驶卡片与状态浮层复用，不改变ManualController或控制连接权限。该路径相对预估额外修改1处，实际总55。
