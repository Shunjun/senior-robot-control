# 批准控制端 DEMO 返修执行计划

依据：2026-11-25 只读审计及最终冻结 DEMO，用户已授权返修。单 owner 顺序实施，不重新设计。

1. 共享控件及 App / RobotPanel 单壳；退出旧 article、同步行与诊断外壳。
2. 设备锚定菜单、独立配对 modal、稳定连接及等待 viewport。
3. 驾驶卡层级、浮层尺寸、唯一地图/视频；导航、任务阶段、地图管理、状态内部结构。
4. 完整日志页及分类设置；摄像头共用 owner、诊断保持 mount 和请求身份。
5. 受影响原回归和定向 bug 回归；生产构建安全 fixture 三尺寸同版截图。
6. 审 diff、正常推原功能分支及创建后续 Draft PR；负责人亲阅验收。

范围沿审计第5节明确文件，不修改后端、控制状态机、依赖或构建配置。共享壳是后续串行前置；日志和设置理论可独立，但本 dispatch 不委派。

Ruling: 摄像头生产初始仍关闭；冻结设计未要求自动启动真实媒体，用户选择在控制/设置间共用并保留。
Ruling: 配对和诊断继续保留原请求 owner，展示迁移不创建模拟状态机。

Git 核对：远端 main=11a26906c9831b000b09e2279ad7ef4ea514d903，tree 与 9d5899d6 相同；已正常 merge main 更新祖先，无产品差异，无已有 open 后续 PR。

## 已批准登录追加（2026-11-25）

用户查看 `/tmp/control-workspace-login-design-proposal-2026-11-25.md` 及三尺寸独立交互预览后回复“可以 就这样吧”。沿相同 concrete、WHEEL 圆环、410px深色圆角卡、单密码输入和橙色提交接入正式 Login；显示/隐藏仅改变输入 type。认证仍沿原 submit → api.login → 清密码 → onLoggedIn，保留 busy 与错误分流。

1. 修改 Login.tsx，新增 Login.module.css；使用共享 Button/IconButton 与 Lucide Eye/EyeOff；移除无消费者的旧登录 CSS。
2. 原 devices 测试补显示/隐藏不提交和 busy单提交连续流程；复用既有认证回归。
3. 同版 build / typecheck / Vitest，静态安全认证 fixture 下做1440/390/844批准/正式对照；不接真实认证。设计预览模拟选择器不进入产品，最终清理临时预览。

Ruling: 390小图仅127×90，无法容纳44px触控工具和交换按钮；唯一地图工具继续放同一底部工具组，主次交换不会重建地图，所有工具仍控制同一个地图。这是触控约束所需的最小适配。

## 执行记录

共享壳/控件、设备与等待、控制场景卡片、日志/设置、批准登录追加均已按同一owner顺序实施。受影响13个Vitest入口187通过；地图/任务末次38通过，日志/UI末次5通过；最终typecheck/build通过，三尺寸Chrome生产客户端e2e分别通过。V1/V2、V4/V6、V7在原main只读副本上3条定向失败，当前通过；V24跨页原校准请求身份另有实质连续测试。

最终证据与明确的能力差异见 `/tmp/control-workspace-visual-repair-delivery-2026-11-25.md`，同版截图/成对索引位于 `/tmp/control-workspace-visual-repair-2026-11-25/`。本记录仅表示实施者交付；负责人亲阅diff和全部视图的最终视觉验收尚未代行。
