# 控制工作区展示层与回归

PR41 本轮按最终冻结 DEMO 重排展示；此前“最少适配”和统一 44px 可见按钮尺寸的结论不再作为验收依据。冻结 `control-demo/index.html` 的 SHA256 为 `d84d64de8b806224756f3d326ca620cbe29e0fe856eb8125f2cb4469de05b754`，登录沿已批准版本保留。

## 展示与业务边界

- App/Shell 管理顶栏、选择车辆、场景内配对与空页；横屏页头随文档滚动。设备菜单只能解除当前选中车辆的配对，保留凭据失效确认、失败重试和重复请求保护。
- ControlSurface 保持唯一地图、视频和驾驶 owner；浮卡、主副画面和底部三组工具共用场景边距。切页/切车/失焦释放仍由现有业务控制器处理。
- 导航默认按状态说明、地图选目标、返回 Home/取消、校准/编辑 Home 排列；地图选点使用独立 banner，先点位置、再点朝向才出现确认；原有拖动和精确坐标入口仍可用。精确坐标、当前位姿设 Home、刷新与诊断在次级详情中。
- 任务列表按名称/开关、真实计划与点数、实例状态、执行/编辑/查看点位排列。地图选点后进入独立编辑卡；默认名称、时间、启用、有序点、取消/保存，标题“高级”入口承载复杂 cron/interval/timezone、只读 UUID 和已有航点操作，不改动未编辑的原计划。
- 地图管理按当前、已保存、新建/导入组织；新建名称和包名操作由对应对话框承载。状态浮层按速度、真实趋势、姿态图、CPU 与性能分类组织，Roll/Pitch 缺失明确显示不可用，Yaw 来自真实遥测。
- 日志保留原文、服务端过滤、游标、跟随和下载；设置分类保留六个驾驶与四个日志字段及版本/unknown 请求/草稿 SI。校准 owner 跨页保持挂载。

未实现的立即执行/FIFO/独立候选与空闲证据/拟合图层/画质与采样写入仍只反馈“暂未实现”，不发送相应请求。真实地图、视频和测量不会替换成 DEMO 房间或意图值；未配置车体几何时仍画箭头，原因位于地图详情。

## Tailwind 归属

Tailwind 4.3.5 使用官方 Vite 插件；`src/theme.css` 是唯一自有样式入口，负责 import、共享 theme tokens 和基础规则。15 份 CSS Modules 及其 import 已移除，展示类名与有限 variants 在各自 TSX 中完整声明，未新增 `@apply` 页面样式表。

运行时数值仍使用局部 inline 样式：摇杆位置、检测框、比例尺宽度、实际驾驶卡高度、航向和 CPU 条。Canvas/SVG/ECharts 的数据和图表配置仍由原绘制层负责。

`src/ids.ts` 优先使用原生 `crypto.randomUUID()`；LAN HTTP 缺少该方法时，用 `crypto.getRandomValues()` 生成 RFC 4122 UUIDv4。所有身份调用点沿原时机生成，重试继续复用原身份，未使用 `Math.random` 或改变协议载荷。

## 验证方式

```sh
npm --prefix web ci
npm --prefix web run typecheck
npm --prefix web test -- --run
npm --prefix web run build
PLAYWRIGHT_BASE_URL=http://<静态预览地址> PLAYWRIGHT_CHANNEL=chrome \
  npm --prefix web exec -- playwright test e2e/control.spec.ts --config web/playwright.config.ts
```

浏览回归只加载生产构建和 `e2e/fixtures/control-fixture.js`，所有 fetch/WS 被截断；静态路由以外网络被阻止，不启动后端或接实车。三个既有参数输入为 1440×1000、390×844、844×390；同尺寸检查实际点击、单 owner、选点入口、任务编辑、配置 SI、日志原文、急停/恢复禁用和比例尺数学。

新增两条 ID helper 单测分别覆盖原生路径及无 randomUUID 的加密 UUIDv4，另有两个独立输入回归分别覆盖鼠标两次点击与键盘第二次 Enter 保持首选位置并设置朝向；复用的配对单测改为仅当前选中车辆删除，同时保留过期刷新、失败、重复与会话过期保护。原 CSS 源码字符串测试因 Modules 删除及 44px 规则撤回而删除，其有效行为由实际浏览器点击/禁用/键盘/不重叠断言继续覆盖。

本轮实际偏差的定向浏览检查嵌入原三个流程：导航默认动作顺序与选点隐藏/取消、任务默认动作与选点/独立编辑、地图分组与新建对话框、配对打开时场景文档坐标不变。横屏比较固定 scrollY=0/170/底部，不能将 viewport 图与 fullPage 图混用；页头不新增 sticky，急停回到页头可达，持续浮动急停与冻结稿的取舍交负责人决定。

最终原图、每页真实差异、版本/命令结果和保留的静态预览进程见 `/tmp/control-workspace-strict-design-delivery-2026-11-25.md`。实施交付不代表负责人已完成视觉验收；PR41 保持 Draft。
