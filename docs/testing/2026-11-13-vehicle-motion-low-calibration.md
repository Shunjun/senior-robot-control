# T1 隔离低速校准与 r3 原因取证

状态：T1 子阶段；所有 measured_bounds 仍未冻结。仅审查固定 +0.3 m/s 计划，不把该速度称为低档 2 km/h 或高档 5 km/h 验收；本轮没有修改产品控制、gate/cap、旧 ROS 测试，没有提交/推送。

## 身份与隔离

- VM：原 Parallels Ubuntu22.04 ARM64，toby；新根 `/home/toby/senior-audit-20261031/calibration-low-20261112-r1`，原 r3 源码/安装/profile/证据保持不变。
- 实际执行源码为 r3 frozen source/install；校准器新修订单独复制进每次 repeat 目录并逐字节 SHA 验证。Python 为 `motion-20261112-r3/.venv/bin/python`，ROS 为 `/opt/ros/humble`；各次 `run.json.environment` 记录实际 PYTHONPATH/LD_LIBRARY_PATH。没有加载宿主或旧 checkout 的产品模块。
- 已审 fixture 根 `motion-20261112-r3/calibration-fixture-reviewed`；manifest SHA `daf50ffeb744ed489334b084b500fc171bda3f5dfa0239bcaa908449593517a6`，运行前逐项验证 manifest.files；固定 plan SHA `ac219e8245d9f64709b9adeb1974058bab5301114ddb383489e5c7daf38aba7f`。
- 独占 `.vm-test.lock`；ROS173、LOCALHOST_ONLY=1、IGN/GZ partition=`vehicle_motion_calibration_review1`。无 Agent、云、Nav2、真实车连接。每次独占 systemd user unit RuntimeMaxSec175/TimeoutStopSec5/KillMode=control-group，calibrator 内部8秒+50ms异常零输出交付、外部 timeout10/kill-after1；finally 仅结束本次进程组。世界从原点静止重新启动，准备好后才启动校准器时限。
- 原 fixture 文件保持与已审 manifest 一致；直接运行其 launch，将 ROS_HOME/ROS_LOG_DIR 指向新 run。首次启动缺 `gz_ros2_control-system` 是直接 ign launch 未复制 ROS 官方 gz_sim helper 的插件搜索环境；在外部补 `IGN_GAZEBO_SYSTEM_PLUGIN_PATH/GZ_SIM_SYSTEM_PLUGIN_PATH=$LD_LIBRARY_PATH`，库本身已安装，没有改 xacro 或控制参数。新 generator start.sh 固化该适配（`//Compatible`）。地面/轮 mu=1 只是假定仿真地面，质量/惯量来自 senior_4wd_dl 当前模型。

## 本轮工具 diff 与负对照

1. 对现场 `graph-probe-2/graph.json` 取证：spawner/create 均已退出；固定 `vehicle_motion_recorder_review1` 只发布 `/rosout`、`/events/write_split`，只订阅所录传感器、参数事件及 dedicated cmd。`gz_ros2_control` 是驱动插件的参数/日志元数据节点。两者各自以固定名称、根命名空间和端点集合证明角色，其他名字、namespace、额外 topic 或 actuator publisher 均拒绝；drive 仍唯一 actuator subscriber，重复 drive/额外串口订阅拒绝。TF helper 原受限角色继续保留。
2. `ign topic -l` wrapper 在刺激 publisher 创建前超过2秒，无运动命令；使用现场 help 确认的 Fortress transport11 list-only binary 直接调用；静止独立探测确认 list 固定发现等待实际2.014–2.016秒，-d .1 对 list 无效，因此固定2.5秒只读发现预算，整体8秒 alarm 和未知世界拒绝不变。串行发现曾使首次 regular 延迟4.14秒、剩余轨迹在8秒 alarm 时异常零输出；现以一个固定线程并行进行独立 ROS/transport 准备，parameter client 也提前参与发现，不扩大任何 service/年龄/运动检查阈值。正常/foreign-world 绑定测试验证观测能同时推进，失败销毁 client/context/executor 且不创建 actuator。阻塞发现之后用一个有界 tick 处理已排队观测，再执行原 source-age/原点静止检查，未放宽任何年龄或预算。
3. 命令 JSONL 新增 emitted steady、wall ns、最近观测 ROS clock/receipt steady，用于追踪时钟映射；无 header Twist 的 bag receipt ROS 时间仍不等于 actuator 应用时间，不把二者误差当零。
4. 新 fixture 参数仅用于下批静态材料：显式 hard_seconds、直线走廊长度或径向墙距；按工作上限×声明线速+3m 检查标称留空空间，保留原模型并扩大地面。直线走廊有8m范围内非对称地标，转弯不能误用该走廊。matcher 搜索包络仅在生成的校准 YAML 随声明速度扩展，产品文件不变；这不表示高档 scan 已可用。

capture 的标准产品流缺 IPC、motion、safety、`/cmd_vel` 是这个专用图的预期边界，仍返回2并逐项标缺。dedicated cmd、joint_states、wheel、truth、clock、scan_motion、raw scan、IMU 由原生 bag 记录；capture.complete 只表示各标准必需流至少一条，始终不等于完整窗口或物理停车。

## r3 manual 失败的精确来源

r3 manual ROS 测试硬编码 `robot_id=route-sim`，与私有副本配置身份不一致（仅保存实际 ID SHA `c0ca17775a8a9f730f0e98023a9327bf7d93aadbf0590122a63ea4a57106d9c7`）。`AdapterCore.__init__` 因存在 server_heartbeat hook 初始 motion_disabled=True；heartbeat 只记录 epoch，随后 control.acquire 走 `_reenable → validate_envelope(raw, configured_id)`，后者抛 `wrong robot`，被映射成 motion_disabled，尚未调用 Arbiter 的 submit。离线按原 envelope 重放得到同一错误；仅在纯 AdapterCore、accepted-submit stub 中换成匹配 ID，返回 accepted 且 motion_disabled=False，不能据此声称真实 Arbiter 或 manual 验收通过。

准确路径：`tests/ros/test_simulation_manual.py` → `ros2/src/senior_robot/senior_robot/agent_adapter.py` 的 handle/_reenable → `src/senior_control/protocol.py:validate_envelope`。这修正上一报告“epoch/lease 待定位”的推断，是测试身份配置问题的直接证据；未重跑整场或修改旧测试。

## r3 navigation 第一次 motion_anomaly

时间统一为原 bag 消息源 ROS 秒；完整结构化 snapshot、原 epoch、同区间匹配方法保存在 `offline/r3-cause-timeline.json`。

| ROS 时刻/区间 | 直接证据 |
| --- | --- |
| 5.0 前 | snapshot recent_intervals 中命令/轮/scan 静止；随后开始正向加右转 |
| 5.8→5.9 | 首条 `/motion/evidence.reason=motion_anomaly`；scan 仍 valid/qualified，并非缺 scan/scan timeout |
| 同区间命令 | 平均 vx=.0966 m/s、wz=-.276 rad/s |
| 同区间轮 | Δx=.00793252m，Δyaw=-.02266667rad；SE2 平均 vx=.07933256m/s、wz=-.22666667rad/s |
| 同区间真值 | Δx=.00293681m，Δyaw=-.00130549rad；平均 vx=.02936845m/s、wz=-.01305493rad/s |
| 同区间 IMU | trapezoid Δyaw=-.00129417rad，接近真值而非轮航向 |
| 同区间 scan | Δx=Δy=Δyaw=0，quality=.9405、非退化；并不代表真实车身完全没动 |
| 首次 latch 累计 | turn_elapsed=.9s > startup .6；turn_command=.12615rad >= yaw_limit .12；turn_body=.01 < .5×turn_command；causes 只有 turn_stall |

调用路径：`motion_fusion._process_pending` 按 scan 区间积分 command → `motion_runtime.qualify_interval` 对齐 wheel/IMU/scan → `SlipMonitor.update` 的累计 turn_stall 分支 → 首次 latch/stop acknowledgement → `_log_first_latch` 和 motion_anomaly 证据；后续 safety/reference 资格消失、RouteService 的 qualified 循环退出并撤销 source。首条 latch 与 wheel/GT/IMU 差异是直接证据；由此断言某个轮胎摩擦参数或 scan 匹配算法单独导致全部导航失败仍是推断。当前 matcher 使用 .02m 栅格/.02rad 角搜索步，单区间真值只有毫米/毫弧度，零 delta 的量化影响需独立验证，不能把高 quality 当准确度证明。轮转动明显大于真值，模型接触/四轮滑移/执行器响应亦须分开取证；不调低 gate 造绿。

## 下批材料（仅生成，禁止本轮执行）

仓库具体 plan：`tests/fixtures/vehicle_motion_calibration/next/{low_forward,high_forward,reverse,turn}.json`；原 review1 plan 字节不改。每次从原点静止、无正反跨越，a=.3、yaw a=.5 都只是刺激命令 ramp，不是测得最大加减速。

| plan | 目标 (m/s,rad/s) | 零/目标/零段(s) | work上限 | 标称上限位移 work×maxv | 几何 | max wheel rad/s |
| --- | --- | --- | --- | --- | --- | --- |
| low_forward | 5/9,0 | 2/6/4=12 | 16s | 8.889m | 直线端墙±20m，地面60m | 7.310 |
| high_forward | 25/18,0 | 2/7/6=15 | 20s | 27.778m | 端墙±40m，地面100m | 18.275 |
| reverse | -.3,0 | 2/3/3=8 | 12s | 3.6m | 端墙±20m，地面60m | 3.947 |
| turn | .25,.5 | 2/4/3=9 | 13s | 3.25m | 径向墙7m，地面50m | 4.872 |

标称位移只假设实际不超过声明线速，未包含未知物理滑行；3m余量不是实测停距。wheel 计算 `(abs(v)+.481154/2*abs(w))/.076`，均小于模型声明20，不擅自提高轮限。直线 y=4.5/-4.7m 的错位地标旨在避免纯平行墙的纵向不可观测；turn 的7m墙位于8m量测范围边缘，实际沿弧线可观测性、扫掠/惯性与停止距离仍未验证。全用同一 mu=1 假定，不能迁移真车。

当前宿主可逐文件审查的静态产物为 `/private/tmp/vehicle-motion-next-20261113-review2/{low_forward,high_forward,reverse,turn}`；4份 launch 的 AST parse 通过，generator SHA `1c5440b605efdbc9e8ad868923f03d2883240f6472d69c6415b8ff676cdb2e40`。这些路径绑定宿主，后续 Linux 使用新冻结源码重新生成并审实际 manifest，不能直接拿宿主启动文件在 VM 执行。

静态生成命令（Linux 冻结根上执行；新 output 不得存在）：

```bash
python -m tools.vehicle_motion_fixture --source-root "$SOURCE" --plan "$SOURCE/tests/fixtures/vehicle_motion_calibration/next/low_forward.json" --output "$RUN/low-fixture" --hard-seconds 16 --corridor-half-length-m 20
python -m tools.vehicle_motion_fixture --source-root "$SOURCE" --plan "$SOURCE/tests/fixtures/vehicle_motion_calibration/next/high_forward.json" --output "$RUN/high-fixture" --hard-seconds 20 --corridor-half-length-m 40
python -m tools.vehicle_motion_fixture --source-root "$SOURCE" --plan "$SOURCE/tests/fixtures/vehicle_motion_calibration/next/reverse.json" --output "$RUN/reverse-fixture" --hard-seconds 12 --corridor-half-length-m 20
python -m tools.vehicle_motion_fixture --source-root "$SOURCE" --plan "$SOURCE/tests/fixtures/vehicle_motion_calibration/next/turn.json" --output "$RUN/turn-fixture" --hard-seconds 13 --landmark-radius-m 7
```

后续运行必须先亲审 Linux 实际生成物/manifest、计划SHA、地标/路径包络、低速响应结论，再独占 lock/unit、核查空域、启动bag/capture并从原点静止；每个新 work 上限对应独立外部 watchdog，finally 回收该 cgroup。倒车 .3 是响应取证，不覆盖倒车配置上限全域；turn .5 不覆盖默认1rad/s组合。低速若揭示执行/估计异常，先实施最小可证修复并独立对照，再考虑这些计划，不直接以更多资格门槛掩盖。

## 两次完整低速结果

成功对照是 **repeat6 / repeat7**，每次全新世界原点静止；calibrator=0、bag=0、标准 capture=2（预期缺流），各100条 regular + 1条 cleanup 命令，原生 bag 均101条 dedicated cmd，最大+.3、wz始终0、没有反向输出。实际校准器 SHA `79fc81ffa470670158d3e86fd7e12ba7bd989a58889eaeaa3068b5e2837d4c9b` 与当前源码一致。

| 指标 | repeat6 | repeat7 |
| --- | --- | --- |
| 启动至 finally 完成 | 27.785s | 28.418s |
| plan记录至cleanup（含准备） | 7.546s | 7.875s |
| 首条regular前准备 | 2.351s | 2.345s |
| 第一至最后regular steady跨度 | 5.144s | 5.480s |
| 最大 regular steady 间隔 | .104s | .123s |
| 稳态同窗 truth/wheel vx | .300/.300m/s（ROS9.634→10.273） | .300/.300m/s（8.774→9.418） |
| 实际joint速度峰值 | 3.94737rad/s | 3.94737rad/s |
| 首个正命令 native ROS | 8.466s | 7.731s |
| truth 首次 ≥.01m/s | 8.480s | 7.760s |
| wheel 首次 ≥.01m/s | 8.530s | 7.810s |
| 正常减速段首个目标（ROS clock映射） | 10.322s | 9.462s |
| 减速开始至随后静止位置 Δx | .137218m | .098668m |
| 首个regular零命令 native ROS | 11.226s | 10.131s |
| 该零命令时 truth/wheel 插值速度 | .010656/.023700m/s | .015000/.030150m/s |
| 后续首个 truth ≤.001m/s 且保持≥.5s 的采样时刻 | 11.240s | 10.160s |
| 零命令以后至最终静止位置 Δx | .00010864m | .00014020m |
| 零命令+.5s之后 truth 样本 | 366条，速度0 | 495条，速度0 |

以上 stop crossing 的 .01/.005/.001m/s、.5s 仅是离线描述曲线的几个截面，全部得到相同停止采样点；没有据此改 acceptance YAML 或设合格门。ground truth 为50Hz，零时刻插值不是精确积分的物理停车时刻；Twist 无 header，bag 时刻是 recorder 最近 ROS clock 的接收标记，也不是驱动应用时刻。可直接说正向响应存在、普通 ramp 减速后停下，不能把14/29ms的接收→采样 crossing 称为已测执行器最大 lag。wheel 的 crossing 更晚，也不能反推车身未动；本轮未读取 diff_drive 实际 rolling-window 参数，平滑导致迟滞仅是待核假设。

两次真值 `.2s` 窗口 Δspeed/ΔROS-time 范围分别约[-.375,+.375]、[-.5224,+.4500]m/s²；这是该有限刺激/调度/仿真时间倍率下的观测斜率，并非测得最大加速/最保守减速能力。命令 `.3` ramp 属于 steady 时间，两种时钟不能混用；第二次工作距8秒只有约.125秒余量，不能声称所有负载下均能正常执行。普通结束的 regular 零速已有约1段保持后才 cleanup，物理停车结论来自 truth 的后续样本，不来自零输出。

### 激光与采样证据

- repeat6 原生 scan-motion 155/155 valid，单 producer epoch；repeat7 173/175 valid，ROS11.5s=`raw_scan_invalid`、11.6s=`initializing`，切换 producer epoch 后恢复，均在运动已停止之后。原始 scan11.5仍存在；输入包含正常的无回波 inf，不足以证明该帧几何无效。
- 具体失效传播可核对 `scan_adapter.on_scan` 的 admission.offer 失败/代际变化 → `_invalid(capture.reason or raw_scan_invalid)` → scan receipt 的 owner invalid → `scan_motion_node` 转发 owner.reason 并重置 epoch → 下一帧 matcher initializing。bag 未录 fenced 中间流或 scan owner/admission 内部代际诊断，现有 log 未给更细分原因，因而不能判定是 ledger、source-clock、排队还是其他 admission 条件；不把该两条文本归因为 matcher 低质量。下一次最小增量是加录 `/safety/raw/scan`、`/perception/scan_candidate`、`/perception/scan_filtered` 与 `/perception/scan_status`，用于解释这个失效，不增加 gate 或资格规则。
- 对每条 valid scan 的 **相同 ROS 起止区间**，真值 pose 插值并转 SE2 body delta，排除缺端点/无效区间，不跨 producer reset。运动区间单帧 |scan vx−truth vx| 均值 .08319/.08389m/s，最大 .16840/.16521m/s；不是“wheel 不等于一个平均 scan 就判故障”。相同连续运动链聚合：repeat6 ROS8.5→11.2 的2.7s，scan平均 .207410 vs truth .204222m/s；repeat7 ROS7.8→10.1 的2.3s，scan平均 .217394 vs truth .213589m/s。该数据说明平均位移较接近、单帧速度受量化影响显著；2cm栅格/0.1s区间的影响是代码与曲线支持的解释，仍需独立分辨率/负对照验证，不冻结 current-speed/error bound。
- 原生 row 数分别19514/21695，与 rosbag metadata 完全一致；wheel466/526、truth777/876、IMU776/876、raw scan155/175、joint1553/1731、clock15531/17235、cmd各101。truth/IMU最大source间隔.02s，wheel.04s，scan/scan-motion.1s，无倒序/ROS reset。
- repeat6 原生 clock 最大source间隔.003s；repeat7 在运动开始前 ROS4.600→4.741、4.750→4.871 缺clock消息，joint4.590→4.810亦缺采样，但truth/IMU继续。不能把缺若干clock消息直接断言成ROS时钟重置。运动工作期间命令记录的 clock receive age 最大.0285/.0897 steady 秒，均未越原.25s检查；所有逐步图/源时效检查正常。
- capture仅捕获542/564个clock，与原生共有498/561个相同源stamp，约为原生clock的3.2/3.3%；不能把其counts当发布频率。其有界循环一次仅spin一个callback且共享多流，另有QoS/调度影响，原生bag才是本轮频率/轨迹主证据。repeat6在工作结束后，clock capture出现.809 steady秒的接收空隙但ROS只推进.021s，当前没有原生接收steady时间可区分VM/仿真暂停和采集停顿，不能声称全程无墙钟暂停；repeat7 capture最大接收空隙.187s在工作前。两份标准 analyzer findings=[]，不意味着缺流/调度一概健康。

### 失败准备记录与清理

- `graph-probe` 缺插件路径、`graph-probe-2` 就绪图，`world-discovery-probe` 固定2秒发现等待取证均无刺激。
- repeat1、2、3在 transport发现前拒绝，repeat4在drive参数服务发现前拒绝，均无 stimulus publisher/无 dedicated cmd。一次复试的临时 `inspect.py` 影射标准库 inspect，入口导入即失败；已改名并保留 startup-shadow 日志，未进入fixture或修改产品。
- repeat5发出76条regular（包括正向与部分减速），8秒 alarm 时仍有+.06目标，异常 cleanup直接零；这是明确的异常退出反例，不能当普通减速/完整计划或候选性能样本。该原始bag/JSONL照存，成功对照独立重启。
- finally 结束本次进程组；外部unit再以本次cgroup回收。`cleanup-final.json`：7个unit均inactive/dead，toby进程中 ROS173/该partition 遗留0；其他用户/域/历史进程未杀。清理SHA `65e6b76b2dbc21cea4849cfe7ab202206ae08646a1730bc14c29212d3985d3cc`。

## 原始材料、可复算命令与审查入口

宿主证据根 **`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t1-low-calibration`**（下称 `$E`）；VM 原件在上文新根 `repeat6/`、`repeat7/`，不覆盖任何r3历史材料。宿主对应 `vm-repeat6/`、`vm-repeat7/`，已逐文件比对VM `evidence-sha256.json`，全部匹配；`samples-v2.jsonl` 由 rosbag2_py 只读导出，非有限值保留为字符串，原始 LaserScan ranges 完整保留在 native bag，导出仅简化raw scan ranges为ray_count。

| 证据 | repeat6 SHA256 | repeat7 SHA256 |
| --- | --- | --- |
| bag/bag_0.db3 | f250a5521003573a0a818cf868fbaea1290f72acb675643d8fec7d9c5c3d52b0 | daf2084c70be20674d5b6c78b4028dbdf2f164d18cb114c072f7fe1969b76bc6 |
| stimulus.jsonl | 2f32dd1de2878edfb27983715375e1906755feacf88abe49a04e10321041b1bf | c6ac4a09e88ebc25b6d698e9492bcdcdf140415868380abc7dd105a728818601 |
| samples-v2.jsonl | 55a6e631ebe210fcd39f32461b82f8c6add559410a3ad2bb054a0207d181660f | 1f32a6d8dcd1bf6660a62ba3a4f7b35c99f57420bc6eb5ef5dc8969c5375fc74 |
| evidence-sha256.json | 8c39e50fe1a09054d29a9380cc6247124b991b9943ee769ede1d4ef53163be94 | ab708b2da05ef79fdd4db79633dd6877e9243df158014174dc4a9de680cdfe05 |

- `$E/low-summary.json`：每项指标、全部scan→truth区间明细；SHA `228d1df039c32cb365894a562a11f047c08a7dd0836a53667c67661fef401879`。
- `$E/repeat{6,7}-motion-curves.csv`：可直接画曲线，列名注明SI/ROS源时间或无header命令receipt时间；SHA分别 `71e807b419aa67a58e8eed26021d1c23217258c3cc9439c4765b35cf510b0fad`、`3f435812bf2fbc836494d5e4b26b550ce387f399f121925cca8bcf70863bc0c0`。
- `$E/vm-offline/r3-cause-timeline.json`：manual复现和navigation首latch完整时间线，SHA `6f477b06a51ecdd01c335381cf971492ae1fa323e19aa42cd0e41c4c4c3c3af5`；输入仍为r3原bag/service.log，不重放运动。
- `$E/review-fix.patch`：校准器/fixture/两份定向测试的实际patch，基于r3 archive的已审文件，fixture基于已审b50fa3...生成器；`review-fix-identity.json`给出两端SHA。当前改动还包括4份next plan、本报告、T1历史报告/计划进度追加，均可直接审仓库文件。
- `$E/run-low.py`、`run-two.py`、`ros-env.sh` 保存本次准确启动、独占unit、外部watchdog和finally回收命令；这是本次固定review1专用重现入口，不是产品开发服务。下一次重复需复制成新run根/新repeat和unit，保留已审plan/fixture SHA，不能直接运行脚本覆盖当前目录。拓展速度仅走上一节新fixture审查。

只读复算与检查：

```bash
python "$E/summarize-low.py" > "$E/recomputed-summary-console.json"
# 同名 analyze 输出以 x 打开，复算使用新的输出名。
.venv/bin/python tools/vehicle_motion_analyze.py --input "$E/vm-repeat6/capture.jsonl" --output "$E/repeat6-recomputed-analysis.json"
.venv/bin/python -m pytest tests/unit/test_vehicle_motion_{analyze,calibrate,fixture}.py -q
.venv/bin/python -m ruff check tools/vehicle_motion_{capture,analyze,calibrate,fixture}.py tests/unit/test_vehicle_motion_{analyze,calibrate,fixture}.py
.venv/bin/python -m mypy --explicit-package-bases tools/vehicle_motion_{capture,analyze,calibrate,fixture}.py --follow-imports=silent --ignore-missing-imports
```

本轮定向测试52 passed（含发现并行进展/失败回收、固定recorder端点负对照、扩展计划及过小场地/转弯误用走廊拒绝）；Ruff、Mypy通过。未扩大无关回归。

可形成的候选仅为这个mu=1无转弯场景在+.3m/s的稳态/响应/归零后物理曲线事实；scan单帧误差、producer重置、接收缺口和近deadline调度说明尚不能冻结通用边界。全部29项 measured_bounds、freeze批准/版本/holdout仍为null/空，高低档、倒车、转弯、滑移、反向制动、Nav2同链路、真实硬件/固件均未凭这两条直线数据合格。下一步先审本diff和低速scan分辨率/失效证据，再决定针对估计的最小改进与受限turn取证，按明确数据覆盖冻结；T1整体未完成，控制改动前冻结门不变。

补充身份/下一轮时限：`$E/frozen-identity.json` 已复核原r3 **564个源码文件、1004个安装文件全部SHA不变**，HEAD=`4dd9da0212f0faa759d495af571056a9b5ec2992`；source-manifest SHA `6f8e9971f7d9778cfa292d2e0d3114f05635a8ef4b8d447b883b7d4bf854c4ef`，install-manifest SHA `7ac9b6b2be894d1261fe3c72bec822d36d3d278584be5015c21866c82df7219b`，同时记录实际 arbiter/agent_adapter/motion_lifecycle 导入文件。VM新根 `analysis/` 已保存摘要、CSV、分析器结果、实际patch/两端SHA、准确运行/导出脚本及 `sha256.json`；后处理只增加导出/索引文件，不改原始bag、capture或stimulus字节。已验证8个无关tracked脏文件与r3快照内容SHA全部一致，`git diff --check`及本轮全部untracked文件的 `git diff --no-index --check` 无空白错误。

下一轮候选的具体外部校准 timeout 分别为low18s/high22s/reverse14s/turn15s，均 `--signal=INT --kill-after=1`，内部work上限仍为表中16/20/12/13s且保留50ms零输出交付。准备阶段最多55秒启动+10秒观测/recorder就绪；同步capture建议分别36/40/32/33秒，bag持续到至少calibrator结束后5秒，单fixture的外部unit总限仍175秒+5秒stop，finally只清理本次cgroup。该时限表是待审材料，不是运行性能已冻结；所有candidate仍使用独占ROS173、localhost及由各run_id派生的独立Gazebo partition，不同时运行。实际响应若超过声明运动包络、感知不具可观测性或无法证明场地余量，先停止该批计划并审核最小修改，不靠扩大gate/cap或临时提高时限执行。
