# T1 丰富地标走廊测量与多源修订（2026-11-13）

状态：有限测量子阶段完成，T1 整体未冻结。低档物理响应可重复，但该场地的扫描测速仍不连续；依授权停止高档，未执行高档、重新运行 manual/navigation 或修改产品控制。新用户要求已同步 spec/plan：多源估计、GPS 可缺失、健康方向感知支持下可限速轮速退化；下面的高档依赖是本次专用扫描测量条件，不是产品永久禁行策略。

## 身份、场景和执行边界

宿主证据根目录（下称 `$E`）：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t1-observable-corridor-r2`。
VM：`/home/toby/senior-audit-20261031/calibration-observable-20261113-r2`，原始数据在 `runs/low_forward-{1,2}`，宿主逐文件副本为 `$E/vm-low_forward-{1,2}`。初次采集故障另存无后缀 `.../t1-observable-corridor` 与 VM `.../calibration-observable-20261113-r1`，不覆盖历史。

- 车型 senior_4wd_dl，原点静止独立启动；世界低档 SHA `1a61a757a1d6aa6c0badc8e733011b85d3fe092b7cd9efbdbfd5b4216587e974`，高档 SHA `47cdf5be42009d10c317afdbbe9b64f14d11f26258230e611dcf6cc91deb13ab`。
- fixture generator 对完整 SDF 树去除新静态箱体后逐节点核验原树；只允许具名 `candidate_diagonal_*` 静态箱体。低档新增 14、高档 26 块，原 physics/plugins/地面/车辆不变。VM 实际 model.xacro/controllers/bridge/scan-motion 逐字节对比原 fixture，launch/start 归一化根路径后相同。`vm-preparation.json` 留存全部绑定。
- 低档 manifest SHA `38a51d064e2a428d58a1d30700615275bcb8f24f67c6667e15429971a3c4f122`；高档 `dd869257e77ce4cd3afb6bd4d2571600b852c0af2a4be5a80fe8dd264a2901eb`（只生成未启动）。源车辆 SHA `44c58e0dc5ea10d5168162442c7a4004a0aee372bfd2162f7ca5e664f1aacdba`。
- 低档原 plan SHA `8fc51752bfce4f5a3337b2e09df2e9f3e8cc2b1962d56c655340549363ec13a2`：2 s 零、6 s +5/9 m/s、4 s 零，ramp .3，240 个常规 50 ms tick；工作上限 16 s、外部 timeout 18 s。fixture unit 上限 175 s、stop 5 s。参数、摩擦、目标、超时未放宽。mu=mu2=1 是仿真假设。
- 独占 `.vm-test.lock`，ROS 173、LOCALHOST_ONLY=1、专用 Gazebo partition；无真实硬件/云。唯一命令目标仍为 `/vehicle_motion_calibration/low_forward/cmd_vel`，发布者由被审 calibrator 独占，drive 是唯一执行器订阅，固定 recorder 仅只读订阅。
- 冻结 product source/install 564/1004 文件复核零差异，摘要分别 `6f8e9971f7d9778cfa292d2e0d3114f05635a8ef4b8d447b883b7d4bf854c4ef` / `7ac9b6b2be894d1261fe3c72bec822d36d3d278584be5015c21866c82df7219b`。实际 Python 与 PYTHONPATH、ROS 路径在各 run.json；没有改系统 Python。
- 本轮 generator SHA `62473d3d572e10fd375a6fc8e6467b4264d828a74f6b899ed8115c06db60f1bc`；calibrator 最终 `e0adde8b1e2c642ea3820236798c541f5ec7701fe5a76e1fc05ce5ac8a6b6b4a`。`tools-sha256.json`、实际 runner 副本及 SHA 一并保存。

## 采集故障、修复与实际覆盖

第一次两场刺激本身完成，但 recorder 均报 `SQLite error (5): database is locked`。直接证据是 bag.log，准备器当时反复对正在写入的 SQLite 执行 COUNT；只读连接仍会阻塞 rollback-journal writer。两场保留为采集负例，不能用它们宣布完整曲线。修复后新目录独立重测两次，不做无界重复。

修复：就绪检查改为固定只读 probe 对每个实际类型订阅并收到消息（10 s 截止），同时核验 recorder 活着、唯一执行出口和图；不再访问实时数据库。recorder 在 fixture 前启动，避免晚发现诊断 topic。原生完整性在 recorder 退出后检查。工具 `native_bag_receipts` 仅用于已停止且 finalized 的 bag；无 metadata 拒绝查询，并明确调用方仍须拥有 recorder 的停止事实。新增失败测试先观察到旧实现错误接受未 finalized 数据库，再修复通过。未增加执行器权限。

两次修复后 bag 均正常退出，SQLite integrity_check=ok，49307 / 49391 条消息；原生 `/clock` 38153/38105，raw scan 382/382，四条诊断流均有 381–383 样本，truth/IMU 1907/1905，wheel 1109/1141，joint 3697/3805，专用 cmd 241/241。刺激前每条非 cmd 流已实际收到样本，`observer_preparation_receipts` 留存计数；DDS 匹配不算收到。241 cmd 包括 240 常规 tick 和 finally 零输出；finally 也在普通结束执行，不表示发生异常，普通减速已由常规段完成。

capture 退出 2 是专用图预期缺 `motion/command/safety/ipc`，不伪装 complete。其 `complete` 仅指所要求流至少一条，不能证明频率、完整窗口或物理停车；本轮缺流原样保留。原生 clock 连续 1 ms、truth/IMU 20 ms、scan 100 ms，均无倒退；未发现 clock 源暂停。wheel 最大源间隔约 .04 s；joint 第一场开始记录较晚（首 stamp 1.19 s，第二场 .06 s），已记录连续区间最大源间隔 .01 s；不能从记录推断启动前消息全部接收。capture 的低频 clock 接收不能当时钟暂停。没有导出的 scheduler starvation 计数，不据此声称产品不存在饥饿。

## 低档实测候选（仅该物理 fixture）

| 观测 | repeat 1 | repeat 2 |
| --- | --- | --- |
| 常规命令 tick | 240/240 | 240/240 |
| 命令最大 steady 间隔 | .064924 s | .054463 s |
| 真值稳态中位 vx | .555555555556 m/s | .555555555556 m/s |
| 轮反馈稳态中位 vx | .555555555556 m/s | .555555555556 m/s |
| 最大关节轮速 | 7.309942 rad/s | 7.309942 rad/s |
| 首个最终常规零（bag ROS 收到时刻） | 14.444 | 14.548 |
| 下一份真值 speed≤.001 且持续 .5 s | 14.460 | 14.560 |
| 零时刻插值后剩余位移 | 4.44 µm | 3.33 µm |
| 最终真值 x | 3.328222 m | 3.345216 m |

`.001 m/s + .5 s` 是本次描述性统计判据，非冻结静止标准。bag 收到 Twist 不是执行器应用时间，20 ms truth 采样也不能支撑微秒级延迟/微米级保证。10/50/90% 真值交叉均在命令相应接收后约 .013–.027 s；轮反馈较迟，完整时刻见 measurements.json，不能直接冻为纯执行器 deadtime。0.2 s 斜率仅描述该 .3 ramp 的响应，含定时量化；不证明最大加速度/可保证减速度。场地 y 向最靠近箱体约 4.17 m，扩张车身外接半径约 .457 m；本次直行 y≈0、x≤3.346 m，相对 19.9 m 走廊端墙和停止区域充足。仍未验证其他摩擦、负载和真实制动。

## Scan 首因、误差与旧场对照

原生全程合格比例为 240/382 与 238/381，主要来自停止后的零运动。以同窗 truth |vx|>.01 筛运动窗，两次均 **3/78** 合格，8 unobservable、34 initializing、33 match_unqualified；不能用静止占比声称运动测速健康。

1. ROS .002：raw receipt 报一次 `raw_source_changed`（启动发现来源），随后 raw/candidate/filtered 有效，无运动中 raw 失效记录。不是证据中的运动重置首因。
2. 从第一帧开始 `unobservable_geometry`。在真实 filtered beams 重放现有九束平面窗规则：初始 220 条有效射线，但只有 18 个合格 normal windows，低于现有 25；translation information=.1181>.05、独立 yaw information=2.189>.01。两场全部 75/76 个 unobservable 均由窗数不足解释，未修改条件。
3. 车体前移后信息条件通过，第一次 `match_unqualified` 在 ROS 7.6/7.7，继而初始化/失败交替。源码 `scan_motion_node.cpp:256` → `LocalMatcher::update` → `scan_matcher.cpp:283` 多条件判定 → node `invalidate(..., restart=true)`，会清 matcher；若此前已有合格发布，还调用 new_epoch。ROS 14.1/14.2 的 epoch 切换对应此路径。节点无效发布将 response/residual/covariance 保留为零，不能把这些零当真实失败指标；当前消息不足以区分 response、协方差、inlier fraction、边界等具体子条件。
4. 三个合格运动窗 scan vx 几乎零，同源时间窗的 truth 非零；最大绝对误差 .140160 / .143160 m/s，另两窗约 .05316 和 .02316。比较使用 source interval 的 SE(2) 对数速度与 truth 位姿同窗，不跨 invalid/epoch 拼接、不与不同时刻轮均值硬比。raw receipt 年龄最大 .010/.008 s，clock 无缺口/倒退；现有证据支持场景/配准问题，不能归为时钟停顿或笼统调度饥饿。

旧走廊 low_forward-4 的 353 filtered 帧全部不可观测：窗数 11–23，translation information 6.85e-5–8.46e-4。新场确实改善信息矩阵（translation 最小 .1119、窗数最高 58/60），却未消除起点窗数不足和运动配准失败；旧负例保留。旧转弯 scan 最大 vx 误差 .24969/.28846、yaw 误差 .0851/.0883，不能与新直行“零 yaw”作同类精度提升比较。旧转弯真值/IMU yaw≈.295，而 wheel=.5 是车体/轮运动差异，轮速闭环不能自行消除这项测量偏差。

**纠正前报告**：`scan_matcher.cpp:257–263` 已有 coarse 后 fine：fine 平移步长 .02 m、yaw .005 rad；.02 rad 是 coarse，不是最终 yaw 分辨率，撤回“再加一次已有 fine pass”的建议。即便 .02/.1=.2 m/s、.005/.1=.05 rad/s 是网格尺度，它们也不等于实际输出量化或可保证误差；Karto 峰值统计及固定 anchor 差分可改变结果。现有 bag 尚不足以把全部误差精确分给离散匹配、注册偏差和信息不足。

## 可立即实施的下一步（需单独派发，不改本轮产品）

最小任务：在 matcher 的现有资格合取处产出结构化失败子原因及原始 response/residual/inlier_count/total/covariance/anchor transform，node 无效路径仍发布这些诊断但保持 valid=false、原 epoch/重置语义；不改 gate、search、分辨率或阈值。新增 targeted C++ 测试覆盖每个拒绝分支、初始化正常负对照；将本轮第 1 次作为开发重放，第 2 次及旧转弯/弱场作为已看过的固定回归，不冒充盲留出。独立 holdout：负责人在实现前冻结现有误差/有效率判据，登记两个未用于开发的新地标布局/seed，各固定直行与转弯计划、每场两次、原点静止和原有界清理；先做静止观测验证，再采新数据。holdout 不参与调参，失败保留并回到开发集处理，不能测试后换窗或改标准。先在相同冻结依赖下离线重放现有 filtered scan，不再跑全场逃避分析。

有准确子原因后，仅改被证实的问题：例如若 existing fine 返回系统性偏置，才评审局部连续配准细化及其协方差；不能重复 fine 或降低资格阈值。若补丰富场地，须让实际 GPU/过滤后的九束平面窗在起点和全路径成立，先静止只读取样验证；此前理想 2D raycast 的 35 窗预测不等于实际 GPU+过滤结果。弱场仍须通过已批准的多源/轮速退化策略覆盖，不能靠一直增地标规避需求。高档继续暂缓；所有 29 项 measured_bounds 与 wheel degraded cap 保持 null。

## GPS 资料核查与验收分层

本机厂家源码为 primary evidence，根目录 `1.WHEELTEC ROS机器人通用资料/7.ROS源码/ROS2源码/Humble源码/src/wheeltec_gps`，四份原始文件 SHA 在 `$E/gps-vendor-sources.json`：

- NMEA driver 的 `/gps/vel` 为 TwistStamped，另有 `/gps/nmea/vel` covariance；VTG 依 valid_fix、RMC 依 fix_valid，按 course 算东/北速度。它把速度 std_dev 写死 .2 m/s（方差 .04）；不能以此证明目标 GPS 的实际精度。
- ublox firmware7plus `~/fix_velocity` 从 NavPVT vel_e/n/d mm/s 转 ENU m/s，协方差用 s_acc 换算平方；原消息同时有 fix/status，仍需时效、状态、协方差可信度及漂移检验。
- 串口配置只是 C94-M8P 示例，不能证明实装型号、RTK/双天线、天线杆臂或真实速度精度。未打开串口/猜凭证/启动硬件驱动。当前产品 xacro 没有 GPS sensor，已有 geo/relocalization 是位置/种子能力，现有 GPS 单测不证明融合测速。

spec/plan 已加入多源质量、source mode、时效/协方差、轮速退化和恢复平滑，并删改激光唯一/缺源一律全停条款。分层矩阵覆盖丰富/长直重复弱特征、狭窄转弯、前后障碍可退、摩擦滑移、动态障碍及 GPS 有效/丢失/漂移、激光测速失效、组合失效与恢复；区分真实物理/传感器与注入。当前只完成本次有限场地测量与旧 bag 对照；GPS/退化控制与其余场景尚待所属任务，理想 GPS 不是硬件证明。

## 复现、审核与清理

`$E` 包含 run-scene.py/run-batch.py、ros-env.sh、prepare.py（各次运行另保存实际 runner）；执行必须保持现有锁和有界 unit，不可绕过本报告的高档暂停结论。离线复现：

```sh
python3 "$E/measure.py" low_forward-1 low_forward-2
python3 "$E/audit-geometry.py"
```

曲线：`curves-low-{1,2}.csv`（truth/wheel/cmd）及 `low_forward-{1,2}-scan-intervals.csv`（同窗 scan/truth/wheel/epoch）。原生 bag SHA：repeat1 `24e5aa5b7844b4f0863cb056996fbc1ed382e1f3cca3c8d9dd92e46ef087776f`；repeat2 `afba203cbf01e74bc6b77fda7bc09125bad405ef8b09e964c08533939c3725f9`。lossless samples SHA 分别 `08ac981765cd64112c3907d2768bd8a3de65120af3d51542f9d2152c2a4b96f1` / `af2d1c541c8358bfe78b1a714be005ca362d698e1ecbd131932c9c03814f0b55`；每场 evidence-sha256.json 与总 artifact-index.json 记录其余文件。

`cleanup-and-identity.json`：两 unit inactive/dead、ControlGroup 空；取得独占锁，/proc 中 ROS173 进程为零；仅回收本次进程组/cgroup，无其他进程操作。原产品 source/install 零差异。没有凭证/私有 profile 导出。

本地检查：57 个针对测试通过，四工具 Ruff/Mypy 通过；真实采集负例未当测试通过。最终实际增量以本轮 before 快照对照的 review-fix.patch 为准；负责人须亲读后提交，未提交/推送。T1 不称首版完成，不声称 5 km/h 闭环或实车停车通过。

实际本地验证命令（均退出 0）：

```sh
.venv/bin/python -m pytest tests/unit/test_vehicle_motion_analyze.py tests/unit/test_vehicle_motion_calibrate.py tests/unit/test_vehicle_motion_fixture.py -q
.venv/bin/python -m ruff check tools/vehicle_motion_{capture,analyze,calibrate,fixture}.py tests/unit/test_vehicle_motion_{analyze,calibrate,fixture}.py
.venv/bin/python -m mypy --explicit-package-bases --follow-imports=silent --ignore-missing-imports tools/vehicle_motion_{capture,analyze,calibrate,fixture}.py
git diff --check
```

未跟踪的新文件另逐一 `git diff --no-index --check /dev/null <file>`，无 whitespace 错误；原有 8 份 tracked 脏文件 SHA 与初始冻结清单相同。`local-verification.json` 记录结果。
