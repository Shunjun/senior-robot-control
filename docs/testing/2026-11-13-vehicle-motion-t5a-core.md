# T5a纯闭环核心（待负责人亲审，不接执行器）

2026-11-13。已实现可运行纯轨迹、P/PI控制及状态机，不是5 km/h、Gazebo或实车验收。没有修改旧safety、ROS/UI/cloud/协议、T1参数或其他既有脏文件，没有启动服务。真实CI仍0、激光移动估计精度差及适用物理边界未冻结的限制保持。

## 已审提交与本轮边界

T4a提交 **`c569658da88eeb111ee1e907a9c2fd3bf4e50332`**，消息`feat: add bounded directional tightening and asynchronous relaxation`。提交前index为空，五文件实际SHA分别与已审t4a-directional/t4a-review2清单完全相同；提交仅含：

- `src/senior_control/motion/contracts.py`
- `src/senior_control/motion/constraints.py`
- `src/senior_control/motion/constraint_worker.py`
- `tests/unit/test_directional_constraints.py`
- `tests/unit/test_constraint_scheduling.py`

提交后index为空，无push。T5a未提交：新增`motion/{trajectory,controller,state_machine}.py`及`tests/unit/test_motion_{trajectory,controller,state_machine}.py`，只更新总计划及本报告；没有再改contracts或已审T4几何。

证据根目录E：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t5a-core`。包含限定8文件review.patch、源码/日志/CSV SHA、提交清单、完整实际源码、RED/GREEN、demo.py及七组曲线CSV/摘要JSON。旧失败曲线保留在before-fixture-direction-fix和before-antiwindup，不覆盖冻结r3。

## 接口与职责

| 模块 | 消费与产出、唯一状态 |
|---|---|
| trajectory | `Trajectory(initial_velocity,initial_acceleration).propose(goal,response,dt)`→不可变TrajectoryStep；`commit(step)`只在最终验证成功后更新v/a。三个有限jerk阶段解析积分到目标速度/零加速度，初始残余加速度不丢失；反向不可避免、非法dt或参数返回显式异常。速度目标/cap来自T4和轮能力，不另造响应参数。 |
| controller | `MotionController(engine,config,initial_v,initial_a).tick(target,context,permission,body,wheel,now,estop=False)`→ControlDecision。持有command轨迹、PI、近静止证据计数和首次停车响应截止；不持有authority、源健康恢复或ESTOP锁存。初次tick仅建立时钟，无输出；异常/长dt不追赶。 |
| state_machine | `MotionStateMachine.observe(RecoveryObservation,now)`与`reduce(MotionEvent,now_steady=...)`集中转换。单一可信composition流分配事件sequence；不是浏览器自报序号。观测键/时刻必须来自T3已准入新证据，不能给旧融合窗口重新编号充数；没有runner、ROS或存储副作用。 |

Feedback是已准入、带原steady stamp/deadline、epoch/sequence、basis及`current_supported`的当前vx/wz投影，不能把BodyMotion区间均值重新盖戳成当前速度。body参考只在有依据时进外环；wheel用于跟踪/不一致检查，在有独立模式风险包络的wheel_imu_limited模式可作受限参考，不成为真实body速度或静止证明。缺可信body不造0，也不积分IMU成速度；source admission/质量仍由T3唯一owner处理。本轮T3→T4→T5测试实际运行ReplayChain，使用其masked partial BodyMotion（average/upper均None）、mode和source epoch，独立合成plant包络支撑限速；wheel-only路径不累积body PI或声称测得真实静止。该T3接口测试用显式synthetic准入1份/0跨度快速构造输入，不替代另行测试的运动恢复5份/.5s，也不改生产资格。

PI无D，幅度及轮组合共同比例限幅，最终candidate在轨迹/PI/轮限之后交`T4.validate_output`，拒绝则不commit轨迹/I。有限速度、加速度和jerk来自同一ResponseBounds及T4模式cap；模式恢复只通过轨迹改变速度。抗饱和包括幅值与jerk/rate限制，允许旧积分减小；停车、hold、参考/来源或模式改变、不一致反馈时清积分，不靠持续抬高驱动追滑移。轮/体差只能报告不一致，不能武断判定某源错误。

`ControlDecision.command=None`是无可执行候选/故障请求，不是“安全零”，也不授权保持旧命令。其identity和deadline取实际消费的T4来源/配置、固定permit deadline、原始目标/反馈/运动/感知/响应/风险期限之交，不重盖时间戳。verified只表示在显式模型条件下逻辑证明成立，不代表该模型已被物理验证。composition必须消费emergency/原因并进入独立故障处置，不能忽略None继续执行旧非零输出。

缺运动permit而停车空间仍成立时，正常制动不瞬时清零：命令沿同ResponseBounds的jerk停车轨迹连续推进。每步仍调用validate_output；该接口按T4设计仅授运动离散许可/显式插值证明，缺permit时返回false，因此制动另消费**最新零目标refine的停车证明**，不把运动授权与停车证明混为一谈。反向请求一直先取零目标，直到新鲜独立body+wheel近静止证据连续达到注入的3份/.1s且命令v/a归零，才切换方向；这是合成测试的近零配置，不冻结实测阈值。默认不主动输出反向速度制动。

首次进入停车时用T4公开stop_axis及原ResponseBounds计算监督终点；后续新scan/反馈/intent generation不续期。到期仍动报告stop_response_failed，没反馈则stop_boundary_without_feedback。最新stop_envelope_blocked优先于该计算诊断，不再重演T4审核中的制动降级。无可行jerk步、模型/轮限不能满足、T4最终否决都明确返回None及原因；不编造物理安全，也不改成用户ESTOP锁存。

## 状态语义

- ordinary obstacle只进入OBSTACLE_HOLD，保留manual/task身份和方向约束能力；解除不走故障恢复。反馈停、execution fault、软件ESTOP、硬件按下分别管理，优先级为ESTOP→执行故障→反馈停→障碍hold→控制源。
- 软件reset必须匹配当前estop_revision、硬件未按下、新鲜健康且实测近静止；不会顺便清fault。fault_recover独立匹配fault_revision、5份独立健康且跨度≥.5s及真实近静止。硬件release来自可信硬件owner，网页reset不能清硬件状态。
- FEEDBACK_HEALTHY只在同源、时刻推进的至少5份健康证据且跨度≥.5s接受；重复tick/seq/stamp、断档、源重启不能累计。observe/reduce共用同一steady时钟，倒退变execution fault而非用户ESTOP。
- manual takeover暂停已接受任务；release/expiry/reconnect/反馈恢复不自动恢复接管任务。明确TASK_RESUME只产出replan_task动作，待T6重规划后的TASK_START才运行；普通运行任务反馈恢复也先请求重规划，不直接恢复旧速度。
- reset关闭旧控制源、保留任务暂停，不复活手动或任务。新MANUAL_START是T2已验证的新authority事件，不在此重写T2 acquire/关闭语义；重放全局事件序号无效。ESTOP覆盖是独立显式分支，可发0但verified=false，不等于物理已停；复位接受后controller.reset_at_stillness只在真实新鲜近静止证据成立时重建零v/a，下一tick才推进。

## 验证与真实曲线

关键行为先RED后GREEN，日志包括初始模块不存在、模式cap降级、状态恢复/时钟、原许可deadline、抗rate饱和、停车响应边界与最新碰撞风险优先级。测试发现并修正S曲线释放交界浮点消去（1e-12尺度数值容差，不是物理容差）、组合轮限达到边界时1–2 ULP舍入差误判为物理不可行（只允许四ULP归一化，实际更大轮限变化仍拒绝，最终归一化candidate重过T4）和fixture中倒车时“向零减速”误标为向外加速度；没有调PI/通过线掩盖失败。

最终 **92 passed**：33项新增T5a，53项直接消费的T4约束/调度回归，6项contracts回归；原330未无理由重扫。严格Mypy21源文件、Ruff、git diff --check通过，命令：

```sh
PYTHONPATH=.:src .venv/bin/pytest -q tests/unit/test_motion_trajectory.py tests/unit/test_motion_controller.py tests/unit/test_motion_state_machine.py tests/unit/test_directional_constraints.py tests/unit/test_constraint_scheduling.py tests/unit/test_motion_contracts.py
MYPYPATH=src:. .venv/bin/mypy --strict --explicit-package-bases -p senior_control.motion -m tools.motion_fusion_replay -m tools.motion_replay_records -m tools.motion_fusion_synthetic
.venv/bin/ruff check src/senior_control/motion/{trajectory,controller,state_machine}.py tests/unit/test_motion_{trajectory,controller,state_machine}.py
PYTHONPATH=.:src:tests/unit .venv/bin/python '/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t5a-core/demo.py'
```

每个demo固定500步、dt=.02s，轮shaft的一阶离散plant为`wheel += .2*(command-wheel)`，body为gain*wheel，x积分body；正常gain=.9，滑移负例gain=.2。理想当前body/wheel传感器仅为合成plant测试，不能当真实T3观测精度证据。normal响应注入A=.4、D=.5、J=2、yaw A=.5/D=.6/J=2、lag=.1；P(.5,.3)、I(.3,.1)、I上限(.2,.1)、轮距.4、轮速上限1、max_tick=.1，均SI且synthetic，响应profile使用原固定20s截止，只有当前plant观测/风险状态每tick重新计算，不给旧profile续期。CSV记录目标/实际候选、shaft/body、位移、参考加速度、I、mode、reason、verified及异常请求；JSON列全部缺输出和失败，没有滤掉不利数据。

| 场景 | 实际摘要（10s） |
|---|---|
| release | 5s时body=.38496m/s，随后仍有约.23227m物理位移才停，最终接近0；没有用发布零替代静止。 |
| reverse | 从+.3m/s开始；首次负命令t=.96s、body约+.00349m/s（近零新证据后），最终倒车后松杆也停下，累计x=-.93774m。 |
| rear_unknown | 无倒车许可，body最终接近0，x=.13905m。 |
| degrade_recover | 无body+wheel限速.2→body恢复+cap恢复，通过轨迹平滑，最终停下，x=1.34807m。 |
| late_feedback | 2–2.4s注入过期样本，20tick执行反馈制动而非PI追旧数据；恢复后继续、最终停下。 |
| estop_reset | 2s急停，2.1s未停稳reset拒绝，2.8s有效reset，直到3.2s新控制事件才运动；旧目标不复活，最终停下。bootstrap及reset当tick两次明确无输出，不是故障。 |
| slip（未通过停车，模型适用负例） | 健康body与wheel持续不一致，I清零并限驱动；D=.5不适用于gain=.2，首次模型监督耗尽后224次stop_response_failed、None，纯plant保持末个命令最终body=.0405m/s、x=.56556m。没有注入未经验证的兜底模型让其“通过”，也没有把None当安全停车；这组不能支撑安全或驱动资格。 |

其余六场仅bootstrap一次无输出（estop/reset两次），没有异常请求。slip模型负例说明生产composition必须有经验证的故障制动/下游watchdog途径；当前纯链只报告无法证明/无法执行的边界，不承诺该接口能刹住所有物理故障。

## 下一步最小实测集合与接线

T5a待审后可做有界仿真接线，不能先等实车所有参数，也不能把合成值生产放行。最小集合是：(1) 当前模式/场地的body参考延迟/预测域与独立风险包络，含轮滑移和静止依据；(2) 有限速度、轮能力/轮距/固件接口、真实A/D/J/lag及侧滑settling；(3) 感知free/unknown/动态预测范围、外参和停止区域；(4) Linux tick/快路径/worker最坏预算、时钟暂停与原证据deadline；(5) 最终唯一输出、进程/通信故障和不响应执行器的已验证兜底制动。只冻结实际适用集合，不以29项空值阻塞纯算法。

T3真实当前预测/统计模型仍有缺口，Feedback.current_supported不能由旧区间均值或未知P伪造；生产adapter需按T3原支持时间/来源桥接。执行器adapter还须区分可信本机制动owner与已关闭的manual authority：过期/release后只允许本机制动，不能以刷新旧authority给迟到命令续命。ControlDecision是内部数值候选，尚不是带唯一输出fence的wire命令。T6消费replan_task和暂停原因，T7投影统一状态，ROS最后唯一出口在完整T4/T5验证后接入。约束几何机械拆分仍另做审阅，不混本批公式。硬件急停/固件PI/真实地面、5 km/h、在线自适应与长期可靠性未完成。

## R1/R2审核修正（本节替代上文仅靠body零目标证明的制动说明）

R1已用负责人反例RED复现。新增RiskEnvelope可选brake_drive_overshoot及brake_tracking_basis（Compatible默认None/空，不自动信任旧profile），独立声明整个零目标命令族/抓地恢复的车体速度可达域：当前body区间与历史命令、本tick候选的凸包，加独立物理驱动超调；同profile还必须支持ResponseBounds停车响应与settling。这不是wheel/body瞬时差、不是把命令改名真速。缺该关系明确missing_brake_tracking_model/None/未验证。

ConstraintEngine.validate_braking消费实际历史command、残余加速度与当前候选；当前body的upper不能裁掉驱动未来域，映射域保留原scope/frame/profile/期限，并使用原_stop/_sweep公式做完整线/角扫掠核验。controller只在Trajectory.propose零目标的解析jerk族中调用，制动候选须通过这份绑定证明，原body停车证明不再替代它。剩余向外加速度（零命令时双向）纳入；不改变旧公式或门槛。正常有效连续制动保留；模型不适用/空间不足则拒绝候选并请求独立异常处置，不伪造硬零。新API并非任意外部轨迹授权；它绑定本纯控制器的零目标解析族，未来执行器/ROS仍不得绕过轨迹生成器。

合成fixture显式声明被动、无额外驱动超调且同响应族可停的假设（overshoot=0）；这不意味着当前车体等于命令，当前体速仍独立保留。真实滑移恢复的超调/延迟/制动力关系尚无实测，生产profile不得填此合成值。新增正反/线角组合、残余加速度、过期/缺模型及近障碍历史命令反例；正常plant加减速继续通过。

R2在旧源码加载下精确复现pause/done均遗留replan_task（r2-red.log）；两条生命周期分支清理_feedback_task。恢复后显式pause仍需TASK_RESUME才给replan；已结束任务不能resume；正常导航仅反馈暂挂的自动重规划和手动接管暂停原回归保留。没有改T2/T3或网络语义。

受影响回归97 passed、严格Mypy18模块和Ruff通过；命令沿用上文六组pytest文件及 -p senior_control.motion。七组demo使用新源码/显式合成关系重跑到独立t5a-r1-review目录，六个正常场景仍最终近零，slip仍225次无候选/224次异常请求、终速.0405，旧负例与原曲线未覆盖。审阅包及前后SHA：chat-scratch/32583a56faed28ac0328929612a923c8/t5a-r1-review；R1对constraints为最小新API/可选字段diff，其余公式不动。T5a仍未提交、无ROS/Gazebo/服务或物理资格声明。

## 制动时间与空间统一（task_13ef385d71bf，待审）

RED：历史命令+.4、当前body+.05，普通被动一阶滞后plant下，原body-only监督先于drive轨迹结束触发stop_response_failed；新增test_normal_body_lag_does_not_expire_before_drive_stop_family实际失败。现T4的BrakingProof同时携带brake与同一次_sweep的绝对deadline_steady=now+horizon_s，只有空间、模型和原时效均合格才提供截止。T5在生成实际候选后消费该证明，并锁定首次有效截止；删除T5重复的stop_axis计算，不复制公式、不增加余量、不在新scan/generation续期。旧validate_braking返回BrakeRequest接口以Compatible薄封装保留。数据新鲜期限与物理停车终点仍是不同含义，逐tick继续检查原观测/模型有效期。

98项受影响回归通过，严格Mypy18模块/Ruff通过。仅重跑受影响的release与slip各500步到新目录t5a-time-review/demo；原CSV、失败记录不覆盖。release结果不变；旧gain=.2滑移demo的body-only超时也属于该缺口，修复后最终body约1.96e-22、x=.395423m，无异常请求。不能为了“保留失败”维持错误超时；原终速.0405的历史失败证据仍保留，不响应车体的stop_response_failed负对照仍通过（原阈值未改）。这些合成结果不证明真实抓地恢复、制动力或滑移能力。

### 下一批实际最小输出接线边界（本批未改）

1. `ros2/src/senior_robot/senior_robot/motion_fusion.py`：把实际ROS观测适配到T3唯一AdmissionOwner/TemporalResolver/NumericalFusion，保留原keys/时刻；当前支持不足则average/upper保持None，不给区间均值改戳。供应有依据的模式及当前风险/drive关系，不能写死synthetic资格。
2. `ros2/src/senior_robot/senior_robot/safety_node.py`：已有steady timer与最终Twist发布者；在这里组装T4预处理/ConstraintScheduler、T5轨迹及状态机，逐候选快核验、独立制动owner、FinalCommandFence、唯一输出/watchdog。不能让旧零输入分支先退出闭环，也不能同时保留两个执行链。改动需对应ROS集成测试与真实加载路径核对。
3. `ros2/src/senior_robot/launch/{safety_motion,simulation_motion}.launch.py`和`config/simulation_motion.yaml`：只做同版参数/profile/话题接线，固定已验证仿真能力，不提高车型限幅。T4几何/公式无需再次重构；T5纯模块无需继续扩矩阵。
4. T6独占且本批只读的`src/senior_control/agent/{composition,ports}.py`、`safety/arbiter.py`及ROS`{agent_adapter,command_node}.py`负责T2已接受目标、source/generation、authority关闭与任务暂停/重规划映射。先由T6给出冻结handoff接口，禁止双写；过期manual只关闭意图授权，本机制动不能继承旧用户运动授权。状态机pause/done原因不得被恢复事件盖掉。

### Linux有界正常验收命令与明确缺口（仅整理，未执行）

现有`tests/ros/test_simulation_manual.py::test_gazebo_manual_motion_and_stop`消费SENIOR_SIM_NAV_SOCKET，但写入的是旧manual协议及固定请求，不能直接充当新低档5/9、高档25/18m/s或T2 authority验收。`tests/ros/test_motion_controller_integration.py`尚不存在；不存在的新测试命令不称为可运行通过线。下一批接线同时只需在该文件实现四个正常场景：低档起停、高档起停、低档左右组合转向、高档受轮限组合转向；各从原点静止、不反向/不注故障。提前按适用模型与场地冻结速度误差、停车距离、稳态时窗及Linux预算，不能测试后调线。

现有工具的可运行命令模板（新独立run，Humble/Fortress、既有本地私有profile副本、robot_id=route-sim、frozen source/install/venv；不得原地覆盖历史）如下。由单个外层脚本持`.vm-test.lock`贯穿整个unit，并用trap/finally stop；每场unit RuntimeMaxSec<=175、TimeoutStopSec=5、KillMode=control-group，仅回收本次unit。运行前检查ROS173无人占用，LOCALHOST_ONLY=1，独立IGN_PARTITION/GZ_PARTITION、db/socket/estop/log，禁真车/原云。

```sh
source /opt/ros/humble/setup.bash
source "$RUN/install/setup.bash"
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="$RUN_ID" GZ_PARTITION="$RUN_ID"
"$RUN/venv/bin/python" -m senior_control.cli doctor --profile "$RUN/state/simulation.json"
"$RUN/venv/bin/python" -m senior_control.cli start --profile "$RUN/state/simulation.json" --headless true
# trap/finally: 同一profile stop，然后只停止本次独占systemd unit
"$RUN/venv/bin/python" -m tools.vehicle_motion_capture --socket "$SOCKET" --seconds 60 --output "$RUN/capture.jsonl"
# 与capture同步，原生bag独立进程组；最终零命令后至少观察5秒真实静止
 timeout --signal=INT --kill-after=2 65 ros2 bag record -o "$RUN/bag" /clock /cmd_vel /motion/scan /wheel/odom /imu/data /ground_truth/odom /safety/status
# 仅已有旧链烟测；与新authority协议不能假称兼容
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SENIOR_SIM_NAV_SOCKET="$SOCKET" timeout --signal=INT --kill-after=2 120 "$RUN/venv/bin/python" -m pytest -p pytest_asyncio.plugin tests/ros/test_simulation_manual.py::test_gazebo_manual_motion_and_stop -q
"$RUN/venv/bin/python" -m senior_control.cli stop --profile "$RUN/state/simulation.json"
```

这些命令需在上述锁/unit/watchdog框架内并行采集、finally清理，不是顺序跑完采集后才刺激；现有私有profile路径由新run复制产生，不输出凭证。新四场刺激runner依赖T6接口及上述输出接线实现，当前确切阻碍是未接线/未实现验收入口和适用模型未核验，不是再等所有29项或增加纯模块。真值只评分，验收记录实际body/轮速、命令/加速度/jerk、轨迹位移和最终发布者身份；不能将calibration直发cmd工具代替本产品闭环验收。
