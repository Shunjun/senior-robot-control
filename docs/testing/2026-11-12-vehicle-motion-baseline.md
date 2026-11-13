# SDD ledger — plan: docs/superpowers/plans/2026-11-12-vehicle-motion-redesign.md

T1 工具实现与只读环境取证，2026-11-12；**T1 尚未完成，所有实测边界未冻结**。本记录不证明可驾驶、物理停车、5 km/h、导航或真实硬件通过。未启动仿真、服务或发送运动刺激，未提交/推送，未修改产品控制源码。

## 本轮范围与决定

- Ruling: 仅在获准文件白名单内工作，把执行账本放在本 baseline 文档，不创建技能默认的 worktree/隐藏任务目录，不执行提交脚本；当前明确派发范围优先于技能通用流程，代价是没有自动化 task-done 回执，需负责人读实际 diff 与下述验证。
- Ruling: capture 是只读 ROS subscriber + `state.get` IPC 轮询，不把 IPC 车速/状态当成收到 ROS 的证明；缺少任何必需 ROS 流均 exit 2，代价是即使没有 truth 的产品运行正常也不能声明完整验收采集。
- Ruling: 当前 `motion_fusion.py` 详细等待诊断在日志中，SafetyGate 只导出 worker_finished/worker_discarded 等部分计数，缺少“提交/过期拒绝/入许可”完整数值窗口；工具保存原始 ROS/IPC，但不从文本猜出 alignment_wait/permit_starvation，代价是这两项真实根因可能仍需后续受审的有界诊断适配或离线数值转换。
- Ruling: analyzer 的 0.5 s 连续窗口、0.21 s 最大间隔、0.25 s 源时效和 0.15 m/s 轮身差异是具名诊断判别策略，**不是冻结验收阈值**；不输出物理停车通过，代价是它负责给可复查线索，不能直接放行新控制器。
- Ruling: calibrate 默认只校验计划；真正发布需 `--execute`、完全匹配的 reviewed SHA、专用域/话题和仿真图检查，不接受车端 profile/凭证；代价是现有默认图无法直接运行该工具，须先审专用 fixture。
- 依赖已按负责人审阅修正：T6 接线依赖 T5，T7 集成依赖 T5/T6；两者可先设计纯模块。T1 冻结实测响应/误差与分配预算，T4/T5 才实测新模块最坏耗时/行为，不用尚未存在的新控制器反定 T1 通过线。

## 工具与证据格式

| 文件 | 责任/边界 |
| --- | --- |
| `tools/vehicle_motion_capture.py` | 最长 3600 s / 128 MiB，JSONL 原始采集；订阅 `/clock /scan /motion/scan /motion/evidence /wheel/odom /imu/data /cmd_vel /safety/status /ground_truth/odom`，每 0.25 s 只读 IPC `state.get`；不发送 heartbeat/velocity，不伪造任务事件；实际 task status 以 state.get 当前携带字段为准 |
| `tools/vehicle_motion_analyze.py` | 原始文件 SHA256、样本索引、具名连续窗口诊断；不改原始文件、不排序抹平乱序，不判物理停止 |
| `tools/vehicle_motion_calibrate.py` | 严格 plan 校验、有限 ramp、仿真证据检查、退出零请求及回收 publisher；不启动 fixture，不读取任何车辆凭证，不是产品控制实现 |
| `tests/fixtures/vehicle_motion_trace.json` | **合成**正刺激：停钟、258 ms 数值等待、轮转车不动、持续过期结果；不是历史现场数据 |
| `config/vehicle-motion-acceptance.yaml` | status=unfrozen，需求目标/模型声明与全部待测 null 分开，不能拿它冒充已验证 capability |

JSONL 第一条 manifest 给 tool SHA、Git HEAD（不含未提交 diff 身份）、域、时长、话题与 QoS；负责人采集时仍须另存冻结源码/安装包/网页摘要。每条 ROS sample 保留 raw 消息、callback receive-steady、原始 ROS stamp（无 header 的 Twist 为 null）、DDS GID/原始 DDS 纳秒时间。DDS 时间不冒充 host steady。扫描平均速度由 body_delta 的 SE(2) log / 真实区间算出；原始 body_delta 一并保留。Inf/NaN 编码成字符串，避免丢弃无返回激光射线；凭证类键脱敏。summary 明列 counts/missing_streams，exit 0 仅表示要求的流至少实际收到一条，**不表示窗口完整或产品通过**。best-effort depth 5 会丢包，callback time 包含本采集器调度，不能当传感硬件接收时刻。

analyzer 保留源路径/SHA与 evidence 的 raw record indices；receive-steady 负责连续窗口，source ROS time 负责区间。轮速与 truth 在相同 scan 区间线性插值积分，只有连续且新鲜的有效 scan + truth + wheel 持续差异才给轮身差异线索，正常加速时不会把瞬时轮速与区间均值直接比较。乱序/换源/间隙打断窗口；过期 scan 不补新时间。permit_starvation 要数值 counter 增量（至少 3 次提交/完成/过期拒绝且 admission 不增加），counter reset 和正常入许可为负对照。真实数据缺这些数值字段时只记不可观测，不伪造诊断。

## 实际验证

先写 fixture/失败测试，看到三工具 skeleton 的 21 项 NotImplementedError 失败；随后实现转绿。额外测试先观察换源跨窗口、缺 watchdog 校验、零发布后立即回收、慢检查后仍发布非零等断言失败，再修正并回归。

```sh
.venv/bin/python -m pytest tests/unit/test_vehicle_motion_analyze.py tests/unit/test_vehicle_motion_calibrate.py -q
# 31 passed（包含正常负对照、缺流/乱序/过期、CLI 默认不执行、保护旧输出文件、授权摘要与清理）
.venv/bin/python -m ruff check tools/vehicle_motion_analyze.py tools/vehicle_motion_capture.py tools/vehicle_motion_calibrate.py tests/unit/test_vehicle_motion_analyze.py tests/unit/test_vehicle_motion_calibrate.py
# All checks passed
.venv/bin/python -m mypy tools/vehicle_motion_capture.py tools/vehicle_motion_analyze.py tools/vehicle_motion_calibrate.py --follow-imports=silent --ignore-missing-imports
# Success: 3 source files；仅忽略本机不存在的 ROS/generated-message 类型库，不是 ROS 运行验证。
```

另外 136 项原有定向回归与新测试联合跑过一次 162 passed（当时新增为 26 项）；最后新增回归后的完整数字以末尾最终验证为准。本轮没有实际 ROS subscription/publisher 运行证据；默认校验 CLI 和合成 analyzer CLI 均实际运行成功，缺 ROS/IPC capture 子进程实际 exit 2。上述工具本地测试不能替代后续仿真。

## 环境探测事实（只读，脱敏）

- 宿主 Darwin：有 `.venv`、`prlctl`，无 ros2/colcon/ign/gz；无需猜 SSH 身份。首次 `prlctl` 沙箱报 ps 权限错误，按工具权限机制只读重试，没有按其提示 init/restart Parallels。
- Parallels 已运行 VM：`Ubuntu Linux 22.04`，UUID `f76a76fb-67f9-406a-ba66-4eef50a063b5`，IPv4 `10.211.55.4`，架构 aarch64；另一 Ubuntu 26.04 VM 停止，未启动它。
- `prlctl exec <uuid> '<完整 shell 命令>'` 可只读访问 guest；该工具会拼接远端 shell 参数，Python `-c` 必须在传入的完整命令里保留 shell 引号。本轮一次引号错误只产生语法错误，修正后读取成功；不使用 shell 插值承载凭证。
- `/opt/ros/humble/setup.bash` 存在；加载后 ros2=`/opt/ros/humble/bin/ros2`、colcon=`/bin/colcon`、ign=`/bin/ign`、xvfb-run=`/bin/xvfb-run`；rclpy 可导入。已安装 libignition-gazebo6 6.18.0、ros-humble-navigation2 1.1.20、ros-humble-ros-gz-bridge 0.244.26。
- 探测时内存总 7927 MiB、available 6657 MiB，swap 使用 0；/home 所在盘可用 39 GiB。`pgrep` 对 ign gazebo/senior_control/safety_node/command_node/nav2 返回 0 个；用户 `senior*` 活动 unit 列表无输出。此为瞬时事实，后续运行前重新核查，并不证明域号 173 永久空闲。
- 现有私有 profile：`/home/toby/senior-sim/simulation.json`，kind=simulation，ros_domain=73，socket=`/home/toby/senior-sim/adapter.sock`（不存在），database/estop 文件存在，credentials 文件存在且目录 0700/凭证文件 0600，未读/输出凭证内容。
- profile ros_setup=`/home/toby/projects/senior-robot-control/ros2/install/setup.bash`（存在），项目 `.venv/bin/python` 存在。源码 HEAD 首次以 root 查询被 Git ownership 检查拒绝，**未**改 safe.directory；随后以 toby 只读查询成功：HEAD=`d6696a1`，Python=`3.10.12`。VM 源提交与当前分支 `4dd9da0` 不同，安装包对应源码尚未核验，不能直接作为当前同版基线。
- 文档约定 `/home/toby/senior-audit-20261031/` 当前不存在；历史材料实际存在 `/home/toby/senior-validation/20261112-*`，未搬移/覆盖。约定统一根需要负责人后续准备，不拿历史目录作本次输出。该历史根未找到 `.venv*`，不能假设某个 Python 环境路径有效。

## 待负责人审阅的明确刺激计划

临时文件（未加入仓库）：`/private/tmp/vehicle-motion-calibration-review-20261112.json`。
SHA256：`ac219e8245d9f64709b9adeb1974058bab5301114ddb383489e5c7daf38aba7f`。

- 专用域 173（候选，运行前核查），world 必须为 `vehicle_motion_calibration`，车型 senior_4wd_dl，初始真值位置距原点≤0.25 m、平面速度≤0.01 m/s 且 |wz|≤0.01 rad/s；源时间必须与推进中的 clock 同步，不能靠重放旧 truth 刷新接收时间。
- 只允许 `/vehicle_motion_calibration/review1/cmd_vel`，禁止 `/cmd_vel`；现有产品节点/未知节点或额外发布者即拒绝；Gazebo /clock 与 truth 唯一来源均需 senior_sim_bridge，drive use_sim_time=true、unstamped Twist、0<cmd_vel_timeout≤0.3 s；时钟持续推进、truth 新鲜，场地界内至少 3 m guard。
- 场地边界 [-20,-20,20,20] m 仅为待审 fixture 条件，不声称场地现已存在或已测可保证制动。
- 1 s 目标零 → 2 s 目标 +0.3 m/s（命令 ramp≤0.3 m/s²，转向零）→ 2 s 目标零；合计 5 s，CLI 工作期限 8 s（含 ROS preflight 的独立 alarm，零值发送清理另预留 0.05 s，外部测试 watchdog 10 s）；无反向刺激。yaw 上限 0.5 rad/s 但本计划不施加 yaw。
- 0.05 s tick；正常结束/异常/信号 finally 请求零、等待一个 tick 让 DDS 发送、销毁 publisher；预检失败在创建 publisher 前退出。进程被 SIGKILL 或系统卡死不能保证 Python finally，故需独立控制器超时；发布零/超时均不是实测物理停止证明。

仅验证命令已运行成功，输出 `/private/tmp/vehicle-motion-calibration-review-20261112.validation.jsonl`：

```sh
.venv/bin/python tools/vehicle_motion_calibrate.py --plan /private/tmp/vehicle-motion-calibration-review-20261112.json --seconds 8 --output /private/tmp/vehicle-motion-calibration-review-20261112.validation.jsonl
```

现有模型 xacro 硬编码 `/drive/cmd_vel_unstamped:=/cmd_vel`，现有 world 名称也不是上述专用 world；因此**不能把该计划直接指向现有产品图运行**。需负责人审临时隔离 fixture：加载同车型 Gazebo 模型、专用空旷 world、只把其 drive 输入 remap 到上述专用 topic，保证没有 safety/Agent/Nav2、没有其他硬件或运动发布者，并确认所有 preflight 条件；该 fixture 尚未创建，本轮不通过改产品 xacro 绕过。初始低速完成后，5 km/h/正反/转弯/不同摩擦刺激必须各有新的受审有限计划与停车场地证据。

## 最小下一步（本轮未执行）

1. 负责人亲读这三工具、测试、YAML、计划依赖 diff 和上述临时刺激 plan；确认后安排后续有界运行，不再向用户重复询问已批准方案。
2. 保留已查 VM HEAD=d6696a1 的记录，以 VM 所有者 toby 继续只读核对工作区和安装包摘要，准备约定统一根下新 run 目录/锁与冻结源，核对 Python/ROS 安装对应本次源；现有 profile 指向旧位置，不可直接当同版新版。源与安装身份确定后，将私有 profile 引用新 frozen install，凭证保留私有，不覆盖原 profile 或历史材料。
3. 最小默认基线可按原 simulation profile 使用既有启动链，但先获得负责人这次有界启动指令，执行 `python -m senior_control.cli doctor --profile /home/toby/senior-sim/simulation.json`；若选择历史原版基线，单列其源/安装 SHA，不能叫新版同版验收。不要把 `config/simulation.yaml` 当私有 CLI profile。
4. 后续受控测试会话设置实际 `ROS_DOMAIN_ID`（原 profile 是73）、`ROS_LOCALHOST_ONLY=1`、独立 ROS_HOME/ROS_LOG_DIR，持有统一根 `.vm-test.lock`；finally/trap 执行对应 stop，单次≤180 s 并回收本次进程组。

```bash
# 仅供负责人后续指令使用；先验证 VM toby 的 Python、profile、域与安装一致。
python -m senior_control.cli start --profile "$VEHICLE_PROFILE" --headless true
# 受控独立终端同时启动 capture 与测试；必须在同一运行窗口，不是顺序先采后跑。
python tools/vehicle_motion_capture.py --socket "$VEHICLE_SOCKET" --seconds 180 --output "$RUN_ROOT/default.jsonl"
SENIOR_SIM_NAV_SOCKET="$VEHICLE_SOCKET" python -m pytest tests/ros/test_simulation_manual.py tests/ros/test_simulation_navigation.py -vv --junitxml="$RUN_ROOT/default.xml"
python tools/vehicle_motion_analyze.py --input "$RUN_ROOT/default.jsonl" --output "$RUN_ROOT/default-analysis.json"
python -m senior_control.cli stop --profile "$VEHICLE_PROFILE"
```

真实 ROS 缺流/对齐诊断缺字段时记录缺口而非通过；需要原生记录同时执行有界 `ros2 bag record`，具体话题见总计划。校准专用 fixture/审核完成后才可运行下列命令，计划摘要必须与受审文件一致；运行前需额外留出 preflight 时间或减少段时长，8 s alarm 不会为了等待环境自动延长：

```bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python tools/vehicle_motion_calibrate.py --plan "$REVIEWED_PLAN" --seconds 8 --output "$RUN_ROOT/calibration.jsonl" --execute --reviewed-sha256 ac219e8245d9f64709b9adeb1974058bab5301114ddb383489e5c7daf38aba7f
```

## 未冻结与审核门

acceptance YAML 中所有 measured_bounds 为 null，freeze 的审阅者/时间/源与安装/web 摘要/原始证据/适用范围/负对照/留出数据均未填。实际 actuator 限幅/加减速/jerk/滞后、测量误差与静止窗、最高速度稳态、停止位移/场地、故障检测窗、手动/感知/许可时效、控制预算分配和导航容差仍须测量；不能用本工具的诊断常数或校准命令 ramp 冒充响应模型。预算冻结不要求先写完新核心；新快路径/PI/恢复行为由 T4/T5 在其所属阶段满足冻结预算。

ROS 真实运行与 VM 安装包对应源身份、专用 fixture 仍未验证；静态检查/本地 fake port 测试只能说明工具逻辑。负责人实际 diff 审阅后再安排采集/刺激；无通过数据不冻结，不把本子阶段完成写成 T1/首版完成。

最终验证：新工具 31 项与现有定向回归联合 **167 passed in 3.52s**（以本轮实际工具输出时间为准）；Ruff 通过，Mypy 在显式忽略缺失 ROS 库类型的模式下 3 文件通过，`git diff --check` 无错误。作者自审已检查退出路径、来源时间、窗口负对照与无控制源码修改，独立负责人实际 diff 审阅仍待执行；本记录不把作者自审冒充独立架构审核。


## 后续审查与实际运行（当前派发）

四条审查修复及默认栈实际运行已完成，详见 `docs/testing/2026-11-12-vehicle-motion-t1-review-baseline.md`。按授权新建统一审计根及三个冻结 run，原先“目录不存在/未运行”仅代表上文早期探测时点；r1/r2 环境与工具失败均保留，r3 两场 adapter/全部必需流可用、capture exit0，旧手动测试 motion_disabled、导航首点 failed，不能声明可驾驶通过。最新专用 fixture 在 VM r3/calibration-fixture-reviewed，已静态展开/编译、未启动/未刺激；29个实测字段继续未冻结。

后续取证：`2026-11-13-vehicle-motion-low-calibration.md` 已记录既有VM的两次固定+.3m/s隔离校准、原生bag/时钟/物理停车与旧r3原因定位；本文件为早期环境/工具基线历史，未测29项边界继续null，未以低速样本替代5km/h或导航验收。
