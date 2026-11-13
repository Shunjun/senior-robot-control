# T1 审查修复与默认基线

状态：本次审查修复、默认同版基线实际运行取证和专用校准 fixture 静态审阅材料已完成；T1 整体仍未完成，实测边界未冻结，fixture 未启动、未执行 `calibration --execute`。本记录不是 5 km/h 验收，也不冻结响应边界。原设计/计划仍以用户已批准版本为准。

## 审查项与证据

| 项 | 修复 | 验证 |
| --- | --- | --- |
| 1：短零目标掩盖更早残留速度 | `command_schedule` 携带完整 vx/wz 状态逐段离散积分，validation 与执行共用同一 schedule；正常终态要求 regular zero 至少保持一个 tick | 给定 1s零/2s+.3/.01s零/.05s零用例在 vx/wz 两轴先失败后被拒绝；延长终段可正常结束且 cleanup 前已为零 |
| 1：时序误差与结束 | 每个 ramp 步均先等待最小 hold，不为赶时限跳过步；调度/检查超预算异常退出，不伪称普通减速完成 | bounded wait jitter 下命令斜率不超 accel/yaw_accel，regular 最终为零；原异常 zero/finally 回收回归保留 |
| 2：采集器/扫描节点误拒 | 明列 `vehicle_motion_capture`、`scan_receipt`、`scan_adapter`、`scan_motion_node` 观测角色；TF 内部 helper 仅允许固定命名模式且现场证明仅 `/tf`、`/tf_static` 等只读 topic 角色 | 采集器/扫描节点可入图；额外 actuator publisher/subscriber 与 CLI daemon 仍拒绝；没有无限通配白名单 |
| 3：producer epoch 被覆盖 | 原 `source_epoch` 保留，另记 producer_epoch 与 publisher_gid；窗口和区间组合键为二者 | 同 GID 换 producer 的窗口/scan 区间均中断；原始 producer 未被改写 |
| 4：区间拼接 | 复现无效记录前后被拼接，以及 ROS reset 后旧匹配 stamp 被用于新 scan；按连续 receipt run 独立积分，在 scan 接收时刻选最新新鲜 run，不回退旧完整 run | malformed row 保留 barrier；两次不完整 ROS 时间经过不能拼全区间，旧 stamp 匹配也不构成轮身故障证据 |

`capture.complete` 仍只表示必需流至少各收到一条，不能证明完整窗口/物理停车；summary/raw counts 不改成验收通过。

## 真实环境运行揭示的修复

- r1 发现 custom context 却调用 global executor，采集器未收到样本即退出；修为显式拥有 context/executor 并关闭回收，新增 binding-level 失败测试。
- r2 发现 Humble 标准 executor 丢弃第二个 callback metadata 参数。采集器复用已存在、带 `//Compatible` 的 `senior_robot.motion_lifecycle.MotionExecutor/with_dds_info`，不修改产品实现。Humble 可能不导出 per-sample publisher_gid：保持 null、明确未知，绝不拿当前 topology GID 冒充这条样本来源；有 metadata 时仍与 producer_epoch 共同划分连续性。
- r1 独立运行 Python 不见系统 lark；r2 已解决 lark，但缺仓库 safety extra 的 Shapely，ROS 自带 launch_testing 插件还导致 pytest 收集失败。r3 使用新 run 内 `--system-site-packages` venv，复用既有依赖并仅向该 venv 安装仓库约束的 NumPy<2/Shapely<3；pytest 显式关闭自动插件并加载 pytest_asyncio，不修改系统 Python、旧 venv 或产品参数。

## 冻结与运行目录

- VM：既有 Parallels Ubuntu22.04 ARM64，toby 现存环境；根 `/home/toby/senior-audit-20261031/`，统一 `.vm-test.lock` 串行。
- 新 run：`motion-20261112-r1`、`motion-20261112-r2`、`motion-20261112-r3`；旧 repo、旧 profile、历史验证目录保持原样。
- 每份快照有 `source-manifest.json`：HEAD=4dd9da0 完整值、564 个文件逐项 SHA、24 个未提交文件内容 SHA；原始归档不含 .git、node_modules、venv、秘密或无关构建产物。
- r1 archive SHA256 `5eeb6dc50f85e2c5a5e1da9fc7600489ba55e5be1cbfa18c5e6f70fc530ee552`。
- r2 archive SHA256 `36e22656bd32b7d939b7dc4f6b30f4f35ef2a12b074d034b10ea9cc0dba39d26`。
- r3 archive SHA256 `f5f2250a696b2edce37307416fc36c3a133b16b032b6e33527d677fb62c55c5a`。
- r1/r2 的 6 包 Release 原生构建、doctor 均通过；加载路径均实际指向各自 frozen `src/src/senior_control` 和 `install/senior_robot/...`。r1 95 s、r2 89 s；构建通过不等于运行依赖完整。
- 私有 profile 均复制到各 run 的 state，凭证只复制不输出，独立 DB/socket/estop/ROS 日志；cloud_url 强制为本机未服务 `wss://127.0.0.1:9/ws/agent`。不连接原云、不连真实车。ROS 域174每场前核查无占用，ROS_LOCALHOST_ONLY=1，IGN/GZ partition 随私有目录唯一。

## 默认基线结果

| run/场景 | 实际结果 | 耗时/清理 |
| --- | --- | --- |
| r1/manual | adapter 未可用；pytest exit1，capture exit1（context/executor）；尚未进入驾驶 | 42.98 s，独占 unit 已停，域174无遗留 |
| r1/navigation | adapter 未可用；pytest exit1，capture exit1；启动依赖异常，不是导航控制结论 | 55.92 s，unit 已停，域174无遗留 |
| r2/manual | adapter 未可用；pytest exit4（ROS pytest 插件收集），capture exit1（Humble metadata callback） | 57.25 s，unit 已停，域174无遗留 |
| r2/navigation | 同类环境/采集问题；缺 Shapely 导致 scan/fusion/gate/adapter 退出 | 43.65 s，unit 已停，域174无遗留 |

这些失败均保留 `capture-process.log`、`service.log`、`pytest.txt`、原生 bag、run.json 和内容 SHA；不把缺流数据或软件启动失败解释为物理停车通过。r3 已解决上述环境/采集问题；实际结果如下，旧失败不作为工具失败或产品通过。

### r3：默认栈实际取证结果

r3 新独立 venv：Python3.10.12、NumPy1.26.4、Shapely2.1.2、Pydantic2.14.1；原生 6 包构建约89 s，doctor 通过。所有564个冻结源文件事后校验无内容变化（source_mismatches=[]）；capture/analyze/calibrate 使用 r3 冻结版本。最后补充的 jitter 测试和 fixture 启动源绑定只影响本地验证/静态审阅，不改已运行产品链；新 generator 单独在 review-tools 归档并记 SHA。

| 场景 | 旧测试真实结果 | 采集与回收 |
| --- | --- | --- |
| manual | 1 failed / 2 passed，运动测试在请求处返回 `motion_disabled`，未进入有意义运动 | adapter 可用，capture exit0，所有必需流已收到；151.88s，stop exit0，域174无遗留 |
| navigation | 1 failed，进入航点0后任务终态 failed；MotionEvidence 后续大量为 `motion_anomaly` | adapter 可用，capture exit0；151.29s，stop exit0，域174无遗留 |

原生 bag 与 Python capture 同窗。manual：scan1357、motion1357、truth3548、wheel3543、IPC586；navigation：scan1344、motion1344、truth3558、wheel3558、IPC586。原生 bag 的 `/clock` 分别135912/134639条，truth6795/6732条，wheel4074/4039条，说明 Python callback 采集存在欠采样；complete=true 仅是流覆盖，传感频率/完整时序应以原生 bag 与源 stamp 核对，不能拿 callback 间隔当传感真实频率。

manual clock：145.633s steady / 135.674s ROS 推进；navigation：146.889s steady / 134.397s ROS 推进。manual 最大绝对命令vx为0，truth vx约3.1e-13m/s（数值静止）；navigation 最大绝对命令vx=.14m/s、wheel vx约.098m/s、truth vx约.06493m/s。各 peak 不保证同刻，不能直接相减当滑移/估计误差；没有达到0.3m/s或5km/h的声明。analyzer 两场 findings=[]，只表示其四类数值窗口未形成，不等于状态健康、物理停车通过或已经复现258ms唯一根因。数值 alignment/expiry admission counters 仍未导出。

两场所有 ROS callback 的 per-sample GID 在此 Humble 版本均缺失，保留null；producer epoch 原值保留。没有把 topology 查到的 endpoint 冒充样本GID。该缺口须随数据审阅，不能据此冻结来源相关误差或新的控制放行条件。

### 原始证据与 SHA

VM run 根：`/home/toby/senior-audit-20261031/motion-20261112-r3/`。原始 JSONL 在 `manual/capture.jsonl`、`navigation/capture.jsonl`；原生 sqlite bag 与 metadata 在各场 `bag/`；测试/JUnit/service/run时间记录在各场目录，均保留。

| 证据 | SHA256 |
| --- | --- |
| source-manifest.json | `6f8e9971f7d9778cfa292d2e0d3114f05635a8ef4b8d447b883b7d4bf854c4ef` |
| install-sha256.txt（构建后安装文件清单） | `1f3a633ebc5d3cc1c912f859e45262f891b742293c59e2180b00faa05f2e1bb9` |
| installed-runtime-files.json（排除运行生成pyc） | `7ac9b6b2be894d1261fe3c72bec822d36d3d278584be5015c21866c82df7219b` |
| manual/capture.jsonl | `f87f271b6e48eeae33d040c57c06bb7b2247eb50207ce7f2503ac81ad2a69057` |
| navigation/capture.jsonl | `d7f1d0e0d50ef741d9f2c656c230667dc9be12de7a80dd8e83623118513a175b` |

完整逐文件 bag/log/fixture SHA 见 VM `review-evidence-sha256.json`；其内容摘要见下方交付索引。宿主审阅副本位于 Orca scratch `t1-review-r3/`：review-summary、review-evidence-sha256、两场 pytest/JUnit/analysis，以及 `vm-calibration-fixture-reviewed/` 实际生成物，不含私有 state/profile/凭证。冻结目录无 .git，capture 的 git revision 可为空，以外部 source manifest、tool SHA 和实际加载路径为身份依据。

每场默认服务有 RuntimeMaxSec=160、TimeoutStopSec=15、KillMode=control-group；采集与 bag 有自己的新进程组，finally 仅回收本场进程组与独占 unit，不按进程名称杀其他用户任务。外层 watchdog 与锁覆盖总执行；pytest 总剩余时限从场景开始计，超时记 `outer_timeout` 而非通过。没有改 gate/cap/扫描限值。

## 待审校准 fixture

源码：`tools/vehicle_motion_fixture.py`；固定低速 plan：`tests/fixtures/vehicle_motion_calibration/plan.json`（与先前 ac219e…aba7f 内容相同）；测试：`tests/unit/test_vehicle_motion_fixture.py`。

仅复用原车型 xacro 和 baseline world 生成新目录：独立 world=`vehicle_motion_calibration`、ROS 域173、IGN_PARTITION=GZ_PARTITION=`vehicle_motion_calibration_review1`；仅 remap `/drive/cmd_vel_unstamped` 至专用 `/vehicle_motion_calibration/review1/cmd_vel`，不改产品 xacro。轮/地面摩擦显式设 mu=mu2=1，是待验证的仿真假设；原质量/惯量/轮半径/轮距/关节能力保留。静态不对称墙面提供 scan 几何，生成时检查其距计划最坏位移与 3m guard；不是开放平地无限制场地。

生成 clock/truth/轮反馈/scan/IMU bridge、source 相同 scan receipt/adapter/matcher，不启动 Agent、Nav2、safety/command motion source；唯一运动发布者须为受审 calibrator，唯一 actuator subscriber 为 drive，观测节点不得新增 actuator 订阅者。标准产品 capture 的 required stream 包含 IPC、motion 与 `/cmd_vel`，这些在此专用 fixture 中可能缺失，必须按 summary 明列；不把 dedicated target 上的刺激记录伪装成产品完整链路。

本机先生成 `/private/tmp/vehicle-motion-fixture-review-20261112-r1/` 并语法编译；VM 实际审阅物为 `/home/toby/senior-audit-20261031/motion-20261112-r3/calibration-fixture-reviewed/`，已完成 xacro 静态展开和 launch Python 编译，**未启动 fixture、未执行刺激**。新增源绑定把 frozen source 放在 PYTHONPATH 首位，避免复用依赖时误载旧核心；运行依赖仍须使用下列已验证环境。生成文件含源 SHA/plan SHA/各文件 SHA、显式 physics 来源。运行命令须负责人审实际生成物后执行：

```sh
# 仅离线生成；现有 reviewed 目录不覆盖。实际生成物已经保留，负责人先审。
python -m tools.vehicle_motion_fixture --source-root "$FROZEN_SOURCE" --output "$RUN_ROOT/new-fixture" --plan "$FROZEN_SOURCE/tests/fixtures/vehicle_motion_calibration/plan.json"
# 以下为后续负责人审阅后执行，本轮未运行：
RUN_ROOT=/home/toby/senior-audit-20261031/motion-20261112-r3
source /opt/ros/humble/setup.bash
source "$RUN_ROOT/install/setup.bash"
export PYTHONPATH="$RUN_ROOT/src/src:$RUN_ROOT/.venv/lib/python3.10/site-packages:/home/toby/projects/senior-robot-control/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}:/usr/lib/python3/dist-packages"
# 独占 transient unit/cgroup，--wait 使锁覆盖完整运行；175s + 5s stop 上限。
FIXTURE_UNIT=senior-calib-fixture-r3-review1
flock -n /home/toby/senior-audit-20261031/.vm-test.lock systemd-run --user --wait --collect --unit="$FIXTURE_UNIT" --property=RuntimeMaxSec=175 --property=TimeoutStopSec=5 --property=KillMode=control-group --property=UMask=0077 --setenv=PYTHONPATH="$PYTHONPATH" /bin/bash "$RUN_ROOT/calibration-fixture-reviewed/start.sh" "$RUN_ROOT/install/setup.bash"
# 仅对本次成功创建的 unit 提前停止；cgroup 回收其子进程，不按名称杀其他 Gazebo。
systemctl --user stop "$FIXTURE_UNIT.service"
# 核对该 unit/cgroup 已退出及域173无本次遗留，保留该次 journal 与场景日志。
```

reviewed generator SHA256：`b50fa3cf4c1ff6d84cadc0817ceb8702278ed25f0cd1006d686b866caa474853`；实际 manifest SHA256：`daf50ffeb744ed489334b084b500fc171bda3f5dfa0239bcaa908449593517a6`；展开 URDF SHA256：`313701e458a46fb336361400de0783dc3bc28e767ea0d1afe3e3344c01addeeb`。source model/physics 与每项实际生成文件 SHA 在该 manifest；CLI 默认校验低速 plan 的授权摘要仍是 ac219e…aba7f。这里没有新增 calibration --execute 运行授权，须负责人先审实际 fixture。


## 未冻结项

`config/vehicle-motion-acceptance.yaml` 的29个实测字段仍为 null；源/安装摘要只证明运行身份，不证明响应、5 km/h、停距、故障阈值或控制预算已达标。数值对齐等待/过期饥饿诊断仍缺产品导出字段，记录缺口不凭文本编造。专用 fixture/实际校准还待审，在线自适应与真车验证仍为独立后续。

## 最终本地验证与交付门

`.venv/bin/python -m pytest tests/unit/test_vehicle_motion_analyze.py tests/unit/test_vehicle_motion_calibrate.py tests/unit/test_vehicle_motion_fixture.py -q`：43 passed；Ruff 四工具/三测试文件通过；`.venv/bin/python -m mypy --explicit-package-bases tools/vehicle_motion_{capture,analyze,calibrate,fixture}.py --follow-imports=silent --ignore-missing-imports` 四工具通过（Mac 缺 ROS 类型库，不当成 ROS运行证明）。真实 Humble capture 已通过两场150秒窗口。未提交/推送，未修改产品控制源码或其他既有脏文件。

本次作者自审已检查四条审查项、异常与普通终态、来源/时钟/缺流语义、源码绑定与场景回收；负责人仍需亲读实际修复 diff 和实际生成 fixture。下一步先审该 fixture，再派发独立低速校准，定位 manual 的 motion_disabled 生命周期与 navigation 的 motion_anomaly，取得正刺激/负对照/响应数据后才冻结边界；不能通过改 gate/cap 或延长超时制造通过。

完整交付索引 review-evidence-sha256.json SHA256：`96c0ee767c50426c4219a0af3d457c5ae94a19996efe606c777b807c99a5e23e`。

## 2026-11-13 后续（不改写上述历史运行）

固定+.3m/s fixture现已按负责人授权完成两次独立正常减速/物理停车取证；完整结果、失败准备样本、源/安装验证、原生bag SHA、清理和下一批未执行材料见 `2026-11-13-vehicle-motion-low-calibration.md`。本记录此前“fixture未启动/未execute”是当时子阶段状态，已由该后续推进，但T1整体未完成且29个边界仍null。

r3 manual的motion_disabled现已定位：测试route-sim身份与私有profile副本不符，AdapterCore `_reenable` 在 `validate_envelope` 的wrong-robot分支拒绝，尚未进入Arbiter；此前epoch/lease等候选不是已证原因。navigation首latch是ROS5.8→5.9累计turn_stall，同窗wheel yaw=-.0226667、truth=-.00130549、IMU=-.00129417、valid scan=0rad；这是轮/身差异与scan量化解释的证据，不能单凭日志归因地面摩擦或算法，也不把旧场景失败改记通过；未重跑两场150秒基线。
