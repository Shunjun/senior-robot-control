# T1 扩展隔离测量：低档、倒车、转向与修正身份的 manual

状态：低档、倒车、转向各两次正常测量完成；高档按场地测量条件失败停止，未运行；身份/急停状态一致的旧 manual 真实通过。已把有依据的数值写入 `config/vehicle-motion-acceptance.yaml.candidates` 七组候选；顶层仍 unfrozen、29项 measured_bounds 仍 null、freeze批准仍空，T1整体未完成。没有修改产品控制链、旧ROS测试、T2a文件或进行提交/推送，不声明现有产品5km/h闭环通过。

## 版本、隔离与固定输入

- VM原件：`/home/toby/senior-audit-20261031/calibration-expanded-20261113-r2`（下称 `$R`）；宿主镜像：`/Users/zhangshunjun/Library/Application Support/orca/chat-scratch/32583a56faed28ac0328929612a923c8/t1-expanded-calibration`（下称 `$E`）。r1只是JSON tuple/list比较导致的静态准备失败，未启动；历史r3/低速run和原profile不覆盖。
- 产品沿用 `motion-20261112-r3/src`、`install`、`.venv/bin/python`：HEAD `4dd9da0212f0faa759d495af571056a9b5ec2992` **加原未提交快照**。完整564文件source manifest SHA `6f8e9971f7d9778cfa292d2e0d3114f05635a8ef4b8d447b883b7d4bf854c4ef`；1004文件install manifest SHA `7ac9b6b2be894d1261fe3c72bec822d36d3d278584be5015c21866c82df7219b`。并行T2a未进入这个运行镜像。
- Linux以已审 generator SHA `1c5440b605efdbc9e8ad868923f03d2883240f6472d69c6415b8ff676cdb2e40` 新生成四fixture；运行前逐项验证manifest文件SHA，并对宿主已审模板做XML/文本、YAML值、launch AST和shell token语义比较，只归一化source/output根路径；所有七文件、非路径元数据一致，记录 `$E/vm-preparation.json`。不因路径替换再改变模型或参数。
- 正常运行的校准器 SHA `c7ef918fe9977e04db4a8326b98129625678ba59a0524eb87461fe9242b00765`；capture SHA `c15719040cdcf5b67e6909643d7c02472564f0fd3196c56c87c500afc76f4f1a`；`frozen-tools-v2` 逐项校验，r2初始工具版本和失败记录保留。capture的git revision为空，因为冻结目录无.git；版本依据上述manifest，不把空revision冒充HEAD证明。
- 独占 `.vm-test.lock`，校准ROS173/localhost、各run_id独立IGN/GZ partition；每次新世界从原点静止，观测type/源/控制器参数后才启动校准。单unit175s+5s stop，calibration外部timeout为18/14/15/22s、kill-after1，内部work16/12/13/20s，未扩大；finally仅回收本次进程组/cgroup。mu=mu2=1、质量/惯量来自当前senior_4wd_dl模型，非真车地面或量产固件资格。

| 固定plan | SHA256 | 请求与分段 | 正常记录 |
| --- | --- | --- | --- |
| low_forward | 8fc51752bfce4f5a3337b2e09df2e9f3e8cc2b1962d56c655340549363ec13a2 | 2s零/6s +5/9m/s/4s零，a=.3 | low_forward-4、5，各240 regular+1 cleanup |
| reverse | 859543aacc703b80a0dd9dc9cd35046e034122c7a8fcf7ae443292d6d5291d15 | 2s零/3s −.3m/s/3s零，a=.3 | reverse-1、2，各160+1 |
| turn | 48753d83031312c825358036d263c290238bbd1e90ecd5aa51f4fc41831b7a9e | 2s零/4s (.25m/s,+.5rad/s)/3s零，a=.3、yaw a=.5 | turn-1、3，各180+1 |
| high_forward | 9ab74ccdc9669ac5b0fab494f90742bbc34a9865ec4ddb27813722d3411f04e7 | 原2/7/6s、25/18m/s计划保留 | **未执行**：低档/倒车走廊激光不可观测，按本次授权条件停止依赖高档 |

四个Linux fixture manifest SHA依次为 `9b45d8b540aa435af73a7bd49f0c08fd8c836f852ee519194668e892fbf726ef`、`1ffc10db269fc39e0069b328ca60452364a43e27403593f69dea525e984307f7`、`4937770909f06ceb6acdcb5835df4237f7fe09d879b51234d94fbdbd93a47ac4`、`5f932734d80fbb30be97500a31244138bfdddb13d7930302badb3994ea50a3db`。扩大matcher搜索速度是这些fixture的测量条件，产品simulation_motion.yaml未变。

## 实际运动与停止（候选，非性能边界）

所有正常calibrator=0、native bag=0；standard capture=2，明确缺产品IPC/motion/safety和`/cmd_vel`，dedicated cmd由bag独立记录。普通轨迹已归零并保持后才cleanup，没有依赖异常zero完成常规减速；倒车从静止开始，没有反向制动。

| run | 含准备work(s) | 稳态truth vx/vy/wz | 同窗wheel vx/wz | joint峰值(rad/s) | bag内laser有效率 |
| --- | --- | --- | --- | --- | --- |
| low_forward-4 | 14.576 | .555556 /≈0/≈0 | .555556/≈0 | 7.30994 | 0/353 |
| low_forward-5 | 14.772 | .555556 /≈0/≈0 | .555556/≈0 | 7.30994 | 0/338 |
| reverse-1 | 10.494 | −.300000 /≈0/≈0 | −.300000/≈0 | 3.94737 | 0/307 |
| reverse-2 | 10.604 | −.300000 /≈0/≈0 | −.300000/≈0 | 3.94737 | 0/301 |
| turn-1 | 11.478 | .246852 /−.018057/.294781 | .250000/.500000 | 4.87222 | 319/319 |
| turn-3 | 11.552 | .246869 /−.018041/.294754 | .250000/.500000 | 4.87222 | 307/307 |

稳态窗口由native命令到达固定目标后+.25s至最后一个该目标采样定义，分别为ROS8.888→12.770、9.410→13.176、10.362→12.115、7.800→9.546、8.876→11.615、10.472→13.219；`measurements.json`保存样本数/范围/均值，而非只报一个点。理想仿真直行误差的浮点级数值不构成通用误差界。转向IMU同窗均值 .294701/.294675rad/s，318/306个同scan区间积分对真值Δyaw最大差 .0002566/.0003023rad，支持车体角响应明显低于轮运动学的事实，不能单凭wheel/scan差异宣称电机故障；地面接触、四轮滑移和车身响应需按模型范围解释。

| run | 首个正常零命令bag ROS(s) | 该时刻插值truth speed/wz | 后续首个近零样本(s) |
| --- | --- | --- | --- |
| low_forward-4 /5 | 14.686 /14.909 | .000389/≈0；.000306/≈0 | 14.700 /14.920 |
| reverse-1 /2 | 13.119 /10.555 | .015/≈0；.015/≈0 | 13.140 /10.580 |
| turn-1 /3 | 12.617 /14.225 | .002882/.017596；.002182/.013234 | 12.640 /14.240 |

“近零”仅用于描述源曲线：speed≤.001m/s、|wz|≤.001rad/s且后续保持≥.5s；同时保留.01/.005截面，不把它们写成接受线。truth为50Hz，Twist无header、bag是recorder最近ROS clock的接收标记，不是执行器应用时间；上述间隔不能冻结毫秒级lag，浮点微小coast也不能作为普适停距。`.2 ROS秒`窗口观测dvx/dt范围：低档约[-.300,+.375]/[-.525,+.450]、倒车[-.300,+.300]/[-.375,+.300]m/s²；这些是steady ramp、调度与仿真倍率共同作用下的**观测斜率**，不是最大加速或保守制动能力，不把命令a=.3冒充实测上限。

## 激光第一拒绝环节、reset与采集缺口

**走廊失败有数值依据。** low_forward-4的353份fenced filtered scan上游均有效，matcher全部在`scan_matcher.cpp:observable()`拒绝；直接重放相同beam/固定参数的point-to-line信息判据：第一帧ROS2.3有效131 rays（已超过100），仅14个连续9-ray平面窗口（要求≥25），平移信息最小特征值 .0001526（要求≥.05）。全部353帧的窗口数和最小特征值都不满足原要求，yaw独立信息并非失败项；不是缺流、低速未达目标或看到一条文本就归因。`geometry-audit.json`逐帧保存raw sequence和数值。因此低档/倒车的scan→truth误差是不可测，不填0；相同走廊高档不执行，也不降低min_rays/observability或扩大gate/cap。

**转向只比较同一ROS区间。** scan和truth pose分别以SE2 log/dt取得区间平均body twist；truth端点插值仅在一个连续源run内，间隔/倒序/同stamp重置有歧义时拒绝，不跨producer epoch或缺端点拼接。运动帧49/49个，|vx误差|均值 .09709/.10518、最大 .24969/.28846m/s；|wz误差|最大 .08510/.08832rad/s。这不是当前速度误差，更不能用轮速度作车身真值。固定离线选项将连续同epoch delta先SE2复合，再比较相同整体区间：.4s窗口最大vx误差 .08654/.08674，.5s为 .07573/.07233m/s，仍高于计划预定的 .06556m/s测量预算要求；不能改用较好p95或测试后放宽预算。窗口中心另增加.2/.25s滞后，所以也不能把平均速度当当前速度。

**本批reset第一可见原因。** 正常六场运动采样内没有scan producer epoch重置；manual-2/3的native链在启动ROS.002s先出现`/safety/raw/scan.capture.valid=false, reason=raw_source_changed`（manual-2 capture generation3、sequence1、有实际raw/bridge GID和boot），之后初始化并持续qualified。对应`scan_receipt`首次raw source发现/代际失效，再传播到scan adapter/relay/matcher；不是运动中车体停住或匹配误差锁存。manual-3 1200/1201 qualified，manual-2 1301/1302，唯一native无效motion样本为initializing。前次review1 ROS11.5的`raw_scan_invalid`在本批未重现，不能声称已解释它的确切admission子判据；此前缺adapter当时now_ros/steady等值，仍需最小诊断字段区分负ROS差、过期和身份变化，不新增资格机制。

**订阅成功不等于收到数据。** 已现场核对新增四topic的真实类型分别为BoundedRawLaserScan、BoundedFilteredScan、FencedFilteredScan、DiagnosticArray，固定recorder只读角色新增这四项，任何发布/额外topic仍拒绝。native实际记录中：low_forward-4和manual-2/3四流均有数据；low_forward-5、reverse-1/2、turn-1的raw/fenced流为0，turn-3 fenced为0（其他三流有数据）。turn-3显式best_effort/volatile覆盖后仍出现fenced0，ready-qos记录publisher和recorder subscriber均reliability2；不能归因为已证QoS不匹配或伪称已修复。原始scan、candidate、motion scan、truth/IMU/wheel及dedicated cmd均保留，缺流在export/counts中明确，不以重跑第三次正常场景掩盖。后续最小采集改进应在已有准备时限内确认native各必需观测流确实入库，再进行运动；不把这种测量准备检查接入产品gate。

原生中值周期：clock .001、scan/motion-scan .1、truth/IMU .02、wheel .03（最大.04）、joint .01 ROS秒；clock有.045/.090等接收缺口，其余所列源周期最大值与原生计数见候选组。工作内clock receive age最大.0545 steady秒，其余场更低，原.25检查未放宽。standard capture吞吐受有界spin/QoS影响，只用于收到的样本与缺流，不当发布频率证明。

## manual：完整身份一致后真实接受、运动、停止

旧r3 wrong-robot→motion_disabled证据仍是历史**环境身份错误**，不作为本次控制拒绝。新profile只连接未服务的本机wss://127.0.0.1:9，ROS174/新partition、独立database/socket/estop，未改测试认证断言或产品gate。

准备过程照存：首次脚本在启动后因tools导入路径缺失退出，未运行测试；manual-2的credential已是route-sim，但我复制的estop仍为旧ID，`PersistentEstop`按设计将foreign state加载为latched=True（原文件latched=False、identity_matches=False，文件SHA前后不变）。它因estop未进入acquire，1 failed/2 passed且车体静止，不是产品身份校验失败。随后在**全新manual-state-3**用产品`PersistentEstop.initialize(...,'route-sim')`创建新的未锁存状态，保留全部旧状态，绝不清除既有急停。保存`initial-estop.json`和actual IPC identity校验（ok、domain174）。

有效环境下仅一次旧manual完整执行：**3 passed in7.27s**，capture=0、bag=0，整个有界观测151.824s。旧测试的control.acquire、30条drive.velocity和drive.stop均通过原ok断言；90个native非零输出，最高+.08m/s，truth峰值+.080000m/s，真实Δx=.118400m。测试观测输出零延迟4.1ms、车体近零首样本延迟19.0ms、保持.220s、truth周期20ms；native零命令ROS4.011时truth仍约.08m/s，4.040s后近零，证明零输出与物理静止有区别。此为旧低速接口的有效回归，不覆盖新authority/TTL、网页/云、5km/h或导航；导航不重跑，沿用前报告ROS5.8→5.9 turn_stall及wheel/scan/IMU同窗证据。

## 原始材料、候选范围与下一步

每个 `$R/runs/<case>` 对应 `$E/vm-<case>`，manual为 `$R/manual-3` / `$E/vm-manual-3`；bag/JSONL、ready graph/QoS、控制参数、run记录和原生导出/逐文件SHA同存。native行数与metadata一致，宿主传输文件逐项核对VM index。`$E/measurements.json` SHA `aa910bc68646dcb0c5eb6d839afce61014a47218bac9ad05c8ba5eb6f6181d6d`；manual-metrics SHA `135b8b119d9440be631c637358b5719871869dd091d3c361b419b36f482dcadc`；geometry-audit SHA `107e84e9d5687db8d3c00c39d756e9c1cea59abf32e6cbb88abd18963ca63024`。七份`*-curves.csv`提供SI/源ROS时间曲线，scan为区间平均body log速度；unheadered command列明确是receipt时间。

| run | native行数 | bag/bag_0.db3 SHA256 |
| --- | --- | --- |
| low_forward-4 | 45762 | 3b6850e8d72d7284c5c83368d6d3ed7cc1a39efc95d32d700ac3c5eefa8ee9ec |
| low_forward-5 | 43571 | ccaf9f7d615db172e04d639f63e7cdcafc9e4cb3e0e2ff3f15c336d3948348c4 |
| reverse-1 | 39815 | db902fd16c6d7d572b8afb99157695238557ba16544533a643d6fd622c520dd3 |
| reverse-2 | 38425 | 66ef4e2cb4d99e7945491e28883f794e2c37042ca61170e8723d459dc63f5e3b |
| turn-1 | 40698 | f46f5661be04f501d0592b4f4cde09da89be0923ab52c59da63b6c4a31d943b9 |
| turn-3 | 39693 | a9a66672fb7c3643964f911b6ac30e13859bdf7ee32a55800d84d9715630ab8f |
| manual-3 | 150075 | a74eb956afbac995ec16cbd0cba4f809178746495b1b00f5b907eeb76f94104e |

`candidates`分组包含两次独立run的直行/倒车/转向稳态、joint反馈、源周期、工作内观测年龄、零后body曲线与scan误差，另保留失败场地和有效旧manual组。它们只是该mu=1车型/速度方向范围的观测候选，不填充或批准顶层冻结值；真车/摩擦变化、满档/倒车5/9、负yaw/原地转向/1rad/s、故障制动、延迟/滑移正负刺激、Nav2、控制tick预算与新闭环性能尚未测。T2a可按已授权并行纯contracts/manual_session并显式注入TTL，本任务不评定其完成状态；未冻结前不接线，T3–T5仍不开始控制实现。

最小后续建议：

1. 先修**测量场地**，不降匹配门槛。已准备静态候选 `/private/tmp/vehicle-motion-corridor-observability-20261113/{low_forward,high_forward}-candidate-world.sdf`、manifest和worlds.patch：保留原world物理/端墙/障碍，新增±5.3m侧面的3×.2×2m、±45°长面，x间距6m；最近y边4.169m，减1.05倍车身外接圆后仍约3.71m留空。理想2D720-ray审查覆盖低档x±10/高档±30m、.5m步长，含1cm距离量化时最少35平面窗口、最小平移信息.2284，均高于既有25/.05；这是静态raycast选型，**没有执行Gazebo或证明Karto/运动质量**。world SHA分别 `1a61a757a1d6aa6c0badc8e733011b85d3fe092b7cd9efbdbfd5b4216587e974`、`47cdf5be42009d10c317afdbbe9b64f14d11f26258230e611dcf6cc91deb13ab`，需要负责人亲审后另派实际场地验证/高档，不自动运行。
2. 再做估计最小改进验证：当前2cm平移/.02rad角步导致短区间差分抖动，单纯拉长平均仍不满足预定误差预算；评估受限的subgrid/局部细化与固定窗口body-pose拟合，保留producer连续性、区间平均→当前上界及年龄/协方差账本，不用轮速代替激光，不因valid/高response而忽略误差。新增有限数值失效原因字段才能解释旧r7精确admission分支，不先增加更多资格层。
3. 负责人亲读实际diff、原始曲线和候选，决定哪些范围可冻结以及尚需负对照；不因本次低档/legacy成功就解除高档/整版验收门。

## 工具/准备修复与复核

本轮产品外的最小代码diff仅为校准recorder允许四个已核对topic的**订阅**，以及`GraphValidationError`保留拒绝时的原始只读图；两项均有先失败后通过的定向测试，额外topic、任何诊断/actuator发布、重复drive等回归保留，未改变资格条件。准备失败不混入正常测量：low_forward-1因虚拟IPC路径104字节超过既有100限制而未创建calibrator；low_forward-2在publisher创建前被完整图检查拒绝，旧版本未保存失败快照，原因不猜测；独立low_forward-debug3按原检查通过后故意在创建publisher前退出。turn-2因drive参数service尚未被新context发现而无stimulus；turn-3在原55s启动/10s观测准备限内先实际读取drive参数，仍保留calibrator内部原.5s服务检查和全部原限制，正常重复一次。没有第三次正常测量或为了凑成功改plan/ramp/时限。

验证命令：

```bash
.venv/bin/python -m pytest tests/unit/test_vehicle_motion_{analyze,calibrate,fixture}.py -q
.venv/bin/python -m ruff check tools/vehicle_motion_{capture,analyze,calibrate,fixture}.py tests/unit/test_vehicle_motion_{analyze,calibrate,fixture}.py
.venv/bin/python -m mypy --explicit-package-bases tools/vehicle_motion_{capture,analyze,calibrate,fixture}.py --follow-imports=silent --ignore-missing-imports
python "$E/measure.py" low_forward-4 low_forward-5 reverse-1 reverse-2 turn-1 turn-3
python "$E/manual-metrics.py"
python "$E/temporal-options.py"
```

离线脚本只复算导出，首次导出原始bag是Linux的`export.py`，以rosbag2_py只读转换为gzip JSONL，保留全部字段与非有限值字符串，逐次核对metadata总数；`measure.py`/`imu-check.py`处理源区间，`geometry-audit.py`重放既有可观测判据，`temporal-options.py`只比较固定离线选项，不改产品。已用解析常twist弧线、反向直线、ROS同stamp重置/不完整新run及源间隙负对照检查SE2与区间方法。验收候选是描述性数据，不通过选择这些离线选项改变原接受标准。后续运行禁止直接复用已存在的run目录，所有脚本/输入/输出SHA和明确缺流随材料提交负责人审核。

最终复核：54项定向单测通过，Ruff/Mypy通过；`git diff --check`及本轮untracked文件逐项空白检查通过，8个既有tracked脏文件与原快照SHA全部一致。`frozen-identity.json`再次核实564源码/1004安装文件均无变化，实际arbiter/adapter/lifecycle导入路径在r3 frozen src/install；`cleanup.json`证实13个本次unit全部inactive/dead、ControlGroup为空，ROS173/174或本次partition遗留进程0。`$R/analysis/sha256.json`（宿主`$E/analysis-sha256.json`）SHA `77eb9ab537c4aa635cdbf0f742d33b94f3302b1f14c3ce2797608b681ad0c728`绑定所存数值、候选、原始实际patch和复算脚本；报告本体以当前仓库文件为准。当前分支与Draft PR #14未提交/推送或合并；由负责人亲审工具/测试/YAML/报告实际diff、场地候选及原始数据后继续。
