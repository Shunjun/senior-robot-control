# 第8—9项实现与验证交付

授权范围的软件实现已完成，待负责人亲审后统一交付原Draft PR26；未提交、推送或部署，未启动服务、ROS、仿真或车辆。第10项心跳、第11项隔离及毫秒迁移没有混入。

证据按节点最后结果去重为1361通过、12明确缺证跳过，**不是单次全绿**。受影响消费者的134项补测及最终6项Nav2/仿真补丁与真实C++机制检查通过。完整mypy剩余6项均为HEAD同类基线错误；全仓Ruff的1项lint与2份格式问题属于禁止修改的独立文件，不能称CI通过。真实Linux IPC、完整Humble构建、DDS、插件ABI及实车行为均未验证，保持Draft部署前置。

固定Fortress的真实0→0 reset hook与多world隔离已经实施；最终说明见下方“最终增量”及[reset执行边界](../reviews/2026-11-16-fortress-reset-boundary.md)。前面的主体批次和失败修复记录保留历史属性，不与增量证据相加。自有整文件/共享独立hunk及保护文件、学习最终依赖指纹见本报告末尾交付清单。

工作区分支 `fix/degraded-drive-speed`，实际HEAD保持 `593e8f13f385892f0bd62cf75c593f7b07efd2da`。

## 环境与命令

macOS26.4 arm64，CPython3.12.15、pytest8.4.2、ruff0.17.3，Apple clang21.0.0；本机无ROS/Humble/colcon，未生成或加载目标插件。`cv2`不存在。Python/macOS monotonic不是本次Linux同域部署验证。

最终集中命令的114份文件名单为 `/tmp/control-clock-test-files.txt`，包含导航限速、core实际输出、所有变更时间消费者和学习冻结版本：

```sh
NAV2_SOURCE_CACHE=/tmp/nav2-1.1.20 \
SIM_CLOCK_SOURCE_CACHE=/tmp/simulation-clock-upstream/src \
.venv/bin/python -m pytest -q --tb=short \
  --junitxml=/tmp/control-clock-final.xml $(cat /tmp/control-clock-test-files.txt)
```

该单次结果 **1357 passed / 1 failed / 12 skipped，172.81秒**。唯一失败是绝对定位anchor早于首份运动测量的旧fixture，原断言要求把当前末速向过去未覆盖区间外推。生产端现在正确拒绝未知前缀；已在合成producer明确增加真实较早wheel端点，保持全部正向输出/来源/未知误差断言，并新增未知前缀仍拒绝的反例。只补测该文件：

```sh
.venv/bin/python -m pytest -q --tb=short \
  tests/unit/test_pose_space_estimation.py \
  --junitxml=/tmp/control-clock-pose-final.xml
```

结果 **8 passed，0.20秒**。按节点最后一次结果去重为1359通过/12跳过；这不是单次全绿。其后没有修改生产代码，未知前缀拒绝未放宽。

另因真实路径消费者修复，集中执行 `/tmp/control-clock-path-final.xml` 的52项，含学习逐段变速/转弯/缺段、完整学习cycle、core JSON链、仿真0.5x/1x/2x与绝对anchor记录路径，**52 passed，69.19秒**；不与上述总数累加。旧335导航证据、学习43项原因补测均保留历史属性，没有重复计入。

## 产品修复与fixture迁移的区别

真实产品回归修复包括：底盘有效包四元数积分使用原包单调dt；同包wheel/IMU保持不变前沿；配置校验沿唯一typed执行topic；RGBD原SE2元组矩阵访问；位姿查询不混host与物理坐标；按有界原始路径逐段消费而非末速乘累计覆盖；仅用真实新scan的物理tick推进物理外推、暂停不按host推进；未测横向分量沿原侧漂预算，不能误判整条线速度未知；旧点超过原deadline的路径段明确未知；原始source ns与receipt float按既有ns精度比较；拒收诊断保留原producer epoch/sequence，不能重复计同一原始根。

测试迁移包括：移除已退役wall-offset/ROS倒退撤销/未来样本队列断言，迁移同版typed消息、SourceTime与computed steady参数。历史JSON没有记录真实producer mono，不能制造新鲜度证据；它们只在fixture产生边界一次性声明明确的合成产生时间/物理坐标，接收端仍严格要求原始字段，不补盖时间。历史原件不变，不能把这些合成控制组合用例称为实测传输、采集时间或RTF通过。

覆盖有效超旧阈值线/角/组合/负方向、低速/零目标以及真实受限输出；系统前后校时、原始deadline、过期队列/重放/旧run/旧boot/来源身份/NaN拒绝、断流停车；仿真速率、暂停、非零world回退、同world producer重启、不同world不误对齐。**不包含尚未实现的0→0 reset执行通知。**

## Reception remap follow-up

The reception child now forwards typed sensor topic remaps. `tests/unit/test_control_process_resources.py` was verified separately after this constructor change: `/tmp/control-clock-reception-remap.xml`, 11 passed in 1.52s, including one new actual-constructor argument test with a fake Process (no ROS/process start). The final deduplicated evidence is therefore 1360 passing nodes and 12 skips, not one all-green batch.

## 独立C++、受控补丁与缺证

- exact生产native receiver和SourceTime/ActuatorAdmission代码已用真实C++编译器及确定性IO/时钟harness执行，验证原计算前沿、排队过期、顺序、boot/来源、零/EOF/NaN闭合；这不是ROS/DDS或Linux socket实测。
- 官方Nav2及仿真源码/补丁/C++检查：`/tmp/control-clock-final-overlay-context.xml` **5 passed，5.60秒**。官方固定版本、tag、commit及同版安装说明见[接入文档](../operations/control-clock-overlays.md)。Nav2 manifest为111原始文件/11被patch文件，包含必要controller/MPPI/AMCL；没有复制维护整个上游。
- 厂家只读资料默认3包及可选GPS/AHRS共5包、29被patch文件，`tools/prepare_sensor_overlay.py ... --optional gps --optional ahrs` 原文件SHA、patch SHA、零fuzz上下文和最终SHA全部通过，日志 `/tmp/control-clock-final-vendor-context.log`，未构建或部署。
- `compileall` 与本任务完整拥有的115份Python文件 ruff F检查通过；共享生命周期/第7项hunk没有整文件格式化。`git -c core.whitespace=cr-at-eol diff --check`通过。
- 12跳过：4组OpenCV依赖模块未收集；2项真实Linux学习memfd/pipe容量及spawn/SCM_RIGHTS服务链；6项真实Linux SOCK_SEQPACKET native输出。它们都不是通过。原native确定性harness一次5秒超时如实留在 `/tmp/control-clock-fixture-pass2.log`，原二进制直接复验及最终集中批次均通过，未通过改超时或产品逻辑掩盖。
- 完整Humble/C++消息生成/ament安装导出/真实DDS/插件ABI/真实串口写时界与实车行为均未验证。厂家driver_read之前设备/内核/SDK缓冲年龄未知；不能据此声称物理采集真时或硬件watchdog。

## 隔离与版本

三份独立禁改文件保持派发字节，SHA256：

| 文件 | SHA256 |
| --- | --- |
| native_receipt.py | 53d08230fb94dce972ed72dbf6fb16532d3b34aea4e5ed7191772a9bd0ef910a |
| tests/test_native_receipt_diagnostics.py | 5c33cba2555e0c0b9b0096a025620fbfdb9ef86ef03bfee53e485dc43c440408 |
| software-estop-owner.md | cf258266704168979f6485658e6c8e08d18bd444e6a1175cfc2f848ff5cab4d1 |

学习最新 `learning-recorded-path-fingerprints.json` own八文件全部逐字节不变；早期dual-clock manifest后续由原学习worker合法返修，不能要求它仍等于旧值。本轮共享Nav依赖的实际最终版本另锁定；学习动作/owner/3秒逻辑、第7项原hunk、清理worker监督边界均不列为本任务自有整文件改动。根表格只更新8—9项实施状态并保留用户原文。

## 最终增量：reset、共享链接与完整静态基线

负责人批准的最小core reset边界已完成，取代本报告前面“0→0待决策/未完成”状态。仅在实际requestedRewind分支通知共享world时钟；修正多ECM各自entity #1可能重复的真实world key问题。core构建闭包59文件/34源文件在外部准备，SDK和共享库精确依赖见最终接入文档，未复制整个engine。

新增真实共享库/三个DSO链接机制、公开符号与共享状态验证通过；覆盖0→0、非零→0单次递增、连续reset、并发reset、暂停、多world、producer独立身份。原始patch上下文及所有最终SHA重新验证：`/tmp/control-clock-all-overlays-final.xml` **6 passed，9.06秒**；厂家五包29文件的最终准备记录 `/tmp/control-clock-vendor-locked-final.log` 全部SHA/零fuzz通过。它们不是真实Humble/完整engine构建或目标加载证明。

为清除新增类型问题而补的类型注解、固定三轴tuple和steady字段类型，集中验证受影响消费者：`/tmp/control-clock-reset-and-types.xml` **134 passed，70.09秒**，包含学习冻结路径/完整cycle、core物理域、GPS、深度、语义、异步视觉、运动/位姿。学习own八文件仍逐字节冻结。各记录按节点最后一次结果去重为 **1361通过、12跳过**，不是单次全绿，不将重叠补测累加。

完整CI命令也已实际核对：

```sh
.venv/bin/python -m ruff check .
.venv/bin/python -m ruff format --check .
.venv/bin/python -m mypy --no-incremental src/senior_control
```

使用 `git archive HEAD` 的只读快照、同一venv/配置得到基线：Ruff lint为0；mypy有68项历史错误。当前完整mypy剩6项、4文件，全部与HEAD基线相同，本任务新增错误为0；当前全仓Ruff仅1项B905，格式检查仅2文件，均属于用户明确禁止修改的native_receipt.py和test_native_receipt_diagnostics.py独立改动。它们保持原SHA，未擅自修复。因此**不能称完整CI通过**；自身可修改范围无新增lint/格式/type问题。原始日志为 `/tmp/control-clock-head-{ruff,format,mypy}.log`、`/tmp/control-clock-ruff-final.json`、`/tmp/control-clock-format-final.log`、`/tmp/control-clock-mypy-final.log`。

最终自有patch为 `docs/testing/2026-11-16-control-clock-owned.diff`；整文件与共享独立hunk清单、三保护文件、学习最终own以及Nav依赖指纹为 `docs/testing/2026-11-16-control-clock-fingerprints.json`。core只列prefetch_scan/_observation/_snapshot、source_time导入和原tick computed参数；其他学习生命周期hunk保持原worker归属。real_runtime/scan_adapter增量与派发时保存内容比较，第7项原hunk未纳入；两个supervisor最终没有本任务导航增量，学习/清理改动全部排除。早期startup时间接口后来已由学习worker的同版实现接手，不将其最终整文件列为本任务所有。
