# 原生年龄输入边界返修

审阅基线 `bc46ac32490fa6b8b20af99f1a8847975d58361d`，原 `fix/degraded-drive-speed` 工作区追加修复，保留前序提交。负责人已认可共同积分路径和完整deque消费修正；累计审阅继续，本次不能代表整体验收。

## 异常原因与最小修复

`originals_current()`原先只先排除了now<stamp，received>now在后方判断；调用Policy.age_ns时会先抛invalid_argument，无法走原candidate_invalid。最后输出前的年龄调用同样没有确定的输入异常边界。

- `sensor_timing::Policy::try_age_ns`返回optional并声明noexcept，统一执行原有时间顺序、finite、elapsed数值范围及饱和加法判定；没有catch(...)或catch(std::exception)。全部是无分配标量校验与计算，异常输入返回nullopt，不制造年龄0。原throwing age_ns API保留并标Compatible，配置构造异常仍正常抛出，未把任意程序错误转成输入拒绝。
- `RgbdCandidateReceipt::capture_age`集中ROS Header→纳秒转换及非法计数；originals_current和最后publish前都使用同一入口并先检查optional，再读年龄。orig非法仍false→candidate_invalid；最后非法明确candidate_invalid，合法但到期仍candidate_expired。来源/ledger/TF/时钟代次判断保持原样。
- AgeSummary新增饱和invalid_clock计数，非法输入不混入avg/max/count或伪造年龄；在既有默认5秒DEBUG汇总中输出，没有每帧新增日志。合法到期的真实年龄和over_hard仍正常保留。
- 软0.3秒/硬1秒和9e9秒elapsed数值范围沿用原实现，没有延长时效或增加绝对steady值门槛；同样有限的大steady epoch、差值为0的情形依旧可计算。

## 调用一致性审查

对仓库C++源码/头文件执行 `rg -n 'age_ns\(' ros2/src --glob '*.hpp' --glob '*.cpp'`，生产回调只有RGB-D这两个使用阶段；现在均经capture_age→try_age_ns。没有残留直接调用throwing age_ns的ROS回调；保留的throwing封装只供显式程序调用和已有纯helper兼容测试。完整输出已存verification JSON。

## 实际验证与未构建项

- 新纯C++输入矩阵先因缺少非抛出API编译失败，随后定向 `test_native_sensor_age_boundary.py` 与原有 `test_shared_sensor_cpp.py` **2 passed**。命令使用原共享venv并设PYTHONPATH=src，精确argv、源码指纹及编译器版本见 `2026-11-15-native-age-boundary-verification.json`。
- 新测试以C++17/Wall/Wextra/Werror及UBSan、float-cast-overflow、不可恢复sanitizer编译并实际执行。覆盖未来received_ros（stamp并不未来）、ROS/steady倒退、负/逆序输入、NaN/±inf、极端steady相减溢出/不可表示elapsed、原数值范围边界、饱和int64加法、.3/1秒边界、非法计数与无假年龄、兼容throwing API和计数饱和；原QueuePolicy/瞬时积压纯helper回归保留。
- ROS测试源码新增 `HeaderAgeBoundaryRejectsInputWithoutThrowingOrFakeAge`，通过既有friend peer调用生产capture_age，使用真实RawCaptureHeader类型和非法输入矩阵，核对无异常/无伪年龄/invalid计数及合法expired区别；相关ClockRollback测试补上原已要求的policy参数，否则默认-1会在构造阶段失败，无法测试clock回退。**这两项ROS测试源码未构建、未执行**，不能作为真实回调/DDS验收。
- 新Python测试Ruff check、format --check通过，git diff --check通过；没有重复跑本轮未影响的201项运动组、前端或全量mypy。

宿主无ROS underlay/colcon，未启动VM、服务或实车，完整生成消息/节点/guard编译链接及真实ROS回调仍缺证据，保留Draft。后续在明确可用的隔离Linux/Humble环境，应构建senior_motion_msgs和senior_sensor_ingress并运行test_rgbd_candidate_receipt等CTest；本机纯header测试不能替代。

未修改停车速度域、decel、自过滤或其他排队项，未改运行配置、main或推送/建PR。本次输入异常处理与软件验证完成，继续交负责人亲审新增diff及未完成的累计范围。
