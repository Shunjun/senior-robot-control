# safety_input 等价耗时优化

基线：`feat/vehicle-architecture-refactor@38071850bc29d4157e33cc8fdad558998e63c800`。
依据：用户最新要求先测耗时，处理真实重复计算；新版 11-12 设计保留原始测量时间、来源、接收和消费时效，11-19 计划要求同输入同语义才复用。只做离线合成输入与纯 Python 回归，不运行服务、ROS、仿真、实车或部署。

## 顺序与设计

1. 本文先落盘，核对基线和现有纯 Python 用例；用主仓已有 `.venv/bin/python`、本工作树源码作前后穿插基准。
2. `scan_gap_compensation.py`：仅在 ranges 没有正无穷时跳过历史 seed 投影循环，因原循环仅将投影落到正无穷的 seed 加入 slots；仍执行本帧全部真实 seed 建立。保留 seed replace 老化、clear、回滚、同帧缓存、迟到姿态 retry、来源及时间语义。任一正无穷输入走原投影计算，不新增开关。
3. `safety_node.py`：复用 safety_scan_capture 返回的 capture；原 stamp 已与 body stamp 相等，使用 capture 的 stamp_ns；删除同次 helper 已证的 raw_source/frame 非空和 ingress finite 检查，及被当前 freshness.scan 的测量年龄蕴含的 receipt 年龄上限。保留 raw_source 与当前 source 相等、_scan_fence/sequence/source、msg.valid、空数组和有界上限、当前消费时效、时钟检查及失效清理。
4. 比较同机同输入下 baseline/candidate 的无缺口、含正无穷、迟到姿态重试、同帧缓存输出；核对实际 prediction index/distance/missing_frames/seed_capture 与原时间、原 message identity；运行已有前五个补偿纯 Python 节点和 freshness/egress 用例，Ruff 与 CRLF/diff 检查后只提交这三文件。

范围：产品修改 2、测试/测试数据 0、配置/构建 0、工具文档新增 1；合计 3 文件，无删除。seed acquisition 计数替换本片不实施，避免新增状态及等价证明负担。测试新增/改写均 0，已有用例仅复用。

## 命令与实际结果

实施后补记准确命令、输出、前后穿插数据及未验证项。离线测量不是 ROS/仿真/实车验收，也不能据此认定现场 native_receipt EOF/exit=2 的根因。

## 重复检查的逐项证明

`_on_scan` 先运行 `safety_scan_capture` 并要求返回非空，然后原样运行 `_scan_raw_admission.offer`；它们之后进入原 source/sequence 流程才到本次修改处。

| 删除/复用 | 已有保证 | 保留的不同语义 |
| --- | --- | --- |
| 再次 `capture_from_message` | 本回调上方 helper 已返回同一 raw_capture 的解析值 | RawCaptureAdmission 原序号/代次、当前准入不变 |
| 再次 `Time.from_msg(msg.stamp)` | helper 已要求 body stamp 与 capture.stamp_ns 相等 | 原始纳秒转秒、无重新盖章 |
| raw_source 非空 | helper 要求 capture.raw_gid 满足非零 GID 且等于 body.raw_source | 仍比较 msg.raw_source 与当前 _scan_source |
| frame_id 非空 | helper 的非空 _text(capture.frame_id,128) 且与 body.frame_id 相等 | 消息与坐标变换语义不改 |
| ingress_steady/ingress_ros 有限 | helper 比较它们与 capture 对应值；SampleTime 将原始字段限制为整数 int64/uint64 纳秒，转秒有限 | 当前 steady 有限性与 freshness.scan 继续检查 |
| receipt age ≤max | freshness.scan 要求 0≤measurement≤received≤now 且 now−measurement<max，故 0≤now−received<max | 不改变严格过期边界；freshness 当前时钟和递增测量判断、失效清理保留 |

保留空数组拒绝和 2048 上限，避免把本片扩成接收边界调整；保留 msg.valid、_scan_fence、sequence、source、全部 invalid 清理和控制输出门禁。无新兼容路径、开关或绕过。seed 的 replace 老化仍原样，未使用消息 sequence 差计算年龄。

## 实际验证（2026-11-21）

运行目录均为固定功能工作树；Python 为主仓已有环境，依赖由本工作树 pyproject 的 pythonpath 选择当前源码。基线为 3807185；候选为本提交两产品文件 diff。没有修改任何测试、配置或其他产品文件。

基线/候选共用准确回归命令（候选在产品修改后运行）：

```sh
PYTHONDONTWRITEBYTECODE=1 '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest -p no:cacheprovider -q \
  ros2/tests/test_scan_gap_compensation.py::test_internal_gap_follows_current_surface_and_real_return_replaces_it \
  ros2/tests/test_scan_gap_compensation.py::test_edge_includes_original_endpoint_without_crossing_a_break_or_extending_coverage \
  ros2/tests/test_scan_gap_compensation.py::test_three_new_acquisitions_expire_without_snapshot_or_prediction_renewal \
  ros2/tests/test_scan_gap_compensation.py::test_motion_and_new_grid_use_measurement_pose_with_nonblocking_same_frame_retry \
  ros2/tests/test_scan_gap_compensation.py::test_abnormal_ranges_are_never_gap_predictions \
  tests/unit/test_safety_freshness.py tests/unit/test_egress_receipt.py
```

结果：基线 **16 passed in 0.40s**，候选 **16 passed in 0.30s**。复用补偿5项、freshness2项、egress9项；覆盖内部/边沿缺口、三次真实 acquisition 老化、位姿/网格变化和迟到姿态同帧 retry、异常回波、原测量时效及 owner 撤销/恢复。新增/改写测试场景均0；没有修复已确认功能 bug，未新增实际 bug 定向回归。

基准准确命令：

```sh
PYTHONDONTWRITEBYTECODE=1 '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' /tmp/safety_input_efficiency_bench.py
```

临时基准脚本仅放 /tmp，源码附后便于复现；不作为产品或测试文件交付。动态载入 git show 3807185 的原补偿实现与候选实现，同进程同机器同输入对象，120帧×720束；4轮按 baseline/candidate、candidate/baseline 交替，每格为本轮每帧耗时中位数，最终再取4轮中位数；缓存每帧重复10次取单次耗时。输入为解析平面合成测距，绝非传感器实测、Gazebo、ROS或实车环境。late_pose 计时段是缺姿态首次调用之后、同帧补齐姿态的重试；缓存计时不含首次计算。

| 模式 | baseline 四轮中位 ms | candidate 四轮中位 ms | 最终 baseline → candidate ms |
| --- | --- | --- | --- |
| 无正无穷 | 4.486229 / 4.566896 / 4.538959 / 4.516917 | 3.040792 / 3.080959 / 3.039458 / 3.017188 | 4.527938 → 3.040125 |
| 含正无穷 | 4.334979 / 4.299583 / 4.304042 / 4.317750 | 4.345395 / 4.319438 / 4.368375 / 4.308979 | 4.310896 → 4.332417 |
| 迟到姿态重试 | 3.606688 / 3.762938 / 3.653813 / 3.603334 | 3.763729 / 3.798875 / 3.661229 / 3.587667 | 3.630250 → 3.712479 |
| 同帧缓存 | 0.003744 / 0.003719 / 0.003865 / 0.003781 | 0.003796 / 0.003775 / 0.003777 / 0.003758 | 0.003763 → 0.003776 |

无缺口每帧中位减少约32.86%，含正无穷约+0.50%、迟到姿态约+2.27%、缓存约+0.36%，后面三项不宣称提速或统计显著性。变化主要来自省略原本没有正无穷目标的 seed 逐点投影，仍保留约3ms真实 seed 构建和原老化成本；不能外推为整条 safety_input/控制节拍耗时或 native_receipt 故障修复。

一致性核对：同一批完整/正无穷输入共240个 acquisition，两实现各有缺姿态首次调用和补齐姿态 retry，再作同帧缓存调用；共480次缺姿态调用和480次缓存重复。严格比较实际预测 index/distance/missing_frames/完整 seed_capture（含原始采样时间和身份），并断言原 result.message is raw.message、stamp/received/received_ros/measurement_steady 不变，缓存返回同一结果对象；全部通过。已有5项补偿回归另验证运动姿态与网格变换等几何规则。

静态检查准确命令（基线和候选均通过）：

```sh
'/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/ruff' check --no-cache ros2/src/senior_perception/senior_perception/scan_gap_compensation.py ros2/src/senior_motion_control/senior_motion_control/safety_node.py
'/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/ruff' format --check --no-cache ros2/src/senior_perception/senior_perception/scan_gap_compensation.py ros2/src/senior_motion_control/senior_motion_control/safety_node.py
git diff --check
```

Ruff lint 输出 All checks passed，format 输出2 files already formatted，diff --check 无输出。最终提交前另逐文件检查 baseline/candidate 的 CRLF 数量，均为0，产品 diff 仅目标分支和重复检查删除，没有整文件换行改写。

限制：未启动或运行任何 ROS-only 用例、真实设备、仿真或服务；实际 SafetyGate 回调、DDS消息类型支持、ROS时钟/TF和进程调度的端到端行为未在本机验证，callback 变化目前由静态蕴含证明与已有 helper/freshness 回归支持。没有复现现场事故，不将 native_receipt age≈1.1s、EOF/exit=2 猜成某个唯一根因；该路径的队列、协议、阈值和诊断没有改动。

<details>
<summary>离线基准源码（恢复到 /tmp/safety_input_efficiency_bench.py 后执行上方命令）</summary>

```python
import importlib.util
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import time
import tomllib
import types

ROOT = Path("/Users/zhangshunjun/workspace/orca/senior-robot-control/vehicle-architecture-refactor")
config = tomllib.loads((ROOT / "pyproject.toml").read_text())
sys.path[:0] = [str(ROOT / p) for p in config["tool"]["pytest"]["ini_options"]["pythonpath"]]
from control_process_test_support import generic_motion_profile
from test_scan_gap_compensation import scan, poses

REL = "ros2/src/senior_perception/senior_perception/scan_gap_compensation.py"


def load(name, source):
    module = types.ModuleType(name)
    sys.modules[name] = module
    exec(compile(source, name, "exec"), module.__dict__)
    return module.ScanGapCompensator


base = load(
    "efficiency_baseline",
    subprocess.check_output(["git", "show", "3807185:" + REL], cwd=ROOT, text=True),
)
new = load("efficiency_candidate", (ROOT / REL).read_text())
profile = generic_motion_profile(length=0.55, width=0.55, sensor_error=0.05, decel=0.5, lag_ms=500)


def frames(gaps):
    rows = []
    for sequence in range(1, 121):
        ranges = [(1.0 - sequence * 0.0001) / math.cos(-0.36 + i * 0.001) for i in range(720)]
        if gaps and sequence > 1:
            ranges[360] = math.inf
        raw = scan(sequence, ranges, angle_min=-0.36)
        raw.message.angle_increment = 0.001
        rows.append((raw, poses(raw)))
    return rows


complete, holes = frames(False), frames(True)


def signature(result, raw):
    assert result.message is raw.message
    assert (result.stamp, result.received, result.received_ros, result.measurement_steady) == (
        raw.stamp,
        raw.received,
        raw.received_ros,
        raw.measurement_steady,
    )
    return tuple(
        (p.index, p.distance, p.missing_frames, p.seed_capture) for p in result.predictions
    )


# Same frame retry, late poses, and finite/gap inputs use identical objects for both versions.
for rows in (complete, holes):
    engines = [base(profile), new(profile)]
    for raw, aligned in rows:
        assert engines[0].update(raw, [], now=raw.received) is raw
        assert engines[1].update(raw, [], now=raw.received) is raw
        outputs = [e.update(raw, aligned, now=raw.received + 0.02) for e in engines]
        assert signature(outputs[0], raw) == signature(outputs[1], raw)
        for engine, output in zip(engines, outputs):
            assert engine.update(raw, [], now=raw.received + 0.03) is output
print(
    "equivalence: 240 acquisitions, 480 missing-pose retries and 480 cached repeats; identical exact prediction fields, capture, times and original message identity"
)


def run(cls, rows, mode):
    engine = cls(profile)
    elapsed = []
    for raw, aligned in rows:
        if mode == "late_pose":
            assert engine.update(raw, [], now=raw.received) is raw
        if mode == "cached":
            result = engine.update(raw, aligned, now=raw.received)
            start = time.perf_counter_ns()
            for _ in range(10):
                assert engine.update(raw, [], now=raw.received + 0.001) is result
            elapsed.append((time.perf_counter_ns() - start) / 10 / 1e6)
        else:
            start = time.perf_counter_ns()
            result = engine.update(raw, aligned, now=raw.received + 0.02)
            elapsed.append((time.perf_counter_ns() - start) / 1e6)
        signature(result, raw)
    return statistics.median(elapsed)


results = {}
for mode, rows in [
    ("complete", complete),
    ("positive_inf", holes),
    ("late_pose", holes),
    ("cached", holes),
]:
    values = {"baseline": [], "candidate": []}
    for round_id in range(4):
        order = (
            [("baseline", base), ("candidate", new)]
            if round_id % 2 == 0
            else [("candidate", new), ("baseline", base)]
        )
        for label, cls in order:
            values[label].append(run(cls, rows, mode))
    results[mode] = values
    results[mode]["baseline_median_ms"] = statistics.median(values["baseline"])
    results[mode]["candidate_median_ms"] = statistics.median(values["candidate"])
print(json.dumps(results, indent=2))
```

</details>


## 附录：原生接收退出原因定位（d3ffc50 后续）

目标仅为下一次退出可定位，不宣称修复积压。基线固定 `d3ffc5019705947d77cb790b55cc8be33cd6678f`、同一功能主分支。产品修改2：`senior_sensor_ingress/src/native_receipt.cpp` 和 `senior_sensor_ingress/senior_sensor_ingress/native_receipt.py`（均在 ros2/src 下）；工具文档修改1为本文；测试/数据、配置/构建、新增、删除均0，合计严格3文件修改。

设计与顺序：先记录本附录，再将已有失败分支的通用2替换为短 enum class 的唯一非零退出码；header/payload 保持共享原100ms写截止，send/poll按原条件失败，不增加热路径时钟调用、不输出子进程stdio、不改FD/parent检查顺序。父进程仅在原失败日志新增 child_exit_reason，保留 child_exit、原failure_reason、先撤证再poll/close/log的顺序。旧二进制2标记 legacy_failure；None只表示 not_observed，负数表示 signal_N，未知数表示 unknown_exit_N，不推断子进程是否仍活着或自动重启。

| 码 | enum成员 / Python标签 | 对应原分支 |
| --- | --- | --- |
| 10 | write_deadline | header或payload的原共用100ms deadline耗尽 |
| 11 | send_failure | send为0或errno非EAGAIN/EWOULDBLOCK/EINTR |
| 12 | poll_failure | poll返回负且errno非EINTR |
| 13 | oversize_payload | 序列化payload大于20MiB |
| 14 | retired_source_exhaustion | retired GID集合达到256 |
| 15 | invalid_arguments | 参数数量/分组/最多16输入的原shape检查 |
| 16 | invalid_fd_or_parent | fd<3或初始parent不等于getppid |
| 17 | parent_death_setup | 原prctl失败或注册后parent改变 |
| 18 | unexpected_exception | 原main捕获std::exception |

合并同因说明：不区分header/payload写阶段，共享同一个截止且无需新增热路径参数；初始fd/parent保持原复合条件，parent-death注册及竞态也保持原短路检查。stoi格式/溢出、构造输入/QoS/类型异常等仍由原catch归unexpected_exception，不重新分类或扩大异常捕获。

验证清单：复用既有纯Python freshness、egress及runtime diagnostics；源级核对所有enum数值唯一、非0且不复用旧2，Python解释与C++枚举逐项相等、原每个失败条件只替换返回码且所有时间调用/FD/协议不变；用内存node/process/socket/logger替身调用真实NativeReceiptGroup._drain失败路径，核对全部已知码、旧2、None、信号、未知码的日志及撤证/close顺序。没有新增测试文件或修改用例；mock和静态检查不得称native build/ROS验收。Linux/Humble真实阻塞socket、send/poll故障及各退出分支仅列后续隔离复现，不在本轮启动。

实际命令与结果实施后补记；本轮不读生产数据/配置，不启服务/设备/ROS/仿真，不部署推送。

### 退出定位实际验证

基线 d3ffc50 与本片候选复用命令：
```sh
PYTHONDONTWRITEBYTECODE=1 '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' -m pytest -p no:cacheprovider -q tests/unit/test_safety_freshness.py tests/unit/test_egress_receipt.py tests/unit/test_runtime_diagnostics.py
'/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/ruff' check --no-cache ros2/src/senior_sensor_ingress/senior_sensor_ingress/native_receipt.py
'/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/ruff' format --check --no-cache ros2/src/senior_sensor_ingress/senior_sensor_ingress/native_receipt.py
git diff --check
```

结果：基线33 passed in 0.28s、候选33 passed in 0.27s；freshness2项、egress9项、runtime diagnostics22项。两版Ruff lint通过、format为1 file already formatted，diff --check通过，三文件CRLF均0。新增/改写测试0、实际bug回归0；产品检查后仅补文档，没有再修改产品。

源码核对通过：9个唯一非零码10..18且避开旧2，分别使用于原9个退出位置；Python九个原因标签与C++逐项相等。C++去掉enum并将cast还原2后与基线逐字节一致，Python去掉新增映射/helper/log字段后与基线逐字节一致，故没有改变任何原条件/时钟调用/FD/关停顺序/协议。首轮临时核对脚本还原Python段落时少保留一空行，断言失败；修正核对脚本后通过，产品未为此变更。

内存替身调用真实 NativeReceiptGroup._drain 和close的5项均通过：10→write_deadline、旧2→legacy_failure、None→not_observed、-15→signal_15、255→unknown_exit_255。每项都核对先invalidate、poll、关socket、再唯一一次log，原reason/child_exit/topics保留且不反序列化；None仍保持原close中的terminate/wait，日志使用原poll状态，不调整异步竞态。其余新码及0正常退出标签只由严格映射覆盖，不冒称触发了真实退出。未新增测试文件，下方是准确内联命令。

<details>
<summary>源级一致性与父失败分支核对命令</summary>

```sh
PYTHONDONTWRITEBYTECODE=1 '/Users/zhangshunjun/Wheel 机器人/senior-robot-control/.venv/bin/python' - <<'PY'
import ast
from collections import Counter
import importlib.util
from pathlib import Path
import re
import subprocess
import sys
import threading
import tomllib
import types

config = tomllib.loads(Path('pyproject.toml').read_text())
sys.path[:0] = [str(Path.cwd() / p) for p in config['tool']['pytest']['ini_options']['pythonpath']]
cpp_path = 'ros2/src/senior_sensor_ingress/src/native_receipt.cpp'
py_path = 'ros2/src/senior_sensor_ingress/senior_sensor_ingress/native_receipt.py'
cpp, py = Path(cpp_path).read_text(), Path(py_path).read_text()
block = re.search(r'// Keep these failure codes aligned.*?enum class ExitCode : int \{(.*?)\n\};\n', cpp, re.S)
assert block
pairs = re.findall(r'([a-z_]+) = (\d+),', block[1])
assert len(pairs) == 9
codes = {int(code): name for name, code in pairs}
assert len(codes) == 9 and all(0 < code < 256 and code != 2 for code in codes)
uses = re.findall(r'static_cast<int>\(ExitCode::([a-z_]+)\)', cpp)
assert Counter(uses) == Counter(name for name, code in pairs)
normalized = cpp[:block.start()] + cpp[block.end():]
normalized = re.sub(r'static_cast<int>\(ExitCode::[a-z_]+\)', '2', normalized)
assert normalized == subprocess.check_output(['git', 'show', 'd3ffc50:' + cpp_path], text=True)
assignment = next(n for n in ast.parse(py).body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '_CHILD_EXIT_REASONS' for t in n.targets))
assert len(assignment.value.keys) == 11
labels = ast.literal_eval(assignment.value)
assert {code: labels[code] for code in codes} == codes
assert labels[2] == 'legacy_failure' and labels[0] == 'normal_exit'
start = py.index('# Keep these labels aligned with ExitCode in native_receipt.cpp.')
end = py.index('\n\ndef received_steady', start)
normalized_py = py[:start] + '\n' + py[end+2:]
normalized_py = normalized_py.replace('                f"child_exit_reason={_child_exit_reason(returncode)}; "\n', '')
assert normalized_py == subprocess.check_output(['git', 'show', 'd3ffc50:' + py_path], text=True)
print('source contract PASS: 9 unique nonzero codes, exact Python labels and 9 exit sites; normalized C++/Python byte-identical to d3ffc50')
spec = importlib.util.spec_from_file_location('receipt_exit_diagnostic_check', py_path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
serialization = types.ModuleType('rclpy.serialization')
def forbidden_deserialize(*args):
    raise AssertionError('failed branch must not deserialize')
serialization.deserialize_message = forbidden_deserialize
sys.modules['rclpy'] = types.ModuleType('rclpy')
sys.modules['rclpy.serialization'] = serialization
for code, reason in [(10, 'write_deadline'), (2, 'legacy_failure'), (None, 'not_observed'), (-15, 'signal_15'), (255, 'unknown_exit_255')]:
    events, errors = [], []
    class Process:
        def __init__(self):
            self.code = code
        def poll(self):
            events.append('poll')
            return self.code
        def terminate(self):
            events.append('terminate')
        def wait(self, timeout):
            assert timeout == 1
            events.append('wait')
            self.code = -15
    class Socket:
        def shutdown(self, how):
            events.append('socket_shutdown')
        def close(self):
            events.append('socket_close')
    class Logger:
        def error(self, message):
            events.append('log')
            errors.append(message)
    logger = Logger()
    group = module.NativeReceiptGroup.__new__(module.NativeReceiptGroup)
    group.node = types.SimpleNamespace(get_logger=lambda: logger)
    group.inputs = [types.SimpleNamespace(topic='/fixture/input', invalidate=lambda: events.append('invalidate'))]
    group._closed = False
    group._started_at = 1.0
    group._lock = threading.Lock()
    group._invalid = {0}
    group._pending = {}
    group._queued_bytes = 0
    group._failed = True
    group._failure_reason = 'OSError: native receipt channel closed'
    group._process = Process()
    group._socket = Socket()
    group._thread = None
    module.process_diagnostics = lambda: types.SimpleNamespace(pump=lambda now: events.append('pump'))
    group._drain()
    assert group._closed
    assert events[:5] == ['pump', 'invalidate', 'poll', 'socket_shutdown', 'socket_close']
    assert events[-1] == 'log' and events.count('log') == 1
    assert ('terminate' in events) == (code is None)
    assert len(errors) == 1
    assert f'reason=OSError: native receipt channel closed; child_exit={code}; child_exit_reason={reason};' in errors[0]
    assert "topics=['/fixture/input']" in errors[0]
    print(f'memory _drain PASS: child_exit={code}, child_exit_reason={reason}, events={events}')
PY
```
</details>

限制及后续：未做native build/ROS消息生成/Linux/Humble运行、真实子进程或真实socket故障；未运行服务/设备/ROS/仿真，未读生产数据或生产配置，未部署推送。获准后在隔离Linux/Humble用原生程序和私有socketpair暂停reader并填满缓冲，验证原100ms写deadline触发码10，再独立验证send/poll错误、超大payload、retired source和启动失败分支；这些仅为后续候选，本轮未执行。新码只能定位触发条件，不能把write_deadline说成某个Python pending队列、fenced scan回调或唯一事故根因；header/payload仍共用一个截止标签，未新增errno或异常文本采集。
