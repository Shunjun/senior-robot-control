# Task 1 foundation report

Status: F foundation implemented on `fix/ros-typesupport-export-order`, starting at `393e7c380172210406d72b7fbebb3c06f4801351`; this is intentionally an intermediate migration, not a runnable whole-stack release. No business consumers A/B/C/D were changed. No service, SSH, vehicle operation, helper worker, compatibility parser, old-type alias, or fake manufacturer proof was introduced.

## Requirements and commits

Read `.superpowers/sdd/2026-11-17-standard-ros-time-migration/task-1-brief.md` and approved `docs/superpowers/specs/2026-11-17-standard-ros-time-design.md`; did not read the full implementation plan. No root `AGENTS.md` exists in the checkout; the supplied AGENTS instructions were followed. Existing typesupport export-order fix is retained: rosidl interfaces register generated targets before the final `ament_export_targets(export_sensor_timing HAS_LIBRARY_TARGET)` hook, and the interface still links the generated C++ typesupport.

Implementation commit: `199f39b6a8a156363981bffa0be21dc5fffdfae0`. This report is the subsequent report-only commit; use `git log -2 --oneline` for both SHAs. The implementation SHA is the code/schema/fixture contract freeze; coordinator may use the report commit as the branch point because its only additional change is this report.

## Frozen contracts

- `src/senior_control/sensors/sample_time.py`: frozen `SampleTime(stamp_ns, received_ros_ns, received_steady_ns, sample_steady_ns, clock_generation)`, keyword-only `map_sample`, frozen `StampInterval(start_ns, end_ns)`, `interval_ns(first, last)`, `ClockObservation(generation, discontinuity)`, and `ClockTracker.observe(*, ros_ns, steady_ns)`.
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/sample_time.hpp`: matching structures/functions in `senior_motion_msgs::time`, positional mapper and observe arguments, signed int64 ROS coordinates and uint64 monotonic/generation fields, plus `uint64_t monotonic_ns()` using CLOCK_MONOTONIC. No old namespace forwarding.
- Both mappers use integer `received_steady_ns - (received_ros_ns - stamp_ns)`. Negative ROS, future measurement, pre-origin mapping, and invalid Python integer widths/types reject. Zero stamp is legal. Tracker starts at generation 1, increments on either clock decreasing, leaves same-value observations unchanged, and does no rate/reset inference. Generation comparison in interval joining is only for one source/window owner's already accepted endpoints; it is not cross-ingress clock certification.
- `SensorTiming.sample_age(time: SampleTime, *, now_steady_ns: int) -> Age` applies the existing 300 ms soft / 1000 ms hard defaults using integer age. Existing explicit-seconds age APIs remain useful for non-measurement callers. C++ `Policy::age_ns(const SampleTime &, uint64_t now_steady_ns)` connects the same mapping to existing policy/statistics; existing queue policies remain intact.
- `RawCaptureHeader.msg` retains stamp, received_ros, frame/GID/source_epoch/bridge_boot_id/capture_generation/sequence/valid/reason; removes SourceTime and float received_steady; requires uint64 `received_steady_ns`, `sample_steady_ns`, `clock_generation`. All containing messages inherit it. Existing bounded derived geometry, duplicate explicit-seconds UI/transport fields, static TF lifecycle, FinalCommand, DriveExecutionFeedback, ScanMotion endpoints and RGB-D eight captures/TF/calibration/egress remain. The capture record is authoritative; consumers must not remap duplicate display fields.
- Native receipt contract frozen for A sender / B receiver: magic `SDR2`, Python struct `!4sBBQQQqq24sBqqQQQI` (115 bytes); magic, kind, input, generation, sequence, callback_steady_ns, RMW source/received timestamps, 24-byte GID, time_mode, stamp_ns, received_ros_ns, original_received_steady_ns, sample_steady_ns, clock_generation, payload_size, followed by unchanged CDR. Modes 0 absent / 1 original ingress / 2 retained derived capture. No SDR1 fallback. A passes standard use_sim_time explicitly to the native child; B implements required `received_sample(info) -> SampleTime`, keeps `received_steady(info)` as callback/IPC age only, exposes `info['sample_time']`; C consumes that API for localization. These business IPC implementations are deliberately not supplied by F.
- Standard output contract for C: `/cmd_vel` Twist; private packet `=ddddQ` unchanged. The two shared native-output harnesses now inspect standard Twist and preserve deadline, EOF, overflow, malformed input, order and sequence checks. The deterministic harness controls CLOCK_MONOTONIC before including the real common header; no production clock injection seam or old timing namespace is retained. The rclcpp shim records topics and values only; it is not DDS or manufacturer admission evidence.
- `senior_motion_msgs` version is `0.3.0`; downstream common-package CMake requirements are `0.3.0 REQUIRED` without EXACT. Removed unused ament_cmake_python install and simulation_timing library. Removed slam_toolbox exact CMake/package pin; linking to installed ordinary SDK libraries and the existing Humble export accommodation remain.
- D's install paths are frozen as `senior_sim/scripts/simulation_camera.py` and `world_reference.py`, installed under lib/senior_sim renamed to `simulation_camera` and `world_reference`. The existing DIRECTORY config install covers D's `senior_sim/config/motion_profile.json` and `navigation_reference.json`; D creates real implementations/data, no F placeholders exist. Removed senior_robot's old simulation_camera console entry point. D's hardware-format motion profile uses no simulation_only field; ROS parameter remains `hardware_motion_profile`, reference remains `hardware_navigation_reference`.

## Shared fixtures and support

New `tests/unit/sensor_time_test_support.py` exposes exactly `sample_time(*, stamp_ns, received_ros_ns, received_steady_ns, clock_generation=1) -> SampleTime` through the real mapper and `capture_fields(*, time: SampleTime) -> dict[str, Any]`. Capture fields contain only stamp/received_ros namespace Time values and the three new ns/generation fields. No old `capture_time`, `wire_time`, synthetic_domain, or proof schedule compatibility API remains.

`control_process_test_support.py` consumes required `sample['time']` and passes it to PointObservation; it no longer patches a proof clock domain or reconstructs legacy poses. `learning_test_support.py` retains a fixed test host identity for LearningRecordOwner (not a measurement proof), replaces simulation_point with `learning_point(source, stamp_ns, received_ros_ns, received_steady_ns, values, sequence=1)` using mandatory time. ROS conftest maps the supplied scan stamp once from explicit clocks into new capture fields. `motion_reference_test_support.py`, integration heartbeat support/agent_fakes, e2e conftest and ROS motion_* shared helpers were inspected; they contain no old measurement protocol and required no changes.

Four fixture JSONs now carry 255 validated time/first_time/last_time dictionaries. Three existing raw_capture records use their actual recorded stamp and ROS/steady receipt values, with old float received_steady replaced by ns. Historical source rows lack ROS-at-receipt, so their newly frozen test inputs explicitly use synthetic received_ros equal to the original stamp and retained first steady receipt; interval starts shift by original ROS duration. Identity-keyed records are reused, including cached re-emissions. Each fixture states this limitation; this is geometry/control replay, not a new assertion about real acquisition/transport timing. Original numeric source/geometry/health values remain unchanged; old snapshot summaries and opaque historical identity strings remain evidence, not production simulation permissions. Condensed rows lacking an original endpoint were not given fabricated samples. B/C own caller migration and assertions after fork; they must consume the frozen fields directly, remove old synthetic_capture_schedule calls, and avoid editing shared fixtures independently.

## Verification and real limits

Host: Darwin arm64, Python 3.12.15, pytest 8.4.2, Apple clang 21.0.0 (`clang-2100.0.123.102`), ruff 0.17.3, mypy 2.4.0. CMake and colcon are absent; `/opt/ros` does not exist. No ROS Humble environment or generated rosidl modules are available.

Executed commands/results (also verified `struct.calcsize("!4sBBQQQqq24sBqqQQQI") == 115`):

```text
.venv/bin/python -m pytest tests/unit/test_sensor_timing.py -q
  BEFORE implementation: 13 passed in 0.04s.
.venv/bin/python -m pytest tests/unit/test_sensor_timing.py tests/unit/test_shared_sensor_cpp.py -q
  AFTER implementation: 30 passed in 2.48s (earlier run 30 passed in 6.77s).
.venv/bin/python ros2/src/senior_motion_msgs/test/test_final_command_frame.py
  8 tests passed, OK, 1.402s.
.venv/bin/mypy src/senior_control/sensors/sample_time.py src/senior_control/sensors/policy.py
  Success: no issues found in 2 source files.
.venv/bin/ruff check src/senior_control/sensors/sample_time.py src/senior_control/sensors/policy.py tests/unit/sensor_time_test_support.py tests/unit/control_process_test_support.py tests/unit/learning_test_support.py tests/unit/test_sensor_timing.py tests/ros/conftest.py
  All checks passed.
git diff --check
  Passed, no output.
command -v colcon
  No command found.
ls /opt/ros
  No such file or directory.
cmake --version
  command not found (exit 127).
```

The two standalone arithmetic tests compile real public helper headers using `c++ -std=c++17 -Wall -Wextra -Werror -I<repo>/ros2/src/senior_motion_msgs/include <temporary source> -o <temporary executable>` and execute their assertions. These cover the exact 99.8/100/20 -> 19.8 mapping, 0.35 consumed age, 300 ms/1 s bands, preserved deadline, 1 ns epoch distinction, zero stamp, signed/unsigned extrema, invalid/future/reversed mapping, same-source interval generation rejection, ordinary ROS/steady rollback and unchanged equal clocks. Existing C++ queue transient crossing/recovery checks passed unchanged. FinalCommand tests preserve the actual native frame/Python decoder behavior through a local test socket; they do not prove ROS publication or vehicle execution.

Source audits (Python inline scripts executed locally, not rosidl generation): CMake's 25 msg entries exactly equal the 25 files; all senior_motion_msgs local msg type references resolve; all package XML parses; seven edited/new support/setup Python files parse; all fixture JSON parses and 255 new SampleTime records validate; all original fixture values compare equal to baseline except the specified raw_capture float-to-ns replacement. To reproduce the schema audit:

```python
from pathlib import Path
import re
root = Path('ros2/src/senior_motion_msgs')
listed = set(re.findall(r'"(msg/[^"]+\.msg)"', (root / 'CMakeLists.txt').read_text()))
actual = {str(p.relative_to(root)) for p in (root / 'msg').glob('*.msg')}
assert listed == actual
known = {p.stem for p in (root / 'msg').glob('*.msg')}
for path in (root / 'msg').glob('*.msg'):
    assert all(t in known for t in re.findall(r'senior_motion_msgs/(\w+)', path.read_text()))
```

Unverified, deliberately deferred to Task 8 after A/B/C/D: real rosidl generation, clean Linux ROS package builds, installed-header consumer/typesupport loading, native sender/receiver/output harnesses with migrated consumers, real DDS, vehicle static graph, simulation, navigation and webpage operation. No stubs were used to claim these passed. Whole downstream suites are expected to fail imports against removed APIs until their owners migrate; no business tests were bulk edited, skipped or weakened in Task 1. Installed-header consumer remains in CMake and now includes ScanMotion, RawCaptureHeader, SampleTime and sensor policy through installed exports.

## Retired artifacts and retained behaviors

Deleted 11 obsolete msg schemas: SourceTime, ActuatorTarget, NavigationSample and eight Timed sensor messages. Deleted time_proof.py, source_clock.py, source_timing/source_transport/actuator_timing/simulation_timing/navigation_timing headers and simulation_timing.cpp. They implement the approved retired proof/patch transport; no alias/wrapper survives.

Deleted `source_time_test_support.py`, fake source_time.hpp/actuator_target.hpp, source_clock_harness.cpp (driver/simulation proof/domain/engine-reset behavior), and actuator_admission_harness.cpp (manufacturer custom ActuatorTarget admission). Purely old proof-field assertions in output harnesses were removed while valid velocity/order/deadline/EOF checks were retained; `/cmd_vel` rejection became standard-topic acceptance. No consumer regression test file was deleted. B retains/migrates valid clock/order/age/geometry/learning tests and removes only obsolete proof/rate/reset expectations; C migrates navigation/output behavior; A retains ingress/replay; D retires obsolete overlay/environment tests. The coordinator ruling leaves test_control_clock_domains.py and bulk consumer tests to B after fork; only the approved pure mapping checks in test_sensor_timing.py were updated here.

## Owned files committed

The implementation commit owns the 50 files below; the report commit additionally owns `.superpowers/sdd/2026-11-17-standard-ros-time-migration/task-1-report.md`.

- `ros2/src/senior_motion_msgs/CMakeLists.txt`
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/actuator_timing.hpp`
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/navigation_timing.hpp`
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/sample_time.hpp`
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/sensor_timing.hpp`
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/simulation_timing.hpp`
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/source_timing.hpp`
- `ros2/src/senior_motion_msgs/include/senior_motion_msgs/source_transport.hpp`
- `ros2/src/senior_motion_msgs/msg/ActuatorTarget.msg`
- `ros2/src/senior_motion_msgs/msg/NavigationSample.msg`
- `ros2/src/senior_motion_msgs/msg/RawCaptureHeader.msg`
- `ros2/src/senior_motion_msgs/msg/SourceTime.msg`
- `ros2/src/senior_motion_msgs/msg/TimedCameraInfo.msg`
- `ros2/src/senior_motion_msgs/msg/TimedImage.msg`
- `ros2/src/senior_motion_msgs/msg/TimedImu.msg`
- `ros2/src/senior_motion_msgs/msg/TimedLaserScan.msg`
- `ros2/src/senior_motion_msgs/msg/TimedNavSatFix.msg`
- `ros2/src/senior_motion_msgs/msg/TimedOdometry.msg`
- `ros2/src/senior_motion_msgs/msg/TimedPose.msg`
- `ros2/src/senior_motion_msgs/msg/TimedTwist.msg`
- `ros2/src/senior_motion_msgs/package.xml`
- `ros2/src/senior_motion_msgs/python/source_clock.py`
- `ros2/src/senior_motion_msgs/src/simulation_timing.cpp`
- `ros2/src/senior_motion_msgs/test/installed_header_consumer/header_consumer.cpp`
- `ros2/src/senior_robot/setup.py`
- `ros2/src/senior_scan_matcher/CMakeLists.txt`
- `ros2/src/senior_sensor_ingress/CMakeLists.txt`
- `ros2/src/senior_sim/CMakeLists.txt`
- `ros2/src/senior_slam_bridge/CMakeLists.txt`
- `ros2/src/senior_slam_bridge/package.xml`
- `src/senior_control/sensors/policy.py`
- `src/senior_control/sensors/sample_time.py`
- `src/senior_control/sensors/time_proof.py`
- `tests/fixtures/motion_reference_backlog.json`
- `tests/fixtures/navigation_startup_reference.json`
- `tests/fixtures/reference_clock_frontier.json`
- `tests/fixtures/reference_nanosecond_boundary.json`
- `tests/ros/conftest.py`
- `tests/support/native_output/actuator_admission_harness.cpp`
- `tests/support/native_output/protocol_harness.cpp`
- `tests/support/native_output/protocol_state_harness.cpp`
- `tests/support/native_output/rclcpp/rclcpp.hpp`
- `tests/support/native_output/senior_motion_msgs/msg/actuator_target.hpp`
- `tests/support/native_output/senior_motion_msgs/msg/source_time.hpp`
- `tests/support/native_output/source_clock_harness.cpp`
- `tests/unit/control_process_test_support.py`
- `tests/unit/learning_test_support.py`
- `tests/unit/sensor_time_test_support.py`
- `tests/unit/source_time_test_support.py`
- `tests/unit/test_sensor_timing.py`

## Remaining protocol inventory and whole-file assignments

Baseline narrow search (required verbatim) ran before edits:

```sh
rg -l 'SourceTime|Timed[A-Z]|ActuatorTarget|NavigationSample|time_proof|sim_step|simulation_only|source_timing|source_clock|simulation_timing|navigation_timing' src ros2 tools tests config
```

Final expanded search includes old parser/type names, private output/topic, proof fields, SDR1 and production simulation-profile selectors, plus active and historical documentation:

```sh
rg -l 'SourceTime|Timed[A-Z]|ActuatorTarget|NavigationSample|time_proof|sim_step|simulation_only|source_timing|source_clock|simulation_timing|navigation_timing|MeasurementTime|MeasurementInterval|SourceFrontier|source_time_test_support|/control/actuator_target|/timed|navigation_run_id|driver_read|measurement_epoch|measurement_tick_ns|capture_start_steady_ns|capture_end_steady_ns|SDR1|motion_core_profile|SimulationMotionProfile|SimulationReference|prepare_(sensor|simulation|nav2)_overlay|source_transport|actuator_timing|physical_support' src ros2 tools tests config deploy .github README.md docs
```

Final search: 189 files; A=15, B=69, C=13, D=34, F=58. The following table is the union of baseline and final hits, so retired common files are accounted for too. All paths have one definite owner; F owns the newly assigned shared policy.py and final evidence/history annotation. Newly explicit assignments include B for test_agent.py, test_native_motion_executor.py, test_motion_core_handoff.py, test_software_estop_owner.py and test_control_process_resources.py; D for tools/vehicle_motion_fixture.py and operational documents. No production hit remains unassigned. Files not matching a retired protocol may still need ordinary standard-message changes under the brief's existing ownership.

| Owner | Path | Task 1 state / next action |
| --- | --- | --- |
| D | `config/hardware-motion.example.json` | deferred to owner |
| D | `config/hardware-motion.json` | deferred to owner |
| D | `config/hardware-motion.schema.json` | deferred to owner |
| D | `docs/operations/control-clock-overlays.md` | deferred to owner |
| D | `docs/operations/hardware-motion-parameters.md` | deferred to owner |
| D | `docs/operations/navigation-monotonic-overlay.md` | deferred to owner |
| D | `docs/operations/simulation.md` | deferred to owner |
| F | `docs/reviews/2026-11-13-vehicle-ui-contracts.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-actuator-clock-boundary.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-control-clock-producer-contract.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-control-clock-system-audit.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-duration-milliseconds-current-proposal.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-fortress-reset-boundary.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-learning-dual-clock-interface.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-learning-path-integration-repair.md` | historical evidence; annotate in integration |
| F | `docs/reviews/2026-11-16-navigation-steady-contract.md` | historical evidence; annotate in integration |
| F | `docs/superpowers/plans/2026-11-12-vehicle-motion-redesign.md` | deferred to owner |
| F | `docs/superpowers/plans/2026-11-17-standard-ros-time-migration.md` | deferred to owner |
| F | `docs/superpowers/specs/2026-11-16-startup-learning-visibility.md` | deferred to owner |
| F | `docs/superpowers/specs/2026-11-17-standard-ros-time-design.md` | deferred to owner |
| F | `docs/testing/2026-11-13-vehicle-motion-normal-runtime.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-13-vehicle-motion-output-wiring.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-14-vehicle-real-control-wiring.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-15-motion-test-migration.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-15-remove-legacy-motion.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-control-clock-concentrated-validation.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-control-clock-fingerprints.json` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-control-clock-owned.diff` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-dual-clock-fingerprints.json` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-dual-clock.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-frozen-reference-owned.diff` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-recorded-path.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-shared-state.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-surface-review-evidence/frozen-learning-extra-raw-gap-probe.log` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-surface-review-evidence/frozen-learning-extra-raw-gap-probe.xml` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-surface-review-evidence/learning-surface-exact-gap.log` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-surface-review-evidence/original-gap-diagnostic.py.txt` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-surface-review-owned.diff` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-learning-surface-review.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-navigation-limits-clock.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-runtime-fault-isolation-owned.diff` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion-evidence/occlusion-final.log` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion-evidence/occlusion-final.xml` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion-evidence/occlusion-first.log` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion-evidence/occlusion-first.xml` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion-evidence/occlusion-normal-regression.log` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion-evidence/occlusion-normal-regression.xml` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion-owned.diff` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-16-startup-learning-occlusion.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-duration-milliseconds-evidence/pytest-learning-gap.log` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-duration-milliseconds-evidence/pytest-learning-gap.xml` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-duration-milliseconds-evidence/test_learning_gap_debug.py.txt` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-duration-milliseconds-owned.diff` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-duration-milliseconds.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-final-review-fixes-before-fingerprints.json` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-final-review-fixes-evidence/package-source-fingerprints.json` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-final-review-fixes-fingerprints.json` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-rgbd-receipt-build-regression.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-startup-final-validation-evidence/pytest-unit.log` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-startup-final-validation-evidence/pytest-unit.xml` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-startup-final-validation-evidence/ruff-format-protected.diff` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-startup-final-validation-evidence/ruff-format.txt` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-startup-final-validation-fingerprints.json` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-startup-final-validation.md` | historical evidence; annotate in integration |
| F | `docs/testing/2026-11-17-test-pruning-evidence/start-source-inventory.txt` | historical evidence; annotate in integration |
| F | `ros2/src/senior_motion_msgs/CMakeLists.txt` | migrated; no final-pattern matches |
| F | `ros2/src/senior_motion_msgs/include/senior_motion_msgs/actuator_timing.hpp` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/include/senior_motion_msgs/source_timing.hpp` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/include/senior_motion_msgs/source_transport.hpp` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/ActuatorTarget.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/RawCaptureHeader.msg` | migrated; no final-pattern matches |
| F | `ros2/src/senior_motion_msgs/msg/SourceTime.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedCameraInfo.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedImage.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedImu.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedLaserScan.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedNavSatFix.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedOdometry.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedPose.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/msg/TimedTwist.msg` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/python/source_clock.py` | retired in Task 1 |
| C | `ros2/src/senior_motion_msgs/src/motion_output_guard.cpp` | deferred to owner |
| A | `ros2/src/senior_motion_msgs/src/native_receipt.cpp` | deferred to owner |
| F | `ros2/src/senior_motion_msgs/src/simulation_timing.cpp` | retired in Task 1 |
| F | `ros2/src/senior_motion_msgs/test/installed_header_consumer/header_consumer.cpp` | migrated; no final-pattern matches |
| D | `ros2/src/senior_robot/config/simulation_motion.yaml` | deferred to owner |
| C | `ros2/src/senior_robot/launch/navigation.launch.py` | deferred to owner |
| C | `ros2/src/senior_robot/launch/safety_motion.launch.py` | deferred to owner |
| D | `ros2/src/senior_robot/launch/simulation_motion.launch.py` | deferred to owner |
| C | `ros2/src/senior_robot/senior_robot/agent_adapter.py` | deferred to owner |
| C | `ros2/src/senior_robot/senior_robot/command_node.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/control_inputs.py` | deferred to owner |
| C | `ros2/src/senior_robot/senior_robot/dedicated_controller.py` | deferred to owner |
| C | `ros2/src/senior_robot/senior_robot/hardware_reference.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/motion_core_output.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/motion_fusion.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/motion_observation_adapter.py` | deferred to owner |
| C | `ros2/src/senior_robot/senior_robot/motion_output.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/motion_profile.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/motion_sensor_runtime.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/native_receipt.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/optional_motion_inputs.py` | deferred to owner |
| B | `ros2/src/senior_robot/senior_robot/safety_node.py` | deferred to owner |
| D | `ros2/src/senior_robot/senior_robot/simulation_motion_profile.py` | deferred to owner |
| D | `ros2/src/senior_robot/senior_robot/simulation_reference.py` | deferred to owner |
| A | `ros2/src/senior_scan_matcher/src/scan_motion_node.cpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/include/senior_sensor_ingress/capture_ledger.hpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/include/senior_sensor_ingress/raw_receipt.hpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/include/senior_sensor_ingress/scan_ledger.hpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/include/senior_sensor_ingress/scan_receipt.hpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/src/raw_receipt.cpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/src/rgbd_candidate_receipt.cpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/src/scan_receipt.cpp` | deferred to owner |
| A | `ros2/src/senior_sensor_ingress/test/test_capture_ledger.cpp` | deferred to owner |
| D | `ros2/src/senior_sim/config/bridge.yaml` | deferred to owner |
| D | `ros2/src/senior_sim/urdf/senior.urdf.xacro` | deferred to owner |
| A | `ros2/src/senior_slam_bridge/src/acquire_scan_receipt.cpp` | deferred to owner |
| A | `ros2/src/senior_slam_bridge/src/bounded_scan_ingress.cpp` | deferred to owner |
| A | `ros2/src/senior_slam_bridge/src/scan_receipt_admission.hpp` | deferred to owner |
| A | `ros2/src/senior_slam_bridge/src/static_lidar_tf_ingress.cpp` | deferred to owner |
| B | `src/senior_control/localization/gps_course.py` | deferred to owner |
| B | `src/senior_control/motion/estimator.py` | deferred to owner |
| B | `src/senior_control/motion/fusion.py` | deferred to owner |
| B | `src/senior_control/motion/hardware_profile.py` | deferred to owner |
| B | `src/senior_control/motion/observations.py` | deferred to owner |
| B | `src/senior_control/motion/pose_estimator.py` | deferred to owner |
| B | `src/senior_control/motion/sample_types.py` | deferred to owner |
| B | `src/senior_control/motion/source_admission.py` | deferred to owner |
| B | `src/senior_control/motion/startup_learning.py` | deferred to owner |
| B | `src/senior_control/motion/temporal.py` | deferred to owner |
| D | `src/senior_control/real_runtime.py` | deferred to owner |
| B | `src/senior_control/safety/self_learning.py` | deferred to owner |
| F | `src/senior_control/sensors/time_proof.py` | retired in Task 1 |
| B | `src/senior_control/vision/independent_motion.py` | deferred to owner |
| B | `src/senior_control/vision/raw_capture.py` | deferred to owner |
| B | `src/senior_control/vision/rgbd_candidate_codec.py` | deferred to owner |
| B | `src/senior_control/vision/semantic_runtime.py` | deferred to owner |
| B | `tests/integration/test_agent.py` | deferred to owner |
| C | `tests/integration/test_manual_wire.py` | deferred to owner |
| C | `tests/ros/test_final_command_dds.py` | deferred to owner |
| B | `tests/ros/test_native_motion_executor.py` | deferred to owner |
| F | `tests/support/native_output/actuator_admission_harness.cpp` | retired in Task 1 |
| F | `tests/support/native_output/protocol_harness.cpp` | migrated; no final-pattern matches |
| F | `tests/support/native_output/protocol_state_harness.cpp` | migrated; no final-pattern matches |
| F | `tests/support/native_output/senior_motion_msgs/msg/actuator_target.hpp` | retired in Task 1 |
| F | `tests/support/native_output/senior_motion_msgs/msg/source_time.hpp` | retired in Task 1 |
| F | `tests/support/native_output/source_clock_harness.cpp` | retired in Task 1 |
| F | `tests/unit/control_process_test_support.py` | migrated; no final-pattern matches |
| F | `tests/unit/source_time_test_support.py` | retired in Task 1 |
| D | `tests/unit/test_common_sensor_composition.py` | deferred to owner |
| B | `tests/unit/test_continuous_scan_control.py` | deferred to owner |
| B | `tests/unit/test_control_clock_domains.py` | deferred to owner |
| B | `tests/unit/test_control_process_resources.py` | deferred to owner |
| B | `tests/unit/test_egress_receipt.py` | deferred to owner |
| B | `tests/unit/test_estimation_review_contracts.py` | deferred to owner |
| B | `tests/unit/test_gps_course.py` | deferred to owner |
| B | `tests/unit/test_independent_motion.py` | deferred to owner |
| A | `tests/unit/test_laser_reference_publication.py` | deferred to owner |
| B | `tests/unit/test_learning_dual_clock.py` | deferred to owner |
| B | `tests/unit/test_learning_frozen_reference.py` | deferred to owner |
| B | `tests/unit/test_learning_record_owner.py` | deferred to owner |
| B | `tests/unit/test_limited_manual_rearm.py` | deferred to owner |
| B | `tests/unit/test_motion_core_chain.py` | deferred to owner |
| B | `tests/unit/test_motion_core_handoff.py` | deferred to owner |
| B | `tests/unit/test_motion_core_perception.py` | deferred to owner |
| B | `tests/unit/test_motion_core_startup.py` | deferred to owner |
| B | `tests/unit/test_motion_fusion.py` | deferred to owner |
| D | `tests/unit/test_motion_launch_profile.py` | deferred to owner |
| B | `tests/unit/test_motion_reference_backlog.py` | deferred to owner |
| B | `tests/unit/test_motion_sources.py` | deferred to owner |
| B | `tests/unit/test_motion_temporal.py` | deferred to owner |
| B | `tests/unit/test_native_motion_observation_adapter.py` | deferred to owner |
| C | `tests/unit/test_navigation_collision_qualification.py` | deferred to owner |
| C | `tests/unit/test_navigation_command_clock.py` | deferred to owner |
| C | `tests/unit/test_navigation_overlay.py` | deferred to owner |
| B | `tests/unit/test_optional_motion_inputs.py` | deferred to owner |
| B | `tests/unit/test_raw_capture.py` | deferred to owner |
| B | `tests/unit/test_reference_clock_frontier.py` | deferred to owner |
| B | `tests/unit/test_reference_nanosecond_boundary.py` | deferred to owner |
| B | `tests/unit/test_relative_interval_regressions.py` | deferred to owner |
| B | `tests/unit/test_rgbd_candidate_codec.py` | deferred to owner |
| B | `tests/unit/test_rgbd_coordinator.py` | deferred to owner |
| B | `tests/unit/test_rgbd_frame_builder.py` | deferred to owner |
| B | `tests/unit/test_rgbd_joiner.py` | deferred to owner |
| B | `tests/unit/test_rgbd_parent_ledger.py` | deferred to owner |
| B | `tests/unit/test_semantic_candidates.py` | deferred to owner |
| D | `tests/unit/test_sim_contract.py` | deferred to owner |
| D | `tests/unit/test_simulation_clock_overlay.py` | deferred to owner |
| D | `tests/unit/test_simulation_reference_admission.py` | deferred to owner |
| B | `tests/unit/test_software_estop_owner.py` | deferred to owner |
| B | `tests/unit/test_startup_learning.py` | deferred to owner |
| B | `tests/unit/test_startup_learning_chain.py` | deferred to owner |
| B | `tests/unit/test_startup_learning_cycle.py` | deferred to owner |
| B | `tests/unit/test_startup_learning_latch.py` | deferred to owner |
| B | `tests/unit/test_startup_learning_review.py` | deferred to owner |
| B | `tests/unit/test_startup_learning_timed.py` | deferred to owner |
| B | `tests/unit/test_tick_evaluation.py` | deferred to owner |
| B | `tests/unit/test_unified_motion_runtime.py` | deferred to owner |
| D | `tests/unit/test_vehicle_motion_fixture.py` | deferred to owner |
| B | `tests/unit/test_wheel_imu_stillness.py` | deferred to owner |
| B | `tools/motion_fusion_synthetic.py` | deferred to owner |
| B | `tools/motion_replay_records.py` | deferred to owner |
| D | `tools/nav2_overlay/navigation-monotonic.patch` | deferred to owner |
| D | `tools/nav2_overlay/system-monotonic.patch` | deferred to owner |
| D | `tools/prepare_simulation_overlay.py` | deferred to owner |
| D | `tools/sensor_overlay/driver-monotonic.patch` | deferred to owner |
| D | `tools/sensor_overlay/optional-ahrs.patch` | deferred to owner |
| D | `tools/sensor_overlay/optional-gps.patch` | deferred to owner |
| D | `tools/simulation_overlay/CMakeLists.txt` | deferred to owner |
| D | `tools/simulation_overlay/core.CMakeLists.txt` | deferred to owner |
| D | `tools/simulation_overlay/gz-sensors.patch` | deferred to owner |
| D | `tools/simulation_overlay/gz-sim.patch` | deferred to owner |
| D | `tools/simulation_overlay/gz_ros2_control.patch` | deferred to owner |
| D | `tools/simulation_overlay/ros2_controllers.patch` | deferred to owner |
| D | `tools/simulation_overlay/ros_gz.patch` | deferred to owner |
| D | `tools/vehicle_motion_fixture.py` | deferred to owner |

## Concerns for the coordinator

1. Freeze F before forking. The intermediate tree intentionally cannot compile whole business packages: old includes/imports are gone and D's real environment scripts/data do not exist yet. Build only common rosidl generation targets when ROS is available; do not fill missing scripts with wrappers/placeholders.
2. A/B must agree on the SDR2 115-byte header and required metadata; transport RMW timestamps are diagnostics, not header stamp. D must pass standard use_sim_time configuration; production modules infer no environment/rate.
3. Consumers must retain original capture records, reject duplicate/out-of-order samples before mapping, fence publisher/lifecycle changes, clear affected windows on ClockObservation.discontinuity and preserve static-TF lifecycle without extending image TTL. F's pure mapper cannot implement consumer-owned admission.
4. B must update old shared-helper imports and simulation_point callers to the new fixture API, preserve valid algorithm/lifecycle regressions and stop parsing opaque historical source identity strings as simulation admission. F alone handles any later shared fixture adjustments serially.
5. Installed package/header export ordering was preserved by inspection but remains unverified on ROS. Task 8 must remove only confirmed stale build/install outputs before checking retired schemas/libs/patch constraints; do not let old generated files hide missing migration.
6. Historical docs/testing/review/spec/plan references in the inventory are evidence to annotate, not delete. The approved current design legitimately names obsolete protocols to specify retirement.

## Task 1 review correction — immutable shared endpoints and assigned validation scope

Status: addressed the P2 in `task-1-review.md` and the coordinator's additional test-scope correction; awaiting coordinator acceptance. This appendix supersedes the earlier sequence-keyed fixture reuse claim and the earlier broad new-test coverage description. The correction commit containing this appendix is the new F freeze; its exact SHA is available with `git log -1 --format=%H` and in the worker completion message. No product validation, business consumer, service, vehicle operation or remote system was changed.

The confirmed defect was in the fixture materialization, not the common mapper. The earlier fixture cache included the derived delta sequence, so a shared raw endpoint could receive another synthetic mapping when used as the start of a later interval. Audited all four fixtures together using `(source, retained source epoch/lifecycle, clock_generation, original stamp_ns)` as the endpoint key; the laser epochs retain publisher GID and ingress lifecycle. Derived interval sequence is deliberately excluded. Selected the complete record with the earliest `received_steady_ns` for each key and copied every field of that record to every occurrence. This also covers cached repeats and reuse across fixture documents, without recalculating an endpoint from a later interval's receipt.

Exactly two conflicting keys and three copies needed changes:

| Fixture | Stamp ns | Copies corrected | Retained received/sample steady ns | Removed age refresh ns |
| --- | ---: | ---: | ---: | ---: |
| motion_reference_backlog.json | 1300000000 | 2 later first_time copies, including cached interval | 125716720020824 | 6831208 per copy |
| reference_clock_frontier.json | 6100000000 | 1 later first_time | 129861541005342 | 6505917 |

`navigation_startup_reference.json` and `reference_nanosecond_boundary.json` already satisfy the invariant and were not rewritten. All four fixtures' historical geometry, numeric observations, source identities, sequences and provenance are unchanged. A recursive comparison against `42ade64`, excluding only endpoint time dictionaries, confirmed exact equality. All 255 endpoint records (184 unique raw identity/lifecycle/generation/stamp keys) and three unchanged RawCaptureHeader records still validate with the production SampleTime type. Repeating the full canonicalization audit found zero conflicting keys and zero copies requiring correction.

Added exactly one focused regression in existing `test_sensor_timing.py`: `test_fixture_shared_endpoints_and_cached_repeats_keep_original_deadline`. It loads all four fixtures without importing unmigrated consumers, groups the 255 records by raw identity/lifecycle/generation/stamp, and requires full record equality plus equal original policy deadlines for every repeated endpoint. It explicitly pins both reviewed laser endpoints, including three backlog occurrences (previous interval end and two later/cached starts) and two frontier occurrences, so the confirmed adjacent-interval and cached-repeat bug cannot silently return.

Removed the new invalid-coordinate parameter matrix, zero/extrema scenario and mixed integer/interval/generation/rollback scenario. Removed the matching unrelated assertions from the new standalone C++ test. The approved Python/C++ 99.8/100/20 -> 19.8 mapping with 0.35 consumed age, 300 ms/1 s boundaries and original deadline remain. All 13 valid pre-migration sensor-timing cases and the existing C++ age/queue regression remain. Product integer/range checks and ClockTracker implementation were not weakened; A/B own their assigned existing integer/generation/clock regressions after fork.

Exact commands/results for this correction:

```text
.venv/bin/python -m pytest tests/unit/test_sensor_timing.py -k fixture_shared -q
  Before fixture correction: 1 failed, 15 deselected in 0.06s.
  Failure reproduced differing received_steady_ns/sample_steady_ns at backlog stamp 1300000000.
.venv/bin/python /tmp/task1-endpoint-fix.py
  Four fixtures: 255 records, 184 keys, two conflicting keys, three copies corrected.
  Repeated after correction: zero conflicting keys, zero copies corrected.
.venv/bin/python -m pytest tests/unit/test_sensor_timing.py tests/unit/test_shared_sensor_cpp.py -q
  First correction run: 1 failed, 16 passed in 2.19s, because the narrowed C++ test compared signed age arithmetic with an unsigned deadline under -Werror.
  Corrected that test-only comparison by explicitly casting the selected sample steady coordinate to int64_t for deadline arithmetic.
  Final run: 17 passed in 1.36s.
.venv/bin/ruff check tests/unit/test_sensor_timing.py
  All checks passed.
.venv/bin/ruff format --check tests/unit/test_sensor_timing.py
  1 file already formatted.
git diff --check
  Passed, no output.
.venv/bin/python --version
  Python 3.12.15.
.venv/bin/pytest --version
  pytest 8.4.2.
.venv/bin/ruff --version
  ruff 0.17.3.
c++ --version
  Apple clang version 21.0.0 (clang-2100.0.123.102), arm64-apple-darwin25.4.0.
```

The temporary canonicalization/audit script traversed each document's sources and poses, gathered references to time/first_time/last_time by the key above, chose the minimum received_steady_ns record, copied the full dictionary, and asserted historical data/provenance equality after stripping only those time dictionaries. The committed focused test provides the durable repeatable audit through the pytest command above; a separate executed inline Python comparison against `git show 42ade64:<fixture>` confirmed unchanged original observations/provenance and validated every endpoint and raw capture.

Only four F files are part of this correction: the two conflicting fixture JSONs, `tests/unit/test_sensor_timing.py`, and this report. Remaining concerns are unchanged: synthetic receipts remain explicit geometry/control replay inputs rather than transport evidence; real Linux ROS generation/build/installed exports/DDS and all business consumers remain unverified until their assigned migration/integration work. No new compatibility path, speculative validation scenario or placeholder was added.
