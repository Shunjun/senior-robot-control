"""Immutable minimal simulation overlay contexts; no simulator is launched."""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from tools.prepare_simulation_overlay import BUNDLE, prepare

ROOT = Path(__file__).resolve().parents[2]


def test_real_shared_world_epoch_source_identity_and_same_payload_transport_cpp(tmp_path):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable; source contract not compiled")
    executable = tmp_path / "source-clock"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "tests/support/native_output"),
            "-I",
            str(ROOT / "ros2/src/senior_motion_msgs/include"),
            str(ROOT / "tests/support/native_output/source_clock_harness.cpp"),
            str(ROOT / "ros2/src/senior_motion_msgs/src/simulation_timing.cpp"),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, timeout=5)


def test_fixed_official_source_contexts_and_physics_period_remain_separate_from_ttl(tmp_path):
    cache = os.environ.get("SIM_CLOCK_SOURCE_CACHE")
    if not cache:
        pytest.skip("requires official immutable selected source cache: SIM_CLOCK_SOURCE_CACHE")
    target = tmp_path / "overlay"
    prepare(Path(cache), target, False)
    driver = (
        target / "ros2_controllers/diff_drive_controller/src/diff_drive_controller.cpp"
    ).read_text()
    assert (
        "period.seconds()" in driver
    )  # physical integration/limit interval was not replaced by host dt
    assert "odometry_.update(left_feedback_mean, right_feedback_mean, time)" in driver
    assert '"~/actuator_target"' in driver
    assert "create_subscription<Twist>" not in driver
    assert "create_subscription<geometry_msgs::msg::Twist>" not in driver
    assert "last_command_msg->timing.capture_start_steady_ns" in driver
    plugin = (target / "gz_ros2_control/gz_ros2_control/src/gz_system.cpp").read_text()
    write = plugin.split("GazeboSimSystem::write(", 1)[1]
    assert write.index("actuator_current") < write.index("JointVelocityCmd(")
    assert "actuator_current ? this->dataPtr->joints_[i].joint_velocity_cmd : 0.0" in write
    sensors = (target / "gz-sim/src/systems/sensors/Sensors.cc").read_text()
    post = sensors.split("Sensors::PostUpdate(", 1)[1]
    assert post.index("monotonic_ns()") < post.index("updateTimeCv.wait")
    assert "sourceStartApplied = this->sourceStart" in sensors
    bridge = (target / "ros_gz/ros_gz_bridge/src/convert/senior_motion_msgs.cpp").read_text()
    assert "monotonic_ns" not in bridge
    assert bridge.count("extract(input.header(), output.timing)") == 5
    core = (target / "gz-sim/src/SimulationRunner.cc").read_text()
    rewind = core.split("if (this->requestedRewind)", 1)[1].split("// Seek", 1)[0]
    assert core.count("reset_simulation_epoch(") == 1
    assert rewind.index("reset_simulation_epoch(") < rewind.index("this->currentInfo.dt =")
    assert "simulation_world_key(" in rewind
    assert "&this->entityCompMgr, worldEntity(this->entityCompMgr)" in rewind
    build = (target / "core-build/CMakeLists.txt").read_text()
    assert "senior_motion_msgs 0.2.0 EXACT REQUIRED" in build
    assert "senior_motion_msgs::simulation_timing" in build
    assert "add_subdirectory" in build and '"${CMAKE_CURRENT_BINARY_DIR}/msgs"' in build
    assert "src/SimulationRunner.cc" in build and "src/systems/" not in build
    manifest = json.loads((BUNDLE / "manifest.json").read_text())
    assert manifest["gz-sim"]["commit"] == "d9b18d85a17400d32494bca4c3589f519aca7e81"
    assert manifest["gz_ros2_control"]["commit"] == "3f341cd49ed579cfb17422582eb14920c203f269"


def test_engine_and_two_plugins_share_one_real_dynamic_epoch_library(tmp_path):
    """Real linked DSOs, not a ROS/whole-engine/target ABI build claim."""
    import ctypes
    import sys

    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable; shared-state linkage not verified")
    suffix = ".dylib" if sys.platform == "darwin" else ".so"
    library = tmp_path / ("libsimulation_timing" + suffix)
    include = ROOT / "ros2/src/senior_motion_msgs/include"
    common = [
        compiler,
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-fPIC",
        "-fvisibility=hidden",
        "-I",
        str(include),
    ]
    identity = (
        ["-Wl,-install_name,@rpath/" + library.name]
        if sys.platform == "darwin"
        else ["-Wl,-soname," + library.name]
    )
    subprocess.run(
        [
            *common,
            "-shared",
            str(ROOT / "ros2/src/senior_motion_msgs/src/simulation_timing.cpp"),
            *identity,
            "-o",
            str(library),
        ],
        check=True,
        capture_output=True,
    )
    # These minimal entry adapters stand in only for link boundaries; all state
    # and reset behavior is the exact production shared library source above.
    exports = {
        "core": "void reset(const char * world) {\n"
        "  senior_motion_msgs::timing::reset_simulation_epoch(world);\n}",
        "sensor": (
            "const char * epoch(const char * world, unsigned long long tick) {\n"
            "  static thread_local std::string value;\n"
            "  value=senior_motion_msgs::timing::simulation_epoch(world,tick);\n"
            "  return value.c_str();\n}"
        ),
        "control": (
            "const char * epoch(const char * world, unsigned long long tick) {\n"
            "  static thread_local std::string value;\n"
            "  value=senior_motion_msgs::timing::simulation_epoch(world,tick);\n"
            "  return value.c_str();\n}"
        ),
    }
    loaded = {}
    for name, body in exports.items():
        body += (
            '\nextern "C" __attribute__((visibility("default"))) '
            "unsigned long long key(const void * ecm, unsigned long long entity) { "
            "return senior_motion_msgs::timing::simulation_world_key(ecm,entity); }"
        )
        source = tmp_path / (name + ".cpp")
        source.write_text(
            '#include "senior_motion_msgs/simulation_timing.hpp"\n'
            'extern "C" __attribute__((visibility("default"))) ' + body
        )
        output = tmp_path / (name + suffix)
        subprocess.run(
            [
                *common,
                "-shared",
                str(source),
                "-L",
                str(tmp_path),
                "-lsimulation_timing",
                "-Wl,-rpath," + str(tmp_path),
                "-o",
                str(output),
            ],
            check=True,
            capture_output=True,
        )
        loaded[name] = ctypes.CDLL(str(output))
        loaded[name].key.argtypes = [ctypes.c_void_p, ctypes.c_ulonglong]
        loaded[name].key.restype = ctypes.c_ulonglong
    reset = loaded["core"].reset
    reset.argtypes = [ctypes.c_char_p]
    reset.restype = None
    for name in ("sensor", "control"):
        loaded[name].epoch.argtypes = [ctypes.c_char_p, ctypes.c_ulonglong]
        loaded[name].epoch.restype = ctypes.c_char_p
    sensor, control = loaded["sensor"].epoch, loaded["control"].epoch
    first_ecm, second_ecm = ctypes.c_int(), ctypes.c_int()
    first_key = loaded["core"].key(ctypes.byref(first_ecm), 1)
    assert loaded["sensor"].key(ctypes.byref(first_ecm), 1) == first_key
    assert loaded["control"].key(ctypes.byref(first_ecm), 1) == first_key
    second_key = loaded["sensor"].key(ctypes.byref(second_ecm), 1)
    assert second_key != first_key  # Separate ECMs can both assign world entity #1.
    world, other = str(first_key).encode(), str(second_key).encode()
    initial = sensor(world, 0)
    unaffected = sensor(other, 10)
    assert control(world, 0) == initial  # Pause/same tick never resets.
    reset(world)  # Executed 0 -> 0 reset.
    one = control(world, 0)
    assert one != initial and one.rsplit(b":", 1)[1] == b"1"
    assert sensor(world, 0) == one
    assert sensor(other, 10) == unaffected
    assert sensor(world, 100) == one
    reset(world)  # Executed nonzero -> 0, exactly once across both plugins.
    two = sensor(world, 0)
    assert two.rsplit(b":", 1)[1] == b"2" and control(world, 0) == two
    reset(world)
    reset(world)  # Consecutive executed resets must each be visible.
    four = control(world, 0)
    assert four.rsplit(b":", 1)[1] == b"4" and sensor(world, 0) == four
    # Reloading a producer adapter still resolves the same shared dependency.
    restarted = ctypes.CDLL(str(tmp_path / ("sensor" + suffix)))
    restarted.epoch.argtypes = [ctypes.c_char_p, ctypes.c_ulonglong]
    restarted.epoch.restype = ctypes.c_char_p
    assert restarted.epoch(world, 0) == four
    from concurrent.futures import ThreadPoolExecutor

    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(reset, [world] * 32))
    assert sensor(world, 0).rsplit(b":", 1)[1] == b"36"
    assert sensor(world, 0) == control(world, 0)
    assert sensor(other, 10) == unaffected
    metadata = subprocess.check_output(
        ["otool", "-L", str(tmp_path / ("core" + suffix))]
        if sys.platform == "darwin"
        else ["readelf", "-d", str(tmp_path / ("core" + suffix))],
        text=True,
    )
    assert library.name in metadata
    symbols = subprocess.check_output(["nm", "-g", str(library)], text=True)
    assert "reset_simulation_epoch" in symbols and "simulation_epoch" in symbols
