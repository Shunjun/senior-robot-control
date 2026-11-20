"""Offline temporary-directory cleanup and mocked startup boundary checks."""

import asyncio
import fcntl
import os
from pathlib import Path
from types import SimpleNamespace as N

import pytest
from senior_perception import learning_cleanup as cleanup

CURRENT = ".startup-learning-" + "a" * 32
OLD = ".startup-learning-" + "b" * 32
SUFFIXES = (
    "",
    ".features.json",
    ".features.json.pending",
    ".pending",
    ".owner.lock",
    ".attempt-" + "c" * 32,
    ".attempt-" + "d" * 32 + ".pending",
)


def run_cleanup(root, claim=None):
    cleanup.cleanup_old_learning_files(
        root, str(claim or root / CURRENT), peer_lock_name=".senior-simulator.lock"
    )


def test_only_exact_old_generated_regular_files_are_removed(tmp_path):
    for name in (CURRENT, OLD):
        for suffix in SUFFIXES:
            (tmp_path / (name + suffix)).write_text("keep current run")
    unrelated = [
        "agent.db",
        ".startup-learning-random",
        OLD + ".backup",
        OLD + ".features.json.tmp",
        OLD + ".features.json.pending.more",
    ]
    for name in unrelated:
        (tmp_path / name).write_text("keep unrelated")
    directory = tmp_path / (".startup-learning-" + "c" * 32)
    directory.mkdir()
    nested = directory / OLD
    nested.write_text("no recursion")
    target = tmp_path / "target"
    target.write_text("no traversal")
    link = tmp_path / (".startup-learning-" + "d" * 32)
    link.symlink_to(target)
    run_cleanup(tmp_path)
    assert all(
        (tmp_path / (CURRENT + suffix)).read_text() == "keep current run" for suffix in SUFFIXES
    )
    assert all(not (tmp_path / (OLD + suffix)).exists() for suffix in SUFFIXES)
    assert all((tmp_path / name).exists() for name in unrelated)
    assert nested.read_text() == "no recursion" and link.is_symlink()
    assert target.read_text() == "no traversal"


def test_orphaned_real_sidecars_are_removed_but_symlink_sidecars_are_not(tmp_path):
    (tmp_path / (OLD + ".features.json.pending")).write_text("interrupted atomic write")
    external = tmp_path / "external"
    external.write_text("untouched")
    linked = tmp_path / (OLD + ".features.json")
    linked.symlink_to(external)
    run_cleanup(tmp_path)
    assert not (tmp_path / (OLD + ".features.json.pending")).exists()
    assert linked.is_symlink() and external.read_text() == "untouched"


@pytest.mark.parametrize("bad_claim", ["other", "../" + CURRENT])
def test_invalid_current_claim_skips_all_deletion(tmp_path, caplog, bad_claim):
    old = tmp_path / OLD
    old.touch()
    run_cleanup(tmp_path, tmp_path / bad_claim)
    assert old.exists() and "skipped" in caplog.text


def test_symlink_root_is_not_followed(tmp_path, caplog):
    real = tmp_path / "real"
    real.mkdir()
    old = real / OLD
    old.touch()
    linked = tmp_path / "linked"
    linked.symlink_to(real, target_is_directory=True)
    run_cleanup(linked)
    assert old.exists() and "cannot open" in caplog.text


def test_active_peer_supervisor_protects_all_run_markers(tmp_path, caplog):
    old = tmp_path / OLD
    old.touch()
    with (tmp_path / ".senior-simulator.lock").open("wb") as peer:
        fcntl.flock(peer.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        run_cleanup(tmp_path)
    assert old.exists() and "skipped or incomplete" in caplog.text
    run_cleanup(tmp_path)
    assert not old.exists()


def test_unlink_failure_logs_and_continues(tmp_path, monkeypatch, caplog):
    old = tmp_path / OLD
    sidecar = tmp_path / (OLD + ".features.json")
    old.touch()
    sidecar.touch()
    original = os.unlink

    def fail_one(path, **kwargs):
        if path == OLD:
            raise PermissionError("simulated permission failure")
        return original(path, **kwargs)

    monkeypatch.setattr(cleanup.os, "unlink", fail_one)
    run_cleanup(tmp_path)
    assert old.exists() and not sidecar.exists()
    assert "simulated permission failure" in caplog.text


def test_enumeration_failure_is_nonfatal(tmp_path, monkeypatch, caplog):
    (tmp_path / OLD).touch()

    def failed_scan(*args):
        raise OSError("enumeration failed")

    monkeypatch.setattr(cleanup.os, "scandir", failed_scan)
    run_cleanup(tmp_path)
    assert (tmp_path / OLD).exists() and "enumeration failed" in caplog.text


class Child:
    def __init__(self, dead=False):
        self.returncode = 2 if dead else None
        self.done = asyncio.Event()
        if dead:
            self.done.set()

    async def wait(self):
        await self.done.wait()
        return self.returncode

    def terminate(self):
        self.returncode = 0
        self.done.set()


class Server:
    def __init__(self, *args, **kwargs):
        pass

    async def start(self):
        pass

    async def close(self):
        pass


@pytest.mark.parametrize("outcome", ["success", "adapter_failed", "agent_exited", "cleanup_failed"])
async def test_hardware_cleanup_only_once_after_verified_live_agent_under_lock(
    tmp_path, monkeypatch, caplog, outcome
):
    from senior_control import docker_start_helper as helper

    old = tmp_path / OLD
    old.touch()
    events = []
    stop = None
    calls = []

    async def prepare(path):
        pass

    async def ready(path, robot, domain, adapter, stopping):
        nonlocal stop
        stop = stopping
        events.append("verified")
        if outcome == "adapter_failed":
            stopping.set()
            raise ValueError("adapter failed")
        return [1, 2, 3]

    async def spawn(*args, env, **kwargs):
        events.append(args[0])
        child = Child(dead=outcome == "agent_exited" and args[0] == "agent")
        if args[0] == "camera":
            stop.set()
        return child

    async def terminate(child, **kwargs):
        if child is not None:
            child.terminate()

    original = cleanup.cleanup_old_learning_files

    def clean(root, claim, **kwargs):
        calls.append(claim)
        assert events[0] == "agent" and events[-1] == "verified"
        with (root / ".agent-service.lock").open("rb") as contender:
            with pytest.raises(BlockingIOError):
                fcntl.flock(contender.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        if outcome == "cleanup_failed":
            # Exercise the production cleaner's nonfatal OS-error handling.
            with monkeypatch.context() as patch:
                patch.setattr(
                    cleanup.os,
                    "scandir",
                    lambda *args: (_ for _ in ()).throw(OSError("failed cleanup")),
                )
                original(root, claim, **kwargs)
        else:
            original(root, claim, **kwargs)
        Path(claim).write_text("current claim retained")

    monkeypatch.setattr(helper, "IpcServer", Server)
    monkeypatch.setattr("senior_perception.learning_service.LearningService", Server)
    monkeypatch.setattr(helper, "_prepare_socket", prepare)
    monkeypatch.setattr(helper, "_record", lambda *args, **kwargs: None)
    monkeypatch.setattr(helper, "read_credentials", lambda *args: N(robot_id="robot"))
    monkeypatch.setattr(helper, "_adapter_command", lambda *args: ["adapter"])
    monkeypatch.setattr(helper, "_motion_command", lambda *args: ["motion"])
    monkeypatch.setattr(helper, "_ros_launch_command", lambda *args: ["camera"])
    monkeypatch.setattr("senior_perception.learning_service.LearningService", Server)
    monkeypatch.setattr(helper, "_wait_for_adapter", ready)
    monkeypatch.setattr(helper, "_terminate_motion_stack", terminate)
    monkeypatch.setattr(helper.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(cleanup, "cleanup_old_learning_files", clean)
    profile = N(ros_domain=None, ros_setup=None)
    assert await helper.supervise(tmp_path, ["agent"], {}, profile=profile) == (
        2 if outcome == "agent_exited" else 0
    )
    assert len(calls) == (1 if outcome in {"success", "cleanup_failed"} else 0)
    assert old.exists() == (outcome != "success")
    if calls:
        assert Path(calls[0]).read_text() == "current claim retained"
    if outcome == "cleanup_failed":
        assert "failed cleanup" in caplog.text


@pytest.mark.parametrize("outcome", ["success", "adapter_failed", "agent_exited"])
async def test_simulation_boundary_preserves_failures_and_cleans_once(
    tmp_path, monkeypatch, outcome
):
    from senior_control import simulation_cli as helper

    old = tmp_path / OLD
    old.touch()
    stop = None
    spawned = []
    calls = []
    profile = N(
        socket=tmp_path / "agent.sock",
        credentials=tmp_path / "device.json",
        video=False,
        ros_domain=43,
        cloud_url="wss://unused.example",
        database=tmp_path / "agent.db",
        ros_setup=tmp_path / "setup.bash",
    )

    async def ready(path, robot, ros, stopping):
        nonlocal stop
        stop = stopping
        if outcome == "adapter_failed":
            stopping.set()
            raise ValueError("adapter failed")

    async def spawn(*args, **kwargs):
        child = Child(dead=outcome == "agent_exited" and not spawned)
        spawned.append(child)
        return child

    async def terminate(child, timeout):
        child.terminate()

    original = cleanup.cleanup_old_learning_files

    def clean(root, claim, **kwargs):
        assert len(spawned) == 2 and all(child.returncode is None for child in spawned)
        with (root / ".senior-simulator.lock").open("rb") as contender:
            with pytest.raises(BlockingIOError):
                fcntl.flock(contender.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        calls.append(claim)
        original(root, claim, **kwargs)
        stop.set()

    monkeypatch.setattr("senior_perception.learning_service.LearningService", Server)
    monkeypatch.setattr(helper, "_wait_for_adapter", ready)
    monkeypatch.setattr(helper.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(helper, "_stop", terminate)
    monkeypatch.setattr(helper, "_notify", lambda *args: None)
    from senior_control import docker_start_helper

    async def stop_group(child):
        if child is not None:
            child.terminate()

    monkeypatch.setattr(docker_start_helper, "_terminate_motion_stack", stop_group)
    monkeypatch.setattr(cleanup, "cleanup_old_learning_files", clean)
    # Production main() holds this existing ownership lock around supervise().
    with (tmp_path / ".senior-simulator.lock").open("wb") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert await helper.supervise(profile, N(robot_id="robot")) == (
            2 if outcome == "agent_exited" else 0
        )
    assert len(calls) == (1 if outcome == "success" else 0)
    assert old.exists() == (outcome != "success")


def test_active_record_writer_preserves_entire_old_run_until_closed(tmp_path):
    for suffix in SUFFIXES:
        (tmp_path / (OLD + suffix)).write_text("old active run")
    with (tmp_path / (OLD + ".owner.lock")).open("rb") as owner:
        fcntl.flock(owner.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        run_cleanup(tmp_path)
        assert all((tmp_path / (OLD + suffix)).exists() for suffix in SUFFIXES)
    run_cleanup(tmp_path)
    assert all(not (tmp_path / (OLD + suffix)).exists() for suffix in SUFFIXES)
