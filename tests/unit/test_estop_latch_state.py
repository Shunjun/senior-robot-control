import json

import pytest

from senior_control.safety.arbiter import PersistentEstop


def test_uninitialized_or_corrupt_state_is_latched(tmp_path):
    path = tmp_path / "estop.json"
    latch = PersistentEstop(path, "paired-car")
    assert latch.latched
    latch.close()
    path.write_text("broken")
    latch = PersistentEstop(path, "paired-car")
    assert latch.latched
    latch.close()


def test_committed_estop_survives_restart_and_is_robot_bound(tmp_path):
    path = tmp_path / "estop.json"
    PersistentEstop.initialize(path, "paired-car")
    latch = PersistentEstop(path, "paired-car")
    assert not latch.latched
    latch.estop()
    latch.close()
    assert json.loads(path.read_text())["latched"] is True
    again = PersistentEstop(path, "paired-car")
    assert again.latched and again.revision == 1
    again.close()
    wrong = PersistentEstop(path, "other-car")
    assert wrong.latched
    wrong.close()
    with pytest.raises(FileExistsError):
        PersistentEstop.initialize(path, "paired-car")


def test_initialization_tool_never_overwrites_existing_state(tmp_path):
    from senior_control.safety.initialize_estop import main

    path = tmp_path / "state.json"
    assert main(["--simulation", "--robot-id", "paired-car", "--state", str(path)]) == 0
    original = path.read_bytes()
    with pytest.raises(FileExistsError):
        main(["--simulation", "--robot-id", "paired-car", "--state", str(path)])
    assert path.read_bytes() == original


def test_initialization_refuses_a_running_owner(tmp_path):
    path = tmp_path / "state.json"
    owner = PersistentEstop(path, "paired-car")
    try:
        with pytest.raises(BlockingIOError):
            PersistentEstop.initialize(path, "paired-car")
        assert not path.exists()
    finally:
        owner.close()
