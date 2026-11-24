"""Release real control resources even when a test fails before fixture yield."""

import pytest


@pytest.fixture(autouse=True)
def close_control_resources(monkeypatch):
    from senior_motion_control.motion_core_output import MotionCoreOutput

    instances = []
    initialize = MotionCoreOutput.__init__

    def owned_initialize(self, *args, **kwargs):
        initialize(self, *args, **kwargs)
        instances.append(self)

    monkeypatch.setattr(MotionCoreOutput, "__init__", owned_initialize)
    yield
    for core in reversed(instances):
        if not core._closed:
            core.close()
