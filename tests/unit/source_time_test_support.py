"""Synthetic source boundaries with explicit host and physical coordinates."""

from types import SimpleNamespace

import pytest

from senior_control.sensors.time_proof import SourceTime

DOMAIN = "00000000-0000-0000-0000-000000000001"
WORLD = "test-world:0"


def capture_time(
    measured,
    *,
    start=None,
    sequence=1,
    epoch="test-producer",
    tick=None,
    world=WORLD,
    basis="sim_step",
):
    # This synthetic producer declares 1x unless a test supplies its physical tick.
    return SourceTime(
        DOMAIN,
        epoch,
        sequence,
        round((measured if start is None else start) * 1e9),
        round(measured * 1e9),
        basis,
        round((measured if tick is None else tick) * 1e9) if basis == "sim_step" else None,
        world if basis == "sim_step" else None,
    )


def wire_time(
    measured,
    *,
    start=None,
    sequence=1,
    epoch="test-producer",
    tick=None,
    world=WORLD,
    basis="sim_step",
):
    proof = capture_time(
        measured, start=start, sequence=sequence, epoch=epoch, tick=tick, world=world, basis=basis
    )
    return SimpleNamespace(
        clock_domain=proof.clock_domain,
        source_epoch=proof.source_epoch,
        sequence=proof.sequence,
        capture_start_steady_ns=proof.capture_start_steady_ns,
        capture_end_steady_ns=proof.capture_end_steady_ns,
        time_basis=proof.time_basis,
        measurement_tick_ns=0 if proof.measurement_tick_ns is None else proof.measurement_tick_ns,
        measurement_epoch="" if proof.measurement_epoch is None else proof.measurement_epoch,
    )


# Imported by pure unit modules at their explicit synthetic boot boundary.
# Kernel boot discovery itself is tested separately; this never changes capture time.
@pytest.fixture(autouse=True)
def synthetic_domain(monkeypatch):
    monkeypatch.setattr("senior_control.sensors.time_proof.monotonic_domain", lambda: DOMAIN)


def synthetic_capture_schedule(document):
    """Rebuild a test producer schedule for legacy geometry-only captures.

    These recordings did not retain producer monotonic time. Their historical
    receipt and ROS labels are chosen here as NEW synthetic emission and 1x
    physical coordinates, respectively. This fixture proves control composition,
    never recorded acquisition age, real transport delay, or physical RTF.
    Identities retain one immutable schedule across packet reuse; receivers do
    not add time. The original recorded JSON is left unchanged.
    """
    from copy import deepcopy
    from dataclasses import asdict

    value = deepcopy(document)

    def visit(node):
        if isinstance(node, list):
            for item in node:
                visit(item)
        elif isinstance(node, dict):
            if "sources" in node and "sampled_steady" in node:
                for row in node["sources"]:
                    sample = row["sample"]
                    end = sample.get("end_ros", sample.get("stamp_ros"))
                    start = sample.get("start_ros", end)
                    if end is None:
                        continue  # Condensed capture contains no original raw sample.
                    host = sample["received_steady"]
                    sample["measurement_steady"] = host
                    sample["start_steady"] = host - (end - start)
                    sample["end_steady"] = host
                    a = capture_time(host, tick=start).measurement
                    b = capture_time(host, tick=end).measurement
                    sample["measurement"] = asdict(b)
                    sample["measurement_interval"] = {"start": asdict(a), "end": asdict(b)}
                for pose in node.get("poses", []):
                    pose["measurement_steady"] = pose["received_steady"]
                    pose["measurement"] = asdict(
                        capture_time(pose["received_steady"], tick=pose["stamp_ros"]).measurement
                    )
            for child in node.values():
                visit(child)

    visit(value)
    return value
