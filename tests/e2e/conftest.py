"""Shared acceptance-test configuration without starting external services."""

from __future__ import annotations

import hashlib
import json
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import pytest
import yaml

if TYPE_CHECKING:
    from collections.abc import Iterator


RECORDED_SCENE_SEED = 210_202_610


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--simulation-profile",
        action="store",
        default=None,
        metavar="PATH",
        help="simulation YAML used for the Linux acceptance run",
    )


@dataclass(frozen=True)
class SimulationProfile:
    source: Path
    database: Path
    api_port: int
    relay_port: int
    scene_seed: int
    config_hash: str


@pytest.fixture
def simulation_profile(
    request: pytest.FixtureRequest,
    tmp_path: Path,
) -> Iterator[SimulationProfile]:
    raw_path = request.config.getoption("--simulation-profile")
    source = Path(raw_path).resolve() if isinstance(raw_path, str) else None
    if source is None:
        source = Path(__file__).resolve().parents[2] / "config" / "simulation.yaml"
    payload = source.read_bytes()
    parsed = yaml.safe_load(payload)
    assert isinstance(parsed, dict)
    assert parsed.get("mode") == "simulation"

    sockets: list[socket.socket] = []
    ports: list[int] = []
    for _ in range(2):
        reservation = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        reservation.bind(("127.0.0.1", 0))
        reservation.listen(1)
        sockets.append(reservation)
        ports.append(int(reservation.getsockname()[1]))
    assert ports[0] != ports[1]

    record = {
        "profile": str(source),
        "database": str(tmp_path / "acceptance.sqlite3"),
        "ports": ports,
        "scene_seed": RECORDED_SCENE_SEED,
    }
    (tmp_path / "simulation-run.json").write_text(
        json.dumps(record, sort_keys=True),
        encoding="utf-8",
    )
    try:
        yield SimulationProfile(
            source=source,
            database=tmp_path / "acceptance.sqlite3",
            api_port=ports[0],
            relay_port=ports[1],
            scene_seed=RECORDED_SCENE_SEED,
            config_hash=hashlib.sha256(payload).hexdigest(),
        )
    finally:
        for reservation in sockets:
            reservation.close()
