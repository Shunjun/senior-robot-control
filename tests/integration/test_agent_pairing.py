from pathlib import Path

import httpx
import pytest
from test_cloud import Cloud
from test_cloud import cloud as cloud

from senior_control.agent.pairing import pair_device, read_credentials


async def test_pair_api_writes_private_credentials_once(cloud: Cloud, tmp_path: Path) -> None:
    csrf = await cloud.login()
    response = await cloud.client.post(
        "/api/pairing-codes", json={}, headers={"X-CSRF-Token": csrf}
    )
    path = tmp_path / "device.json"
    robot_id = await pair_device(
        "wss://cloud.test/ws/agent",
        response.json()["code"],
        "robot",
        path,
        transport=httpx.ASGITransport(app=cloud.app),
    )
    assert path.stat().st_mode & 0o777 == 0o600
    credentials = read_credentials(path)
    assert credentials.robot_id == robot_id
    assert credentials.device_token
    original = path.read_bytes()
    with pytest.raises(FileExistsError):
        await pair_device(
            "wss://cloud.test/ws/agent",
            "unused",
            "robot",
            path,
            transport=httpx.ASGITransport(app=cloud.app),
        )
    assert path.read_bytes() == original


async def test_pair_failure_leaves_no_credential_file(cloud: Cloud, tmp_path: Path) -> None:
    path = tmp_path / "device.json"
    with pytest.raises(ValueError, match="pairing failed"):
        await pair_device(
            "wss://cloud.test/ws/agent",
            "invalid",
            "robot",
            path,
            transport=httpx.ASGITransport(app=cloud.app),
        )
    assert not path.exists()
    assert not list(tmp_path.glob(".device.json.*"))
