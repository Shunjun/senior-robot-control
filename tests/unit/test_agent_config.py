from pathlib import Path

import pytest

from senior_control.agent.composition import AgentConfig


def test_agent_config_uses_explicit_environment_and_ignores_other_senior_keys() -> None:
    config = AgentConfig.from_environment(
        {
            "SENIOR_ROBOT_ID": "robot-a",
            "SENIOR_CLOUD_URL": "wss://cloud.test/ws/agent",
            "SENIOR_DEVICE_TOKEN": "secret",
            "SENIOR_AGENT_SOCKET": "/tmp/a.sock",
            "SENIOR_DATABASE_PATH": "/tmp/a.db",
            "SENIOR_UNRELATED": "ignored",
        }
    )
    assert config.robot_id == "robot-a"
    assert config.socket_path == Path("/tmp/a.sock")
    assert "secret" not in repr(config)


def test_agent_config_requires_encrypted_remote_transport() -> None:
    with pytest.raises(ValueError):
        AgentConfig(
            "robot-a", "ws://example.com/ws/agent", "secret", Path("/tmp/a.db"), Path("/tmp/a.sock")
        )


async def test_composition_opens_and_closes_database(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from senior_control.agent import main as agent_module
    from senior_control.agent.composition import run_agent
    from senior_control.storage.database import Database

    seen: list[bool] = []
    real_exit = Database.__aexit__

    async def close(self: Database, *args: object) -> None:
        await real_exit(self, None, None, None)
        seen.append(not self.is_open)

    async def run(self: agent_module.Agent) -> None:
        return None

    monkeypatch.setattr(Database, "__aexit__", close)
    monkeypatch.setattr(agent_module.Agent, "run", run)
    monkeypatch.setattr(agent_module.Agent, "install_signal_handlers", lambda self: None)
    config = AgentConfig(
        "robot-a", "wss://cloud.test/ws/agent", "secret", tmp_path / "robot.db", Path("/tmp/a.sock")
    )
    await run_agent(config)
    assert seen == [True]


def test_pair_cli_reads_code_privately_and_uses_credential_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import asyncio
    import sys

    import senior_control.agent.pairing as pairing
    from senior_control.agent import main as agent_module

    seen: list[tuple[str, str, str, Path]] = []

    async def pair(url: str, code: str, name: str, path: Path) -> str:
        seen.append((url, code, name, path))
        return "robot-a"

    def run_isolated(coro: object) -> object:
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)  # type: ignore[arg-type]
        finally:
            loop.close()

    path = tmp_path / "credentials.json"
    monkeypatch.setattr(agent_module.asyncio, "run", run_isolated)
    monkeypatch.setattr(pairing, "pair_device", pair)
    monkeypatch.setattr("getpass.getpass", lambda prompt: "one-time-secret")
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "agent",
            "--pair",
            "--cloud-url",
            "wss://cloud.test/ws/agent",
            "--device-name",
            "robot",
            "--credentials-file",
            str(path),
        ],
    )
    assert agent_module.main() == 0
    assert seen == [("wss://cloud.test/ws/agent", "one-time-secret", "robot", path)]


def test_token_file_requires_private_permissions(tmp_path: Path) -> None:
    from senior_control.agent.pairing import read_token

    token = tmp_path / "token"
    token.write_text("secret")
    token.chmod(0o644)
    with pytest.raises(ValueError, match="private"):
        read_token(token)
    token.chmod(0o600)
    assert read_token(token) == "secret"
