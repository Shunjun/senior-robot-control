import json
from pathlib import Path

from senior_control import cli


def test_pair_preserves_existing_credentials(tmp_path: Path, monkeypatch, capsys) -> None:
    credential = tmp_path / "credential.json"
    credential.write_text("existing secret")
    profile = tmp_path / "profile.json"
    profile.write_text(json.dumps({"kind": "simulation", "credentials": str(credential)}))
    monkeypatch.setattr(cli, "getpass", lambda _: (_ for _ in ()).throw(AssertionError()))

    assert (
        cli.main(
            [
                "pair",
                "--profile",
                str(profile),
                "--cloud-url",
                "https://example.test",
                "--name",
                "test",
            ]
        )
        == 1
    )
    assert credential.read_text() == "existing secret"
    assert "existing secret" not in capsys.readouterr().err


def test_pair_prompts_and_uses_tls(tmp_path: Path, monkeypatch) -> None:
    credential = tmp_path / "credential.json"
    profile = tmp_path / "profile.json"
    profile.write_text(json.dumps({"kind": "real", "credentials": str(credential)}))
    monkeypatch.setattr(cli, "getpass", lambda _: "one-time code")

    async def pair(url: str, code: str, name: str, path: Path) -> str:
        assert (url, code, name, path) == (
            "wss://example.test",
            "one-time code",
            "test",
            credential,
        )
        return "robot-a"

    monkeypatch.setattr(cli, "pair_device", pair)
    assert (
        cli.main(
            [
                "pair",
                "--profile",
                str(profile),
                "--cloud-url",
                "https://example.test",
                "--name",
                "test",
            ]
        )
        == 0
    )


def test_doctor_reports_missing_build_without_writing(tmp_path: Path, capsys) -> None:
    credential = tmp_path / "credential.json"
    credential.write_text('{"robot_id":"robot-a","device_token":"secret"}')
    credential.chmod(0o600)
    profile = tmp_path / "profile.json"
    profile.write_text(
        json.dumps(
            {
                "kind": "simulation",
                "credentials": str(credential),
                "ros_setup": str(tmp_path / "install" / "setup.bash"),
                "database": str(tmp_path / "robot.db"),
                "socket": str(tmp_path / "robot.sock"),
            }
        )
    )
    assert cli.main(["doctor", "--profile", str(profile)]) == 1
    output = capsys.readouterr().out
    assert "ROS build" in output
    assert "secret" not in output
    assert sorted(p.name for p in tmp_path.iterdir()) == ["credential.json", "profile.json"]
