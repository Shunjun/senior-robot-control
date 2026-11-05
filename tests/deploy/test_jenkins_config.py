from pathlib import Path

ROOT = Path(__file__).parents[2]


def test_web_build_forces_real_data_and_installs_build_dependencies():
    pipeline = (ROOT / "Jenkinsfile").read_text()
    assert "VITE_DEMO_DATA=0" in pipeline
    assert "npm ci --include=dev" in pipeline


def test_cloud_image_installs_websocket_protocol():
    dockerfile = (ROOT / "deploy/Dockerfile.cloud").read_text()
    assert "websockets==15.0.1" in dockerfile
    assert "uvicorn.protocols.websockets.websockets_impl" in dockerfile


def test_diagnostics_redacts_credential_values(tmp_path):
    import subprocess
    import sys

    credential = tmp_path / "credential"
    credential.write_text('SENIOR_SECRET=secret-value\nADMIN_PASSWORD_HASH="hash-value"\n')
    result = subprocess.run(
        [sys.executable, str(ROOT / "deploy/redact-logs.py"), str(credential)],
        input="secret-value hash-value\n",
        text=True,
        capture_output=True,
        check=True,
    )
    assert result.stdout == "[redacted] [redacted]\n"


def test_credential_validation_and_compose_preserve_scrypt(tmp_path):
    import os
    import subprocess
    import sys

    import yaml

    fake_hash = "scrypt$16384$8$1$" + "ab" * 16 + "$" + "cd" * 32
    credential = tmp_path / "env"
    credential.write_text(
        "DOMAIN='robot.example.com'\nHTTP_PORT='8098'\nHTTPS_PORT='8449'\nACME_EMAIL='ops@example.com'\nSENIOR_SECRET='"
        + "x" * 32
        + "'\nADMIN_PASSWORD_HASH='"
        + fake_hash
        + "'\n"
    )
    validator = ROOT / "deploy/validate-jenkins-env.py"
    result = subprocess.run(
        [sys.executable, str(validator), str(credential)],
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout == "robot.example.com false 8449\n"
    enabled = tmp_path / "enabled"
    enabled.write_text(
        credential.read_text()
        + "ENABLE_TURN='true'\nTURN_REALM='turn.example.com'\n"
        + "TURN_LISTEN_IP='0.0.0.0'\nTURN_EXTERNAL_IP='192.0.2.1'\n"
        + f"TURN_SHARED_SECRET='{'y' * 32}'\n"
    )
    result = subprocess.run(
        [sys.executable, str(validator), str(enabled)],
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout == "robot.example.com true 8449\n"
    env = dict(
        PATH=os.environ["PATH"],
        CLOUD_IMAGE="senior-control-cloud:test",
        RELEASE_DIR="/tmp/release",
        DEPLOY_ROOT="/tmp/deploy",
    )
    env.pop("ADMIN_PASSWORD_HASH", None)
    env.pop("SENIOR_SECRET", None)
    result = subprocess.run(
        [
            "docker",
            "compose",
            "--env-file",
            str(credential),
            "-f",
            str(ROOT / "deploy/compose.jenkins.yaml"),
            "config",
        ],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )
    # Compose's rendered YAML escapes literal dollars as $$ for reloading.
    rendered = yaml.safe_load(result.stdout)["services"]["cloud"]["environment"][
        "ADMIN_PASSWORD_HASH"
    ]
    assert rendered.replace("$$", "$") == fake_hash
    assert yaml.safe_load(result.stdout)["services"]["caddy"]["environment"]["ACME_EMAIL"] == (
        "ops@example.com"
    )
    caddy = yaml.safe_load(result.stdout)["services"]["caddy"]
    assert {port["target"]: port["published"] for port in caddy["ports"]} == {
        80: "8098",
        443: "8449",
    }

    import shutil

    manual = tmp_path / "manual"
    manual.mkdir()
    for name in ("compose.sh", "compose.yaml", "validate-jenkins-env.py"):
        shutil.copyfile(ROOT / "deploy" / name, manual / name)
    for settings, expected in (
        (credential, {"cloud", "caddy"}),
        (enabled, {"cloud", "caddy", "coturn"}),
    ):
        shutil.copyfile(settings, manual / ".env")
        result = subprocess.run(
            ["bash", str(manual / "compose.sh"), "config", "--services"],
            env=env,
            capture_output=True,
            text=True,
            check=True,
        )
        assert set(result.stdout.splitlines()) == expected
    manual_config = subprocess.run(
        ["bash", str(manual / "compose.sh"), "config"],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )
    manual_ports = yaml.safe_load(manual_config.stdout)["services"]["caddy"]["ports"]
    assert manual_ports == caddy["ports"]

    parsed = subprocess.run(
        [
            "docker",
            "compose",
            "--env-file",
            str(credential),
            "-f",
            str(ROOT / "deploy/compose.jenkins.yaml"),
            "config",
            "--environment",
        ],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )
    assert "ADMIN_PASSWORD_HASH=" + fake_hash in parsed.stdout.splitlines()
    for extra in (
        "ENABLE_TURN='yes'\n",
        "COMPOSE_FILE='bad'\n",
        "SENIOR_SECRET='duplicate'\n",
        "DOMAIN='duplicate.example.com'\n",
        "OTHER='bad'\n",
        "TURN_REALM='<placeholder>'\n",
        "TURN_REALM=unquoted\n",
    ):
        bad = tmp_path / "bad"
        bad.write_text(credential.read_text() + extra)
        result = subprocess.run(
            [sys.executable, str(validator), str(bad)], capture_output=True, text=True
        )
        assert result.returncode != 0
        assert fake_hash not in result.stderr
        if extra.startswith("COMPOSE_FILE="):
            assert "unsupported setting on line 7" in result.stderr
            assert extra.strip() not in result.stderr
        if extra.startswith("TURN_REALM=unquoted"):
            assert "line 7 must use" in result.stderr
            assert "unquoted" not in result.stderr


def test_credential_rejects_truncated_scrypt_hash_without_logging_it(tmp_path):
    import subprocess
    import sys

    truncated = "scrypt$" + "ab" * 32
    credential = tmp_path / "env"
    credential.write_text(
        "DOMAIN='robot.example.com'\nHTTP_PORT='8098'\nHTTPS_PORT='8449'\n"
        f"ACME_EMAIL='ops@example.com'\nSENIOR_SECRET='{'x' * 32}'\n"
        f"ADMIN_PASSWORD_HASH='{truncated}'\n"
    )
    result = subprocess.run(
        [sys.executable, str(ROOT / "deploy/validate-jenkins-env.py"), str(credential)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert "ADMIN_PASSWORD_HASH must be a complete scrypt hash" in result.stderr
    assert truncated not in result.stderr


def test_proxy_address_is_caddy_address_in_isolated_backend():
    import ipaddress

    import yaml

    compose = yaml.safe_load((ROOT / "deploy/compose.jenkins.yaml").read_text())
    cloud = compose["services"]["cloud"]
    caddy = compose["services"]["caddy"]
    backend = compose["networks"]["backend"]
    trusted = ipaddress.ip_address(cloud["environment"]["TRUSTED_PROXY"])
    assert backend["internal"] is True
    assert trusted == ipaddress.ip_address(caddy["networks"]["backend"]["ipv4_address"])
    assert trusted in ipaddress.ip_network(backend["ipam"]["config"][0]["subnet"])
    assert set(cloud["networks"]) == {"backend"}
    cloud_ip = ipaddress.ip_address(cloud["networks"]["backend"]["ipv4_address"])
    gateway = ipaddress.ip_address(backend["ipam"]["config"][0]["gateway"])
    assert cloud_ip != trusted and cloud_ip != gateway and trusted != gateway
    assert cloud_ip in ipaddress.ip_network(backend["ipam"]["config"][0]["subnet"])
    assert "default" in caddy["networks"]


def test_pipeline_uses_github_push_webhook():
    pipeline = (ROOT / "Jenkinsfile").read_text()
    assert "githubPush()" in pipeline
    assert "pollSCM" not in pipeline


def test_pipeline_leaves_deployment_settings_in_credential():
    pipeline = (ROOT / "Jenkinsfile").read_text()
    assert "ACME_EMAIL" not in pipeline
    assert "ENABLE_TURN" not in pipeline
