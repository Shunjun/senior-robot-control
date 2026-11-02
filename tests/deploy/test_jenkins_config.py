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
        "SENIOR_SECRET='" + "x" * 32 + "'\nADMIN_PASSWORD_HASH='" + fake_hash + "'\n"
    )
    validator = ROOT / "deploy/validate-jenkins-env.py"
    subprocess.run([sys.executable, str(validator), str(credential)], check=True)
    env = dict(
        PATH=os.environ["PATH"],
        DOMAIN="rosrouter.doomain.cn",
        ACME_EMAIL="ops@example.com",
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
        "COMPOSE_FILE='bad'\n",
        "SENIOR_SECRET='duplicate'\n",
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


def test_pipeline_polls_for_scm_changes():
    pipeline = (ROOT / "Jenkinsfile").read_text()
    assert "pollSCM('H/5 * * * *')" in pipeline
