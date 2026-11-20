"""Fixed operator actions; Shell owns syntax and interactive input.

The installed platform supplies actual per-invocation effects. This module never
imports the legacy SDK or selects a second database/runtime owner.
"""

import sys
from dataclasses import replace
from pathlib import Path
from typing import TextIO

from vehicle_manager import cli
from vehicle_manager.platform_ports import DockerOperatorPorts, OperatorPorts


def main(arguments: list[str], *, ports: OperatorPorts, secret_input: TextIO = sys.stdin) -> int:
    """Private positional action protocol, deliberately not an operator argparse."""
    if len(arguments) != 7:
        print("Invalid normalized operator request.", file=sys.stderr)
        return 2
    action, selection, path, headless, dry_run, cloud_url, name = arguments
    if (
        action
        not in {
            "start",
            "stop",
            "status",
            "doctor",
            "deploy",
            "pair",
            "pair-preflight",
            "log-route",
        }
        or selection not in {"profile", "control-dir"}
        or headless not in {"true", "false"}
        or dry_run not in {"true", "false"}
        or not path
    ):
        print("Invalid normalized operator request.", file=sys.stderr)
        return 2
    if action == "pair":
        # The old _pair callback binds its own getpass; use the same canonical
        # implementation with this invocation's stdin effect instead.
        ports = replace(ports, getpass=lambda _: read_pair_code(secret_input))
        pair_ports = ports
        ports = replace(
            ports, pair=lambda profile, url, name: cli._pair(profile, url, name, ports=pair_ports)
        )
    return cli.execute(
        action,
        profile_path=Path(path) if selection == "profile" else None,
        control_dir=Path(path) if selection == "control-dir" else None,
        headless=headless,
        dry_run=dry_run == "true",
        cloud_url=cloud_url,
        name=name,
        native_only=True,
        ports=ports,
    )


def docker_action(
    *, ports: OperatorPorts, runtime_ports: DockerOperatorPorts, source: TextIO = sys.stdin
) -> int:
    """One fixed request from Shell, using original container runtime owners.

    Host systemd stays in Shell. The mapped filesystem is written only by the
    original atomic install owner; receipts fence rollback to those exact files.
    """
    import json
    import re
    import stat

    from vehicle_manager import deployment, simulation
    from vehicle_manager.profile import load_profile

    try:
        request = json.loads(source.readline())
        action = request["action"]
        root = Path(request["host_root"])
        mapped = Path(request["runtime_root"])
        inspection = request["inspection"]
        container = request["container"]
        docker = request["docker"]
        if (
            not isinstance(container, str)
            or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}", container)
            or not Path(docker).is_absolute()
            or inspection["State"]["Running"] is not True
            or deployment.container_directory(root, inspection["Mounts"]) != mapped
        ):
            raise ValueError("invalid running-container binding")
        profile = load_profile(mapped / "config.json", control_dir=root)
        if profile.kind != "real":
            raise ValueError("Docker simulation is not supported")
        if profile.container not in {None, container}:
            raise ValueError("selected container differs from configuration")
        profile = profile.model_copy(update={"container": container})
        unit = cli._unit(root / "config.json")
        contents = ""
        if action not in {"pair", "pair-preflight"}:
            deployment._validate_paths(profile, mapped, max_path=runtime_ports.max_path)
            contents = deployment.render_service(
                profile,
                str(mapped.stat().st_uid),
                container_root=mapped,
                docker_executable=docker,
                start_container=False,
            )
        if action == "metadata":
            print(json.dumps({"unit": unit, "user": False}))
        elif action == "unit":
            print(contents, end="")
        elif action == "preview":
            print(f"Environment: {root / 'agent.env'}")
            print(f"Service: {root / unit} (linked by systemd on the host)")
            print(f"Runtime data directory: {mapped}")
            print(contents)
        elif action == "install":
            # The host checks Linux/root and link ownership before this request.
            # Re-check the mounted original directory/credentials, preserving UID.
            deployment.private_directory(mapped)
            from vehicle_manager.agent.pairing import read_credentials

            read_credentials(mapped / "device.json")
            if (mapped / "device.json").stat().st_uid != mapped.stat().st_uid:
                raise ValueError("credentials and control directory must have the same owner")
            files = deployment.install_artifacts(
                mapped,
                unit,
                deployment.environment(profile, mapped),
                contents,
                install_once=ports.install_once,
            )
            try:
                print(
                    json.dumps(
                        [
                            {
                                "name": file.name,
                                "device": file.stat().st_dev,
                                "inode": file.stat().st_ino,
                            }
                            for file in files
                        ]
                    )
                )
            except BaseException:
                for file in reversed(files):
                    file.unlink()
                raise
        elif action == "rollback":
            receipt = request["receipt"]
            if len(receipt) != 2 or {entry["name"] for entry in receipt} != {"agent.env", unit}:
                raise ValueError("invalid deployment receipt")
            for entry in reversed(receipt):
                file = mapped / entry["name"]
                info = file.lstat()
                if not stat.S_ISREG(info.st_mode) or (info.st_dev, info.st_ino) != (
                    entry["device"],
                    entry["inode"],
                ):
                    raise ValueError("deployment artifact changed; refusing rollback")
                file.unlink()
        elif action in {"pair", "pair-preflight", "doctor", "status"}:
            runtime = runtime_ports.runtime_profile(mapped)
            if action in {"pair", "pair-preflight"}:
                url, name = request["cloud_url"], request["name"]
                if action == "pair-preflight":
                    cli.validate_pair(runtime, url, name, ports=ports)
                else:
                    pair_ports = replace(ports, getpass=lambda _: read_pair_code(source))
                    cli._pair(runtime, url, name, ports=pair_ports)
                    print("Device paired.")
            elif action == "doctor":
                deployment.private_directory(mapped)
                from vehicle_manager.agent.pairing import read_credentials

                read_credentials(runtime.credentials)
                runtime_ports.validate_runtime(runtime)
                print("Agent Python and control directory verified; ROS IPC is checked at startup.")
                print("Physical runtime is unverified.")
            else:
                return simulation.service_status(
                    unit,
                    runtime,
                    user=False,
                    probe=runtime_ports.probe,
                    host_properties=request["properties"],
                    host_event=request["event"],
                )
        else:
            raise ValueError("unknown Docker operator action")
        return 0
    except (OSError, ValueError, KeyError, TypeError, EOFError, KeyboardInterrupt):
        print(
            "Docker action failed. Check the running container, private bind mount and profile.",
            file=sys.stderr,
        )
        return 1


def read_pair_code(source: TextIO) -> str:
    line = source.readline()
    if not line:
        raise EOFError
    return line.removesuffix("\n")
