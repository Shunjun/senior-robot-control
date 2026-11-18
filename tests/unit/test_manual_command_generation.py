"""Manual intent must use the generation established by its preceding zero barrier."""

import ast
import json
import os
import subprocess
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from senior_control.agent.manual_control import ManualIngress
from senior_control.errors import ErrorCode
from senior_control.models import Velocity
from senior_control.motion.contracts import ManualIntent
from senior_control.motion.manual_authority import ManualAuthorityError, ManualAuthorityState
from senior_control.protocol import ManualIntentEnvelope, RecoveryEnvelope, SimpleEnvelope
from senior_control.safety.arbiter import Arbiter, CommandResult, FinalCommandFence
from senior_control.settings.models import DrivingSettings, simulation_settings_profile


def command_class():
    """Extract production methods without importing ROS; optional ref proves the regression."""
    path = "ros2/src/senior_robot/senior_robot/command_node.py"
    ref = os.environ.get("SENIOR_TEST_COMMAND_REF")
    source = (
        subprocess.check_output(["git", "show", f"{ref}:{path}"], text=True)
        if ref
        else Path(path).read_text()
    )
    methods = {
        "manual_request", "submit", "_close_manual", "publish_command",
        "_publish_manual_target", "_publish_barrier",
    }
    tree = ast.parse(source, filename=path)
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "CommandNode"
    )
    cls.bases = []
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in methods]
    assert {node.name for node in cls.body} == methods
    module = ast.Module(
        body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), cls],
        type_ignores=[],
    )
    namespace = {
        "replace": replace, "FinalCommand": SimpleNamespace, "ManualIntent": ManualIntent,
        "ManualAuthorityError": ManualAuthorityError, "ManualAuthorityState": ManualAuthorityState,
        "ManualIntentEnvelope": ManualIntentEnvelope, "RecoveryEnvelope": RecoveryEnvelope,
        "SimpleEnvelope": SimpleEnvelope, "DrivingSettings": DrivingSettings,
        "CommandResult": CommandResult, "ErrorCode": ErrorCode,
    }
    exec(compile(ast.fix_missing_locations(module), path, "exec"), namespace)
    return namespace["CommandNode"]


def test_manual_intents_use_prepared_and_stopped_barrier_generations():
    node = command_class()()
    clock = SimpleNamespace(now=10.0)
    node._monotonic_clock = SimpleNamespace(monotonic=lambda: clock.now)
    node._owner_boot, node._generation, node._active = "command-boot", 1, False
    node._navigation_source, node._manual_target = None, None
    node._manual_ingress = ManualIngress("robot", "command-boot", ttl=0.3)
    node._arbiter = Arbiter("robot", 300)
    node.get_parameter = lambda name: SimpleNamespace(value="robot")
    fence = FinalCommandFence()
    publisher = b"command-publisher"
    fence.set_publisher(publisher)
    frames, admitted = [], []

    def publish(frame):
        frames.append(frame)
        admitted.append(fence.accept(
            publisher, frame.owner_boot_id, frame.generation, frame.sequence,
            frame.steady_deadline, Velocity(frame.vx, frame.wz), clock.now,
            session=frame.session_id, epoch=frame.connection_epoch,
        ))

    node._final_publisher = SimpleNamespace(publish=publish)
    driving = simulation_settings_profile().driving_defaults.model_dump()

    def envelope(kind, payload, sequence):
        return {
            "v": 1, "robot_id": "robot", "session_id": "controller",
            "connection_epoch": 1, "controller_generation": 1,
            "request_id": f"request-{sequence}", "seq": sequence,
            "expires_at": 100.0, "type": kind, "payload": payload,
        }

    def request(kind, payload, sequence):
        return node.manual_request({
            "envelope": json.dumps(envelope(kind, payload, sequence)),
            "driving": driving, "received_steady": clock.now,
            "deadline_steady": clock.now + 0.3,
        })

    prepared = request("control.acquire", {}, 1)
    assert prepared["ok"] and prepared["state"] == "prepared"
    authority = prepared["authority_id"]
    for gesture in (1, 2):
        barrier = frames[-1]
        assert admitted[-1] and barrier.sequence == 0
        assert barrier.vx == barrier.wz == 0
        assert barrier.close_reason == ("prepared" if gesture == 1 else "release")
        clock.now += 0.1
        result = request("drive.intent", {
            "authority_id": authority, "gesture": gesture, "sequence": gesture,
            "linear": 0.1, "turn": 0, "gear": "low",
        }, gesture * 2)
        intent = frames[-1]
        assert result["ok"] and intent.vx > 0 and intent.close_reason == "intent"
        assert admitted[-1], "FinalCommandFence rejected the actual manual intent"
        assert intent.generation == barrier.generation
        assert intent.sequence == gesture and intent.source_id == authority
        assert fence.usable(clock.now)
        clock.now += 0.1
        stop = SimpleEnvelope.model_validate(envelope(
            "drive.stop", {"authority_id": authority, "gesture": gesture}, gesture * 2 + 1,
        ))
        assert node.submit(stop).accepted
        assert frames[-1].generation > intent.generation
        assert admitted[-1] and not fence.usable(clock.now)
        assert node._manual_ingress.view(clock.now).state is ManualAuthorityState.PREPARED
        clock.now += 1.0  # Stopped ownership survives the old active sample's TTL.
