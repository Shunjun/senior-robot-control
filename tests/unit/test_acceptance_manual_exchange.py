"""Exercise the ROS acceptance test's actual manual transport boundary without ROS."""

import importlib.util
import json
from pathlib import Path

import pytest


@pytest.fixture
def acceptance():
    path = Path(__file__).resolve().parents[1] / "ros/test_motion_core_acceptance.py"
    spec = importlib.util.spec_from_file_location("acceptance_manual_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("kind", ["control.acquire", "control.release"])
@pytest.mark.parametrize("ok", [True, False])
async def test_acquire_and_release_return_the_actual_request_response(acceptance, kind, ok):
    receipt = {"ok": ok, "accepted": ok, "authority_id": "authority"}
    requests = []

    class Requests:
        async def request(self, operation, **fields):
            requests.append((operation, fields))
            return receipt

    class Intents:
        async def send(self, *_args, **_fields):
            pytest.fail("acquire/release must use the request/response channel")

    envelope = {"type": kind, "seq": 7, "payload": {"authority_id": "authority"}}
    result = await acceptance.manual_exchange(Requests(), Intents(), envelope, 12.0)
    assert result is receipt  # Including rejection; never invent successful acceptance.
    assert len(requests) == 1
    operation, fields = requests[0]
    assert operation == "manual" and json.loads(fields["envelope"]) == envelope
    assert fields["received_steady"] == 12 and fields["deadline_steady"] == 12.2
    assert fields["driving"] == acceptance.DRIVING


async def test_intent_is_one_way_and_does_not_return_a_transport_receipt(acceptance):
    sent = []

    class Requests:
        async def request(self, *_args, **_fields):
            pytest.fail("intent must never wait on the request/response channel")

    class Intents:
        async def send(self, operation, **fields):
            sent.append((operation, fields))
            # Local transport completion is not a vehicle receipt.
            return {"ok": True, "accepted": True}

    envelope = {"type": "drive.intent", "seq": 8, "payload": {"linear": 1}}
    result = await acceptance.manual_exchange(Requests(), Intents(), envelope, 12.0)
    assert result is None
    assert len(sent) == 1
    operation, fields = sent[0]
    assert operation == "manual_intent" and json.loads(fields["envelope"]) == envelope
    assert fields["received_steady"] == 12 and fields["deadline_steady"] == 12.2
    assert fields["driving_version"] == 0 and fields["driving"] == acceptance.DRIVING
