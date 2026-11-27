"""Project ROS wiring: one definition source shared by runtime and native builds."""

import json
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType, SimpleNamespace


@dataclass(frozen=True)
class Publisher:
    name: str
    namespace: str


def load_wiring():
    return json.loads(Path(__file__).with_suffix(".json").read_text())


_definition = load_wiring()
TOPICS = SimpleNamespace(**_definition["topics"])
PUBLISHERS = MappingProxyType(
    {key: Publisher(**value) for key, value in _definition["publishers"].items()}
)


def publisher(role: str) -> Publisher:
    return PUBLISHERS[role]


def matching_publisher(endpoints, name: str, namespace: str, message_type: str | None = None):
    """Inspect the existing graph snapshot once; GID is diagnostic metadata only."""
    if len(endpoints) != 1:
        return None
    endpoint = endpoints[0]
    if endpoint.node_name != name or endpoint.node_namespace != namespace:
        return None
    if message_type is not None and endpoint.topic_type != message_type:
        return None
    return endpoint


def publisher_identity(endpoints, role: str, message_type: str | None = None) -> str:
    """Return a nonempty availability marker with optional diagnostic identity."""
    expected = publisher(role)
    endpoint = matching_publisher(endpoints, expected.name, expected.namespace, message_type)
    if endpoint is None:
        return ""
    return bytes(endpoint.endpoint_gid).hex() or "available"
