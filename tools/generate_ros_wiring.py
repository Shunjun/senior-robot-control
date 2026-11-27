"""Generate build artifacts from the single project ROS wiring definition."""

import argparse
import json
import re
from pathlib import Path


def generate(definition: Path, output: Path, template: Path | None = None) -> None:
    data = json.loads(definition.read_text())
    if template is None:
        entries = dict(data["topics"])
        for role, owner in data["publishers"].items():
            entries[f"node_{role}"] = owner["name"]
            entries[f"ns_{role}"] = owner["namespace"]
        lines = [
            "// Generated from ros_wiring.json. Do not edit this artifact.",
            "#pragma once",
            "#include <string>",
            "namespace senior_sensor_ingress::wiring {",
        ]
        lines += [
            f"inline const std::string {key} = {json.dumps(value)};"
            for key, value in entries.items()
        ]
        lines += ["}  // namespace senior_sensor_ingress::wiring"]
        content = "\n".join(lines) + "\n"
    else:
        content = re.sub(
            r"@wiring:([a-z0-9_]+)@", lambda match: data["topics"][match[1]], template.read_text()
        )

        # Source YAML keys identify roles, installed keys address configured nodes.
        def node_key(match):
            owner = data["publishers"].get(match[1])
            if owner is None:
                return match[0]
            name = owner["namespace"].rstrip("/") + "/" + owner["name"]
            return json.dumps(name) + ":"

        content = re.sub(r"^([a-zA-Z_][a-zA-Z_0-9]*):", node_key, content, flags=re.M)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--definition", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--template", type=Path)
    args = parser.parse_args()
    generate(args.definition, args.output, args.template)


if __name__ == "__main__":
    main()
