"""Formal management Agent entry with the original CLI and exit semantics."""

from vehicle_manager.agent.application import main as application_main
from vehicle_manager.agent.config import AgentConfig
from vehicle_manager.agent.production import run_agent


def main() -> int:
    return application_main(configuration=AgentConfig.from_environment, run_agent=run_agent)


if __name__ == "__main__":
    raise SystemExit(main())
