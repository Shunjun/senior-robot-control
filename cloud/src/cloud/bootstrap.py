"""Production cloud entry point; imports and factory construction start no service."""

import os
from collections.abc import Mapping
from importlib import import_module
from pathlib import Path

from fastapi import FastAPI
from pydantic import SecretStr

from cloud.application import create_app
from cloud.config import CloudSettings
from cloud.storage import Database


def create_app_from_environment(environment: Mapping[str, str] | None = None) -> FastAPI:
    """Read exactly the existing deployment fields, preserving their explicit paths."""
    values = os.environ if environment is None else environment
    return create_app(
        CloudSettings(secret=SecretStr(values["SENIOR_SECRET"])),
        database=Database(Path(values["DATABASE_PATH"])),
        admin_password_hash=values["ADMIN_PASSWORD_HASH"],
        allowed_origins=(values["ALLOWED_ORIGIN"],),
    )


def main() -> None:
    # The server extra is loaded only when the production entry point is invoked.
    uvicorn = import_module("uvicorn")

    app = create_app_from_environment()
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        proxy_headers=True,
        forwarded_allow_ips=os.environ["TRUSTED_PROXY"],
    )


if __name__ == "__main__":
    main()
