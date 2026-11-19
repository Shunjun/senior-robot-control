"""Independent cloud composition; construction opens no socket or database."""

from collections.abc import Sequence

from fastapi import FastAPI

from cloud.app import create_app as _create_app
from cloud.config import CloudSettings
from cloud.diagnostics import DiagnosticAsyncQueue, DiagnosticDeque, configure_diagnostics
from cloud.limits import CloudLimits
from cloud.migrations import migrate
from cloud.schema import historical_migrations
from cloud.storage import Database


def create_app(
    settings: CloudSettings,
    *,
    database: Database,
    admin_password_hash: str,
    allowed_origins: Sequence[str],
    limits: CloudLimits | None = None,
) -> FastAPI:
    """Wire cloud owners, preserving explicit DB ownership and the existing lifespan."""

    async def prepare_database() -> None:
        await migrate(database, historical_migrations())

    return _create_app(
        settings,
        database=database,
        admin_password_hash=admin_password_hash,
        allowed_origins=allowed_origins,
        limits=limits,
        prepare_database=prepare_database,
        configure_diagnostics=lambda: configure_diagnostics(settings.diagnostics),
        deque_factory=DiagnosticDeque,
        queue_factory=DiagnosticAsyncQueue,
    )
