"""//Compatible: historical imports forward to the sole cloud implementation."""

from collections.abc import Sequence

from cloud.app import CLOUD_STATEMENTS, EPOCH_STATEMENTS
from cloud.app import CSRF_HEADER as CSRF_HEADER
from cloud.app import SESSION_COOKIE as SESSION_COOKIE
from cloud.app import BodyLimitMiddleware as BodyLimitMiddleware
from cloud.app import LoginBody as LoginBody
from cloud.app import PairBody as PairBody
from cloud.app import _ASGIReceive as _ASGIReceive
from cloud.app import _ASGISend as _ASGISend
from cloud.app import _bearer as _bearer
from cloud.app import _BodyTooLarge as _BodyTooLarge
from cloud.app import create_app as _create_app
from cloud.limits import CloudLimits
from fastapi import FastAPI

from senior_control.config import CloudSettings
from senior_control.runtime.diagnostics import (
    DiagnosticAsyncQueue,
    DiagnosticDeque,
    configure_diagnostics,
)
from senior_control.storage.database import Database
from senior_control.storage.migrations import Migration, migrate
from senior_control.storage.schema import DOMAIN_MIGRATIONS

# //Compatible: the original migration objects use the same SQL and version numbers.
CLOUD_MIGRATION = Migration(4, CLOUD_STATEMENTS)
EPOCH_MIGRATION = Migration(5, EPOCH_STATEMENTS)


def create_app(
    settings: CloudSettings,
    *,
    database: Database,
    admin_password_hash: str,
    allowed_origins: Sequence[str],
    limits: CloudLimits | None = None,
) -> FastAPI:
    """//Compatible: production bootstrap; inject the existing owners without opening them."""

    async def prepare_database() -> None:
        await migrate(database, [*DOMAIN_MIGRATIONS, CLOUD_MIGRATION, EPOCH_MIGRATION])

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
