"""Original agent journal SQL data; the journal execution owner stays in composition."""

AGENT_JOURNAL_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE IF NOT EXISTS agent_operation (
        operation_id TEXT PRIMARY KEY,
        robot_id TEXT NOT NULL,
        fingerprint TEXT NOT NULL,
        status TEXT NOT NULL,
        error TEXT,
        revision INTEGER
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS agent_deleted_task (
        task_id TEXT PRIMARY KEY,
        robot_id TEXT NOT NULL,
        operation_id TEXT NOT NULL,
        revision INTEGER NOT NULL
    )
    """,
)
