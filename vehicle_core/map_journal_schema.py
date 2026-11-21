"""Original map journal schema declarations; migration ordering belongs to management."""

OPERATION_JOURNAL_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE map_operation (
        operation_id TEXT PRIMARY KEY,
        action TEXT NOT NULL,
        payload TEXT NOT NULL,
        status TEXT NOT NULL,
        rollback_confirmed INTEGER NOT NULL,
        relocalized INTEGER NOT NULL
    )
    """,
    """
    CREATE TABLE map_operation_phase (
        operation_id TEXT NOT NULL,
        seq INTEGER NOT NULL,
        phase TEXT NOT NULL,
        PRIMARY KEY (operation_id, seq),
        FOREIGN KEY (operation_id) REFERENCES map_operation (operation_id)
    )
    """,
    """
    CREATE TABLE map_task_link (
        task_id TEXT PRIMARY KEY,
        definition TEXT NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        invalidated INTEGER NOT NULL
    )
    """,
    """
    CREATE TABLE map_home_link (
        robot_id TEXT PRIMARY KEY,
        source_robot_id TEXT NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        waypoint_id TEXT NOT NULL,
        status TEXT NOT NULL,
        adopted INTEGER NOT NULL
    )
    """,
)
