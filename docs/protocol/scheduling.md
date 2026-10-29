# Local scheduling

The robot SQLite database is the authority for task definitions, revisions,
enabled state, and execution history. `TaskRunner.tick` only decides. It does
not write the database and does not call navigation. `TaskExecutor` claims an
instance, commits it, releases the write lock, and then calls the injected
navigation port with the persisted `execution_id`.

Use one `TaskStore` for the life of the process. Preparing another store on
the same database starts a new boot and moves in-flight executions to
`needs_review` so a restarted process does not send them again.

Schedules are either an interval or a five-field cron expression. Cron has no
seconds field. The fields are minute, hour, day of month, month, and day of
week. `0` and `7` are both Sunday. When day of month and day of week are both
restricted, a date matches if either field matches. A field is unrestricted
only when its text is `*`.

An interval is anchored at the task's UTC `created_at`. The first occurrence is
one interval after that instant, and later occurrences stay on that grid.
Daily and weekly schedules use the task timezone. The default timezone is
`Asia/Shanghai`.

A wall-clock time that falls in a daylight-saving gap is skipped. A repeated
wall-clock time runs once, at the earlier UTC offset.

A trigger is eligible only inside the current scan window, `(last_tick, now]`.
The first scan has no previous tick and does not catch up. If wall time and
monotonic time disagree by more than two seconds, the discontinuous interval is
recorded as a skip and only the monotonic tail of the scan can start work.
Triggers missed while offline, busy, or outside the current window are skipped
and are not replayed. The next occurrence can start a new instance.

Execution also requires a server connection, a matching active map, a valid map
link, localization, a clear safety state, and no manual driving. Those facts
arrive in `ExecutionContext`. This component does not open a cloud session or
evaluate the safety gate itself. Manual driving or a disconnect pauses a
running instance. Releasing manual control or reconnecting does not resume it.
An invalidated `map_task_link` blocks that task on the old map and leaves the
task definition in place. The link does not store a copy of the schedule.

The claim is one SQLite transaction. The same task and occurrence cannot be
claimed twice; the second claim conflicts. If the process dies after the claim
commits and before navigation is sent, the next process start marks that
instance `needs_review` and does not send it. Cancelling while navigation is in
progress also ends in `needs_review` with rollback unconfirmed. A missing
navigation port does not connect to Nav2 and does not report success. A full
disk does not confirm the write.
