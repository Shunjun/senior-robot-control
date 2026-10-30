import { useEffect, useState, type ReactElement } from "react";

import styles from "./Tasks.module.css";

export type TaskSchedule =
  | { kind: "interval"; every_seconds: number }
  | { kind: "cron"; expression: string };

export interface TaskDraft {
  task_id: string;
  timezone: string;
  schedule: TaskSchedule;
}

export function ScheduleEditor({ value, onSave }: {
  value: TaskDraft | null;
  onSave: (draft: TaskDraft) => void;
}): ReactElement {
  const [taskId, setTaskId] = useState(value?.task_id ?? "");
  const [timezone, setTimezone] = useState(value?.timezone ?? "Asia/Shanghai");
  const [kind, setKind] = useState<TaskSchedule["kind"]>(value?.schedule.kind ?? "interval");
  const [interval, setIntervalValue] = useState(
    value?.schedule.kind === "interval" ? String(value.schedule.every_seconds) : "86400",
  );
  const [cron, setCron] = useState(
    value?.schedule.kind === "cron" ? value.schedule.expression : "0 9 * * 1",
  );

  useEffect(() => {
    if (value === null) return;
    setTaskId(value.task_id);
    setTimezone(value.timezone);
    setKind(value.schedule.kind);
    if (value.schedule.kind === "interval") setIntervalValue(String(value.schedule.every_seconds));
    else setCron(value.schedule.expression);
  }, [value]);

  function save(): void {
    const schedule: TaskSchedule =
      kind === "interval"
        ? { kind, every_seconds: Math.max(1, Number.parseInt(interval, 10) || 1) }
        : { kind, expression: cron.trim() };
    onSave({ task_id: taskId.trim(), timezone: timezone.trim() || "Asia/Shanghai", schedule });
  }

  return (
    <div className={styles.editor}>
      <label>
        任务编号
        <input aria-label="任务编号" value={taskId} onChange={(event) => setTaskId(event.target.value)} />
      </label>
      <label>
        时区
        <input aria-label="时区" value={timezone} onChange={(event) => setTimezone(event.target.value)} />
      </label>
      <label>
        计划类型
        <select
          aria-label="计划类型"
          value={kind}
          onChange={(event) => setKind(event.target.value as TaskSchedule["kind"])}
        >
          <option value="interval">固定间隔</option>
          <option value="cron">Cron（五字段）</option>
        </select>
      </label>
      {kind === "interval" ? (
        <label>
          间隔秒数
          <input
            aria-label="间隔秒数"
            type="number"
            min="1"
            value={interval}
            onChange={(event) => setIntervalValue(event.target.value)}
          />
        </label>
      ) : (
        <label>
          Cron 表达式
          <input aria-label="Cron 表达式" value={cron} onChange={(event) => setCron(event.target.value)} />
        </label>
      )}
      <button type="button" onClick={save} disabled={taskId.trim() === "" || (kind === "cron" && cron.trim() === "")}>
        保存草稿
      </button>
    </div>
  );
}
