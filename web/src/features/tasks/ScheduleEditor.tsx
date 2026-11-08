import type { ReactElement } from "react";

import styles from "./Tasks.module.css";

export type TaskSchedule =
  | { kind: "interval"; every_seconds: number }
  | { kind: "cron"; expression: string };

export interface TaskDraft {
  task_id: string;
  timezone: string;
  kind: TaskSchedule["kind"];
  interval: string;
  cron: string;
}

export const NEW_TASK: TaskDraft = {
  task_id: "",
  timezone: "Asia/Shanghai",
  kind: "interval",
  interval: "86400",
  cron: "0 9 * * 1",
};

export function draftSchedule(value: TaskDraft): TaskSchedule | null {
  if (value.kind === "cron") {
    const expression = value.cron.trim();
    return expression.split(/\s+/).length === 5 ? { kind: "cron", expression } : null;
  }
  const every_seconds = Number(value.interval);
  return Number.isSafeInteger(every_seconds) && every_seconds > 0
    ? { kind: "interval", every_seconds }
    : null;
}

interface ScheduleEditorProps {
  value: TaskDraft;
  onChange: (draft: TaskDraft) => void;
  editing: boolean;
}

export function ScheduleEditor({ value, onChange, editing }: ScheduleEditorProps): ReactElement {
  return (
    <div className={styles.editor}>
      <label>
        任务编号
        <input value={value.task_id} readOnly={editing}
          onChange={(event) => onChange({ ...value, task_id: event.target.value })} />
      </label>
      <label>
        时区
        <input value={value.timezone} onChange={(event) => onChange({ ...value, timezone: event.target.value })} />
      </label>
      <label>
        计划类型
        <select value={value.kind}
          onChange={(event) => onChange({ ...value, kind: event.target.value as TaskDraft["kind"] })}>
          <option value="interval">固定间隔</option>
          <option value="cron">Cron（五字段）</option>
        </select>
      </label>
      {value.kind === "interval" ? (
        <label>
          间隔秒数
          <input type="number" min="1" value={value.interval}
            onChange={(event) => onChange({ ...value, interval: event.target.value })} />
        </label>
      ) : (
        <label>
          Cron 表达式
          <input value={value.cron} onChange={(event) => onChange({ ...value, cron: event.target.value })} />
        </label>
      )}
    </div>
  );
}
