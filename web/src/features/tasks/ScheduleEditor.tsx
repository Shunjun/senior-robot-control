import type { ReactElement } from "react";
import {
  draftSchedule,
  scheduleSummary,
  type TaskDraft,
  type ScheduleDraft,
} from "./schedule";
import { ScheduleFields } from "./ScheduleFields";
import styles from "./Tasks.module.css";

export {
  draftSchedule,
  NEW_TASK,
  type TaskDraft,
  type TaskSchedule,
} from "./schedule";
interface ScheduleEditorProps {
  value: TaskDraft;
  onChange: (draft: TaskDraft) => void;
  editing: boolean;
  namesSupported?: boolean;
  compact?: boolean;
}
export function ScheduleEditor({
  value,
  onChange,
  editing,
  namesSupported = true,
  compact = false,
}: ScheduleEditorProps): ReactElement {
  function changeSchedule(patch: Partial<ScheduleDraft>): void {
    onChange({ ...value, ...patch, scheduleEdited: true });
  }
  return (
    <div className={styles.editor}>
      <label className={styles.fullWidth}>
        任务名称
        <input
          value={namesSupported ? value.name : value.task_id}
          readOnly={!namesSupported}
          onChange={(event) => onChange({ ...value, name: event.target.value })}
        />
      </label>
      <details className={styles.fullWidth} open={!compact}>
        <summary>任务编号与时区</summary>
        <label className={`${styles.note} ${styles.fullWidth}`}>
          内部编号
          <input value={value.task_id} readOnly />
        </label>
        <p className={styles.note}>
          名称可修改、可重复；
          {editing ? "编辑时编号保持不变。" : "编号用于区分同名任务。"}
        </p>
        <label>
          时区
          <input
            value={value.timezone}
            onChange={(event) =>
              onChange({ ...value, timezone: event.target.value })
            }
          />
        </label>
      </details>
      <label>
        计划类型
        <select
          value={value.mode}
          onChange={(event) => {
            const current = draftSchedule(value);
            changeSchedule({
              mode: event.target.value as TaskDraft["mode"],
              ...(event.target.value === "advanced" && current?.kind === "cron"
                ? { cron: current.expression }
                : {}),
            });
          }}
        >
          <option value="daily">每天</option>
          <option value="weekly">每周</option>
          <option value="interval">固定间隔</option>
          <option value="advanced">高级 Cron</option>
        </select>
      </label>
      <ScheduleFields value={value} onChange={changeSchedule} />
      <p className={styles.scheduleSummary} aria-label="计划摘要">
        {scheduleSummary(value)}
      </p>
      {!compact && (value.mode === "daily" || value.mode === "weekly") && (
        <p className={`${styles.note} ${styles.fullWidth}`}>
          按所选时区的当地时间执行，不是从现在起每隔 24 小时。
        </p>
      )}
    </div>
  );
}
