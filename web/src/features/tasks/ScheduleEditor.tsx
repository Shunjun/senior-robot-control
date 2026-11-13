import type { ReactElement } from 'react';
import { draftSchedule, scheduleSummary, type TaskDraft, type ScheduleDraft } from './schedule';
import { ScheduleFields } from './ScheduleFields';
import styles from './Tasks.module.css';

export { draftSchedule, NEW_TASK, type TaskDraft, type TaskSchedule } from './schedule';
interface ScheduleEditorProps {
  value: TaskDraft;
  onChange: (draft: TaskDraft) => void;
  editing: boolean;
}
export function ScheduleEditor({ value, onChange, editing }: ScheduleEditorProps): ReactElement {
  function changeSchedule(patch: Partial<ScheduleDraft>): void {
    onChange({ ...value, ...patch, scheduleEdited: true });
  }
  return <div className={styles.editor}>
    <label>任务编号
      <input value={value.task_id} readOnly={editing} onChange={event => onChange({ ...value, task_id: event.target.value })} />
    </label>
    <label>时区
      <input value={value.timezone} onChange={event => onChange({ ...value, timezone: event.target.value })} />
    </label>
    <label>计划类型
      <select value={value.mode} onChange={event => {
        const current = draftSchedule(value);
        changeSchedule({ mode: event.target.value as TaskDraft['mode'],
          ...(event.target.value === 'advanced' && current?.kind === 'cron' ? { cron: current.expression } : {}) });
      }}>
        <option value="daily">每天</option><option value="weekly">每周</option>
        <option value="interval">固定间隔</option><option value="advanced">高级 Cron</option>
      </select>
    </label>
    <ScheduleFields value={value} onChange={changeSchedule} />
    <p className={styles.scheduleSummary} aria-label="计划摘要">{scheduleSummary(value)}</p>
    {(value.mode === 'daily' || value.mode === 'weekly') && <p className={`${styles.note} ${styles.fullWidth}`}>按所选时区的当地时间执行，不是从现在起每隔 24 小时。</p>}
  </div>;
}
