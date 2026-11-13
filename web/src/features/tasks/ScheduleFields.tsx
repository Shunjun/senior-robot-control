import type { ReactElement } from 'react';
import { DAY_NAMES, UNITS, WEEKDAYS, type ScheduleDraft } from './schedule';
import styles from './Tasks.module.css';

interface ScheduleFieldsProps {
  value: ScheduleDraft;
  onChange: (patch: Partial<ScheduleDraft>) => void;
}
export function ScheduleFields({ value, onChange }: ScheduleFieldsProps): ReactElement {
  if (value.mode === 'advanced') return <label className={styles.fullWidth}>
    Cron 表达式
    <input aria-label="Cron 表达式" value={value.cron} onChange={event => onChange({ cron: event.target.value })} spellCheck={false} />
    <span className={styles.note}>高级五字段：分 时 日 月 星期；由车端校验。</span>
  </label>;
  if (value.mode === 'interval') return <>
    <label>间隔数值
      <input type="number" min="0" step="any" value={value.interval}
        onChange={event => onChange({ interval: event.target.value })} />
    </label>
    <label>间隔单位
      <select value={value.intervalUnit} onChange={event => onChange({ intervalUnit: event.target.value as ScheduleDraft['intervalUnit'] })}>
        {Object.entries(UNITS).map(([unit, item]) => <option key={unit} value={unit}>{item.label}</option>)}
      </select>
    </label>
  </>;
  return <>
    <label>执行时间
      <input type="time" value={value.time} onChange={event => onChange({ time: event.target.value })} />
    </label>
    {value.mode === 'weekly' && <fieldset className={styles.weekdays}>
      <legend>执行星期（可多选）</legend>
      {WEEKDAYS.map(day => <label key={day}>
        <input type="checkbox" checked={value.weekdays.includes(day)} onChange={event => onChange({
          weekdays: event.target.checked ? [...value.weekdays, day] : value.weekdays.filter(item => item !== day),
        })} />{DAY_NAMES[day]}
      </label>)}
    </fieldset>}
  </>;
}
