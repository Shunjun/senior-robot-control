import type { ReactElement } from 'react';
import { DAY_NAMES, UNITS, WEEKDAYS, type ScheduleDraft } from './schedule';

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  fullWidth: [
    "tasks-full-width [grid-column:1/-1]"
  ].join(" "),
  note: [
    "tasks-note text-muted text-[10px] leading-[1.8] mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  weekdays: [
    "tasks-weekdays flex flex-wrap gap-1 border border-solid border-line mt-0",
    "mr-0 mb-0 ml-0 pt-1 pr-1.5 pb-1 pl-1.5 [&&_label]:flex",
    "[&&_label]:items-center [&&_label]:gap-1 [&&_label]:text-[10px] [&_input]:w-[16px] [&_input]:h-[16px] [&_input]:accent-amber"
  ].join(" "),
} as const;

interface ScheduleFieldsProps {
  value: ScheduleDraft;
  onChange: (patch: Partial<ScheduleDraft>) => void;
}
export function ScheduleFields({ value, onChange }: ScheduleFieldsProps): ReactElement {
  if (value.mode === 'advanced') return <label className={ui.fullWidth}>
    Cron 表达式
    <input aria-label="Cron 表达式" value={value.cron} onChange={event => onChange({ cron: event.target.value })} spellCheck={false} />
    <span className={ui.note}>高级五字段：分 时 日 月 星期；由车端校验。</span>
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
    {value.mode === 'weekly' && <fieldset className={ui.weekdays}>
      <legend>执行星期（可多选）</legend>
      {WEEKDAYS.map(day => <label key={day}>
        <input type="checkbox" checked={value.weekdays.includes(day)} onChange={event => onChange({
          weekdays: event.target.checked ? [...value.weekdays, day] : value.weekdays.filter(item => item !== day),
        })} />{DAY_NAMES[day]}
      </label>)}
    </fieldset>}
  </>;
}
