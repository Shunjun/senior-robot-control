import type { TaskSchedule } from '../../api/workspace';

export type { TaskSchedule };
export type ScheduleMode = 'daily' | 'weekly' | 'interval' | 'advanced';
export type IntervalUnit = 'seconds' | 'minutes' | 'hours' | 'days';
export const UNITS: Record<IntervalUnit, { seconds: number; label: string }> = {
  seconds: { seconds: 1, label: '秒' }, minutes: { seconds: 60, label: '分钟' },
  hours: { seconds: 3600, label: '小时' }, days: { seconds: 86400, label: '天' },
};
export const WEEKDAYS = [1, 2, 3, 4, 5, 6, 0] as const;
export const DAY_NAMES: Record<number, string> = {
  1: '周一', 2: '周二', 3: '周三', 4: '周四', 5: '周五', 6: '周六', 0: '周日',
};
export interface ScheduleDraft {
  mode: ScheduleMode;
  time: string;
  weekdays: readonly number[];
  interval: string;
  intervalUnit: IntervalUnit;
  cron: string;
  originalSchedule: TaskSchedule | null;
  scheduleEdited: boolean;
}
export interface TaskDraft extends ScheduleDraft {
  task_id: string;
  timezone: string;
}
const INITIAL_SCHEDULE: ScheduleDraft = {
  mode: 'daily', time: '09:00', weekdays: [1],
  interval: '1', intervalUnit: 'days', cron: '0 9 * * *',
  originalSchedule: null, scheduleEdited: false,
};
export const NEW_TASK: TaskDraft = { task_id: '', timezone: 'Asia/Shanghai', ...INITIAL_SCHEDULE };

function clock(time: string): [number, number] | null {
  if (!/^\d{2}:\d{2}$/.test(time)) return null;
  const [hour, minute] = time.split(':').map(Number) as [number, number];
  return hour <= 23 && minute <= 59 ? [hour, minute] : null;
}

/** Preserve the original wire representation until a schedule control is edited. */
export function draftSchedule(value: ScheduleDraft): TaskSchedule | null {
  //Compatible: retain existing wire text/values until a schedule control changes.
  if (value.originalSchedule && !value.scheduleEdited) return value.originalSchedule;
  if (value.mode === 'advanced') {
    const expression = value.cron.trim();
    return expression.split(/\s+/).length === 5 ? { kind: 'cron', expression } : null;
  }
  if (value.mode === 'interval') {
    const every_seconds = Number(value.interval) * UNITS[value.intervalUnit].seconds;
    return value.interval.trim() && Number.isSafeInteger(every_seconds) && every_seconds > 0
      ? { kind: 'interval', every_seconds } : null;
  }
  const at = clock(value.time);
  if (!at) return null;
  const days = WEEKDAYS.filter(day => value.weekdays.includes(day));
  if (value.mode === 'weekly' && days.length === 0) return null;
  return { kind: 'cron', expression: `${at[1]} ${at[0]} * * ${value.mode === 'daily' ? '*' : days.join(',')}` };
}

function cronDays(field: string): number[] | null {
  const result = new Set<number>();
  for (const item of field.split(',')) {
    if (!/^\d(?:-\d)?$/.test(item)) return null;
    const [start, last] = item.split('-').map(Number);
    const end = last ?? start!;
    if (start === undefined || start > end || end > 7) return null;
    for (let day = start; day <= end; day++) result.add(day % 7);
  }
  return WEEKDAYS.filter(day => result.has(day));
}

/** Common numeric cron forms are editable; all other forms retain the advanced text. */
export function scheduleDraft(schedule: TaskSchedule): ScheduleDraft {
  const draft: ScheduleDraft = { ...INITIAL_SCHEDULE, originalSchedule: { ...schedule } };
  if (schedule.kind === 'interval') {
    const unit = (['days', 'hours', 'minutes', 'seconds'] as const)
      .find(key => schedule.every_seconds % UNITS[key].seconds === 0) ?? 'seconds';
    return { ...draft, mode: 'interval', interval: String(schedule.every_seconds / UNITS[unit].seconds), intervalUnit: unit };
  }
  draft.cron = schedule.expression;
  draft.mode = 'advanced';
  const fields = schedule.expression.trim().split(/\s+/);
  if (fields.length !== 5 || fields[2] !== '*' || fields[3] !== '*') return draft;
  const [minute, hour, , , days] = fields;
  if (!/^\d{1,2}$/.test(minute!) || !/^\d{1,2}$/.test(hour!) || Number(minute) > 59 || Number(hour) > 23) return draft;
  const time = `${hour!.padStart(2, '0')}:${minute!.padStart(2, '0')}`;
  if (days === '*') return { ...draft, mode: 'daily', time };
  const weekdays = cronDays(days!);
  return weekdays?.length ? { ...draft, mode: 'weekly', time, weekdays } : draft;
}

export function scheduleSummary(value: TaskDraft): string {
  let plan: string;
  switch (value.mode) {
    case 'daily': plan = `每天 ${value.time || '请选择时间'}`; break;
    case 'weekly': plan = `每周 ${WEEKDAYS.filter(day => value.weekdays.includes(day)).map(day => DAY_NAMES[day]).join('、') || '请选择星期'} ${value.time || '请选择时间'}`; break;
    case 'interval': plan = `每 ${value.interval || '—'} ${UNITS[value.intervalUnit].label}（固定间隔，按任务创建时间起算）`; break;
    case 'advanced': plan = `高级计划：${value.cron}`; break;
  }
  return `${plan} · 时区 ${value.timezone || '未填写'}`;
}
