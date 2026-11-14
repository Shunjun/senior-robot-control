import { describe, expect, it } from 'vitest';
import { draftSchedule, NEW_TASK, scheduleDraft, scheduleSummary } from './schedule';

describe('visual schedule conversion', () => {
  it('keeps daily wall time distinct from a fixed 24-hour interval', () => {
    expect(draftSchedule({ ...NEW_TASK, mode: 'daily', time: '09:15', scheduleEdited: true }))
      .toEqual({ kind: 'cron', expression: '15 9 * * *' });
    expect(draftSchedule({ ...NEW_TASK, mode: 'interval', interval: '1', intervalUnit: 'days', scheduleEdited: true }))
      .toEqual({ kind: 'interval', every_seconds: 86400 });
  });
  it('maps common daily and weekly cron losslessly until explicitly edited', () => {
    const original = { kind: 'cron', expression: '05 09 * * 1-5' } as const;
    const draft = scheduleDraft(original);
    expect(draft).toMatchObject({ mode: 'weekly', time: '09:05', weekdays: [1, 2, 3, 4, 5] });
    const task = { ...NEW_TASK, ...draft, timezone: 'Europe/Berlin' };
    expect(draftSchedule(task)).toEqual(original);
    expect(draftSchedule({ ...NEW_TASK, ...draft, time: '10:30', scheduleEdited: true }))
      .toEqual({ kind: 'cron', expression: '30 10 * * 1,2,3,4,5' });
    expect(scheduleDraft({ kind: 'cron', expression: '0 0 * * *' })).toMatchObject({ mode: 'daily', time: '00:00' });
    expect(scheduleDraft({ kind: 'cron', expression: '0 8 * * 7' })).toMatchObject({ weekdays: [0] });
  });
  it('preserves complex cron verbatim on metadata-only edits', () => {
    const original = { kind: 'cron', expression: '  */15 9-17 * * 1-5  ' } as const;
    const draft = { ...NEW_TASK, ...scheduleDraft(original), task_id: 'renamed' };
    expect(draft.mode).toBe('advanced');
    expect(draft.cron).toBe(original.expression);
    expect(draftSchedule(draft)).toEqual(original);
    expect(scheduleSummary(draft)).toContain(original.expression);
  });
  it('roundtrips interval units and rejects incomplete visual values', () => {
    const original = { kind: 'interval', every_seconds: 7200 } as const;
    const draft = { ...NEW_TASK, ...scheduleDraft(original) };
    expect(draft).toMatchObject({ interval: '2', intervalUnit: 'hours' });
    expect(draftSchedule(draft)).toEqual(original);
    expect(draftSchedule({ ...draft, interval: '3', scheduleEdited: true })).toEqual({ kind: 'interval', every_seconds: 10800 });
    expect(draftSchedule({ ...NEW_TASK, mode: 'weekly', weekdays: [], scheduleEdited: true })).toBeNull();
    expect(draftSchedule({ ...NEW_TASK, time: '', scheduleEdited: true })).toBeNull();
    expect(draftSchedule({ ...NEW_TASK, mode: 'interval', interval: '-1', scheduleEdited: true })).toBeNull();
    expect(scheduleSummary(draft)).toContain('Asia/Shanghai');
  });
});
