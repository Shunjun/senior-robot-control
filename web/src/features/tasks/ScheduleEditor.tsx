import type { ReactElement, ReactNode } from "react";
import {
  draftSchedule,
  scheduleSummary,
  type TaskDraft,
  type ScheduleDraft,
} from "./schedule";
import { ScheduleFields } from "./ScheduleFields";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  note: [
    "tasks-note text-muted text-[10px] leading-[1.8] mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  editor: [
    "tasks-editor block mt-3 [&>label+label]:mt-3 [&_label]:grid [&_label]:gap-[7px] [&_label]:text-field [&_label]:text-[11px]",
    "[&_input]:min-w-0 [&_input]:w-full [&_input]:pt-2.5 [&_input]:pr-2.5 [&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:text-input-ink",
    "[&_input]:bg-none [&_input]:bg-input [&_input]:border [&_input]:border-solid [&_input]:border-login-input-border-color [&_input]:rounded-input [&_select]:min-w-0 [&_select]:w-full",
    "[&_select]:pt-2.5 [&_select]:pr-2.5 [&_select]:pb-2.5 [&_select]:pl-2.5 [&_select]:text-input-ink [&_select]:bg-none [&_select]:bg-input",
    "[&_select]:border [&_select]:border-solid [&_select]:border-login-input-border-color [&_select]:rounded-input [&_input[readonly]]:text-muted [&_input[readonly]]:text-[10px]"
  ].join(" "),
  planSummary: [
    "tasks-plan-summary text-field text-[11px] [&_p]:mt-[7px] [&_p]:mr-0 [&_p]:mb-0 [&_p]:ml-0 [&_p]:text-ink"
  ].join(" "),
  scheduleDetails: [
    "tasks-schedule-details text-[10px] text-muted [&_summary]:cursor-pointer [&[open]]:grid [&[open]]:gap-3 [&_label]:mt-2"
  ].join(" "),
} as const;

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
  showAdvanced?: boolean;
  children?: ReactNode;
}
export function ScheduleEditor({
  value,
  onChange,
  editing,
  namesSupported = true,
  compact = false,
  showAdvanced = false,
  children,
}: ScheduleEditorProps): ReactElement {
  function changeSchedule(patch: Partial<ScheduleDraft>): void {
    onChange({ ...value, ...patch, scheduleEdited: true });
  }
  const scheduleControls = (
    <>
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
      {(!compact || value.mode !== "daily") && (
        <ScheduleFields value={value} onChange={changeSchedule} />
      )}
      <label>
        时区
        <input
          value={value.timezone}
          onChange={(event) =>
            onChange({ ...value, timezone: event.target.value })
          }
        />
      </label>
      <label>
        内部编号
        <input value={value.task_id} readOnly />
      </label>
      <p className={ui.note}>
        名称可修改、可重复；
        {editing ? "编辑时编号保持不变。" : "编号用于区分同名任务。"}
      </p>
      <p className={ui.note} aria-label="计划摘要">
        {scheduleSummary(value)}
      </p>
    </>
  );
  return (
    <div className={ui.editor}>
      <label>
        任务名称
        <input
          value={namesSupported ? value.name : value.task_id}
          readOnly={!namesSupported}
          maxLength={40}
          placeholder="例如：一层巡检"
          onChange={(event) => onChange({ ...value, name: event.target.value })}
        />
      </label>
      {compact &&
        (value.mode === "daily" ? (
          <label>
            每日执行时间
            <input
              aria-label="执行时间"
              type="time"
              value={value.time}
              onChange={(event) => changeSchedule({ time: event.target.value })}
            />
          </label>
        ) : (
          <div className={ui.planSummary}>
            <span>执行计划</span>
            <p>{scheduleSummary(value)}</p>
            <span className={ui.note}>
              在高级计划中编辑；原计划保持不变。
            </span>
          </div>
        ))}
      {children}
      {(!compact || showAdvanced) && <details open={!compact} className={ui.scheduleDetails}>
        <summary>高级计划与任务信息</summary>
        {scheduleControls}
      </details>}
    </div>
  );
}
