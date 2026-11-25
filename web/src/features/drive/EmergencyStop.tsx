import { useEffect, useRef, useState, type ReactElement } from 'react';
import { CommandError, describeCommandError, type RobotLink } from '../../api/client';
import { Square } from "lucide-react";
import { useVehicleView } from '../devices/useVehicleView';

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  estopDock: [
    "drive-estop-dock flex flex-wrap items-center gap-1 relative ml-auto max-w-[min(340px,_calc(100vw_-_24px))]",
    "[&[data-compact=true]_.drive-estop-tag]:hidden max-[460.001px]:ml-0"
  ].join(" "),
  estop: [
    "drive-estop inline-flex items-center justify-center gap-1 pt-[9px] pr-[13px] pb-[9px] pl-[13px] min-w-0 max-[460.001px]:pr-2 max-[460.001px]:pl-2",
    "border border-solid border-drive-estop-border-color rounded-control text-drive-estop-color bg-none bg-drive-estop-background-color cursor-pointer",
    "relative before:content-[''] [&::before]:absolute [&::before]:inset-[-4.5px]"
  ].join(" "),
  estopCap: [
    "drive-estop-cap text-xs max-[460.001px]:text-xs"
  ].join(" "),
  estopTag: [
    "drive-estop-tag text-xs"
  ].join(" "),
  recoveryActions: [
    "drive-recovery-actions absolute top-[calc(100%_+_10px)] right-0 w-[260px] max-w-[calc(100vw_-_24px)] flex flex-wrap",
    "gap-1.5 pt-3 pr-3 pb-3 pl-3 bg-none bg-drive-recovery-actions-background-color border",
    "border-solid border-line rounded-panel shadow-ui-card z-[35] [&>p]:[flex-basis:100%] [&>p]:mt-0 [&>p]:mr-0",
    "[&>p]:mb-0 [&>p]:ml-0 [&>p]:text-xs [&>p]:leading-[1.8] [&>p]:text-warning-ink [&_.drive-ghost]:text-xs"
  ].join(" "),
  ghost: [
    "drive-ghost min-h-[44px] min-w-[44px] pt-2 pr-3 pb-2 pl-3 border",
    "border-solid border-amber rounded-control text-amber bg-none bg-transparent text-xs",
    "cursor-pointer [&:disabled]:opacity-[0.45] [&:disabled]:cursor-not-allowed [&:focus-visible]:[outline:2px_solid_var(--color-amber)] [&:focus-visible]:[outline-offset:2px]"
  ].join(" "),
  quiet: [
    "drive-quiet text-muted mt-2.5 mr-0 mb-0 ml-0 leading-[1.8] text-xs"
  ].join(" "),
} as const;

export interface EmergencyStopProps {
  link: RobotLink;
  compact?: boolean;
}
export function EmergencyStop({ link, compact = false }: EmergencyStopProps): ReactElement {
  const { frame, connected, available } = useVehicleView(link.vehicle);
  const [pending, setPending] = useState(false), [note, setNote] = useState<string | null>(null);
  const generation = useRef(0);
  useEffect(() => () => {
    generation.current++;
  }, [link]);
  async function press(): Promise<void> {
    const mine = ++generation.current;
    setPending(true);
    setNote(null);
    try {
      const request = link.request('safety.estop', {});
      link.vehicle?.fenceManual();
      await request;
      if (mine === generation.current) setNote('车端已应答，等待车端状态确认');
    }
    catch (error) {
      if (mine === generation.current) setNote(`急停未确认：${error instanceof CommandError ? describeCommandError(error.code) : '无法发送'}。请立即使用车上实体急停`);
    }
    finally {
      if (mine === generation.current) setPending(false);
    }
  }
  const confirmed = frame?.confirmed_estop_status;
  const resetAllowed = frame?.allowed_actions.includes('estop_reset') === true;
  async function recover(kind: 'estop_reset' | 'fault_recover'): Promise<void> {
    try {
      await link.vehicle?.recover(kind);
      setNote('恢复请求已应答，等待车端状态确认');
    } catch (error) {
      setNote(String(error));
    }
  }
  return (
    <div className={ui.estopDock} data-compact={compact}>
      <button
        className={ui.estop}
        aria-label="急停"
        onClick={() => void press()}
      >
        <Square size={6} fill="currentColor" aria-hidden="true" /><span className={ui.estopCap}>急停</span>
        <span className={ui.estopTag}>E-STOP</span>
      </button>
      {(confirmed === "software_latched" ||
        resetAllowed ||
        frame?.allowed_actions.includes("fault_recover") ||
        pending ||
        note) && (
        <div className={ui.recoveryActions}>
          {confirmed === "software_latched" && (
            <p>软件急停已锁存</p>
          )}
          {pending && <p role="status">急停等待确认</p>}
          {note && (
            <p role={note.startsWith("急停未确认") ? "alert" : "status"}>
              {note}
            </p>
          )}
          {(confirmed === "software_latched" || resetAllowed) && (
            <>
              <button
                className={ui.ghost}
                disabled={!connected || !available || !resetAllowed}
                onClick={() => void recover("estop_reset")}
              >
                复位软件急停
              </button>
              {confirmed === "software_latched" && !resetAllowed && (
                <p className={ui.quiet}>
                  车端尚未许可复位，等待运动证据有效并确认停稳。
                </p>
              )}
            </>
          )}
          {frame?.allowed_actions.includes("fault_recover") && (
            <button
              className={ui.ghost}
              disabled={!connected || !available}
              onClick={() => void recover("fault_recover")}
            >
              恢复执行故障
            </button>
          )}
        </div>
      )}
    </div>
  );
}
