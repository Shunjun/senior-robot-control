import { useEffect, useMemo, useRef, useState, type ReactElement } from "react";
import type { RobotLink } from "../../api/client";
import type { RobotSnapshot } from "../../api/protocol";
import { useMeasuredSpeed, useVehicleView } from "../devices/useVehicleView";
import type { ReleaseReason } from "./Joystick";
import { DriveJoystick } from "./DriveJoystick";
import { ManualController, type ManualPhase } from "./ManualController";
import { displaySpeed, useSpeedUnit } from "../../display/speedUnits";
import { Button } from "../../components/ui/Button";
import { SegmentedControl } from "../../components/ui/SegmentedControl";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  drive: [
    "drive-drive grid gap-0 max-[460.001px]:grid-cols-[minmax(0,_1fr)_146px] max-[460.001px]:gap-[5px_12px] max-[460.001px]:[&>.drive-quiet]:[grid-column:1/-1] max-[460.001px]:[&>.drive-quiet]:text-xs max-[460.001px]:[&>.drive-quiet]:leading-[1.5]",
    "max-[460.001px]:[&>.drive-quiet]:mt-1"
  ].join(" "),
  driveHead: [
    "drive-drive-head flex items-center justify-between gap-2.5 [&_h3]:text-base [&_h3]:font-medium [&_h3]:mt-0",
    "[&_h3]:mr-0 [&_h3]:mb-[5px] [&_h3]:ml-0 max-[460.001px]:[grid-column:1] max-[460.001px]:[&_h3]:text-sm"
  ].join(" "),
  english: [
    "drive-english font-normal text-xs leading-[normal] font-mono tracking-[1px] text-english"
  ].join(" "),
  subtitle: [
    "drive-subtitle text-xs leading-[1.8] text-muted mt-0 mr-0 mb-0 ml-0",
    "max-[460.001px]:[grid-column:1] max-[460.001px]:text-xs"
  ].join(" "),
  measurement: [
    "drive-measurement font-light text-4xl leading-[normal] font-number tracking-[-1.4px] mt-[15px] mr-0",
    "mb-[3px] ml-0 [&_small]:font-normal [&_small]:text-xs [&_small]:leading-[normal] [&_small]:font-ui [&_small]:tracking-[0] [&_small]:text-muted",
    "max-[760.001px]:text-3xl max-[760.001px]:mt-2.5 max-[460.001px]:[grid-column:1] max-[460.001px]:text-3xl max-[460.001px]:mt-0 max-[460.001px]:mr-0 max-[460.001px]:mb-0 max-[460.001px]:ml-0"
  ].join(" "),
  gears: [
    "drive-gears w-full max-[460.001px]:[grid-column:1]"
  ].join(" "),
  readings: [
    "drive-readings flex flex-wrap justify-center gap-5.5 text-xs text-drive-readings-color whitespace-nowrap max-[460.001px]:[grid-column:2]",
    "max-[460.001px]:text-xs max-[460.001px]:gap-[5px]"
  ].join(" "),
  leaseTag: [
    "drive-lease-tag text-drive-lease-tag-color text-xs mt-[15px] mr-0 mb-2.5 ml-0 max-[760.001px]:mt-2.5",
    "max-[760.001px]:mr-0 max-[760.001px]:mb-2.5 max-[760.001px]:ml-0 max-[460.001px]:[grid-column:1] max-[460.001px]:[grid-row:5] max-[460.001px]:mt-1 max-[460.001px]:mr-0 max-[460.001px]:mb-1",
    "max-[460.001px]:ml-0 max-[460.001px]:text-xs"
  ].join(" "),
  authority: [
    "drive-authority [&_button]:w-full max-[460.001px]:[grid-column:1] max-[460.001px]:[grid-row:6] max-[460.001px]:[&_button]:text-xs max-[460.001px]:[&_button]:pt-2 max-[460.001px]:[&_button]:pr-2 max-[460.001px]:[&_button]:pb-2",
    "max-[460.001px]:[&_button]:pl-2"
  ].join(" "),
  quiet: [
    "drive-quiet text-muted mt-2.5 mr-0 mb-0 ml-0 leading-[1.8] text-xs"
  ].join(" "),
  failure: [
    "drive-failure text-warning-ink bg-none bg-red-wash pt-2 pr-2 pb-2 pl-2",
    "rounded-md text-xs max-[460.001px]:[grid-column:1/-1] max-[460.001px]:text-xs max-[460.001px]:leading-[1.5] max-[460.001px]:mt-1"
  ].join(" "),
} as const;

export interface DriveViewProps {
  robotId: string;
  link: RobotLink;
  snapshot: RobotSnapshot | null;
  active?: boolean;
  compact?: boolean;
}
export function DriveView({
  robotId,
  link,
  active = true,
  compact = false,
}: DriveViewProps): ReactElement {
  const unit = useSpeedUnit();
  const view = useVehicleView(link.vehicle);
  const [phase, setPhase] = useState<ManualPhase>("none");
  const [error, setError] = useState<string | null>(null);
  const [resetToken, reset] = useState(0);
  const [gear, setGear] = useState<"low" | "high">("low");
  const values = useRef({ linear: 0, turn: 0 });
  const controller = useMemo(
    () =>
      link.vehicle
        ? new ManualController(link.vehicle, (next, issue) => {
            setPhase(next);
            if (issue) setError(issue);
          })
        : null,
    [link, robotId],
  );
  const measured = useMeasuredSpeed(view);
  const turn =
    measured !== null &&
    view.frame?.body_reference &&
    !view.frame.body_reference.stale
      ? view.frame.body_reference.average.wz
      : null;
  const enabled = active && view.manual_connected;
  useEffect(() => {
    if (!enabled) {
      controller?.release();
      values.current = { linear: 0, turn: 0 };
      reset((n) => n + 1);
    }
  }, [controller, enabled]);
  useEffect(() => {
    if (view.frame?.manual_authority?.state === "closed") {
      controller?.observeClosed(view.frame.manual_authority.authority_id);
    }
  }, [controller, view.frame?.manual_authority]);
  useEffect(() => {
    controller?.attach();
    const stop = () => {
      controller?.release();
      values.current = { linear: 0, turn: 0 };
      reset((n) => n + 1);
    };
    const hidden = () => {
      if (document.visibilityState === "hidden") stop();
    };
    window.addEventListener("blur", stop);
    window.addEventListener("pagehide", stop);
    document.addEventListener("visibilitychange", hidden);
    return () => {
      window.removeEventListener("blur", stop);
      window.removeEventListener("pagehide", stop);
      document.removeEventListener("visibilitychange", hidden);
      controller?.dispose();
    };
  }, [controller]);
  useEffect(() => {
    if (phase === "none") {
      values.current = { linear: 0, turn: 0 };
      reset((n) => n + 1);
    }
  }, [phase]);
  function change(linear: number, turn: number): void {
    values.current = { linear, turn };
    controller?.update(linear, turn, gear);
  }
  function press(linear: number, turn: number): boolean {
    if (controller?.phase === "prepared") {
      if (!controller.begin(linear, turn, gear)) return false;
    } else if (controller?.phase === "active")
      controller.update(linear, turn, gear);
    else return false;
    values.current = { linear, turn };
    return true;
  }
  function stopGesture(reason: ReleaseReason): void {
    if (reason === "blur" || reason === "hidden") {
      release();
      return;
    }
    controller?.stopGesture();
    values.current = { linear: 0, turn: 0 };
    reset((n) => n + 1);
  }
  function release(): void {
    controller?.release();
    values.current = { linear: 0, turn: 0 };
    reset((n) => n + 1);
  }
  return (
    <section
      className={ui.drive}
      data-compact={compact}
      aria-label="手动驾驶"
    >
      <header className={ui.driveHead}>
        <h3>手动驾驶</h3>
        <span className={ui.english}>DRIVE</span>
      </header>
      <p className={ui.subtitle}>一个摇杆，控制速度与转向。</p>
      <p data-testid="measured-speed" className={ui.measurement}>
        {measured === null ? "不可用" : displaySpeed(measured, unit).toFixed(2)}{" "}
        <small>{unit}</small>
      </p>
      <div className={ui.gears}>
        <SegmentedControl
          label="档位"
          value={gear}
          onChange={(value) => {
            setGear(value);
            controller?.update(
              values.current.linear,
              values.current.turn,
              value,
            );
          }}
          options={[
            { value: "low", label: "低速" },
            { value: "high", label: "高速" },
          ]}
        />
      </div>
      <DriveJoystick
        disabled={!enabled || !["prepared", "active"].includes(phase)}
        resetToken={resetToken}
        onPress={press}
        onChange={change}
        onRelease={stopGesture}
      />
      <div className={ui.readings} aria-label="车辆测量读数">
        <span>
          速度{" "}
          <b>
            {measured === null
              ? "不可用"
              : displaySpeed(measured, unit).toFixed(2)}
          </b>{" "}
          {unit}
        </span>
        <span>
          转向 <b>{turn === null ? "不可用" : turn.toFixed(2)}</b> rad/s
        </span>
      </div>
      <p className={ui.leaseTag}>
        <span aria-hidden="true">{phase === "none" ? "○ " : "● "}</span>
        {
          {
            none: "未获取控制权",
            preparing: "等待车端准备",
            prepared: "已准备 · 等待新按压",
            active: "手动会话进行中",
            stopping: "正在停车 · 保留控制权",
          }[phase]
        }
      </p>
      <div className={ui.authority}>
        {phase === "none" ? (
          <Button
            variant="primary"
            disabled={!enabled}
            onClick={() => {
              setError(null);
              void controller?.prepare();
            }}
          >
            获取控制权
          </Button>
        ) : (
          <Button variant="outline" onClick={release}>
            释放控制权
          </Button>
        )}
      </div>
      <p className={ui.quiet}>
        靠近中轴自动吸附 · 松手归零
        <br />
        切换页面或操作卡片时释放控制权
      </p>
      {error && (
        <p role="alert" className={ui.failure}>
          {error}
        </p>
      )}
    </section>
  );
}
