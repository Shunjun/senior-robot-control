import { useLayoutEffect, useRef, useState, type ReactElement } from "react";

import type { RobotLink } from "../../api/client";
import type { RobotSnapshot, RobotSummary } from "../../api/protocol";
import type { WorkspacePage } from "./useWorkspaceView";
import { Button } from "../../components/ui/Button";
import { VehicleTools } from "./VehicleTools";
import { ControlSurface } from "./ControlSurface";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  robot: [
    "devices-robot relative h-[var(--viewport-height)] min-h-[var(--viewport-min)] border border-solid border-devices-pairing-command-border-color rounded-scene",
    "overflow-hidden bg-none bg-app-placeholder-background-color isolate shadow-devices-robot min-[1500px]:min-h-[730px]"
  ].join(" "),
  controlPage: [
    "devices-control-page absolute inset-0"
  ].join(" "),
  connectionEmpty: [
    "devices-connection-empty absolute inset-0 flex flex-col items-center justify-center text-center",
    "pt-6 pr-6 pb-6 pl-6 text-muted [&_h2]:text-ink [&_h2]:text-[23px] [&_h2]:font-[450]",
    "[&_p]:text-[12px] [&_p]:leading-[2] [&_p]:max-w-[360px]"
  ].join(" "),
  connectionNotice: [
    "devices-connection-notice absolute left-4 right-4 top-15.5 z-[28] flex items-center",
    "gap-3 pt-2.5 pr-3 pb-2.5 pl-3 border border-solid border-devices-connection-notice-border-color",
    "rounded-control bg-none bg-devices-connection-notice-background-color text-[11px] [&_p]:mt-0 [&_p]:mr-0 [&_p]:mb-0 [&_p]:ml-0",
    "[&_button]:flex-none max-[460.001px]:left-3 max-[460.001px]:right-3 max-[460.001px]:flex-wrap"
  ].join(" "),
} as const;

export interface RobotPanelProps {
  robot: RobotSummary;
  link: RobotLink;
  /** The control socket closed. The app re-checks the session, since a 401 ends it. */
  onLinkLost: () => void;
  page?: WorkspacePage;
  onPageChange?: (page: WorkspacePage) => void;
  statusOpen?: boolean;
  onStatusClose?: () => void;
}

type LinkState = "connecting" | "open" | "closed";

// ---------------------------------------------------------------- panel

export function RobotPanel({
  robot,
  link,
  onLinkLost,
  page = "control",
  statusOpen = false,
  onPageChange,
  onStatusClose,
}: RobotPanelProps): ReactElement {
  const [snapshot, setSnapshot] = useState<RobotSnapshot | null>(null);
  const [linkState, setLinkState] = useState<LinkState>("connecting");
  const [closeCode, setCloseCode] = useState<number | null>(null);
  const [attempt, setAttempt] = useState(0);
  const lost = useRef(onLinkLost);
  lost.current = onLinkLost;

  useLayoutEffect(() => {
    let cancelled = false;
    setSnapshot(null);
    setLinkState("connecting");
    setCloseCode(null);
    const offSnapshot = link.subscribe((next) => {
      // Frames still in flight for a previous robot must not repaint this one.
      if (!cancelled && next.robot_id === robot.id) setSnapshot(next);
    });
    const offStatus = link.subscribeStatus((status) => {
      if (cancelled || status.robotId !== robot.id || status.state !== "closed")
        return;
      setLinkState("closed");
      setSnapshot((previous) =>
        previous === null
          ? null
          : { ...previous, stale: true, connected: false },
      );
      setCloseCode(status.closeCode);
      lost.current();
    });
    link
      .connect(robot.id)
      .then(() => {
        if (!cancelled) setLinkState("open");
      })
      .catch(() => {
        if (!cancelled) setLinkState("closed");
      });
    return () => {
      cancelled = true;
      offSnapshot();
      offStatus();
      link.disconnect();
    };
  }, [link, robot.id, attempt]);

  const [cameraOn, setCameraOn] = useState(true);
  return (
    <div
      className={ui.robot}
      data-freshness={
        snapshot &&
        !snapshot.stale &&
        snapshot.connected &&
        linkState === "open"
          ? "live"
          : "unavailable"
      }
    >
      <div hidden={page !== "control"} className={ui.controlPage}>
        {snapshot === null ? (
          <div className={ui.connectionEmpty} role="status">
            <h2>
              {linkState === "connecting"
                ? "正在连接车辆…"
                : linkState === "open"
                  ? "正在同步车辆状态"
                  : "车辆连接已断开"}
            </h2>
            <p>
              {linkState === "open"
                ? "车辆已连接，正在准备实时地图与控制。"
                : "等待车辆连接，实时数据将显示在这里。"}
            </p>
            {linkState === "closed" && (
              <Button onClick={() => setAttempt((n) => n + 1)}>重新连接</Button>
            )}
          </div>
        ) : (
          <ControlSurface
            robotId={robot.id}
            link={link}
            snapshot={snapshot}
            active={page === "control"}
            statusOpen={statusOpen}
            onStatusClose={onStatusClose}
            cameraOn={cameraOn}
            onCameraChange={setCameraOn}
          />
        )}
        {snapshot !== null &&
          (linkState === "closed" || snapshot.stale || !snapshot.connected) && (
            <div className={ui.connectionNotice} role="alert">
              <div>
                <p>车辆连接已断开，正在显示最后已知状态。</p>
                <details className="mt-1 text-[9px] text-muted"><summary className="cursor-pointer">连接详情</summary>
                  <p>缓存快照 · 非实时</p>
                  <p>关闭码 {closeCode ?? "未提供"} · 最后同步 {new Date(snapshot.last_synced_at * 1000).toLocaleTimeString("zh-CN", {hour12:false})}</p>
                </details>
              </div>
              <Button onClick={() => setAttempt((n) => n + 1)}>重新连接</Button>
            </div>
          )}
      </div>
      <VehicleTools
        key={`tools-${robot.id}`}
        {...(link.vehicle ? { port: link.vehicle } : {})}
        page={page}
        onBack={() => onPageChange?.("control")}
        cameraOn={cameraOn}
        onCameraChange={setCameraOn}
      />
    </div>
  );
}
