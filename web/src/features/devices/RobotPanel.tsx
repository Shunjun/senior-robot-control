import { useLayoutEffect, useRef, useState, type ReactElement } from "react";

import type { RobotLink } from "../../api/client";
import type { RobotSnapshot, RobotSummary } from "../../api/protocol";
import type { WorkspacePage } from "./useWorkspaceView";
import { Button } from "../../components/ui/Button";
import { VehicleTools } from "./VehicleTools";
import { ControlSurface } from "./ControlSurface";
import styles from "./Devices.module.css";

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

  const [cameraOn, setCameraOn] = useState(false);
  return (
    <div
      className={styles.robot}
      data-freshness={
        snapshot &&
        !snapshot.stale &&
        snapshot.connected &&
        linkState === "open"
          ? "live"
          : "unavailable"
      }
    >
      <div hidden={page !== "control"} className={styles.controlPage}>
        {snapshot === null ? (
          <div className={styles.connectionEmpty} role="status">
            <h2>
              {linkState === "connecting"
                ? "正在建立控制通道…"
                : linkState === "open"
                  ? "等待车端快照"
                  : "控制通道已断开"}
            </h2>
            <p>
              {linkState === "open"
                ? "控制通道已连通，车端尚未上报状态。收到之前不显示任何车辆数据。"
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
            <div className={styles.connectionNotice} role="alert">
              <p>
                控制通道已断开
                {closeCode === null ? "" : `（关闭码 ${closeCode}）`}
                ，当前显示的不是实时状态。<span>缓存快照 · 非实时</span> ·
                最后同步{" "}
                {new Date(snapshot.last_synced_at * 1000).toLocaleTimeString(
                  "zh-CN",
                  { hour12: false },
                )}
              </p>
              <Button onClick={() => setAttempt((n) => n + 1)}>重新连接</Button>
            </div>
          )}
      </div>
      <VehicleTools
        key={`tools-${robot.id}`}
        {...(link.vehicle ? { port: link.vehicle } : {})}
        page={page}
        cameraOn={cameraOn}
        onCameraChange={setCameraOn}
      />
    </div>
  );
}
