import { useLayoutEffect, useRef, useState, type ReactElement } from "react";

import type { RobotLink } from "../../api/client";
import type { RobotSnapshot, RobotSummary } from "../../api/protocol";
import { EmergencyStop } from "../drive/EmergencyStop";
import { VehicleStateView } from "./VehicleStateView";
import { VehicleTools } from "./VehicleTools";
import { ControlSurface } from "./ControlSurface";
import styles from "./Devices.module.css";

export interface RobotPanelProps {
  robot: RobotSummary;
  link: RobotLink;
  /** The control socket closed. The app re-checks the session, since a 401 ends it. */
  onLinkLost: () => void;
}

type LinkState = "connecting" | "open" | "closed";

// ---------------------------------------------------------------- panel

export function RobotPanel({ robot, link, onLinkLost }: RobotPanelProps): ReactElement {
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
      if (cancelled || status.robotId !== robot.id || status.state !== "closed") return;
      setLinkState("closed");
      setSnapshot((previous) => previous === null ? null : { ...previous, stale: true, connected: false });
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

  const presence = linkState !== "open" || snapshot === null || snapshot.stale
    ? "unknown"
    : snapshot.connected ? "online" : "offline";

  return (
    <article className={styles.panel} data-workspace="true">
      <header className={styles.panelHead}>
        <div>
          <p className={styles.eyebrow}>设备 / {robot.id}</p>
          <h2 className={styles.panelTitle}>{robot.name}</h2>
        </div>
        <span
          className={`${styles.presence} ${presence === "unknown" ? styles.presenceUnknown : presence === "online" ? styles.presenceOnline : styles.presenceOffline}`}
        >
          {presence === "unknown" ? "状态未知" : presence === "online" ? "在线" : "离线"}
        </span>
        <EmergencyStop key={`${robot.id}:${attempt}`} link={link} compact />
      </header>

      {linkState === "connecting" && (
        <p className={styles.quiet} role="status">
          正在建立控制通道…
        </p>
      )}
      {linkState === "closed" && (
        <div className={styles.failure} role="alert">
          <p>
            控制通道已断开{closeCode === null ? "" : `（关闭码 ${closeCode}）`}
            ，当前显示的不是实时状态。
          </p>
          <button type="button" className={styles.ghost} onClick={() => setAttempt((n) => n + 1)}>
            重新连接
          </button>
        </div>
      )}
      {snapshot === null ? (
        linkState === "open" && (
          <div className={styles.waiting}>
            <p className={styles.emptyTitle}>等待车端快照</p>
            <p className={styles.quiet}>
              控制通道已连通，车端尚未上报状态。收到之前不显示任何车辆数据。
            </p>
          </div>
        )
      ) : (
        <>
          <p className={styles.quiet}>
            <span>{snapshot.stale ? "缓存快照 · 非实时" : "实时快照"}</span> · 最后同步 {new Date(snapshot.last_synced_at * 1000).toLocaleTimeString("zh-CN", { hour12: false })}</p>
          <VehicleStateView key={`state-${robot.id}`} {...(link.vehicle ? { port: link.vehicle } : {})} />
          <ControlSurface robotId={robot.id} link={link} snapshot={snapshot} />
        </>
      )}
      <VehicleTools key={`tools-${robot.id}`} {...(link.vehicle ? { port: link.vehicle } : {})} />
    </article>
  );
}
