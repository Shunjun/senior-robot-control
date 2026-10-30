import { useEffect, useRef, useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import type { RobotSnapshot, RobotSummary } from "../../api/protocol";
import styles from "./Devices.module.css";

export interface RobotPanelProps {
  robot: RobotSummary;
  link: RobotLink;
  /** The control socket closed. The app re-checks the session, since a 401 ends it. */
  onLinkLost: () => void;
}

type LinkState = "connecting" | "open" | "closed";

function useNowSeconds(): number {
  const [now, setNow] = useState(() => Date.now() / 1000);
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now() / 1000), 1_000);
    return () => clearInterval(timer);
  }, []);
  return now;
}

function formatClock(seconds: number): string {
  return new Date(seconds * 1000).toLocaleTimeString("zh-CN", { hour12: false });
}

function formatAge(seconds: number): string {
  const age = Math.max(0, Math.floor(seconds));
  if (age < 1) return "刚刚";
  if (age < 60) return `${age} 秒前`;
  if (age < 3600) return `${Math.floor(age / 60)} 分钟前`;
  return `${Math.floor(age / 3600)} 小时前`;
}

// ---------------------------------------------------------------- emergency stop

type StopState =
  | { phase: "idle" }
  | { phase: "sending" }
  | { phase: "confirmed" }
  | { phase: "failed"; reason: string };

interface EmergencyStopProps {
  link: RobotLink;
}

/**
 * Fixed e-stop. "Confirmed" is shown only after the robot answered ok. Anything else, including
 * silence, is reported as not confirmed so the operator reaches for the physical stop.
 */
function EmergencyStop({ link }: EmergencyStopProps): ReactElement {
  const [state, setState] = useState<StopState>({ phase: "idle" });

  async function press(): Promise<void> {
    setState({ phase: "sending" });
    try {
      await link.request("safety.estop", {});
      setState({ phase: "confirmed" });
    } catch (error) {
      const reason =
        error instanceof CommandError ? describeCommandError(error.code) : "无法发送指令";
      setState({ phase: "failed", reason });
    }
  }

  return (
    <div className={styles.estopDock}>
      {state.phase === "failed" && (
        <p className={styles.estopFailure} role="alert">
          急停未确认：{state.reason}。请立即使用车上的物理急停。
        </p>
      )}
      {state.phase === "confirmed" && (
        <p className={styles.estopConfirmed} role="status">
          急停已确认：车端已应答
        </p>
      )}
      <button
        type="button"
        className={styles.estop}
        aria-label="急停"
        disabled={state.phase === "sending"}
        onClick={() => void press()}
      >
        <span aria-hidden="true" className={styles.estopCap}>
          急停
        </span>
        <span aria-hidden="true" className={styles.estopTag}>
          {state.phase === "sending" ? "发送中" : "E-STOP"}
        </span>
      </button>
    </div>
  );
}

// ---------------------------------------------------------------- readout

function Readout({ snapshot }: { snapshot: RobotSnapshot }): ReactElement {
  const now = useNowSeconds();
  const age = formatAge(now - snapshot.last_synced_at);
  const { stale } = snapshot;
  return (
    <section className={styles.readout} data-stale={stale}>
      <div className={styles.freshness}>
        {stale ? (
          <>
            <span className={styles.stalePill}>缓存快照 · 非实时</span>
            <span className={styles.syncNote}>
              最后同步 {formatClock(snapshot.last_synced_at)}（{age}）
            </span>
          </>
        ) : (
          <>
            <span className={styles.livePill}>实时快照</span>
            <span className={styles.syncNote}>同步于 {age}</span>
          </>
        )}
      </div>
      {stale && (
        <p className={styles.staleWarning}>
          以下数据是云端缓存，不代表车辆当前状态。恢复实时前不要据此下达指令。
        </p>
      )}
      <p className={styles.bigStatus}>{snapshot.execution_status}</p>
      <dl className={styles.facts}>
        <div>
          <dt>车端连接</dt>
          <dd>{snapshot.connected ? "已连接" : "未连接"}</dd>
        </div>
        <div>
          <dt>定位</dt>
          <dd>{snapshot.localized ? "已定位" : "未定位"}</dd>
        </div>
        <div>
          <dt>地图</dt>
          <dd>
            {snapshot.map_ref === null
              ? "无地图"
              : `${snapshot.map_ref.map_id} · r${snapshot.map_ref.reference_revision}`}
          </dd>
        </div>
        <div>
          <dt>Home</dt>
          <dd>{snapshot.home === null ? "未设置" : `${snapshot.home.waypoint_id} · ${snapshot.home.status}`}</dd>
        </div>
        <div>
          <dt>任务版本</dt>
          <dd>{snapshot.task_revision}</dd>
        </div>
        <div>
          <dt>可执行任务</dt>
          <dd>{snapshot.runnable_task_ids.length}</dd>
        </div>
      </dl>
    </section>
  );
}

// ---------------------------------------------------------------- panel

export function RobotPanel({ robot, link, onLinkLost }: RobotPanelProps): ReactElement {
  const [snapshot, setSnapshot] = useState<RobotSnapshot | null>(null);
  const [linkState, setLinkState] = useState<LinkState>("connecting");
  const [closeCode, setCloseCode] = useState<number | null>(null);
  const [attempt, setAttempt] = useState(0);
  const lost = useRef(onLinkLost);
  lost.current = onLinkLost;

  useEffect(() => {
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

  return (
    <>
    <article className={styles.panel}>
      <header className={styles.panelHead}>
        <div>
          <p className={styles.eyebrow}>设备 / {robot.id}</p>
          <h2 className={styles.panelTitle}>{robot.name}</h2>
        </div>
        <span
          className={`${styles.presence} ${robot.online ? styles.presenceOnline : styles.presenceOffline}`}
        >
          {robot.online ? "在线" : "离线"}
        </span>
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
        <Readout snapshot={snapshot} />
      )}
    </article>
    {/* Outside the clipped panel: a clip-path would cut off the fixed button. */}
    <EmergencyStop link={link} />
    </>
  );
}
