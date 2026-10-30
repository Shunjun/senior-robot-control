import { useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import type { MapRef, ResponsePayload, RobotSnapshot } from "../../api/protocol";
import styles from "./Map.module.css";

export interface HomeControlsProps {
  /** Null until the robot's first snapshot arrives. Nothing is enabled without a live one. */
  snapshot: RobotSnapshot | null;
  link: RobotLink;
}

type Outcome = { kind: "ok"; text: string } | { kind: "error"; text: string };

type ActionName = "start" | "resume" | "cancel" | "home";

const ACTION_LABEL: Record<ActionName, string> = {
  start: "开始导航",
  resume: "继续导航",
  cancel: "取消导航",
  home: "设为 Home",
};

const STATUS_TEXT: Record<string, string> = {
  accepted: "已受理，等待执行",
  dispatching: "正在下发",
  running: "执行中",
  succeeded: "已完成",
  failed: "执行失败",
  needs_review: "需要人工核查",
};

function sameRef(a: MapRef, b: MapRef): boolean {
  return a.map_id === b.map_id && a.reference_revision === b.reference_revision;
}

function isLive(snapshot: RobotSnapshot | null): snapshot is RobotSnapshot {
  return snapshot !== null && !snapshot.stale && snapshot.connected;
}

/** Every reason navigation may not start. Empty means it may. */
function navigationBlocks(snapshot: RobotSnapshot | null): string[] {
  if (snapshot === null) return ["等待车端快照，收到之前不能下达导航指令。"];
  if (!isLive(snapshot)) return ["缓存快照，不是车辆当前状态，不能下达导航指令。"];
  const reasons: string[] = [];
  if (snapshot.home === null) {
    reasons.push("Home 未设置，不能开始导航。");
  } else if (snapshot.map_ref === null || !sameRef(snapshot.home.map_ref, snapshot.map_ref)) {
    reasons.push("Home 不属于当前地图，不能开始导航。");
  }
  if (!snapshot.localized) reasons.push("导航需要先完成定位。");
  return reasons;
}

export function HomeControls({ snapshot, link }: HomeControlsProps): ReactElement {
  const [pending, setPending] = useState<ActionName | null>(null);
  const [outcome, setOutcome] = useState<Outcome | null>(null);
  const [confirmReplace, setConfirmReplace] = useState(false);

  const live = isLive(snapshot);
  const blocks = navigationBlocks(snapshot);
  const canNavigate = blocks.length === 0;
  const canSetHome = live && snapshot.localized && snapshot.map_ref !== null;

  async function run(action: ActionName, type: string, payload: unknown): Promise<void> {
    setPending(action);
    setOutcome(null);
    try {
      const response = await link.request<ResponsePayload>(type, payload);
      const status = response.status ?? null;
      const detail = status === null ? "" : `（${STATUS_TEXT[status] ?? status}）`;
      setOutcome({ kind: "ok", text: `车端已应答：${ACTION_LABEL[action]}${detail}` });
    } catch (error) {
      const reason = error instanceof CommandError ? describeCommandError(error.code) : "无法发送指令";
      setOutcome({ kind: "error", text: `${ACTION_LABEL[action]}未完成：${reason}` });
    } finally {
      setPending((current) => (current === action ? null : current));
    }
  }

  function setHome(): void {
    if (snapshot?.home != null && !confirmReplace) {
      setConfirmReplace(true);
      return;
    }
    setConfirmReplace(false);
    void run("home", "home.set", { operation_id: crypto.randomUUID(), body: {} });
  }

  const home = snapshot?.home ?? null;
  const localization =
    snapshot === null ? "未知" : snapshot.localized ? "已定位" : "重新定位未完成：车辆还没有匹配到地图";

  return (
    <section className={styles.home} aria-label="Home 与导航">
      <dl className={styles.facts}>
        <div>
          <dt>Home</dt>
          <dd>{home === null ? "未设置" : `${home.waypoint_id} · ${home.status}`}</dd>
        </div>
        <div>
          <dt>定位</dt>
          <dd>
            <span role="status" aria-label="定位状态" data-localized={snapshot?.localized === true}>
              {localization}
            </span>
          </dd>
        </div>
      </dl>

      {blocks.length > 0 && (
        <ul className={styles.blockedList}>
          {blocks.map((reason) => (
            <li key={reason} className={styles.blocked}>
              {reason}
            </li>
          ))}
        </ul>
      )}

      <div className={styles.actions}>
        <button
          type="button"
          className={styles.primary}
          disabled={!canNavigate || pending === "start"}
          onClick={() => void run("start", "navigation.start", {})}
        >
          开始导航
        </button>
        <button
          type="button"
          className={styles.tool}
          disabled={!canNavigate || pending === "resume"}
          onClick={() => void run("resume", "navigation.resume", {})}
        >
          继续导航
        </button>
        <button
          type="button"
          className={styles.tool}
          disabled={!live || pending === "cancel"}
          onClick={() => void run("cancel", "navigation.cancel", {})}
        >
          取消导航
        </button>
      </div>

      <div className={styles.actions}>
        {confirmReplace ? (
          <>
            <button type="button" className={styles.primary} onClick={setHome}>
              确认替换 Home
            </button>
            <button type="button" className={styles.tool} onClick={() => setConfirmReplace(false)}>
              保留原 Home
            </button>
          </>
        ) : (
          <button
            type="button"
            className={styles.tool}
            disabled={!canSetHome || pending === "home"}
            onClick={setHome}
          >
            设为 Home
          </button>
        )}
      </div>
      {live && !canSetHome && <p className={styles.quiet}>设置 Home 需要车辆已定位并在地图上。</p>}

      {outcome?.kind === "error" && (
        <p className={styles.failure} role="alert">
          {outcome.text}
        </p>
      )}
      {outcome?.kind === "ok" && (
        <p className={styles.confirmed} role="status">
          {outcome.text}
        </p>
      )}
    </section>
  );
}
