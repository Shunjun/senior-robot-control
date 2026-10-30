import { useEffect, useState, type ReactElement } from "react";

import type { MapRef, RobotSnapshot } from "../../api/protocol";
import type { OccupancySnapshot, Pose } from "./MapCanvas";
import styles from "./Map.module.css";

/** A navigation goal. The map reference says which map the pose is in. */
export interface MapGoal {
  map_ref: MapRef;
  pose: Pose;
}

/** A starting guess for relocalization. It seeds scan matching and is never proof of position. */
export interface PoseHint {
  map_ref: MapRef;
  pose: Pose | null;
  source: "manual";
}

export interface PosePickerProps {
  /** The robot's own snapshot: what is live, localized and which map it is on. */
  snapshot: RobotSnapshot;
  /** The map on screen. Its reference must match the robot's. */
  map: OccupancySnapshot;
  draft: Pose | null;
  onDraftChange: (pose: Pose | null) => void;
  onGoal: (goal: MapGoal) => void;
  onHint: (hint: PoseHint) => void;
}

type Mode = "goal" | "hint";

interface Fields {
  x: string;
  y: string;
  degrees: string;
}

const EMPTY: Fields = { x: "", y: "", degrees: "0" };
const TOLERANCE = 1e-9;

function sameRef(a: MapRef | null, b: MapRef | null): boolean {
  return a !== null && b !== null && a.map_id === b.map_id && a.reference_revision === b.reference_revision;
}

function parseNumber(text: string): number | null {
  if (text.trim() === "") return null;
  const value = Number(text);
  return Number.isFinite(value) ? value : null;
}

/** Wrap to (-pi, pi]. */
function normalizeYaw(yaw: number): number {
  let wrapped = yaw % (2 * Math.PI);
  if (wrapped > Math.PI) wrapped -= 2 * Math.PI;
  if (wrapped <= -Math.PI) wrapped += 2 * Math.PI;
  return wrapped;
}

function parseFields(fields: Fields): Pose | null {
  const x = parseNumber(fields.x);
  const y = parseNumber(fields.y);
  const degrees = parseNumber(fields.degrees);
  if (x === null || y === null || degrees === null) return null;
  return { x, y, yaw: normalizeYaw((degrees * Math.PI) / 180) };
}

function toFields(pose: Pose): Fields {
  return {
    x: pose.x.toFixed(2),
    y: pose.y.toFixed(2),
    degrees: ((pose.yaw * 180) / Math.PI).toFixed(1),
  };
}

function samePose(a: Pose, b: Pose): boolean {
  return (
    Math.abs(a.x - b.x) < TOLERANCE && Math.abs(a.y - b.y) < TOLERANCE && Math.abs(a.yaw - b.yaw) < TOLERANCE
  );
}

/** Why nothing may be sent for this mode, or null when it may. */
function blockReason(snapshot: RobotSnapshot, map: OccupancySnapshot, mode: Mode): string | null {
  const what = mode === "goal" ? "下达目标" : "提交定位提示";
  if (snapshot.stale || !snapshot.connected) {
    return `缓存地图，不能${what}。车端恢复实时前不要据此操作。`;
  }
  if (snapshot.map_ref === null || map.map_ref === null) {
    return `没有可用的地图，不能${what}。`;
  }
  if (!sameRef(snapshot.map_ref, map.map_ref)) {
    return `地图版本与车端不一致，不能${what}。`;
  }
  if (mode === "goal" && !snapshot.localized) {
    return "定位未完成，不能下达目标。先用重定位提示完成定位。";
  }
  return null;
}

export function PosePicker({
  snapshot,
  map,
  draft,
  onDraftChange,
  onGoal,
  onHint,
}: PosePickerProps): ReactElement {
  const [mode, setMode] = useState<Mode>("goal");
  const [fields, setFields] = useState<Fields>(EMPTY);

  // A pick on the map writes the fields; typing must not be reformatted under the cursor.
  useEffect(() => {
    if (draft === null) return;
    const typed = parseFields(fields);
    if (typed !== null && samePose(typed, draft)) return;
    setFields(toFields(draft));
    // `fields` is read only to skip echoing the operator's own typing.
  }, [draft]);

  function edit(patch: Partial<Fields>): void {
    const next = { ...fields, ...patch };
    setFields(next);
    onDraftChange(parseFields(next));
  }

  const pose = parseFields(fields);
  const reason = blockReason(snapshot, map, mode);
  const invalid = pose === null;
  const blocked = reason !== null || invalid;
  const mapRef = map.map_ref;

  function submit(): void {
    if (blocked || pose === null || mapRef === null) return;
    if (mode === "goal") onGoal({ map_ref: mapRef, pose });
    else onHint({ map_ref: mapRef, pose, source: "manual" });
  }

  return (
    <section className={styles.picker} aria-label="位姿设定">
      <fieldset className={styles.modes}>
        <legend className={styles.legend}>设定内容</legend>
        <label className={styles.choice}>
          <input type="radio" name="pose-mode" checked={mode === "goal"} onChange={() => setMode("goal")} />
          <span>目标点</span>
        </label>
        <label className={styles.choice}>
          <input type="radio" name="pose-mode" checked={mode === "hint"} onChange={() => setMode("hint")} />
          <span>重定位提示</span>
        </label>
      </fieldset>

      <div className={styles.fields}>
        <label className={styles.field}>
          <span>X (m)</span>
          <input
            inputMode="decimal"
            autoComplete="off"
            value={fields.x}
            onChange={(event) => edit({ x: event.target.value })}
          />
        </label>
        <label className={styles.field}>
          <span>Y (m)</span>
          <input
            inputMode="decimal"
            autoComplete="off"
            value={fields.y}
            onChange={(event) => edit({ y: event.target.value })}
          />
        </label>
        <label className={styles.field}>
          <span>朝向 (°)</span>
          <input
            inputMode="decimal"
            autoComplete="off"
            value={fields.degrees}
            onChange={(event) => edit({ degrees: event.target.value })}
          />
        </label>
      </div>

      {mapRef !== null && (
        <p className={styles.mapRef}>
          地图 {mapRef.map_id} · r{mapRef.reference_revision}
        </p>
      )}
      {reason !== null && (
        <p className={styles.blocked} role="note">
          {reason}
        </p>
      )}
      {reason === null && invalid && (
        <p className={styles.quiet}>输入 X、Y 和朝向，或在地图上按下选点。</p>
      )}
      {mode === "hint" && (
        <p className={styles.quiet}>提示只是扫描匹配的起点，提交后不代表已经定位。</p>
      )}

      <button type="button" className={styles.primary} disabled={blocked} onClick={submit}>
        {mode === "goal" ? "确认目标" : "提交定位提示"}
      </button>
    </section>
  );
}
