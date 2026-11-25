import { useEffect, useState, type ReactElement } from "react";

import type { MapRef, RobotSnapshot } from "../../api/protocol";
import type { OccupancySnapshot, Pose } from "./MapCanvas";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  poseBanner: [
    "map-pose-banner absolute top-[65px] left-[50%] [transform:translateX(-50%)] z-[14] w-[max-content] max-w-[90%]",
    "pt-[11px] pr-[15px] pb-[11px] pl-[15px] bg-none bg-map-pose-banner-background-color border border-solid",
    "border-map-pose-banner-border-color rounded-lg text-[12px] shadow-map-pose-banner max-[760.001px]:top-15 max-[760.001px]:text-[10px] max-[460.001px]:w-[max-content]"
  ].join(" "),
  bannerRow: [
    "map-banner-row flex items-center gap-2.5 flex-wrap [&_button]:text-[11px] [&_button]:pt-1.5 [&_button]:pr-[9px]",
    "[&_button]:pb-1.5 [&_button]:pl-[9px] max-[460.001px]:justify-center max-[460.001px]:gap-1.5"
  ].join(" "),
  primary: [
    "map-primary pt-[9px] pr-[13px] pb-[9px] pl-[13px] border border-solid",
    "rounded-control cursor-pointer bg-none bg-amber border-amber text-white [&:disabled]:opacity-[0.36] [&:disabled]:cursor-not-allowed"
  ].join(" "),
  tool: [
    "map-tool pt-[9px] pr-[13px] pb-[9px] pl-[13px] text-ink bg-none",
    "bg-control border border-solid border-devices-item-border-color rounded-control cursor-pointer [&:disabled]:opacity-[0.36] [&:disabled]:cursor-not-allowed"
  ].join(" "),
  quiet: [
    "map-quiet text-muted text-[11px] leading-[1.8] mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  coordinateDetails: [
    "map-coordinate-details mt-2 text-muted text-[9px] [&_summary]:cursor-pointer"
  ].join(" "),
  fields: [
    "map-fields grid grid-cols-[repeat(3,_minmax(0,_1fr))] gap-3 mt-2 mr-0 mb-2 ml-0",
    "max-[460.001px]:gap-1.5"
  ].join(" "),
  field: [
    "map-field grid gap-[7px] text-[11px] text-field [&_input]:min-w-0 [&_input]:w-full [&_input]:pt-2.5",
    "[&_input]:pr-2.5 [&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:text-input-ink [&_input]:bg-none [&_input]:bg-input [&_input]:border",
    "[&_input]:border-solid [&_input]:border-login-input-border-color [&_input]:rounded-input max-[460.001px]:[&_input]:pt-1.5 max-[460.001px]:[&_input]:pr-1.5 max-[460.001px]:[&_input]:pb-1.5 max-[460.001px]:[&_input]:pl-1.5"
  ].join(" "),
} as const;

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
  onHome?: (goal: MapGoal) => void;
  pending?: boolean;
  orientationReady?: boolean;
  selectionMode?: Mode;
  onCancel?: () => void;
  onRefresh?: () => void;
}

type Mode = "goal" | "hint" | "home";

interface Fields {
  x: string;
  y: string;
  degrees: string;
}

const EMPTY: Fields = { x: "", y: "", degrees: "0" };
const TOLERANCE = 1e-9;

function sameRef(a: MapRef | null, b: MapRef | null): boolean {
  return (
    a !== null &&
    b !== null &&
    a.map_id === b.map_id &&
    a.reference_revision === b.reference_revision
  );
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
    Math.abs(a.x - b.x) < TOLERANCE &&
    Math.abs(a.y - b.y) < TOLERANCE &&
    Math.abs(a.yaw - b.yaw) < TOLERANCE
  );
}

/** Why nothing may be sent for this mode, or null when it may. */
function blockReason(
  snapshot: RobotSnapshot,
  map: OccupancySnapshot,
  mode: Mode,
): string | null {
  const what =
    mode === "home"
      ? "更新 Home"
      : mode === "goal"
        ? "下达目标"
        : "提交定位提示";
  if (snapshot.stale || !snapshot.connected) {
    return `缓存地图，不能${what}。车端恢复实时前不要据此操作。`;
  }
  if (snapshot.map_ref === null || map.map_ref === null) {
    return `没有可用的地图，不能${what}。`;
  }
  if (!sameRef(snapshot.map_ref, map.map_ref)) {
    return `地图版本与车端不一致，不能${what}。`;
  }
  if (mode === "home" && !snapshot.localized)
    return "车辆定位完成后才可编辑本地图 Home。";
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
  onHome,
  pending = false,
  orientationReady = true,
  selectionMode,
  onCancel,
  onRefresh,
}: PosePickerProps): ReactElement {
  const mode = selectionMode ?? "goal";
  const [fields, setFields] = useState<Fields>(EMPTY);
  useEffect(() => {
    if (!onCancel) return;
    const escape = (event: KeyboardEvent) => { if (event.key === "Escape") onCancel(); };
    document.addEventListener("keydown", escape);
    return () => document.removeEventListener("keydown", escape);
  }, [onCancel]);

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
  const blocked = reason !== null || invalid || pending || !orientationReady;
  const mapRef = map.map_ref;

  function submit(): void {
    if (blocked || pose === null || mapRef === null) return;
    if (mode === "home") onHome?.({ map_ref: mapRef, pose });
    else if (mode === "goal") onGoal({ map_ref: mapRef, pose });
    else onHint({ map_ref: mapRef, pose, source: "manual" });
  }

  return (
      <section className={ui.poseBanner} aria-label="地图位姿选取">
        <div className={ui.bannerRow}>
          <span>
            {draft && !orientationReady
              ? "再点击地图，设置朝向"
              : draft
              ? `${mode === "home" ? "Home" : mode === "hint" ? "校准位置" : "导航目标"}已选定`
              : "先点击位置，再点击设置朝向"}
          </span>
          {orientationReady && (draft || fields.x !== "" || fields.y !== "") && (
          <button
            aria-label={mode === "home" ? "更新 Home 坐标" : mode === "hint" ? "提交定位提示" : "确认目标"}
            type="button"
            className={ui.primary}
            disabled={blocked}
            onClick={submit}
          >
            确认
          </button>
          )}
          {draft && orientationReady && (
            <button
              type="button"
              className={ui.tool}
              disabled={pending}
              onClick={() => {
                setFields(EMPTY);
                onDraftChange(null);
              }}
            >
              重选
            </button>
          )}
          {(!draft || !orientationReady) && <button type="button" className={ui.tool} onClick={onCancel}>取消</button>}
        </div>
        {reason && <p className={ui.quiet}>{reason}</p>}
        <details className={ui.coordinateDetails}>
          <summary>精确坐标与朝向</summary>
          <div className={ui.fields}>
            {(
              [
                ["x", "X (m)"],
                ["y", "Y (m)"],
                ["degrees", "朝向 (°)"],
              ] as const
            ).map(([key, label]) => (
              <label className={ui.field} key={key}>
                {label}
                <input
                  inputMode="decimal"
                  value={fields[key]}
                  onChange={(event) => edit({ [key]: event.target.value })}
                />
              </label>
            ))}
          </div>
          {onRefresh && (
            <button type="button" className={ui.tool} onClick={onRefresh}>
              刷新地图
            </button>
          )}
          <p className={ui.quiet}>
            {mode === "hint"
              ? "提示只是扫描匹配的起点，提交后不代表已经定位。"
              : mode === "home"
                ? "Home 与车辆定位独立保存。"
                : "确认后向当前地图下达目标。"}
          </p>
        </details>
      </section>
    );

}
