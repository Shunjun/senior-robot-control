import { useLayoutEffect, useMemo, useRef, useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import type { ResponsePayload, RobotSnapshot } from "../../api/protocol";
import { DriveView } from "../drive/DriveView";
import { HomeControls } from "../map/HomeControls";
import { MapCanvas, type OccupancySnapshot, type Pose } from "../map/MapCanvas";
import { useWorkspaceMap } from "../map/useWorkspaceMap";
import { PosePicker, type MapGoal, type PoseHint } from "../map/PosePicker";
import { MapManager } from "../mapping/MapManager";
import { TaskList } from "../tasks/TaskList";
import { VideoPanel } from "../video/VideoPanel";
import styles from "./OperatorWorkspace.module.css";

export interface ControlSurfaceProps {
  robotId: string;
  link: RobotLink;
  snapshot: RobotSnapshot;
}

export type WorkspaceMode = "manual" | "navigation" | "tasks" | "mapping";
export type PrimaryView = "video" | "map";

const MODES: ReadonlyArray<[WorkspaceMode, string]> = [
  ["manual", "手动驾驶"],
  ["navigation", "导航与定位"],
  ["tasks", "任务管理"],
  ["mapping", "地图管理"],
];

/** Metres per cell for the empty grid shown until a map layer reaches the browser. */
const PLACEHOLDER_RESOLUTION = 0.05;

function describePose(pose: Pose | null): string {
  if (pose === null) return "无位姿";
  return `X ${pose.x.toFixed(2)} m, Y ${pose.y.toFixed(2)} m, 朝向 ${((pose.yaw * 180) / Math.PI).toFixed(1)}°`;
}

/** Drive and map controls for the selected vehicle and its current map reference. */
export function ControlSurface({ robotId, link, snapshot }: ControlSurfaceProps): ReactElement {
  const [mapLoad, setMapLoad] = useState(0);
  const [mode, setMode] = useState<WorkspaceMode>("manual");
  const [manualView, setManualView] = useState<PrimaryView>("video");
  const [navigationView, setNavigationView] = useState<PrimaryView>("map");
  const view = mode === "manual" ? manualView : mode === "navigation" ? navigationView : "map";
  const setView = mode === "manual" ? setManualView : setNavigationView;
  const { map: occupancy, gridId, waypoints, message, displayOnly, waypointWrites, navigationReady } =
    useWorkspaceMap(link, snapshot, { enabled: view === "map" || mode === "navigation", catalog: mode === "tasks", refresh: mapLoad });
  const refKey = JSON.stringify([snapshot.map_ref, gridId]);
  const [selection, setSelection] = useState<{ refKey: string; pose: Pose | null }>({ refKey, pose: null });
  const [feedback, setFeedback] = useState<{ refKey: string; note: string } | null>(null);
  useLayoutEffect(() => {
    setSelection({ refKey, pose: null });
  }, [refKey]);
  const draft = selection.refKey === refKey ? selection.pose : null;
  const note = feedback?.refKey === refKey ? feedback.note : null;
  const workspace = occupancy !== undefined && gridId !== undefined && waypoints !== undefined
    ? { map: occupancy, grid_id: gridId, waypoints, waypointWrites: waypointWrites === true }
    : undefined;

  function setDraft(pose: Pose | null): void {
    setSelection({ refKey, pose });
  }

  // The occupancy layer has no transport to the browser yet: the grid is empty, and the
  // robot's own snapshot supplies the map reference that goals are checked against.
  const placeholder: OccupancySnapshot = useMemo(
    () => ({
      map_ref: snapshot.map_ref,
      origin: { x: 0, y: 0, yaw: 0 },
      resolution: PLACEHOLDER_RESOLUTION,
      cells: [],
    }),
    [snapshot.map_ref],
  );

  const map = occupancy ?? placeholder;
  const activeMap = occupancy !== undefined && displayOnly !== true;
  const pickMap = activeMap ? map : { ...map, map_ref: null };

  const directNavigation = activeMap && navigationReady === true;
  const [navigationBusy, setNavigationBusy] = useState(false);
  const navigationScope = useRef(0);
  useLayoutEffect(() => {
    navigationScope.current += 1;
    setNavigationBusy(false);
    setFeedback(null);
    return () => { navigationScope.current += 1; };
  }, [robotId, link, refKey, gridId, directNavigation, snapshot.connected, snapshot.stale, mode]);

  async function runNavigation(action: "goal" | "return_home", pose?: Pose): Promise<void> {
    if (!directNavigation || gridId === undefined || !snapshot.localized || snapshot.map_ref === null || navigationBusy) return;
    const scope = navigationScope.current;
    const label = action === "goal" ? "目标" : "返航";
    setNavigationBusy(true);
    setFeedback({ refKey, note: `等待车端确认${label}…` });
    try {
      const response = await link.request<ResponsePayload>(`navigation.${action}`, {
        operation_id: crypto.randomUUID(),
        body: { ...snapshot.map_ref, grid_id: gridId, ...pose },
      });
      if (scope === navigationScope.current) {
        setFeedback({ refKey, note: response.ok ? `车端已受理${label}。` : `车端未确认${label}。` });
      }
    } catch (error) {
      if (scope === navigationScope.current) {
        const reason = error instanceof CommandError ? describeCommandError(error.code) : "无法发送";
        setFeedback({ refKey, note: `${label}未确认：${reason}。` });
      }
    } finally {
      if (scope === navigationScope.current) setNavigationBusy(false);
    }
  }

  function onGoal(goal: MapGoal): void {
    if (directNavigation) {
      void runNavigation("goal", goal.pose);
    } else {
      //Compatible: older vehicles only support a local goal preview.
      setFeedback({ refKey, note: `目标已设定（${describePose(goal.pose)}），未发送：车端未开放目标指令。` });
    }
  }

  function onHint(hint: PoseHint): void {
    setFeedback({ refKey, note: `定位提示已设定（${describePose(hint.pose)}），未发送：车端未开放定位提示指令。` });
  }

  return (
    <div className={styles.workspace} data-mode={mode}>
      <div className={styles.toolbar}>
        <nav className={styles.switches} aria-label="车辆场景">
          {MODES.map(([value, label]) => (
            <button key={value} type="button" aria-pressed={mode === value} onClick={() => setMode(value)}>
              {label}
            </button>
          ))}
        </nav>
        {view === "map" && mode !== "manual" && (
          <div className={styles.switches}>
            <button type="button" onClick={() => setMapLoad((previous) => previous + 1)}>刷新地图</button>
          </div>
        )}
        {(mode === "manual" || mode === "navigation") && (
          <div className={styles.switches} role="group" aria-label="主视图">
            <button type="button" aria-pressed={view === "video"} onClick={() => setView("video")}>视频</button>
            <button type="button" aria-pressed={view === "map"} onClick={() => setView("map")}>地图</button>
          </div>
        )}
      </div>
      {mode === "tasks" && <TaskList snapshot={snapshot} link={link}
        workspace={workspace} map={occupancy} unavailable={message} />}
      <div className={styles.stage} data-tasks={mode === "tasks"}>
        {mode !== "tasks" && <div className={styles.primary}>
          {view === "video" ? (
            <VideoPanel link={link} live={snapshot.connected && !snapshot.stale} compact />
          ) : (
            <>
              {message !== undefined && <p className={styles.quiet}>{message}</p>}
              {(occupancy !== undefined || mode === "navigation") && (
                <MapCanvas
                  key={gridId ?? "unavailable"}
                  map={map}
                  compact={occupancy !== undefined}
                  stale={snapshot.stale || !snapshot.connected}
                  pickEnabled={mode === "navigation" && activeMap}
                  draft={mode === "navigation" ? draft : null}
                  onPick={setDraft}
                />
              )}
            </>
          )}
        </div>}
        <div className={styles.operations} data-manual={mode === "manual"}>
          <DriveView robotId={robotId} link={link} snapshot={snapshot} active={mode === "manual"} compact />
          {mode === "navigation" && (
            <>
              <HomeControls
                key={`home:${refKey}`}
                snapshot={snapshot}
                link={link}
                homeMapAvailable={activeMap && waypointWrites === true}
                {...(directNavigation
                  ? { directNavigation: { onReturnHome: () => void runNavigation("return_home"), pending: navigationBusy } }
                  : { navigationUnavailable: "车端未报告已验证的导航能力，无法开始、继续或返航。" })}
              />
              <p className={styles.quiet}>
                {directNavigation ? "定位提示保存在当前页面，车端尚不接收。"
                  : "目标与定位提示为本地预览，车端尚不接收。"}
              </p>
              <PosePicker
                key={`pose:${refKey}`}
                snapshot={snapshot}
                map={pickMap}
                draft={activeMap ? draft : null}
                onDraftChange={setDraft}
                pending={navigationBusy}
                onGoal={onGoal}
                onHint={onHint}
              />
              {note !== null && <p className={styles.quiet} role="status">{note}</p>}
            </>
          )}
          {mode === "mapping" && <MapManager robotId={robotId} snapshot={snapshot} link={link} />}
        </div>
      </div>
    </div>
  );
}
