import { useMemo, useState, type ReactElement } from "react";

import type { RobotLink } from "../../api/client";
import type { RobotSnapshot } from "../../api/protocol";
import { DriveView } from "../drive/DriveView";
import { HomeControls } from "../map/HomeControls";
import { MapCanvas, type OccupancySnapshot, type Pose } from "../map/MapCanvas";
import { PosePicker, type MapGoal, type PoseHint } from "../map/PosePicker";
import { MapManager } from "../mapping/MapManager";
import { TaskList } from "../tasks/TaskList";
import { VideoPanel } from "../video/VideoPanel";
import styles from "./Devices.module.css";

export interface ControlSurfaceProps {
  robotId: string;
  link: RobotLink;
  snapshot: RobotSnapshot;
}

/** Metres per cell for the empty grid shown until a map layer reaches the browser. */
const PLACEHOLDER_RESOLUTION = 0.05;

function describePose(pose: Pose | null): string {
  if (pose === null) return "无位姿";
  return `X ${pose.x.toFixed(2)} m, Y ${pose.y.toFixed(2)} m, 朝向 ${((pose.yaw * 180) / Math.PI).toFixed(1)}°`;
}

/**
 * Drive and map controls of the selected robot, on the snapshot and link the panel already has.
 * No protocol message carries a goal or a relocalization hint, so the picker's output is kept
 * on screen and never sent.
 */
export function ControlSurface({ robotId, link, snapshot }: ControlSurfaceProps): ReactElement {
  const [draft, setDraft] = useState<Pose | null>(null);
  const [note, setNote] = useState<string | null>(null);

  // The occupancy layer has no transport to the browser yet: the grid is empty, and the
  // robot's own snapshot supplies the map reference that goals are checked against.
  const map: OccupancySnapshot = useMemo(
    () => ({
      map_ref: snapshot.map_ref,
      origin: { x: 0, y: 0, yaw: 0 },
      resolution: PLACEHOLDER_RESOLUTION,
      cells: [],
    }),
    [snapshot.map_ref],
  );

  function onGoal(goal: MapGoal): void {
    setNote(`目标已设定（${describePose(goal.pose)}），未发送：协议还没有目标指令。`);
  }

  function onHint(hint: PoseHint): void {
    setNote(`定位提示已设定（${describePose(hint.pose)}），未发送：协议还没有定位提示指令。`);
  }

  return (
    <div className={styles.controlSurface}>
      <DriveView robotId={robotId} link={link} snapshot={snapshot} />
      <HomeControls snapshot={snapshot} link={link} />
      <p className={styles.quiet}>地图栅格数据还没有送到浏览器：下方只有坐标系和位姿设定。</p>
      <MapCanvas
        map={map}
        stale={snapshot.stale || !snapshot.connected}
        pickEnabled
        draft={draft}
        onPick={setDraft}
      />
      <PosePicker
        snapshot={snapshot}
        map={map}
        draft={draft}
        onDraftChange={setDraft}
        onGoal={onGoal}
        onHint={onHint}
      />
      {note !== null && (
        <p className={styles.quiet} role="status">
          {note}
        </p>
      )}
      <TaskList snapshot={snapshot} link={link} />
      <MapManager robotId={robotId} snapshot={snapshot} link={link} />
      <VideoPanel link={link} />
    </div>
  );
}
