import { useEffect, useLayoutEffect, useMemo, useRef, useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import type { ResponsePayload, RobotSnapshot } from "../../api/protocol";
import { readMappingCatalog, sendMapCommand } from "../../api/mapping";
import { DriveView } from "../drive/DriveView";
import { HomeControls } from "../map/HomeControls";
import { type OccupancySnapshot, type Pose } from "../map/MapCanvas";
import { useWorkspaceMap } from "../map/useWorkspaceMap";
import { PosePicker, type MapGoal, type PoseHint } from "../map/PosePicker";
import { TaskExecutionControls } from "./VehicleStateView";
import { StatusOverlay } from "./StatusOverlay";
import { Switch } from "../../components/ui/Switch";
import { IconButton } from "../../components/ui/IconButton";
import { ArrowLeftRight, Gamepad2, Navigation, ListOrdered, Map as MapIcon } from "lucide-react";
import { MapManager } from "../mapping/MapManager";
import { TaskList } from "../tasks/TaskList";
import { VideoPanel } from "../video/VideoPanel";
import styles from "./OperatorWorkspace.module.css";

export interface ControlSurfaceProps {
  robotId: string;
  link: RobotLink;
  snapshot: RobotSnapshot;
  active?: boolean;
  statusOpen?: boolean;
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

function navigationUnavailableMessage(reason: string | null | undefined): string {
  //Compatible: legacy snapshots have no concrete navigation diagnostic.
  if (reason == null) return "车端未报告已验证的导航能力，无法开始、继续或返航。";
  const explanations: Record<string, string> = {
    "map or localization generation changed": "地图或定位代次已变化，请等待重新定位。",
    "map or localization unavailable": "当前地图或定位尚未就绪。",
    "planner state unavailable": "路径规划器或运动控制器尚未就绪。",
    "heartbeat or control authority unavailable": "车辆心跳或控制权限尚未就绪。",
    "navigation unavailable": "车端导航尚未就绪。",
  };
  const sensorPrefix = "safety status unavailable (";
  const explanation = reason.startsWith(sensorPrefix) && reason.endsWith(")")
    ? `传感器安全检查尚未通过：${reason.slice(sensorPrefix.length, -1)}。`
    : explanations[reason] ?? `车端导航尚未就绪：${reason}。`;
  return `无法导航：${explanation}`;
}

/** Drive and map controls for the selected vehicle and its current map reference. */
export function ControlSurface({ robotId, link, snapshot, active = true, statusOpen = false }: ControlSurfaceProps): ReactElement {
  const [mapLoad, setMapLoad] = useState(0);
  const [mode, setMode] = useState<WorkspaceMode>("manual");
  const [view, setView] = useState<PrimaryView>("map");
  const [performanceExpired, setPerformanceExpired] = useState(false);
  useEffect(() => {
    setPerformanceExpired(false);
    const timer=setTimeout(() => setPerformanceExpired(true),3500);
    return () => clearTimeout(timer);
  }, [snapshot.performance?.sequence]);
  const [statusTab, setStatusTab] = useState<"motion" | "performance">("motion");
  const [cameraOn, setCameraOn] = useState(false);
  const { map: occupancy, gridId, waypoints, message, displayOnly, waypointWrites, navigationReady } =
    useWorkspaceMap(link, snapshot, { enabled: true, catalog: mode === "tasks", refresh: mapLoad });
  //Compatible: legacy interactions keep their original per-grid scope.
  const referenceVersion = snapshot.map_grid_id === undefined ? gridId : snapshot.map_generation;
  const refKey = JSON.stringify([snapshot.map_ref, referenceVersion]);
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

  // Keep an empty preview until the current occupancy map has loaded.
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
  const mappingEdits = snapshot.mapping_protocol_version === 2;
  const pickMap = activeMap || mappingEdits ? map : { ...map, map_ref: null };
  const liveMapTelemetry = activeMap && snapshot.connected && !snapshot.stale && snapshot.localized
    && snapshot.map_generation !== undefined && snapshot.map_generation === referenceVersion
    && snapshot.map_ref !== null && map.map_ref !== null
    && snapshot.map_ref.map_id === map.map_ref.map_id
    && snapshot.map_ref.reference_revision === map.map_ref.reference_revision;

  //Compatible: older vehicles advertise navigation eligibility only in the map manifest.
  const directNavigation = activeMap && (snapshot.navigation_ready ?? navigationReady) === true;
  const route = snapshot.route_progress;
  const routeActive = activeMap && route !== undefined && route.map_generation === referenceVersion
    && (route.status === "starting" || route.status === "running" || route.status === "cancel_pending");
  const [navigationBusy, setNavigationBusy] = useState(false);
  const navigationScope = useRef(0);
  useLayoutEffect(() => {
    navigationScope.current += 1;
    setNavigationBusy(false);
    setFeedback(null);
    return () => { navigationScope.current += 1; };
  }, [robotId, link, refKey, snapshot.connected, snapshot.stale, mode]);

  async function runNavigation(action: "goal" | "return_home", pose?: Pose): Promise<void> {
    if (!directNavigation || gridId === undefined || !snapshot.localized || snapshot.map_ref === null || navigationBusy || routeActive) return;
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

  async function editMappingPose(kind: "map.calibrate" | "map.home", hint: MapGoal): Promise<void> {
    if (!mappingEdits || hint.pose === null || snapshot.map_ref === null || !snapshot.connected
      || snapshot.stale || navigationBusy || routeActive) return;
    const owner = navigationScope.current;
    const id = crypto.randomUUID();
    const label = kind === "map.home" ? "Home 坐标更新" : "车辆局部匹配";
    setNavigationBusy(true);
    setFeedback({ refKey, note: `正在进行${label}…` });
    try {
      const catalog = await readMappingCatalog(link, robotId);
      if (owner !== navigationScope.current) return;
      const selected = catalog.selection.map_ref;
      if (selected?.map_id !== hint.map_ref.map_id || selected.reference_revision !== hint.map_ref.reference_revision) {
        throw new CommandError("conflict", "地图选择已变化");
      }
      let status: ResponsePayload["status"] = "running";
      try {
        const reply = await sendMapCommand(link, kind, id, {
          ...hint.map_ref, ...hint.pose, expected_generation: catalog.selection.generation,
          ...(kind === "map.home" ? { expected_home_revision: snapshot.home_revision } : {}),
        });
        status = reply.status;
      } catch (reason) {
        if (!(reason instanceof CommandError) || (reason.code !== "timeout" && reason.code !== "operation_unknown")) throw reason;
      }
      while (owner === navigationScope.current && !["succeeded", "failed", "needs_review"].includes(status ?? "")) {
        await new Promise(resolve => setTimeout(resolve, 1000));
        if (owner !== navigationScope.current) return;
        const reply = await link.request<ResponsePayload>("operation.query", { operation_id: id, body: {} });
        if (!reply.ok || reply.operation_id !== id || reply.status == null) throw new CommandError("operation_unknown", "匹配结果未确认");
        status = reply.status;
      }
      if (owner === navigationScope.current) {
        setFeedback({ refKey, note: status === "succeeded" ? `${label}已完成。` : `${label}未完成，请检查车辆状态后重试。` });
        setMapLoad(value => value + 1);
      }
    } catch (reason) {
      if (owner === navigationScope.current) setFeedback({ refKey,
        note: `${label}未完成：${reason instanceof CommandError ? describeCommandError(reason.code) : "连接未确认"}。` });
    } finally {
      if (owner === navigationScope.current) setNavigationBusy(false);
    }
  }

  function onHint(hint: PoseHint): void {
    if (mappingEdits && hint.pose !== null) { void editMappingPose("map.calibrate", { map_ref: hint.map_ref, pose: hint.pose }); return; }
    //Compatible: original mapping protocol has no manual calibration operation.
    setFeedback({ refKey, note: `定位提示已设定（${describePose(hint.pose)}），未发送：车端未开放定位提示指令。` });
  }

  const icons = { manual: Gamepad2, navigation: Navigation, tasks: ListOrdered, mapping: MapIcon };
  return <TaskList snapshot={snapshot} link={link} workspace={workspace} map={map} unavailable={message}
    mapCanvasProps={mode === 'tasks' ? {mini:cameraOn && view === 'video'} : {
      mini:cameraOn && view === 'video',
      map, robot: liveMapTelemetry ? snapshot.robot_pose ?? null : null,
      scanPoints: liveMapTelemetry ? snapshot.scan_points ?? [] : [],
      pickEnabled: mode === 'navigation' && activeMap, draft: mode === 'navigation' ? draft : null,
      onPick: setDraft, taskRoute: [], waypointMarkers: [], onAddWaypoint: () => {},
    }}
    renderWorkspace={(mapView, taskPanel) => <div className={styles.workspace} data-mode={mode} data-primary={cameraOn ? view : 'map'}>
      <div className={styles.previewHead}>
        <span>{cameraOn && view === 'video' ? '摄像头' : '实时地图'}</span>
        {occupancy === undefined && message && <p className={styles.mapMessage} role="status">{message}</p>}
        <Switch label="摄像头" checked={cameraOn} onChange={next => { setCameraOn(next); if (!next) setView('map'); }} />
      </div>
      <div className={styles.mapViewport}>
        {mapView}
        {cameraOn && view === 'video' && <div className={styles.swap}><IconButton label="交换主次画面" onClick={() => setView('map')}><ArrowLeftRight /></IconButton></div>}
      </div>
      {cameraOn && <div className={styles.videoViewport}>
        <VideoPanel link={link} live={active && snapshot.connected && !snapshot.stale} compact />
        {view === 'map' && <div className={styles.swap}><IconButton label="交换主次画面" onClick={() => setView('video')}><ArrowLeftRight /></IconButton></div>}
      </div>}
      <div className={styles.status} hidden={!statusOpen}><StatusOverlay snapshot={snapshot} port={link.vehicle} tab={statusTab} onTabChange={setStatusTab} performanceExpired={performanceExpired} visible={active && statusOpen} /></div>
      <div className={styles.operations} data-manual={mode === 'manual'}>
        <div hidden={mode !== 'manual'}><DriveView robotId={robotId} link={link} snapshot={snapshot} active={active && mode === 'manual'} compact /></div>
        <div hidden={mode !== 'tasks'}>{taskPanel}<TaskExecutionControls port={link.vehicle} /></div>
        {mode === 'navigation' && <>
          <HomeControls key={`home:${refKey}`} snapshot={snapshot} link={link} homeMapAvailable={activeMap && waypointWrites === true}
            {...(directNavigation ? { directNavigation: { onReturnHome: () => void runNavigation('return_home'), pending: navigationBusy || routeActive } }
              : { navigationUnavailable: navigationUnavailableMessage(snapshot.navigation_block_reason) })} />
          <button type="button" onClick={() => setMapLoad(n => n + 1)}>刷新地图</button>
          <PosePicker key={`pose:${refKey}`} snapshot={snapshot} map={pickMap} draft={snapshot.map_grid_id === undefined && !activeMap ? null : draft}
            onDraftChange={setDraft} pending={navigationBusy || routeActive} onGoal={onGoal} onHint={onHint}
            {...(mappingEdits ? { onHome: (goal: MapGoal) => void editMappingPose('map.home', goal) } : {})} />
          {note && <p role="status">{note}</p>}
        </>}
        {mode === 'mapping' && <MapManager robotId={robotId} snapshot={snapshot} link={link} />}
      </div>
      <nav className={styles.dock} aria-label="车辆场景">{MODES.map(([value, label]) => {
        const Icon = icons[value];
        return <button type="button" key={value} aria-label={label} aria-pressed={mode === value} onClick={() => setMode(value)}><Icon size={18} /><span>{label}</span></button>;
      })}</nav>
      <div className={styles.legend} aria-label="地图图例">
        <span><i data-kind="stable" />稳定结构</span><span><i data-kind="free" />确认空闲</span><span><i data-kind="unknown" />未知</span><span><i data-kind="scan" />实时扫描</span>
        <button type="button" onClick={() => setFeedback({ refKey, note:'暂未实现：独立候选层 / 空闲证据层 / 线段拟合' })}>更多图层</button>
      </div>
      {note?.startsWith('暂未实现') && <p className={styles.toast} role="status">{note}</p>}
    </div>} />;
}
