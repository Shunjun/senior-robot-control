import { newId } from "../../ids";
import {
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
  type ReactElement,
} from "react";

import {
  CommandError,
  describeCommandError,
  type RobotLink,
} from "../../api/client";
import type { ResponsePayload, RobotSnapshot } from "../../api/protocol";
import { readMappingCatalog, sendMapCommand } from "../../api/mapping";
import { DriveView } from "../drive/DriveView";
import { HomeControls } from "../map/HomeControls";
import { type OccupancySnapshot, type Pose } from "../map/MapCanvas";
import { useWorkspaceMap } from "../map/useWorkspaceMap";
import { PosePicker, type MapGoal, type PoseHint } from "../map/PosePicker";
import { StatusOverlay } from "./StatusOverlay";
import { Switch } from "../../components/ui/Switch";

import { IconButton } from "../../components/ui/IconButton";
import {
  ArrowLeftRight,
  Gamepad2,
  Navigation,
  ListOrdered,
  Map as MapIcon,
} from "lucide-react";
import { MapManager } from "../mapping/MapManager";
import { TaskList } from "../tasks/TaskList";
import { VideoPanel } from "../video/VideoPanel";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  workspace: [
    "workspace-workspace [--edge:16px] [--bottom:84px] relative w-full h-full min-h-full overflow-hidden",
    "isolate bg-none bg-map-unknown [&[data-primary=video]_.workspace-map-viewport]:absolute [&[data-primary=video]_.workspace-map-viewport]:inset-auto [&[data-primary=video]_.workspace-map-viewport]:left-[var(--edge)] [&[data-primary=video]_.workspace-map-viewport]:bottom-[var(--bottom)] [&[data-primary=video]_.workspace-map-viewport]:w-[250px]",
    "[&[data-primary=video]_.workspace-map-viewport]:h-[155px] [&[data-primary=video]_.workspace-map-viewport]:border [&[data-primary=video]_.workspace-map-viewport]:border-solid [&[data-primary=video]_.workspace-map-viewport]:border-workspace-video-viewport-border-color [&[data-primary=video]_.workspace-map-viewport]:rounded-preview [&[data-primary=video]_.workspace-map-viewport]:overflow-hidden [&[data-primary=video]_.workspace-map-viewport]:z-[3] [&[data-primary=video]_.workspace-map-viewport]:bg-none",
    "[&[data-primary=video]_.workspace-map-viewport]:bg-graphite-900 [&[data-primary=video]_.workspace-map-viewport]:shadow-workspace-video-viewport [&[data-primary=video]_.workspace-video-viewport]:inset-0 [&[data-primary=video]_.workspace-video-viewport]:w-auto [&[data-primary=video]_.workspace-video-viewport]:h-auto [&[data-primary=video]_.workspace-video-viewport]:border-0 [&[data-primary=video]_.workspace-video-viewport]:border-none [&[data-primary=video]_.workspace-video-viewport]:border-current",
    "[&[data-primary=video]_.workspace-video-viewport]:[border-radius:0] [&[data-primary=video]_.workspace-video-viewport]:z-[0] [&[data-mode=tasks]_.workspace-operations:has([data-task-stage=editor])]:w-[315px] [&[data-mode=tasks]_.workspace-operations:has([data-task-stage=editor])]:p-[19px] [&[data-mode=tasks]_.workspace-operations:has([data-task-stage=editor])]:z-[11] [&[data-task-picking=true]_.workspace-operations]:hidden [&[data-nav-picking=true]_.workspace-operations]:hidden min-[1500px]:[&[data-primary=video]_.workspace-map-viewport]:w-[300px] min-[1500px]:[&[data-primary=video]_.workspace-map-viewport]:h-[180px]",
    "max-[760.001px]:[--edge:12px] max-[760.001px]:[--bottom:80px] max-[760.001px]:[&[data-primary=video]_.workspace-map-viewport]:w-[180px] max-[760.001px]:[&[data-primary=video]_.workspace-map-viewport]:h-[125px] max-[460.001px]:[&[data-mode=tasks]_.workspace-operations:has([data-task-stage=editor])]:left-[var(--edge)] max-[460.001px]:[&[data-mode=tasks]_.workspace-operations:has([data-task-stage=editor])]:w-auto max-[460.001px]:[&[data-mode=tasks]_.workspace-operations:has([data-task-stage=editor])]:max-h-[calc(100%_-_var(--bottom)_-_72px)] max-[460.001px]:[&[data-primary=video]_.workspace-map-viewport]:w-[127px]",
    "max-[460.001px]:[&[data-primary=video]_.workspace-map-viewport]:h-[90px] max-[460.001px]:[&[data-primary=video]_.workspace-map-viewport]:top-[65px] max-[460.001px]:[&[data-primary=video]_.workspace-map-viewport]:bottom-auto max-[460.001px]:[&[data-mode=manual]_.workspace-video-viewport]:top-auto max-[460.001px]:[&[data-mode=manual]_.workspace-video-viewport]:bottom-[calc(var(--bottom)_+_var(--drive-card-height,_250px)_+_12px)] max-[460.001px]:[&[data-mode=manual][data-primary=video]_.workspace-map-viewport]:top-auto max-[460.001px]:[&[data-mode=manual][data-primary=video]_.workspace-map-viewport]:bottom-[calc(var(--bottom)_+_var(--drive-card-height,_250px)_+_12px)] max-[460.001px]:[&[data-primary=video]_.workspace-video-viewport]:inset-0",
    "max-[460.001px]:[&[data-primary=video]_.workspace-video-viewport]:w-auto max-[460.001px]:[&[data-primary=video]_.workspace-video-viewport]:h-auto",
    "max-[460.001px]:[--bottom:7rem] max-[460.001px]:[&[data-mode=manual]_.workspace-toast]:top-24 max-[460.001px]:[&[data-mode=manual]_.workspace-toast]:bottom-auto"
  ].join(" "),
  previewHead: [
    "workspace-preview-head absolute top-[var(--edge)] left-[var(--edge)] right-[var(--edge)] flex items-center gap-3",
    "z-[6] pointer-events-none [&_button]:pointer-events-auto max-[760.001px]:gap-[5px]"
  ].join(" "),
  viewName: [
    "workspace-view-name text-xs text-workspace-view-name-color bg-none bg-workspace-view-name-background-color pt-2 pr-3 pb-2",
    "pl-3 border border-solid border-devices-pairing-receipt-border-color rounded-md max-[760.001px]:text-xs max-[760.001px]:pt-[7px] max-[760.001px]:pr-[7px]",
    "max-[760.001px]:pb-[7px] max-[760.001px]:pl-[7px] max-[460.001px]:whitespace-nowrap max-[460.001px]:text-xs max-[460.001px]:pt-[7px] max-[460.001px]:pr-2 max-[460.001px]:pb-[7px] max-[460.001px]:pl-2",
    "max-[460.001px]:max-w-[112px] max-[460.001px]:overflow-hidden max-[460.001px]:text-ellipsis"
  ].join(" "),
  badge: [
    "workspace-badge pt-1 pr-[7px] pb-1 pl-[7px] rounded-xs bg-none bg-workspace-badge-background-color",
    "text-workspace-badge-color text-xs max-[460.001px]:hidden"
  ].join(" "),
  grow: [
    "workspace-grow flex-1"
  ].join(" "),
  mapMessage: [
    "workspace-map-message absolute top-11.5 left-0 max-w-full pt-2 pr-3 pb-2",
    "pl-3 bg-none bg-workspace-map-message-background-color rounded-md text-xs pointer-events-none"
  ].join(" "),
  mapViewport: [
    "workspace-map-viewport absolute inset-0 [&>div]:h-full [&>div>section]:h-full"
  ].join(" "),
  swap: [
    "workspace-swap absolute right-2 top-2 z-[8] [&_button]:w-[32px] [&_button]:h-[30px] [&_button]:pt-[5px]",
    "[&_button]:pr-[5px] [&_button]:pb-[5px] [&_button]:pl-[5px] [&_button]:bg-none [&_button]:bg-map-tool [&_button]:border [&_button]:border-solid [&_button]:border-workspace-swap-border-color",
    "[&_button]:rounded-input [&_button_svg]:w-full [&_button_svg]:h-full max-[460.001px]:right-1.5 max-[460.001px]:top-1.5 max-[460.001px]:[&_button]:w-[29px] max-[460.001px]:[&_button]:h-[27px]"
  ].join(" "),
  videoViewport: [
    "workspace-video-viewport absolute inset-auto left-[var(--edge)] bottom-[var(--bottom)] w-[250px] h-[155px] border",
    "border-solid border-workspace-video-viewport-border-color rounded-preview overflow-hidden z-[3] bg-none bg-graphite-900 shadow-workspace-video-viewport",
    "min-[1500px]:w-[300px] min-[1500px]:h-[180px] max-[760.001px]:w-[180px] max-[760.001px]:h-[125px] max-[460.001px]:w-[127px] max-[460.001px]:h-[90px] max-[460.001px]:top-[65px] max-[460.001px]:bottom-auto"
  ].join(" "),
  status: [
    "workspace-status absolute left-[var(--edge)] top-[65px] w-[310px] max-h-[calc(100%_-_150px)] overflow-auto z-[13]",
    "max-[760.001px]:w-[280px] max-[760.001px]:top-15 max-[460.001px]:w-[260px]"
  ].join(" "),
  operations: [
    "workspace-operations absolute right-[var(--edge)] bottom-[var(--bottom)] w-[286px] max-h-[calc(100%_-_var(--bottom)_-_72px)] overflow-auto z-[7]",
    "rounded-card bg-none bg-card border border-solid border-line shadow-ui-card [scrollbar-width:thin]",
    "[scrollbar-color:#63675c_transparent] pt-[19px] pr-[19px] pb-[19px] pl-[19px] backdrop-blur-[15px] [&>div[hidden]]:hidden min-[1500px]:w-[310px]",
    "max-[760.001px]:w-[235px] max-[760.001px]:pt-3.5 max-[760.001px]:pr-3.5 max-[760.001px]:pb-3.5 max-[760.001px]:pl-3.5 max-[460.001px]:w-[202px] max-[460.001px]:pt-3 max-[460.001px]:pr-3",
    "max-[460.001px]:pb-3 max-[460.001px]:pl-3 max-[460.001px]:max-h-[58%] max-[460.001px]:[&[data-manual=true]]:left-[var(--edge)] max-[460.001px]:[&[data-manual=true]]:w-auto max-[460.001px]:[&[data-manual=true]]:overflow-visible max-[460.001px]:[&[data-manual=true]]:max-h-[calc(100%_-_var(--bottom)_-_72px)]"
  ].join(" "),
  bottomBar: [
    "workspace-bottom-bar absolute left-[var(--edge)] right-[var(--edge)] bottom-3 grid grid-cols-[minmax(0,_1fr)_auto_minmax(0,_1fr)] items-end",
    "gap-3 z-[15] pointer-events-none [&>*]:pointer-events-auto max-[760.001px]:bottom-2.5 max-[760.001px]:gap-1.5 max-[460.001px]:gap-1"
  ].join(" "),
  mapTools: [
    "workspace-map-tools min-w-0"
  ].join(" "),
  dock: [
    "workspace-dock flex gap-[3px] pt-1.5 pr-1.5 pb-1.5 pl-1.5 border",
    "border-solid border-workspace-dock-border-color rounded-dock bg-none bg-devices-connection-notice-background-color shadow-workspace-dock [&_button]:flex [&_button]:items-center",
    "[&_button]:justify-center [&_button]:border-0 [&_button]:border-none [&_button]:border-current [&_button]:rounded-dock-item [&_button]:bg-none [&_button]:bg-transparent [&_button]:text-workspace-dock-color",
    "[&_button]:pt-3 [&_button]:pr-4 [&_button]:pb-3 [&_button]:pl-4 [&_button]:cursor-pointer [&_button]:whitespace-nowrap [&_button]:text-xs",
    "[&_button_svg]:hidden [&_button[aria-pressed=true]]:bg-none [&_button[aria-pressed=true]]:bg-amber [&_button[aria-pressed=true]]:text-white max-[1050.001px]:pt-1 max-[1050.001px]:pr-1 max-[1050.001px]:pb-1 max-[1050.001px]:pl-1",
    "max-[1050.001px]:gap-px max-[1050.001px]:[&_button]:text-xs max-[1050.001px]:[&_button]:pt-2.5 max-[1050.001px]:[&_button]:pr-[11px] max-[1050.001px]:[&_button]:pb-2.5 max-[1050.001px]:[&_button]:pl-[11px] max-[760.001px]:[&_button]:pt-2.5 max-[760.001px]:[&_button]:pr-2",
    "max-[760.001px]:[&_button]:pb-2.5 max-[760.001px]:[&_button]:pl-2 max-[760.001px]:[&_button]:text-xs max-[460.001px]:w-[140px] max-[460.001px]:justify-center max-[460.001px]:pt-[3px] max-[460.001px]:pr-[3px] max-[460.001px]:pb-[3px]",
    "max-[460.001px]:pl-[3px] max-[460.001px]:gap-px max-[460.001px]:rounded-panel max-[460.001px]:[&_button]:w-[32px] max-[460.001px]:[&_button]:min-w-0 max-[460.001px]:[&_button]:pt-2 max-[460.001px]:[&_button]:pr-[3px] max-[460.001px]:[&_button]:pb-2",
    "max-[460.001px]:[&_button]:pl-[3px] max-[460.001px]:[&_button]:text-xs max-[460.001px]:[&_button]:leading-[1.5] max-[460.001px]:[&_button]:whitespace-normal max-[460.001px]:[&_button]:rounded-md max-[460.001px]:[&_button]:flex-none"
  ].join(" "),
  legend: [
    "workspace-legend relative justify-self-end text-xs rounded-input bg-none bg-workspace-legend-background-color border",
    "border-solid border-workspace-legend-border-color text-workspace-legend-color [&_span]:flex [&_span]:gap-[5px] [&_span]:items-center [&_span]:whitespace-nowrap [&_i]:w-[7px]",
    "[&_i]:h-[7px] [&_i]:bg-none [&_i]:bg-map-structure [&_i]:rounded-2xs [&_i[data-kind=free]]:bg-none [&_i[data-kind=free]]:bg-map-free [&_i[data-kind=unknown]]:bg-none [&_i[data-kind=unknown]]:bg-map-unknown",
    "[&_i[data-kind=scan]]:bg-none [&_i[data-kind=scan]]:bg-amber [&_i[data-kind=scan]]:rounded-full [&_button]:pt-0 [&_button]:pr-0 [&_button]:pb-0 [&_button]:pl-0 [&_button]:bg-none",
    "[&_button]:bg-transparent [&_button]:border-0 [&_button]:border-none [&_button]:border-current [&_button]:text-inherit [&_button]:cursor-pointer [&_.workspace-legend-toggle]:hidden",
    "max-[460.001px]:w-[65px] max-[460.001px]:[&_span]:gap-[3px] max-[460.001px]:[&_i]:w-[5px] max-[460.001px]:[&_i]:h-[5px]"
  ].join(" "),
  legendItems: [
    "workspace-legend-items flex flex-wrap gap-2 items-center justify-end pt-2 pr-2.5",
    "pb-2 pl-2.5 [&>button]:w-[14px] [&>button]:h-[14px] [&>button]:text-xs [&>button]:leading-[1] max-[1050.001px]:gap-[5px_7px] max-[1050.001px]:text-xs",
    "max-[760.001px]:text-xs max-[760.001px]:pt-1.5 max-[760.001px]:pr-1.5 max-[760.001px]:pb-1.5 max-[760.001px]:pl-1.5 max-[760.001px]:gap-1 max-[460.001px]:flex-col max-[460.001px]:items-start",
    "max-[460.001px]:justify-start max-[460.001px]:text-xs max-[460.001px]:pt-[5px] max-[460.001px]:pr-[5px] max-[460.001px]:pb-[5px] max-[460.001px]:pl-[5px] max-[460.001px]:gap-[3px] max-[460.001px]:[&>button]:[align-self:flex-end]",
    "max-[460.001px]:[&>button]:h-[8px] max-[460.001px]:[&>button]:text-xs"
  ].join(" "),
  toast: [
    "workspace-toast absolute left-[50%] bottom-[91px] [transform:translateX(-50%)] z-[27] pt-[11px] pr-[17px]",
    "pb-[11px] pl-[17px] bg-none bg-workspace-toast-background-color border border-solid border-workspace-toast-border-color rounded-lg",
    "text-workspace-toast-color text-xs max-w-[70%] pointer-events-none shadow-workspace-toast max-[760.001px]:bottom-19.5 max-[760.001px]:max-w-[90%]"
  ].join(" "),
} as const;

export interface ControlSurfaceProps {
  robotId: string;
  link: RobotLink;
  snapshot: RobotSnapshot;
  active?: boolean;
  statusOpen?: boolean;
  onStatusClose?: (() => void) | undefined;
  cameraOn?: boolean;
  onCameraChange?: (enabled: boolean) => void;
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

function navigationUnavailableMessage(
  reason: string | null | undefined,
): string {
  //Compatible: legacy snapshots have no concrete navigation diagnostic.
  if (reason == null)
    return "车端未报告已验证的导航能力，无法开始、继续或返航。";
  const explanations: Record<string, string> = {
    "map or localization generation changed":
      "地图或定位代次已变化，请等待重新定位。",
    "map or localization unavailable": "当前地图或定位尚未就绪。",
    "planner state unavailable": "路径规划器或运动控制器尚未就绪。",
    "heartbeat or control authority unavailable":
      "车辆心跳或控制权限尚未就绪。",
    "navigation unavailable": "车端导航尚未就绪。",
  };
  const sensorPrefix = "safety status unavailable (";
  const explanation =
    reason.startsWith(sensorPrefix) && reason.endsWith(")")
      ? `传感器安全检查尚未通过：${reason.slice(sensorPrefix.length, -1)}。`
      : (explanations[reason] ?? `车端导航尚未就绪：${reason}。`);
  return `无法导航：${explanation}`;
}

/** Drive and map controls for the selected vehicle and its current map reference. */
export function ControlSurface({
  robotId,
  link,
  snapshot,
  active = true,
  statusOpen = false,
  onStatusClose,
  cameraOn: controlledCamera,
  onCameraChange,
}: ControlSurfaceProps): ReactElement {
  const workspaceElement = useRef<HTMLDivElement>(null);
  const operationsElement = useRef<HTMLDivElement>(null);
  useLayoutEffect(() => {
    const element = operationsElement.current;
    if (!element || !active || element.clientWidth === 0) return;
    const update = () =>
      workspaceElement.current?.style.setProperty(
        "--drive-card-height",
        `${element.getBoundingClientRect().height}px`,
      );
    const observer = new ResizeObserver(update);
    observer.observe(element);
    update();
    return () => observer.disconnect();
  }, [active]);
  const [mapLoad, setMapLoad] = useState(0);
  const [navPick, setNavPick] = useState<"goal" | "hint" | "home" | null>(null);
  const [mode, setMode] = useState<WorkspaceMode>("manual");
  const [view, setView] = useState<PrimaryView>("map");
  const [performanceExpired, setPerformanceExpired] = useState(false);
  useEffect(() => {
    setPerformanceExpired(false);
    const timer = setTimeout(() => setPerformanceExpired(true), 3500);
    return () => clearTimeout(timer);
  }, [snapshot.performance?.sequence]);
  const [statusTab, setStatusTab] = useState<"motion" | "performance">(
    "motion",
  );
  const [localCamera, setLocalCamera] = useState(true);
  const cameraOn = controlledCamera ?? localCamera;
  const setCameraOn = onCameraChange ?? setLocalCamera;
  const {
    map: occupancy,
    gridId,
    waypoints,
    message,
    displayOnly,
    waypointWrites,
    navigationReady,
  } = useWorkspaceMap(link, snapshot, {
    enabled: true,
    catalog: mode === "tasks",
    refresh: mapLoad,
  });
  //Compatible: legacy interactions keep their original per-grid scope.
  const referenceVersion =
    snapshot.map_grid_id === undefined ? gridId : snapshot.map_generation;
  const refKey = JSON.stringify([snapshot.map_ref, referenceVersion]);
  const [selection, setSelection] = useState<{
    refKey: string;
    pose: Pose | null;
    headingReady: boolean;
  }>({ refKey, pose: null, headingReady: false });
  const [feedback, setFeedback] = useState<{
    refKey: string;
    note: string;
  } | null>(null);
  useLayoutEffect(() => {
    setSelection({ refKey, pose: null, headingReady: false });
    setNavPick(null);
  }, [refKey]);
  useEffect(() => {
    setNavPick(null);
  }, [mode, active]);
  const draft = selection.refKey === refKey ? selection.pose : null;
  const note = feedback?.refKey === refKey ? feedback.note : null;
  const workspace =
    occupancy !== undefined && gridId !== undefined && waypoints !== undefined
      ? {
          map: occupancy,
          grid_id: gridId,
          waypoints,
          waypointWrites: waypointWrites === true,
        }
      : undefined;

  function setDraft(pose: Pose | null, headingReady = true): void {
    setSelection({ refKey, pose, headingReady });
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
  const liveMapTelemetry =
    activeMap &&
    snapshot.connected &&
    !snapshot.stale &&
    snapshot.localized &&
    snapshot.map_generation !== undefined &&
    snapshot.map_generation === referenceVersion &&
    snapshot.map_ref !== null &&
    map.map_ref !== null &&
    snapshot.map_ref.map_id === map.map_ref.map_id &&
    snapshot.map_ref.reference_revision === map.map_ref.reference_revision;

  //Compatible: older vehicles advertise navigation eligibility only in the map manifest.
  const directNavigation =
    activeMap && (snapshot.navigation_ready ?? navigationReady) === true;
  const route = snapshot.route_progress;
  const routeActive =
    activeMap &&
    route !== undefined &&
    route.map_generation === referenceVersion &&
    (route.status === "starting" ||
      route.status === "running" ||
      route.status === "cancel_pending");
  const [navigationBusy, setNavigationBusy] = useState(false);
  const navigationScope = useRef(0);
  useLayoutEffect(() => {
    navigationScope.current += 1;
    setNavigationBusy(false);
    setFeedback(null);
    return () => {
      navigationScope.current += 1;
    };
  }, [robotId, link, refKey, snapshot.connected, snapshot.stale, mode]);

  async function runNavigation(
    action: "goal" | "return_home",
    pose?: Pose,
  ): Promise<void> {
    if (
      !directNavigation ||
      gridId === undefined ||
      !snapshot.localized ||
      snapshot.map_ref === null ||
      navigationBusy ||
      routeActive
    )
      return;
    const scope = navigationScope.current;
    const label = action === "goal" ? "目标" : "返航";
    setNavigationBusy(true);
    setFeedback({ refKey, note: `等待车端确认${label}…` });
    try {
      const response = await link.request<ResponsePayload>(
        `navigation.${action}`,
        {
          operation_id: newId(),
          body: { ...snapshot.map_ref, grid_id: gridId, ...pose },
        },
      );
      if (scope === navigationScope.current) {
        setFeedback({
          refKey,
          note: response.ok ? `车端已受理${label}。` : `车端未确认${label}。`,
        });
        if (response.ok) setNavPick(null);
      }
    } catch (error) {
      if (scope === navigationScope.current) {
        const reason =
          error instanceof CommandError
            ? describeCommandError(error.code)
            : "无法发送";
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
      setFeedback({
        refKey,
        note: `目标已设定（${describePose(goal.pose)}），未发送：车端未开放目标指令。`,
      });
    }
  }

  async function editMappingPose(
    kind: "map.calibrate" | "map.home",
    hint: MapGoal,
  ): Promise<void> {
    if (
      !mappingEdits ||
      hint.pose === null ||
      snapshot.map_ref === null ||
      !snapshot.connected ||
      snapshot.stale ||
      navigationBusy ||
      routeActive
    )
      return;
    const owner = navigationScope.current;
    const id = newId();
    const label = kind === "map.home" ? "Home 坐标更新" : "车辆局部匹配";
    setNavigationBusy(true);
    setFeedback({ refKey, note: `正在进行${label}…` });
    try {
      const catalog = await readMappingCatalog(link, robotId);
      if (owner !== navigationScope.current) return;
      const selected = catalog.selection.map_ref;
      if (
        selected?.map_id !== hint.map_ref.map_id ||
        selected.reference_revision !== hint.map_ref.reference_revision
      ) {
        throw new CommandError("conflict", "地图选择已变化");
      }
      let status: ResponsePayload["status"] = "running";
      try {
        const reply = await sendMapCommand(link, kind, id, {
          ...hint.map_ref,
          ...hint.pose,
          expected_generation: catalog.selection.generation,
          ...(kind === "map.home"
            ? { expected_home_revision: snapshot.home_revision }
            : {}),
        });
        status = reply.status;
      } catch (reason) {
        if (
          !(reason instanceof CommandError) ||
          (reason.code !== "timeout" && reason.code !== "operation_unknown")
        )
          throw reason;
      }
      while (
        owner === navigationScope.current &&
        !["succeeded", "failed", "needs_review"].includes(status ?? "")
      ) {
        await new Promise((resolve) => setTimeout(resolve, 1000));
        if (owner !== navigationScope.current) return;
        const reply = await link.request<ResponsePayload>("operation.query", {
          operation_id: id,
          body: {},
        });
        if (!reply.ok || reply.operation_id !== id || reply.status == null)
          throw new CommandError("operation_unknown", "匹配结果未确认");
        status = reply.status;
      }
      if (owner === navigationScope.current) {
        setFeedback({
          refKey,
          note:
            status === "succeeded"
              ? `${label}已完成。`
              : `${label}未完成，请检查车辆状态后重试。`,
        });
        if (status === "succeeded") setNavPick(null);
        setMapLoad((value) => value + 1);
      }
    } catch (reason) {
      if (owner === navigationScope.current)
        setFeedback({
          refKey,
          note: `${label}未完成：${reason instanceof CommandError ? describeCommandError(reason.code) : "连接未确认"}。`,
        });
    } finally {
      if (owner === navigationScope.current) setNavigationBusy(false);
    }
  }

  function onHint(hint: PoseHint): void {
    if (mappingEdits && hint.pose !== null) {
      void editMappingPose("map.calibrate", {
        map_ref: hint.map_ref,
        pose: hint.pose,
      });
      return;
    }
    //Compatible: original mapping protocol has no manual calibration operation.
    setFeedback({
      refKey,
      note: `定位提示已设定（${describePose(hint.pose)}），未发送：车端未开放定位提示指令。`,
    });
    setNavPick(null);
  }

  const [toolbarTarget, setToolbarTarget] = useState<HTMLDivElement | null>(
    null,
  );
  const icons = {
    manual: Gamepad2,
    navigation: Navigation,
    tasks: ListOrdered,
    mapping: MapIcon,
  };
  return (
    <TaskList
      snapshot={snapshot}
      link={link}
      onPickStart={() => { setView("map"); onStatusClose?.(); }}
      workspace={workspace}
      map={map}
      unavailable={message}
      mapCanvasProps={
        mode === "tasks"
          ? {
              mini: cameraOn && view === "video",
              toolbarTarget:
                cameraOn && view === "video" ? null : toolbarTarget,
            }
          : {
              toolbarTarget:
                cameraOn && view === "video" ? null : toolbarTarget,
              mini: cameraOn && view === "video",
              map,
              robot: liveMapTelemetry ? (snapshot.robot_pose ?? null) : null,
              scanPoints: liveMapTelemetry ? (snapshot.scan_points ?? []) : [],
              pickEnabled:
                mode === "navigation" && navPick !== null && activeMap,
              draft: mode === "navigation" ? draft : null,
              onPick: (pose) => setDraft(pose, false),
              onPickHeading: (pose) => setDraft(pose),
              taskRoute: [],
              waypointMarkers: [],
              onAddWaypoint: () => {},
            }
      }
      renderWorkspace={(mapView, taskPanel, pickingBanner) => (
        <div
          ref={workspaceElement}
          className={ui.workspace}
          data-task-picking={mode === "tasks" && pickingBanner != null}
          data-mode={mode}
          data-nav-picking={mode === "navigation" && navPick !== null}
          data-primary={cameraOn ? view : "map"}
        >
          <div className={ui.previewHead}>
            <span className={ui.viewName}>
              {view === "video" && cameraOn
                ? "前视摄像头"
                : occupancy ? "当前地图" : "等待地图"}
            </span>
            {view !== "video" && occupancy && (
              <span className={ui.badge}>
                {snapshot.stale ? "非实时" : (snapshot.mapping_protocol_version ?? 0) >= 1 ? "自动保存" : "实时地图"}
              </span>
            )}
            <div className={ui.grow} />
            {occupancy === undefined && message && (
              <p className={ui.mapMessage} role="status">
                {message}
              </p>
            )}
            <Switch
              label="摄像头"
              checked={cameraOn}
              onChange={(next) => {
                setCameraOn(next);
                if (!next) setView("map");
              }}
            />
          </div>
          <div className={ui.mapViewport}>
            {mapView}
            {cameraOn && view === "video" && (
              <div className={ui.swap}>
                <IconButton label="交换主次画面" onClick={() => setView("map")}>
                  <ArrowLeftRight />
                </IconButton>
              </div>
            )}
          </div>
          {cameraOn && (
            <div className={ui.videoViewport}>
              <VideoPanel
                link={link}
                live={active && snapshot.connected && !snapshot.stale}
                compact
              />
              {view === "map" && (
                <div className={ui.swap}>
                  <IconButton
                    label="交换主次画面"
                    onClick={() => setView("video")}
                  >
                    <ArrowLeftRight />
                  </IconButton>
                </div>
              )}
            </div>
          )}
          <div
            className={ui.status}
            hidden={
              !statusOpen ||
              navPick !== null ||
              (mode === "tasks" && pickingBanner != null)
            }
          >
            <StatusOverlay
              onClose={onStatusClose}
              snapshot={snapshot}
              port={link.vehicle}
              tab={statusTab}
              onTabChange={setStatusTab}
              performanceExpired={performanceExpired}
              visible={active && statusOpen}
            />
          </div>
          {mode === "tasks" && pickingBanner}
          {mode === "navigation" && navPick !== null && (
            <PosePicker
              key={`pose:${refKey}:${navPick}`}
              snapshot={snapshot}
              map={pickMap}
              draft={
                snapshot.map_grid_id === undefined && !activeMap ? null : draft
              }
              onDraftChange={setDraft}
              orientationReady={selection.headingReady}
              pending={navigationBusy || routeActive}
              onRefresh={() => setMapLoad((n) => n + 1)}
              onGoal={onGoal}
              onHint={onHint}
              selectionMode={navPick}
              onCancel={() => {
                setNavPick(null);
                setDraft(null);
              }}
              {...(mappingEdits
                ? {
                    onHome: (goal: MapGoal) =>
                      void editMappingPose("map.home", goal),
                  }
                : {})}
            />
          )}
          <div
            ref={operationsElement}
            className={ui.operations}
            data-manual={mode === "manual"}
          >
            <div hidden={mode !== "manual"}>
              <DriveView
                robotId={robotId}
                link={link}
                snapshot={snapshot}
                active={active && mode === "manual"}
                compact
              />
            </div>
            <div hidden={mode !== "tasks"}>{taskPanel}</div>
            {mode === "navigation" && (
              <>
                <HomeControls
                  key={`home:${refKey}`}
                  snapshot={snapshot}
                  link={link}
                  homeMapAvailable={activeMap && waypointWrites === true}
                  goalAvailable={
                    activeMap &&
                    snapshot.localized &&
                    !routeActive &&
                    !navigationBusy &&
                    (snapshot.navigation_ready ?? navigationReady) !== false
                  }
                  onPickGoal={() => {
                    setView("map");
                    setDraft(null);
                    setNavPick("goal");
                    onStatusClose?.();
                  }}
                  onCalibrate={() => {
                    setView("map");
                    setDraft(null);
                    setNavPick("hint");
                    onStatusClose?.();
                  }}
                  {...(mappingEdits
                    ? {
                        onEditHome: () => {
                          setView("map");
                          setDraft(null);
                          setNavPick("home");
                        onStatusClose?.();
                        },
                      }
                    : {})}
                  onRefresh={() => setMapLoad((n) => n + 1)}
                  {...(directNavigation
                    ? {
                        directNavigation: {
                          onReturnHome: () => void runNavigation("return_home"),
                          pending: navigationBusy || routeActive,
                        },
                      }
                    : {
                        navigationUnavailable: navigationUnavailableMessage(
                          snapshot.navigation_block_reason,
                        ),
                      })}
                />
                {note && <p role="status">{note}</p>}
              </>
            )}
            {mode === "mapping" && (
              <MapManager
                compact
                overlayTarget={workspaceElement.current}
                robotId={robotId}
                snapshot={snapshot}
                link={link}
              />
            )}
          </div>
          <section className={ui.bottomBar} aria-label="视图底部工具">
            <div className={ui.mapTools} ref={setToolbarTarget} />
            <nav className={ui.dock} aria-label="车辆场景">
              {MODES.map(([value, label]) => {
                const Icon = icons[value];
                return (
                  <button
                    type="button"
                    key={value}
                    aria-label={label}
                    aria-pressed={mode === value}
                    onClick={() => setMode(value)}
                  >
                    <Icon size={18} />
                    <span>{value === "navigation" ? "导航定位" : label}</span>
                  </button>
                );
              })}
            </nav>
            <div className={ui.legend} aria-label="地图图例">
              <div className={ui.legendItems}>
                <span>
                  <i data-kind="stable" />
                  稳定结构
                </span>
                <span>
                  <i data-kind="free" />
                  已确认空闲
                </span>
                <span>
                  <i data-kind="unknown" />
                  未知
                </span>
                <span>
                  <i data-kind="scan" />
                  实时点
                </span>
                <button
                  type="button"
                  aria-label="更多图层"
                  title="更多图层"
                  onClick={() =>
                    setFeedback({
                      refKey,
                      note: "暂未实现：独立候选层 / 空闲证据层 / 线段拟合",
                    })
                  }
                >
                  ···
                </button>
              </div>
            </div>
          </section>
          {note?.startsWith("暂未实现") && (
            <p className={ui.toast} role="status">
              {note}
            </p>
          )}
        </div>
      )}
    />
  );
}
