import { useCurrentScan } from "../map/useCurrentScan";
import { newId } from "../../ids";
import { isTaskName, normalizeTaskName } from "../../api/vehicleState";
import {
  useEffect,
  useRef,
  useState,
  type ReactElement,
  type DragEvent,
} from "react";

import {
  CommandError,
  describeCommandError,
  type RobotLink,
} from "../../api/client";
import type {
  ResponsePayload,
  RobotSnapshot,
  MapPoint,
  TaskSummary,
} from "../../api/protocol";
import {
  getTaskDetails,
  type TaskDetails,
  type WorkspaceWaypoint,
} from "../../api/workspace";
import {
  MapCanvas,
  type OccupancySnapshot,
  type Pose,
  type MapCanvasProps,
} from "../map/MapCanvas";
import { listCurrentWaypoints } from "../map/useWorkspaceMap";
import { NEW_TASK, ScheduleEditor } from "./ScheduleEditor";
import {
  draftSchedule,
  newTaskDraft,
  scheduleDraft,
  scheduleSummary,
  type TaskDraft,
} from "./schedule";
import { X, Plus, GripVertical, ArrowUp, ArrowDown } from "lucide-react";
import { TaskExecutionControls } from "../devices/VehicleStateView";
import { Button } from "../../components/ui/Button";
import { Switch } from "../../components/ui/Switch";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  map: [
    "tasks-map min-w-0 min-h-0 h-full overflow-hidden [&>.tasks-note]:absolute [&>.tasks-note]:left-4 [&>.tasks-note]:top-25",
    "[&>.tasks-note]:pt-[5px] [&>.tasks-note]:pr-2 [&>.tasks-note]:pb-[5px] [&>.tasks-note]:pl-2 [&>.tasks-note]:bg-none [&>.tasks-note]:bg-tasks-map-background-color [&>.tasks-note]:rounded-sm [&>.tasks-note]:text-xs",
    "[&>.tasks-note]:pointer-events-none"
  ].join(" "),
  warning: [
    "tasks-warning mt-2 mr-0 mb-2 ml-0 pt-2 pr-2 pb-2",
    "pl-2 bg-none bg-amber-wash rounded-md text-xs"
  ].join(" "),
  note: [
    "tasks-note text-muted text-xs leading-[1.8] mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  picking: [
    "tasks-picking absolute top-[65px] left-[50%] [transform:translateX(-50%)] z-[14] flex items-center",
    "gap-2.5 w-[max-content] max-w-[90%] pt-[11px] pr-[15px] pb-[11px] pl-[15px] border",
    "border-solid border-map-pose-banner-border-color rounded-lg bg-none bg-map-pose-banner-background-color text-xs shadow-map-pose-banner [&_button]:text-xs",
    "[&_button]:pt-1.5 [&_button]:pr-[9px] [&_button]:pb-1.5 [&_button]:pl-[9px] max-[760.001px]:top-15 max-[760.001px]:text-xs max-[460.001px]:flex-wrap max-[460.001px]:justify-center",
    "max-[460.001px]:gap-1.5 max-[460.001px]:w-[max-content] max-[460.001px]:[&>span]:[flex-basis:100%] max-[460.001px]:[&>span]:text-center"
  ].join(" "),
  panel: [
    "tasks-panel min-w-0 [&[data-task-stage=editor]_.tasks-header_h3]:text-base [&[data-task-stage=editor]_.tasks-header_h3]:font-bold [&_.tasks-close]:border-0 [&_.tasks-close]:border-none [&_.tasks-close]:border-current [&_.tasks-close]:pt-1 [&_.tasks-close]:pr-[7px] [&_.tasks-close]:pb-1",
    "[&_.tasks-close]:pl-[7px] [&_.tasks-close]:bg-none [&_.tasks-close]:bg-transparent [&_.tasks-close]:text-quiet [&_.tasks-close]:text-xs [&[hidden]]:hidden"
  ].join(" "),
  header: [
    "tasks-header flex items-center justify-between gap-2.5 [&_h3]:text-base [&_h3]:font-medium [&_h3]:mt-0",
    "[&_h3]:mr-0 [&_h3]:mb-[5px] [&_h3]:ml-0 max-[460.001px]:[&_h3]:text-sm"
  ].join(" "),
  english: [
    "tasks-english font-normal text-xs leading-[normal] font-mono tracking-[1px] text-english"
  ].join(" "),
  close: [
    "tasks-close"
  ].join(" "),
  body: [
    "tasks-body min-w-0"
  ].join(" "),
  description: [
    "tasks-description text-muted leading-[1.8] mt-0 mr-0 mb-0 ml-0 text-xs",
    "max-[460.001px]:text-xs"
  ].join(" "),
  taskActions: [
    "tasks-task-actions flex gap-[7px] mt-3 [&>button]:flex-1 [&>button]:text-xs [&>button]:gap-1 [&>button]:px-[9px] [&+.tasks-task-item]:mt-3"
  ].join(" "),
  taskItem: [
    "tasks-task-item border-t [border-top-style:solid] border-t-control-hover pt-3.5 pr-0 pb-[9px] pl-0",
    "mt-0"
  ].join(" "),
  taskIdentity: [
    "tasks-task-identity flex items-center justify-between gap-2.5 [&_strong]:min-w-0 [&_strong]:[overflow-wrap:anywhere] [&_strong]:text-xs",
    "[&_strong]:font-medium"
  ].join(" "),
  taskStatus: [
    "tasks-task-status text-xs text-map-stale-banner-color block mt-1.5"
  ].join(" "),
  taskButtons: [
    "tasks-task-buttons flex gap-[5px] flex-wrap mt-[9px] [&_button]:text-xs [&_button]:pt-[7px] [&_button]:pr-2.5",
    "[&_button]:pb-[7px] [&_button]:pl-2.5 max-[460.001px]:gap-1 max-[460.001px]:[&_button]:pt-1.5 max-[460.001px]:[&_button]:pr-2 max-[460.001px]:[&_button]:pb-1.5 max-[460.001px]:[&_button]:pl-2"
  ].join(" "),
  taskDetails: [
    "tasks-task-details text-xs text-muted mt-[7px] [&_summary]:cursor-pointer [&_code]:block [&_code]:[overflow-wrap:anywhere] [&_code]:mt-[7px]",
    "[&_code]:mr-0 [&_code]:mb-[7px] [&_code]:ml-0 [&_button]:text-xs [&_button]:pt-[5px] [&_button]:pr-2 [&_button]:pb-[5px] [&_button]:pl-2"
  ].join(" "),
  queueInfo: [
    "tasks-queue-info text-xs bg-none bg-control rounded-md pt-[9px] pr-[9px] pb-[9px]",
    "pl-[9px] mt-3 text-tasks-queue-info-color leading-[1.8]"
  ].join(" "),
  routePreview: [
    "tasks-route-preview [&_button]:pt-[7px] [&_button]:pr-[9px] [&_button]:pb-[7px] [&_button]:pl-[9px] [&_button]:text-xs [&_button]:text-inherit [&_button]:bg-none",
    "[&_button]:bg-control [&_button]:border [&_button]:border-solid [&_button]:border-devices-item-border-color [&_button]:rounded-control [&_button]:cursor-pointer pt-2.5 pr-2.5",
    "pb-2.5 pl-2.5 mt-3 mr-0 mb-3 ml-0 border border-solid",
    "border-line rounded-md [&_h4]:text-xs [&_h4]:mt-0 [&_h4]:mr-0 [&_h4]:mb-2 [&_h4]:ml-0"
  ].join(" "),
  fields: [
    "tasks-fields border-0 border-none border-current mt-0 mr-0 mb-0 ml-0",
    "pt-0 pr-0 pb-0 pl-0 min-w-0 [&[hidden]]:hidden"
  ].join(" "),
  enableRow: [
    "tasks-enable-row flex items-center justify-between gap-2 text-muted text-xs mt-3.5"
  ].join(" "),
  pointHead: [
    "tasks-point-head flex items-center justify-between gap-2 text-muted text-xs mt-4.5"
  ].join(" "),
  route: [
    "tasks-route list-none pt-0 pr-0 pb-0 pl-0 mt-3 mr-0",
    "mb-3 ml-0 [&_li]:bg-none [&_li]:bg-control [&_li]:border [&_li]:border-solid [&_li]:border-logs-caption-border-bottom-color [&_li]:rounded-md",
    "[&_li]:mb-[5px] [&_li]:pt-[7px] [&_li]:pr-2 [&_li]:pb-[7px] [&_li]:pl-2 [&_li]:text-xs [&_li]:cursor-grab [&_.tasks-task-row]:flex-nowrap",
    "[&_button]:grid [&_button]:place-items-center [&_button]:flex-none [&_button]:pt-[3px] [&_button]:pr-[5px] [&_button]:pb-[3px] [&_button]:pl-[5px] [&_button]:text-xs",
    "[&_button]:bg-none [&_button]:bg-control [&_button]:border [&_button]:border-solid [&_button]:border-devices-item-border-color [&_button]:rounded-control [&_button]:cursor-pointer [&_button:disabled]:opacity-[0.36]",
    "[&_button:disabled]:cursor-not-allowed max-[460.001px]:[&_.tasks-task-row]:gap-[5px]"
  ].join(" "),
  taskRow: [
    "tasks-task-row [&_label]:grid [&_label]:gap-[7px] [&_label]:text-field [&_label]:text-xs [&_input]:min-w-0 [&_input]:w-full [&_input]:pt-2.5",
    "[&_input]:pr-2.5 [&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:text-input-ink [&_input]:bg-none [&_input]:bg-input [&_input]:border",
    "[&_input]:border-solid [&_input]:border-login-input-border-color [&_input]:rounded-input [&_select]:min-w-0 [&_select]:w-full [&_select]:pt-2.5 [&_select]:pr-2.5 [&_select]:pb-2.5",
    "[&_select]:pl-2.5 [&_select]:text-input-ink [&_select]:bg-none [&_select]:bg-input [&_select]:border [&_select]:border-solid [&_select]:border-login-input-border-color",
    "[&_select]:rounded-input flex items-center gap-[7px] flex-wrap"
  ].join(" "),
  grip: [
    "tasks-grip text-tasks-grip-color flex-none"
  ].join(" "),
  pointNumber: [
    "tasks-point-number grid place-items-center w-[21px] h-[21px] rounded-full bg-none bg-amber",
    "text-white text-xs flex-none"
  ].join(" "),
  pointLabel: [
    "tasks-point-label flex-1 min-w-0 whitespace-nowrap [&_small]:text-muted [&_small]:text-xs [&_small]:ml-[5px] max-[460.001px]:text-xs",
    "max-[460.001px]:[&_small]:text-xs max-[460.001px]:[&_small]:ml-[3px]"
  ].join(" "),
  waypointDetails: [
    "tasks-waypoint-details [&_summary]:cursor-pointer mt-2.5 text-xs text-muted [&_.tasks-task-row]:mt-2 [&_.tasks-task-row]:mr-0 [&_.tasks-task-row]:mb-2",
    "[&_.tasks-task-row]:ml-0 [&_button]:pt-[7px] [&_button]:pr-[9px] [&_button]:pb-[7px] [&_button]:pl-[9px] [&_button]:text-xs [&_button]:text-inherit [&_button]:bg-none",
    "[&_button]:bg-control [&_button]:border [&_button]:border-solid [&_button]:border-devices-item-border-color [&_button]:rounded-control [&_button]:cursor-pointer"
  ].join(" "),
  footer: [
    "tasks-footer [&_p]:text-xs [&_p]:[overflow-wrap:anywhere] [&_p]:mt-2.5 [&_p]:mr-0 [&_p]:mb-2.5 [&_p]:ml-0"
  ].join(" "),
  workspace: [
    "tasks-workspace grid grid-cols-[minmax(0,_3fr)_minmax(320px,_2fr)] gap-1.5 [&_.tasks-panel]:pt-[19px] [&_.tasks-panel]:pr-[19px] [&_.tasks-panel]:pb-[19px] [&_.tasks-panel]:pl-[19px]",
    "max-[680.001px]:grid-cols-[1fr]"
  ].join(" "),
} as const;

export interface TaskWorkspaceData {
  grid_id: string;
  map: OccupancySnapshot;
  waypoints: readonly WorkspaceWaypoint[];
  waypointWrites?: boolean;
}

interface TaskListProps {
  snapshot: RobotSnapshot;
  link: RobotLink;
  workspace?: TaskWorkspaceData | undefined;
  unavailable?: string | undefined;
  onPickStart?: (() => void) | undefined;
  map?: OccupancySnapshot | undefined;
  mapCanvasProps?: Partial<MapCanvasProps> | undefined;
  renderWorkspace?:
    | ((
        mapView: ReactElement,
        panel: ReactElement,
        picking?: ReactElement | null,
      ) => ReactElement)
    | undefined;
}

export function TaskList({
  snapshot,
  link,
  workspace,
  unavailable,
  map,
  mapCanvasProps,
  renderWorkspace, onPickStart,
}: TaskListProps): ReactElement {
  const currentScan = useCurrentScan(snapshot);
  const ref = snapshot.map_ref;
  //Compatible: older snapshots omit map_generation; still require matching map_ref.
  const matches =
    ref !== null &&
    workspace?.map.map_ref?.map_id === ref.map_id &&
    workspace.map.map_ref.reference_revision === ref.reference_revision &&
    ((snapshot.map_grid_id ?? snapshot.map_generation) === undefined ||
      workspace.grid_id === (snapshot.map_grid_id ?? snapshot.map_generation));
  const live = snapshot.connected && !snapshot.stale;
  const scope = JSON.stringify([
    snapshot.robot_id,
    ref?.map_id,
    ref?.reference_revision,
    (snapshot.map_grid_id === undefined
      ? (snapshot.map_generation ?? workspace?.grid_id)
      : snapshot.map_generation) ?? null,
    live,
  ]);
  return (
    <TaskEditor
      key={scope}
      currentScan={currentScan}
      snapshot={snapshot}
      link={link}
      workspace={live && matches ? workspace : undefined}
      unavailable={unavailable}
      map={map}
      mapCanvasProps={mapCanvasProps}
      renderWorkspace={renderWorkspace}
      onPickStart={onPickStart}
    />
  );
}

function TaskEditor({
  snapshot,
  currentScan,
  link,
  workspace,
  unavailable,
  map,
  mapCanvasProps,
  renderWorkspace, onPickStart,
}: TaskListProps & { currentScan: readonly MapPoint[] }): ReactElement {
  //Compatible: standalone task editor callers retain their combined list/editor layout.
  const [stage, setStage] = useState<"list" | "editor">(
    renderWorkspace ? "list" : "editor",
  );
  const [details, setDetails] = useState<Record<string, TaskDetails>>({});
  const detailReads = useRef(new Set<string>());
  useEffect(() => {
    if (
      !renderWorkspace ||
      !workspace ||
      !snapshot.connected ||
      snapshot.stale ||
      !snapshot.map_ref
    )
      return;
    for (const task of snapshot.tasks ?? []) {
      const key = `${task.task_id}:${task.revision}`;
      if (detailReads.current.has(key)) continue;
      detailReads.current.add(key);
      void getTaskDetails(
        link,
        snapshot.robot_id,
        snapshot.map_ref,
        task.task_id,
      )
        .then((value) => {
          if (
            alive.current &&
            value.revision === task.revision &&
            value.grid_id === workspace.grid_id
          )
            setDetails((previous) => ({ ...previous, [task.task_id]: value }));
        })
        .catch(() => {
          /* A list may still be used without a readable definition. */
        });
    }
  }, [snapshot.tasks, workspace, link, renderWorkspace]);
  const [draft, setDraft] = useState<TaskDraft>(newTaskDraft);
  const [showAdvanced, setShowAdvanced] = useState(false);
  const [planEnabled, setPlanEnabled] = useState(true);
  const [editing, setEditing] = useState<TaskDetails | null>(null);
  const [preview, setPreview] = useState<TaskDetails | null>(null);
  const [routeBlocked, setRouteBlocked] = useState(false);
  const [waypointIds, setWaypointIds] = useState<string[]>([]);
  const [pickingRoute, setPickingRoute] = useState(false);
  const [draftPoints, setDraftPoints] = useState<
    Array<WorkspaceWaypoint & { operationId: string }>
  >([]);
  const confirmedCount = useRef(0);
  const alive = useRef(true);
  useEffect(() => {
    alive.current = true;
    return () => {
      alive.current = false;
    };
  }, []);
  const [selected, setSelected] = useState("");
  const [highlight, setHighlight] = useState<string | null>(null);
  const [pending, setPending] = useState(false);
  const [message, setMessage] = useState<string | null>(null);
  const [conflict, setConflict] = useState(false);
  const [creatingWaypoint, setCreatingWaypoint] = useState(false);
  const [waypointDraft, setWaypointDraft] = useState<Pose | null>(null);
  const [waypointName, setWaypointName] = useState("");
  const [catalog, setCatalog] = useState<WorkspaceWaypoint[] | null>(null);
  const saveWaypointOperation = useRef<AbortController | null>(null);
  const waypointAttempt = useRef<{
    body: string;
    operationId: string;
  } | null>(null);
  useEffect(() => () => saveWaypointOperation.current?.abort(), [link]);

  const namesSupported =
    snapshot.vehicle_ui?.capabilities.includes("task.names.v1") === true;
  function taskLabel(task: { task_id: string; name?: string }): string {
    //Compatible: an old vehicle always displays its stable ID.
    return namesSupported ? (task.name ?? task.task_id) : task.task_id;
  }
  const live = snapshot.connected && !snapshot.stale;
  const progress =
    live && snapshot.route_progress?.map_generation === snapshot.map_generation
      ? snapshot.route_progress
      : undefined;
  const progressTask = (snapshot.tasks ?? []).find(
    (task) => task.task_id === progress?.task_id,
  );
  const progressLabel = taskLabel(
    progressTask ?? { task_id: progress?.task_id ?? "导航" },
  );
  const persisted = catalog ?? workspace?.waypoints ?? [];
  const available = [...persisted, ...draftPoints];
  const waypoints = new Map(
    available.map((waypoint) => [waypoint.waypoint_id, waypoint]),
  );
  const resolved = waypointIds.map((id) => waypoints.get(id));
  const route = resolved.filter((point) => point !== undefined);
  const missingWaypoint = route.length !== waypointIds.length;
  const previewPoints = preview?.waypoint_ids.map((id) => waypoints.get(id));
  const previewMissing =
    previewPoints?.some((point) => point === undefined) === true;
  const routeMissing = preview === null ? missingWaypoint : previewMissing;
  const mapRoute = routeBlocked || routeMissing ? [] : (previewPoints ?? route);
  const schedule = draftSchedule(draft);
  const normalizedName = normalizeTaskName(draft.name);
  const canSubmit =
    live &&
    workspace !== undefined &&
    schedule !== null &&
    draft.task_id.trim() !== "" &&
    (namesSupported ? isTaskName(normalizedName) : editing !== null) &&
    route.length > 0 &&
    !missingWaypoint &&
    !pending &&
    !conflict &&
    !creatingWaypoint &&
    !pickingRoute &&
    preview === null;

  function clearWaypointDraft(): void {
    setCreatingWaypoint(false);
    setWaypointDraft(null);
    setWaypointName("");
    waypointAttempt.current = null;
  }

  async function saveWaypoint(): Promise<void> {
    const ref = snapshot.map_ref;
    const id = waypointName.trim();
    if (
      pending ||
      !live ||
      !workspace?.waypointWrites ||
      ref === null ||
      waypointDraft === null ||
      !id ||
      waypoints.has(id)
    )
      return;
    const operation = new AbortController();
    saveWaypointOperation.current = operation;
    setPending(true);
    setMessage("等待车端确认保存航点…");
    let confirmed = false;
    try {
      const body = {
        ...ref,
        ...waypointDraft,
        grid_id: workspace.grid_id,
        waypoint_id: id,
      };
      const signature = JSON.stringify(body);
      if (waypointAttempt.current?.body !== signature) {
        waypointAttempt.current = {
          body: signature,
          operationId: newId(),
        };
      }
      const response = await link.request<ResponsePayload>("waypoint.create", {
        operation_id: waypointAttempt.current.operationId,
        body,
      });
      if (operation.signal.aborted) return;
      if (!response.ok) {
        setMessage("车端未确认保存航点。");
        return;
      }
      confirmed = true;
      clearWaypointDraft();
      const current = await listCurrentWaypoints(
        link,
        snapshot.robot_id,
        ref,
        workspace.grid_id,
        operation.signal,
      );
      setCatalog(current);
      if (current.some((waypoint) => waypoint.waypoint_id === id)) {
        setSelected(id);
        setHighlight(id);
        setMessage("航点已保存，可加入路线。");
      } else {
        setMessage("车端已保存航点，但当前目录暂不可选用该航点。");
      }
    } catch (error) {
      if (operation.signal.aborted) return;
      setMessage(
        confirmed
          ? "航点已保存，目录刷新失败。请重新读取地图。"
          : `航点保存未确认：${error instanceof CommandError ? describeCommandError(error.code) : "无法发送"}。`,
      );
    } finally {
      if (!operation.signal.aborted) setPending(false);
      if (saveWaypointOperation.current === operation)
        saveWaypointOperation.current = null;
    }
  }

  function addWaypoint(id: string): void {
    if (pending || !waypoints.has(id)) return;
    setWaypointIds((previous) => [...previous, id]);
    setHighlight(id);
  }

  function moveWaypoint(index: number, direction: -1 | 1): void {
    setWaypointIds((previous) => {
      const next = [...previous];
      const neighbor = index + direction;
      [next[index], next[neighbor]] = [next[neighbor]!, next[index]!];
      return next;
    });
  }

  function newTask(): void {
    setShowAdvanced(false);
    setStage(renderWorkspace ? "list" : "editor");
    clearWaypointDraft();
    setPickingRoute(false);
    setDraftPoints([]);
    confirmedCount.current = 0;
    setEditing(null);
    setPlanEnabled(true);
    setPreview(null);
    setRouteBlocked(false);
    setDraft(newTaskDraft());
    setWaypointIds([]);
    setHighlight(null);
    setMessage(null);
    setConflict(false);
  }

  async function openTask(
    taskId: string,
    mode: "edit" | "view" = "edit",
  ): Promise<void> {
    if (
      !live ||
      pending ||
      workspace === undefined ||
      snapshot.map_ref === null
    )
      return;
    setPending(true);
    setMessage("正在读取车端任务…");
    setRouteBlocked(true);
    setPreview(null);
    clearWaypointDraft();
    try {
      const task = await getTaskDetails(
        link,
        snapshot.robot_id,
        snapshot.map_ref,
        taskId,
      );
      if (task.grid_id !== workspace.grid_id) {
        setMessage("地图已变化，请重新读取地图后编辑。");
        setConflict(true);
        return;
      }
      setDetails(previous => ({ ...previous, [task.task_id]: task }));
      setRouteBlocked(false);
      setHighlight(null);
      setMessage(null);
      if (mode === "view") {
        setPreview(task);
        return;
      }
      setStage("editor");
      if (renderWorkspace && workspace.waypointWrites) { onPickStart?.(); setPickingRoute(true); }
      setEditing(task);
      setPlanEnabled(task.enabled);
      setDraft({
        ...NEW_TASK,
        task_id: task.task_id,
        name: task.name ?? task.task_id,
        timezone: task.timezone,
        ...scheduleDraft(task.schedule),
      });
      //Compatible: a task stored before route editing still contains a one-item waypoint array.
      setWaypointIds([...task.waypoint_ids]);
      setHighlight(null);
      setConflict(false);
      setMessage(null);
    } catch (error) {
      setMessage(
        error instanceof CommandError && error.code === "conflict"
          ? "地图或车端版本已变化，请重新读取当前地图与任务。"
          : `无法读取任务：${error instanceof CommandError ? describeCommandError(error.code) : "响应无效"}。`,
      );
    } finally {
      setPending(false);
    }
  }

  function reorderWaypoint(
    event: DragEvent<HTMLLIElement>,
    index: number,
  ): void {
    event.preventDefault();
    const raw = event.dataTransfer.getData("text/plain");
    const from = Number(raw);
    if (
      pending ||
      workspace === undefined ||
      preview !== null ||
      raw === "" ||
      !Number.isInteger(from) ||
      from < 0 ||
      from >= waypointIds.length
    )
      return;
    const next = [...waypointIds];
    const point = next.splice(from, 1)[0]!;
    next.splice(index, 0, point);
    setWaypointIds(next);
  }

  async function queryDraftPoint(
    operationId: string,
  ): Promise<ResponsePayload | null> {
    try {
      const reply = await link.request<ResponsePayload>("operation.query", {
        operation_id: operationId,
        body: {},
      });
      return reply.ok && reply.operation_id === operationId ? reply : null;
    } catch {
      return null;
    }
  }

  async function submit(): Promise<void> {
    if (
      !canSubmit ||
      schedule === null ||
      snapshot.map_ref === null ||
      workspace === undefined
    )
      return;
    setPending(true);
    setMessage("等待车端确认保存…");
    const body = {
      ...snapshot.map_ref,
      grid_id: workspace.grid_id,
      task_id: draft.task_id,
      ...(namesSupported ? { name: normalizedName } : {}),
      created_at: editing?.created_at ?? new Date().toISOString(),
      timezone: draft.timezone.trim() || NEW_TASK.timezone,
      schedule,
      waypoint_ids: waypointIds,
      enabled: planEnabled,
    };
    let waypointRecovery: "failed" | "unknown" | null = null;
    try {
      for (const id of waypointIds) {
        const point = draftPoints.find(
          (candidate) => candidate.waypoint_id === id,
        );
        if (!point) continue;
        if (!workspace.waypointWrites) throw new Error("航点写入不可用");
        try {
          const reply = await link.request<ResponsePayload>("waypoint.create", {
            operation_id: point.operationId,
            body: {
              ...snapshot.map_ref,
              grid_id: workspace.grid_id,
              waypoint_id: id,
              x: point.x,
              y: point.y,
              yaw: point.yaw,
            },
          });
          if (!reply.ok) throw new Error("航点保存未确认");
        } catch (error) {
          if (!alive.current) return;
          const outcome = await queryDraftPoint(point.operationId);
          if (!alive.current) return;
          if (outcome?.status !== "succeeded") {
            waypointRecovery =
              outcome?.status === "failed" ? "failed" : "unknown";
            // Only the matching journal's terminal FAILED proves no commit occurred.
            // A new ID is used on the next explicit submit, never an automatic retry.
            if (waypointRecovery === "failed") {
              setDraftPoints((previous) =>
                previous.map((candidate) =>
                  candidate.operationId === point.operationId
                    ? { ...candidate, operationId: newId() }
                    : candidate,
                ),
              );
            }
            throw error;
          }
        }
        if (!alive.current) return;
        confirmedCount.current++;
        const confirmed: WorkspaceWaypoint = {
          waypoint_id: id,
          x: point.x,
          y: point.y,
          yaw: point.yaw,
          kind: "waypoint",
          status: "active",
        };
        setCatalog((previous) => [
          ...(previous ?? workspace.waypoints),
          confirmed,
        ]);
        setDraftPoints((previous) =>
          previous.filter((candidate) => candidate.waypoint_id !== id),
        );
      }
      if (!alive.current) return;
      const response = await link.request<ResponsePayload>(
        editing === null ? "task.create" : "task.update",
        {
          operation_id: newId(),
          ...(editing === null ? {} : { expected_revision: editing.revision }),
          body,
        },
      );
      if (!alive.current) return;
      if (response.ok) {
        // Require a fresh revision before another update. An old summary is never promoted.
        newTask();
        setMessage("车端已确认保存任务。");
      } else {
        setMessage(
          confirmedCount.current > 0
            ? `已确认 ${confirmedCount.current} 个航点，任务尚未保存。草稿已保留。`
            : "保存未确认。",
        );
      }
    } catch (error) {
      if (!alive.current) return;
      const versionConflict =
        waypointRecovery !== "unknown" &&
        error instanceof CommandError &&
        error.code === "conflict";
      if (versionConflict) setConflict(true);
      const recoveryMessage = versionConflict
        ? "车端版本已变化，请重载车端版本。"
        : waypointRecovery === "failed"
          ? "车端已确认失败且未提交，可再次提交重试失败点。"
          : waypointRecovery === "unknown"
            ? "结果未知，保留原操作；再次提交只查询或重放原操作，需人工核查。"
            : "请核查任务结果后再提交。";
      const progressMessage =
        confirmedCount.current > 0
          ? `已确认 ${confirmedCount.current} 个航点，任务尚未保存。草稿已保留，`
          : "保存未确认，草稿已保留。";
      setMessage(progressMessage + recoveryMessage);
    } finally {
      setPending(false);
    }
  }

  async function changeTask(
    task: TaskSummary,
    action: "disable" | "enable" | "delete",
  ): Promise<void> {
    if (!live || pending) return;
    setPending(true);
    setMessage("等待车端确认变更…");
    try {
      const response = await link.request<ResponsePayload>(
        action === "delete" ? "task.delete" : "task.enable",
        {
          operation_id: newId(),
          expected_revision: task.revision,
          body:
            action === "delete"
              ? { task_id: task.task_id }
              : { task_id: task.task_id, enabled: action === "enable" },
        },
      );
      setMessage(response.ok ? "车端已确认任务变更。" : "车端未确认任务变更。");
    } catch (error) {
      setMessage(
        `任务变更未确认：${error instanceof CommandError ? describeCommandError(error.code) : "无法发送"}`,
      );
    } finally {
      setPending(false);
    }
  }

  const displayedMap = live ? (workspace?.map ?? map) : undefined;
  const liveMapTelemetry =
    live &&
    snapshot.localized &&
    workspace !== undefined &&
    snapshot.map_generation !== undefined &&
    (snapshot.map_grid_id ?? snapshot.map_generation) === workspace.grid_id &&
    snapshot.map_ref !== null &&
    displayedMap?.map_ref != null &&
    snapshot.map_ref.map_id === displayedMap.map_ref.map_id &&
    snapshot.map_ref.reference_revision ===
      displayedMap.map_ref.reference_revision;
  const mapView = (
    <div className={ui.map}>
      {displayedMap === undefined ? (
        <p className={ui.warning}>{unavailable ?? "地图不可用"}</p>
      ) : (
        <>
          <MapCanvas
            key={JSON.stringify([
              snapshot.robot_id,
              snapshot.map_ref,
              snapshot.map_generation ?? workspace?.grid_id,
            ])}
            map={displayedMap}
            stale={!live}
            compact
            robot={liveMapTelemetry ? (snapshot.robot_pose ?? null) : null}
            scanPoints={liveMapTelemetry ? currentScan : []}
            taskRoute={
              workspace === undefined
                ? []
                : mapRoute.filter((point) => point !== undefined)
            }
            waypointMarkers={available}
            highlightedWaypoint={highlight}
            draft={waypointDraft}
            pickEnabled={(creatingWaypoint || pickingRoute) && !pending}
            onPick={(pose) => {
              if (!pickingRoute) {
                setWaypointDraft(pose);
                return;
              }
              const id = `point-${newId().slice(0, 16)}`;
              setDraftPoints((previous) => [
                ...previous,
                {
                  ...pose,
                  waypoint_id: id,
                  kind: "waypoint",
                  status: "draft",
                  operationId: newId(),
                },
              ]);
              setWaypointIds((previous) => [...previous, id]);
            }}
            {...(pending || creatingWaypoint || pickingRoute || preview !== null
              ? {}
              : { onAddWaypoint: addWaypoint })}
            {...mapCanvasProps}
          />
          {(mapCanvasProps?.robot !== undefined
            ? mapCanvasProps.robot === null
            : !liveMapTelemetry || snapshot.robot_pose == null) && (
            <p className={ui.note}>车辆位置暂不可用</p>
          )}
        </>
      )}
    </div>
  );
  const pickingBanner = pickingRoute ? (
    <div className={ui.picking} role="status">
      <span>地图选点 · 已选 {waypointIds.length} 点</span>
      <Button
        disabled={!waypointIds.length || pending}
        onClick={() => setWaypointIds((ids) => ids.slice(0, -1))}
      >
        撤销上一点
      </Button>
      <Button
        variant="primary"
        disabled={!waypointIds.length || pending}
        onClick={() => {
          setPickingRoute(false);
          setStage("editor");
        }}
      >
        下一步
      </Button>
      <Button
        onClick={() => {
          setPickingRoute(false);
          setStage("list");
        }}
      >
        取消选点
      </Button>
    </div>
  ) : null;
  function beginTask(): void {
    newTask();
    setStage("editor");
    if (renderWorkspace && workspace?.waypointWrites) { onPickStart?.(); setPickingRoute(true); }
  }
  const listVisible = (stage === "list" || !renderWorkspace) && !pickingRoute;
  const panel = (
    <section
      className={ui.panel}
      aria-label="任务管理"
      data-task-stage={stage}
    >
      <header className={ui.header}>
        <h3>
          {stage === "editor"
            ? editing
              ? "编辑任务"
              : "新建任务"
            : "任务管理"}
        </h3>
        {stage === "list" ? (
          <span className={ui.english}>TASKS</span>
        ) : (
          renderWorkspace && (
            <div className="flex items-center gap-1">
              <Button
                className={ui.close}
                aria-expanded={showAdvanced}
                disabled={pending}
                onClick={() => setShowAdvanced((value) => !value)}
              >
                高级
              </Button>
              <Button
                aria-label="关闭任务编辑"
                className={ui.close}
                disabled={pending}
                onClick={newTask}
              >
                <X size={16} />
              </Button>
            </div>
          )
        )}
      </header>
      <div className={ui.body}>
        {listVisible && (
          <>
            <p className={ui.description}>
              地图标点建任务，定时触发后执行。
            </p>
            <div className={ui.taskActions}>
              <Button
                variant="primary"
                disabled={pending || !workspace || !namesSupported}
                onClick={beginTask}
              >
                <Plus size={14} />
                新建任务
              </Button>
              <Button onClick={() => setMessage("暂未实现：FIFO 等待队列")}>
                等待队列
              </Button>
            </div>
            {(snapshot.tasks ?? []).length === 0 && (
              <p className={ui.note}>暂无任务</p>
            )}
            {(snapshot.tasks ?? []).map((task) => {
              const definition = details[task.task_id];
              const plan =
                definition && definition.revision === task.revision
                  ? definition
                  : null;
              return (
                <article className={ui.taskItem} key={task.task_id}>
                  <div className={ui.taskIdentity}>
                    <strong>{taskLabel(task)}</strong>
                    <Switch
                      compact
                      label={task.enabled ? "停用计划" : "启用计划"}
                      checked={task.enabled}
                      disabled={!live || pending}
                      onChange={(enabled) =>
                        void changeTask(task, enabled ? "enable" : "disable")
                      }
                    />
                  </div>
                  <p className={ui.note}>
                    {plan
                      ? `${scheduleSummary({ ...NEW_TASK, ...scheduleDraft(plan.schedule), timezone: plan.timezone }).split(" · 时区")[0]} · ${plan.waypoint_ids.length} 个点 · `
                      : ""}
                    {task.enabled ? "定时已启用" : "定时已停用"}
                  </p>
                  <span className={ui.taskStatus}>
                    {progress?.task_id === task.task_id
                      ? `${progressLabel}${progress.active_waypoint_index !== null ? ` · 第 ${progress.active_waypoint_index + 1} 个航点` : ""}`
                      : "执行状态以车端报告为准"}
                  </span>
                  <div className={ui.taskButtons}>
                    <Button
                      variant="primary"
                      onClick={() => setMessage("暂未实现：立即执行已保存任务")}
                    >
                      立即执行
                    </Button>
                    <Button
                      aria-label={`编辑 ${task.task_id}`}
                      disabled={!workspace || pending}
                      onClick={() => void openTask(task.task_id)}
                    >
                      编辑
                    </Button>
                    <Button
                      aria-label={`查看路线 ${task.task_id}`}
                      aria-pressed={preview?.task_id === task.task_id}
                      disabled={!workspace || pending}
                      onClick={() => void openTask(task.task_id, "view")}
                    >
                      查看点位
                    </Button>
                  </div>
                  <TaskExecutionControls
                    port={link.vehicle}
                    compact
                    definitionId={task.task_id}
                  />
                  <details className={ui.taskDetails}>
                    <summary>任务详情</summary>
                    <code>
                      {task.task_id} · r{task.revision}
                    </code>
                    <Button
                      disabled={!live || pending}
                      onClick={() => void changeTask(task, "delete")}
                    >
                      删除
                    </Button>
                  </details>
                </article>
              );
            })}
            <div className={ui.queueInfo}>
              <TaskExecutionControls
                port={link.vehicle}
                compact
                unmatchedIds={(snapshot.tasks ?? []).map(
                  (task) => task.task_id,
                )}
              />
              {progress
                ? `${progressLabel} · ${progress.status}`
                : "当前无路线执行报告"}
              <br />
              FIFO 等待队列暂未实现。
            </div>
          </>
        )}
        {!namesSupported && (
          <p className={ui.warning}>
            车端需升级以支持独立任务名称；当前仅显示编号，可编辑已有任务的其他设置。
          </p>
        )}
        {workspace === undefined && (
          <p className={ui.warning}>
            {!live
              ? "快照已过期或车辆断开，路线编辑已锁定。"
              : (unavailable ??
                "当前地图与版本不匹配或缺少航点目录，路线查看与编辑已锁定。")}
          </p>
        )}
        {preview !== null && (
          <details className={ui.routePreview} aria-label="所选任务路线">
            <summary>点位详情 · {taskLabel(preview)}</summary>
            <h4>任务路线 · {taskLabel(preview)}</h4>
            <p className={ui.note}>内部编号 {preview.task_id}</p>
            <p className={ui.note}>
              {scheduleSummary({
                ...NEW_TASK,
                ...scheduleDraft(preview.schedule),
                timezone: preview.timezone,
              })}
            </p>
            <ol>
              {preview.waypoint_ids.map((id, index) => (
                <li key={`${index}:${id}`}>{id}</li>
              ))}
            </ol>
            {previewMissing && (
              <p className={ui.warning}>
                路线含当前目录中不存在的航点，无法显示完整路线。
              </p>
            )}
            <button type="button" onClick={() => setPreview(null)}>
              返回当前草稿
            </button>
          </details>
        )}
        <fieldset
          hidden={stage !== "editor" || pickingRoute}
          className={ui.fields}
          disabled={!workspace || pending || preview !== null}
        >
          <ScheduleEditor
            compact={renderWorkspace !== undefined}
            showAdvanced={showAdvanced}
            value={draft}
            onChange={setDraft}
            namesSupported={namesSupported}
            editing={editing !== null}
          >
            <div className={ui.enableRow}>
              <span>启用定时计划</span>
              <Switch
                compact
                label="启用此计划"
                checked={planEnabled}
                disabled={pending}
                onChange={setPlanEnabled}
              />
            </div>
            <div className={ui.pointHead}>
              <span>任务点 · 拖拽排序</span>
              {workspace?.waypointWrites && (
                <Button
                  className={ui.close}
                  onClick={() => { onPickStart?.(); setPickingRoute(true); }}
                >
                  ＋ 继续标点
                </Button>
              )}
            </div>
            <ol aria-label="任务航点顺序" className={ui.route}>
              {waypointIds.map((id, index) => (
                <li
                  key={`${index}:${id}`}
                  draggable={
                    !pending && workspace !== undefined && preview === null
                  }
                  onDragStart={(event) =>
                    event.dataTransfer.setData("text/plain", String(index))
                  }
                  onDragOver={(event) => event.preventDefault()}
                  onDrop={(event) => reorderWaypoint(event, index)}
                >
                  <div
                    className={ui.taskRow}
                    onFocus={() => setHighlight(id)}
                  >
                    <GripVertical size={13} className={ui.grip} />
                    <span className={ui.pointNumber}>{index + 1}</span>
                    <span
                      className={ui.pointLabel}
                      data-waypoint-id={id}
                      title={id}
                    >
                      任务点 {index + 1}
                      <small>
                        {waypoints.get(id)
                          ? `${waypoints.get(id)!.x.toFixed(1)}, ${waypoints.get(id)!.y.toFixed(1)}`
                          : id}
                      </small>
                    </span>
                    <button
                      type="button"
                      aria-label={`上移 ${id}`}
                      disabled={index === 0}
                      onClick={() => moveWaypoint(index, -1)}
                    >
                      <ArrowUp size={13} />
                    </button>
                    <button
                      type="button"
                      aria-label={`下移 ${id}`}
                      disabled={index === waypointIds.length - 1}
                      onClick={() => moveWaypoint(index, 1)}
                    >
                      <ArrowDown size={13} />
                    </button>
                    <button
                      type="button"
                      aria-label={`移除 ${id}`}
                      onClick={() =>
                        setWaypointIds(
                          waypointIds.filter((_, offset) => offset !== index),
                        )
                      }
                    >
                      <X size={13} />
                    </button>
                  </div>
                </li>
              ))}
            </ol>
            <p className={ui.note}>连线表示执行顺序，不表示规划路径。</p>
            {(renderWorkspace === undefined || showAdvanced || creatingWaypoint) && <details
              open={renderWorkspace === undefined || creatingWaypoint}
              className={ui.waypointDetails}
            >
              <summary>高级航点编辑</summary>
              <p className={ui.note}>
                车端状态 {snapshot.execution_status} · 任务目录 r
                {snapshot.task_revision}
              </p>
              {workspace?.waypointWrites && (
                <Button onClick={() => { onPickStart?.(); setPickingRoute(true); }}>
                  多点路线选取
                </Button>
              )}
              <div className={ui.taskRow}>
                <label>
                  可用航点
                  <select
                    value={selected}
                    onChange={(event) => {
                      setSelected(event.target.value);
                      setHighlight(event.target.value);
                    }}
                  >
                    <option value="">请选择</option>
                    {available.map((point) => (
                      <option key={point.waypoint_id}>
                        {point.waypoint_id}
                      </option>
                    ))}
                  </select>
                </label>
                <button
                  type="button"
                  disabled={!waypoints.has(selected)}
                  onClick={() => addWaypoint(selected)}
                >
                  加入路线
                </button>
              </div>
              {workspace?.waypointWrites === true &&
                (creatingWaypoint ? (
                  <div className={ui.taskRow}>
                    <p className={ui.note}>
                      {waypointDraft === null
                        ? "在地图空闲区域选点，可拖动设置朝向。"
                        : `已选位置 X ${waypointDraft.x.toFixed(2)} m · Y ${waypointDraft.y.toFixed(2)} m`}
                    </p>
                    <label>
                      新航点编号
                      <input
                        value={waypointName}
                        onChange={(event) =>
                          setWaypointName(event.target.value)
                        }
                      />
                    </label>
                    <button
                      type="button"
                      disabled={
                        waypointDraft === null ||
                        waypointName.trim() === "" ||
                        waypoints.has(waypointName.trim())
                      }
                      onClick={() => void saveWaypoint()}
                    >
                      保存航点
                    </button>
                    <button type="button" onClick={clearWaypointDraft}>
                      取消新航点
                    </button>
                  </div>
                ) : (
                  <button
                    type="button"
                    onClick={() => setCreatingWaypoint(true)}
                  >
                    在地图上新建航点
                  </button>
                ))}
            </details>}
            {missingWaypoint && (
              <p className={ui.warning}>
                路线含当前目录中不存在的航点，提交已锁定。
              </p>
            )}
          </ScheduleEditor>
        </fieldset>
      </div>
      <footer className={ui.footer}>
        {message !== null && <p role="status">{message}</p>}
        {conflict && editing !== null && (
          <Button
            disabled={pending}
            onClick={() => void openTask(editing.task_id)}
          >
            重载车端版本
          </Button>
        )}
        {stage === "editor" && !pickingRoute && (
          <div className={ui.taskActions}>
            {renderWorkspace && (
              <Button disabled={pending} onClick={newTask}>
                取消
              </Button>
            )}
            <Button
              variant="primary"
              disabled={!canSubmit}
              onClick={() => void submit()}
            >
              保存任务
            </Button>
          </div>
        )}
      </footer>
    </section>
  );
  return renderWorkspace ? (
    renderWorkspace(mapView, panel, pickingBanner)
  ) : (
    <div className={ui.workspace}>
      {mapView}
      {pickingBanner}
      {panel}
    </div>
  );
}
