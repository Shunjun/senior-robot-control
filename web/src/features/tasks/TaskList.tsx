import { isTaskName } from "../../api/vehicleState";
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
import { Switch } from "../../components/ui/Switch";
import styles from "./Tasks.module.css";

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
  map?: OccupancySnapshot | undefined;
  mapCanvasProps?: Partial<MapCanvasProps> | undefined;
  renderWorkspace?:
    | ((mapView: ReactElement, panel: ReactElement) => ReactElement)
    | undefined;
}

export function TaskList({
  snapshot,
  link,
  workspace,
  unavailable,
  map,
  mapCanvasProps,
  renderWorkspace,
}: TaskListProps): ReactElement {
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
      snapshot={snapshot}
      link={link}
      workspace={live && matches ? workspace : undefined}
      unavailable={unavailable}
      map={map}
      mapCanvasProps={mapCanvasProps}
      renderWorkspace={renderWorkspace}
    />
  );
}

function TaskEditor({
  snapshot,
  link,
  workspace,
  unavailable,
  map,
  mapCanvasProps,
  renderWorkspace,
}: TaskListProps): ReactElement {
  const [draft, setDraft] = useState<TaskDraft>(newTaskDraft);
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
  const [drawerOpen, setDrawerOpen] = useState(true);
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

  const namesSupported = snapshot.vehicle_ui?.capabilities.includes('task.names.v1') === true;
  function taskLabel(task: { task_id: string; name?: string }): string {
    //Compatible: an old vehicle always displays its stable ID.
    return namesSupported ? task.name ?? task.task_id : task.task_id;
  }
  const live = snapshot.connected && !snapshot.stale;
  const progress =
    live && snapshot.route_progress?.map_generation === snapshot.map_generation
      ? snapshot.route_progress
      : undefined;
  const progressTask = (snapshot.tasks ?? []).find(task => task.task_id === progress?.task_id);
  const progressLabel = taskLabel(progressTask ?? { task_id: progress?.task_id ?? "导航" });
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
  const canSubmit =
    live &&
    workspace !== undefined &&
    schedule !== null &&
    draft.task_id.trim() !== "" &&
    (namesSupported ? isTaskName(draft.name.trim()) : editing !== null) &&
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
          operationId: crypto.randomUUID(),
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
    clearWaypointDraft();
    setPickingRoute(false);
    setDraftPoints([]);
    confirmedCount.current = 0;
    setEditing(null);
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
      setRouteBlocked(false);
      setHighlight(null);
      setMessage(null);
      if (mode === "view") {
        setPreview(task);
        return;
      }
      setEditing(task);
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
      ...(namesSupported ? { name: draft.name.trim() } : {}),
      created_at: editing?.created_at ?? new Date().toISOString(),
      timezone: draft.timezone.trim() || NEW_TASK.timezone,
      schedule,
      waypoint_ids: waypointIds,
      enabled: editing?.enabled ?? true,
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
                    ? { ...candidate, operationId: crypto.randomUUID() }
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
          operation_id: crypto.randomUUID(),
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
          operation_id: crypto.randomUUID(),
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
    <div className={styles.map}>
      {displayedMap === undefined ? (
        <p className={styles.warning}>{unavailable ?? "地图不可用"}</p>
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
            scanPoints={liveMapTelemetry ? (snapshot.scan_points ?? []) : []}
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
              const id = `point-${crypto.randomUUID().slice(0, 16)}`;
              setDraftPoints((previous) => [
                ...previous,
                {
                  ...pose,
                  waypoint_id: id,
                  kind: "waypoint",
                  status: "draft",
                  operationId: crypto.randomUUID(),
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
            <p className={styles.note}>车辆位置暂不可用</p>
          )}
        </>
      )}
    </div>
  );
  const panel = (
    <section
      className={styles.panel}
      aria-label="任务管理"
      data-open={drawerOpen}
    >
      <header className={styles.header}>
        <h3>任务与计划</h3>
        <button
          type="button"
          className={styles.drawerToggle}
          aria-expanded={drawerOpen}
          onClick={() => setDrawerOpen(!drawerOpen)}
        >
          {drawerOpen ? "收起任务编辑" : "展开任务编辑"}
        </button>
      </header>
      <div className={styles.body}>
        <p className={styles.note}>
          车端状态 {snapshot.execution_status} · 任务目录 r
          {snapshot.task_revision}
        </p>
        {progress && (
          <p className={styles.note}>
            {progressLabel}
            {progress.active_waypoint_index !== null &&
              ` · 第 ${progress.active_waypoint_index + 1} 个航点`}
          </p>
        )}
        <div>
          <button
            type="button"
            onClick={() => setMessage("暂未实现：FIFO 等待队列")}
          >
            等待队列
          </button>
          {(snapshot.tasks ?? []).length === 0 && <p>暂无任务</p>}
          {(snapshot.tasks ?? []).map((task) => (
            <div
              className={styles.taskRow}
              key={task.task_id}
              data-selected={
                (preview?.task_id ?? editing?.task_id) === task.task_id
              }
            >
              <div className={styles.taskIdentity}>
                <strong>{taskLabel(task)}</strong>
                <code>{task.task_id}</code>
              </div>
              <button
                type="button"
                aria-label={`查看路线 ${task.task_id}`}
                aria-pressed={preview?.task_id === task.task_id}
                disabled={!workspace || pending}
                onClick={() => void openTask(task.task_id, "view")}
              >
                查看路线
              </button>
              <button
                type="button"
                aria-label={`编辑 ${task.task_id}`}
                disabled={!workspace || pending}
                onClick={() => void openTask(task.task_id)}
              >
                编辑
              </button>
              <Switch
                label={task.enabled ? "停用计划" : "启用计划"}
                checked={task.enabled}
                disabled={!live || pending}
                onChange={(enabled) =>
                  void changeTask(task, enabled ? "enable" : "disable")
                }
              />
              <button
                type="button"
                onClick={() => setMessage("暂未实现：立即执行已保存任务")}
              >
                立即执行
              </button>
              <button
                type="button"
                disabled={!live || pending}
                onClick={() => void changeTask(task, "delete")}
              >
                删除
              </button>
            </div>
          ))}
        </div>
        {!namesSupported && <p className={styles.warning}>车端需升级以支持独立任务名称；当前仅显示编号，可编辑已有任务的其他设置。</p>}
        {workspace === undefined && (
          <p className={styles.warning}>
            {!live
              ? "快照已过期或车辆断开，路线编辑已锁定。"
              : (unavailable ??
                "当前地图与版本不匹配或缺少航点目录，路线查看与编辑已锁定。")}
          </p>
        )}
        {preview !== null && (
          <section className={styles.routePreview} aria-label="所选任务路线">
            <h4>任务路线 · {taskLabel(preview)}</h4>
            <p className={styles.note}>内部编号 {preview.task_id}</p>
            <p className={styles.note}>
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
              <p className={styles.warning}>
                路线含当前目录中不存在的航点，无法显示完整路线。
              </p>
            )}
            <button type="button" onClick={() => setPreview(null)}>
              返回当前草稿
            </button>
          </section>
        )}
        <fieldset
          className={styles.fields}
          disabled={!workspace || pending || preview !== null}
        >
          <legend>
            {editing === null ? "新建任务" : `编辑任务 · r${editing.revision}`}
          </legend>
          {workspace?.waypointWrites && (
            <button
              type="button"
              onClick={() => setPickingRoute(!pickingRoute)}
            >
              {pickingRoute ? "完成选点" : "多点路线选取"}
            </button>
          )}
          {pickingRoute && (
            <p>
              在地图上依次选点；键盘方向键移动，Enter
              添加中心点。完成后编辑定时与顺序，再保存。
            </p>
          )}
          <ScheduleEditor
            value={draft}
            onChange={setDraft}
            namesSupported={namesSupported}
            editing={editing !== null}
          />
          <div className={styles.taskRow}>
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
                  <option key={point.waypoint_id}>{point.waypoint_id}</option>
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
              <div className={styles.taskRow}>
                <p className={styles.note}>
                  {waypointDraft === null
                    ? "在地图空闲区域选点，可拖动设置朝向。"
                    : `已选位置 X ${waypointDraft.x.toFixed(2)} m · Y ${waypointDraft.y.toFixed(2)} m`}
                </p>
                <label>
                  新航点编号
                  <input
                    value={waypointName}
                    onChange={(event) => setWaypointName(event.target.value)}
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
              <button type="button" onClick={() => setCreatingWaypoint(true)}>
                在地图上新建航点
              </button>
            ))}
          <ol aria-label="任务航点顺序" className={styles.route}>
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
                  className={styles.taskRow}
                  onFocus={() => setHighlight(id)}
                >
                  <code>{id}</code>
                  <button
                    type="button"
                    aria-label={`上移 ${id}`}
                    disabled={index === 0}
                    onClick={() => moveWaypoint(index, -1)}
                  >
                    上移
                  </button>
                  <button
                    type="button"
                    aria-label={`下移 ${id}`}
                    disabled={index === waypointIds.length - 1}
                    onClick={() => moveWaypoint(index, 1)}
                  >
                    下移
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
                    移除
                  </button>
                </div>
              </li>
            ))}
          </ol>
          {missingWaypoint && (
            <p className={styles.warning}>
              路线含当前目录中不存在的航点，提交已锁定。
            </p>
          )}
        </fieldset>
      </div>
      <footer className={styles.footer}>
        {message !== null && <p role="status">{message}</p>}
        {conflict && editing !== null && (
          <button
            type="button"
            disabled={pending}
            onClick={() => void openTask(editing.task_id)}
          >
            重载车端版本
          </button>
        )}
        <button
          type="button"
          disabled={pending || !workspace || !namesSupported}
          onClick={newTask}
        >
          新建任务
        </button>
        <button
          type="button"
          className={styles.primary}
          disabled={!canSubmit}
          onClick={() => void submit()}
        >
          提交任务
        </button>
      </footer>
    </section>
  );
  return renderWorkspace ? (
    renderWorkspace(mapView, panel)
  ) : (
    <div className={styles.workspace}>
      {mapView}
      {panel}
    </div>
  );
}
