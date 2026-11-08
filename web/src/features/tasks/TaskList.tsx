import { useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import type { ResponsePayload, RobotSnapshot, TaskSummary } from "../../api/protocol";
import { getTaskDetails, type TaskDetails, type WorkspaceWaypoint } from "../../api/workspace";
import { MapCanvas, type OccupancySnapshot } from "../map/MapCanvas";
import { draftSchedule, NEW_TASK, ScheduleEditor, type TaskDraft } from "./ScheduleEditor";
import styles from "./Tasks.module.css";

export interface TaskWorkspaceData {
  grid_id: string;
  map: OccupancySnapshot;
  waypoints: readonly WorkspaceWaypoint[];
}

interface TaskListProps {
  snapshot: RobotSnapshot;
  link: RobotLink;
  workspace?: TaskWorkspaceData | undefined;
  unavailable?: string | undefined;
  map?: OccupancySnapshot | undefined;
}

export function TaskList({ snapshot, link, workspace, unavailable, map }: TaskListProps): ReactElement {
  const ref = snapshot.map_ref;
  const matches = ref !== null && workspace?.map.map_ref?.map_id === ref.map_id
    && workspace.map.map_ref.reference_revision === ref.reference_revision;
  const live = snapshot.connected && !snapshot.stale;
  const scope = JSON.stringify([snapshot.robot_id, ref?.map_id, ref?.reference_revision, matches ? workspace.grid_id : null, live]);
  return <TaskEditor key={scope} snapshot={snapshot} link={link}
    workspace={live && matches ? workspace : undefined} unavailable={unavailable} map={map} />;
}

function TaskEditor({ snapshot, link, workspace, unavailable, map }: TaskListProps): ReactElement {
  const [draft, setDraft] = useState<TaskDraft>(NEW_TASK);
  const [editing, setEditing] = useState<TaskDetails | null>(null);
  const [waypointIds, setWaypointIds] = useState<string[]>([]);
  const [selected, setSelected] = useState("");
  const [highlight, setHighlight] = useState<string | null>(null);
  const [pending, setPending] = useState(false);
  const [message, setMessage] = useState<string | null>(null);
  const [conflict, setConflict] = useState(false);
  const [drawerOpen, setDrawerOpen] = useState(true);
  const live = snapshot.connected && !snapshot.stale;
  const available = workspace?.waypoints ?? [];
  const waypoints = new Map(available.map((waypoint) => [waypoint.waypoint_id, waypoint]));
  const resolved = waypointIds.map((id) => waypoints.get(id));
  const route = resolved.filter((point) => point !== undefined);
  const missingWaypoint = route.length !== waypointIds.length;
  const schedule = draftSchedule(draft);
  const canSubmit = live && workspace !== undefined && schedule !== null && draft.task_id.trim() !== ""
    && route.length > 0 && !missingWaypoint && !pending && !conflict;

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
    setEditing(null);
    setDraft(NEW_TASK);
    setWaypointIds([]);
    setHighlight(null);
    setMessage(null);
    setConflict(false);
  }

  async function openTask(taskId: string): Promise<void> {
    if (!live || pending || workspace === undefined || snapshot.map_ref === null) return;
    setPending(true);
    setMessage("正在读取车端任务…");
    try {
      const task = await getTaskDetails(link, snapshot.robot_id, snapshot.map_ref, taskId);
      if (task.grid_id !== workspace.grid_id) {
        setMessage("地图已变化，请重新读取地图后编辑。");
        setConflict(true);
        return;
      }
      setEditing(task);
      setDraft({
        ...NEW_TASK,
        task_id: task.task_id,
        timezone: task.timezone,
        kind: task.schedule.kind,
        interval: task.schedule.kind === "interval" ? String(task.schedule.every_seconds) : NEW_TASK.interval,
        cron: task.schedule.kind === "cron" ? task.schedule.expression : NEW_TASK.cron,
      });
      //Compatible: a task stored before route editing still contains a one-item waypoint array.
      setWaypointIds([...task.waypoint_ids]);
      setHighlight(null);
      setConflict(false);
      setMessage(null);
    } catch (error) {
      setMessage(`无法读取任务：${error instanceof CommandError ? describeCommandError(error.code) : "响应无效"}。`);
    } finally {
      setPending(false);
    }
  }

  async function submit(): Promise<void> {
    if (!canSubmit || schedule === null || snapshot.map_ref === null || workspace === undefined) return;
    setPending(true);
    setMessage("等待车端确认保存…");
    const body = {
      ...snapshot.map_ref,
      grid_id: workspace.grid_id,
      task_id: draft.task_id.trim(),
      created_at: editing?.created_at ?? new Date().toISOString(),
      timezone: draft.timezone.trim() || NEW_TASK.timezone,
      schedule,
      waypoint_ids: waypointIds,
      enabled: editing?.enabled ?? true,
    };
    try {
      const response = await link.request<ResponsePayload>(editing === null ? "task.create" : "task.update", {
        operation_id: crypto.randomUUID(),
        ...(editing === null ? {} : { expected_revision: editing.revision }),
        body,
      });
      if (response.ok) {
        // Require a fresh revision before another update. An old summary is never promoted.
        newTask();
        setMessage("车端已确认保存任务。");
      } else {
        setMessage("保存未确认。");
      }
    } catch (error) {
      if (error instanceof CommandError && error.code === "conflict") {
        setConflict(true);
        setMessage("车端版本已变化。已保留草稿，可重新读取车端版本。");
      } else {
        setMessage(`保存未确认：${error instanceof CommandError ? describeCommandError(error.code) : "无法发送"}`);
      }
    } finally {
      setPending(false);
    }
  }

  async function changeTask(task: TaskSummary, action: "pause" | "enable" | "delete"): Promise<void> {
    if (!live || pending) return;
    setPending(true);
    setMessage("等待车端确认变更…");
    try {
      const response = await link.request<ResponsePayload>(action === "delete" ? "task.delete" : "task.enable", {
        operation_id: crypto.randomUUID(),
        expected_revision: task.revision,
        body: action === "delete" ? { task_id: task.task_id }
          : { task_id: task.task_id, enabled: action === "enable" },
      });
      setMessage(response.ok ? "车端已确认任务变更。" : "车端未确认任务变更。");
    } catch (error) {
      setMessage(`任务变更未确认：${error instanceof CommandError ? describeCommandError(error.code) : "无法发送"}`);
    } finally {
      setPending(false);
    }
  }

  const displayedMap = live ? workspace?.map ?? map : undefined;
  return (
    <div className={styles.workspace}>
      <div className={styles.map}>
        {displayedMap === undefined ? (
          <p className={styles.warning}>{unavailable ?? "地图不可用"}</p>
        ) : (
          <MapCanvas key={workspace?.grid_id ?? "read-only"} map={displayedMap} stale={!live} compact
            taskRoute={missingWaypoint ? [] : route}
            waypointMarkers={available}
            highlightedWaypoint={highlight}
            {...(pending ? {} : { onAddWaypoint: addWaypoint })} />
        )}
      </div>
      <section className={styles.panel} aria-label="任务管理" data-open={drawerOpen}>
        <header className={styles.header}>
          <h3>任务与计划</h3>
          <button type="button" className={styles.drawerToggle} aria-expanded={drawerOpen}
            onClick={() => setDrawerOpen(!drawerOpen)}>
            {drawerOpen ? "收起任务编辑" : "展开任务编辑"}
          </button>
        </header>
        <div className={styles.body}>
          <p className={styles.note}>车端状态 {snapshot.execution_status} · 任务目录 r{snapshot.task_revision}</p>
          <div>
            {(snapshot.tasks ?? []).length === 0 && <p>暂无任务</p>}
            {(snapshot.tasks ?? []).map((task) => (
              <div className={styles.taskRow} key={task.task_id}>
                <code>{task.task_id}</code>
                <button type="button" aria-label={`编辑 ${task.task_id}`} disabled={!workspace || pending}
                  onClick={() => void openTask(task.task_id)}>编辑</button>
                <button type="button" disabled={!live || pending}
                  onClick={() => void changeTask(task, task.enabled ? "pause" : "enable")}>
                  {task.enabled ? "暂停" : "启用"}
                </button>
                <button type="button" disabled={!live || pending}
                  onClick={() => void changeTask(task, "delete")}>删除</button>
              </div>
            ))}
          </div>
          {workspace === undefined && <p className={styles.warning}>
            {!live ? "快照已过期或车辆断开，路线编辑已锁定。" : unavailable ?? "缺少当前地图与航点目录，路线编辑已锁定。"}
          </p>}
          <fieldset className={styles.fields} disabled={!workspace || pending}>
            <legend>{editing === null ? "新建任务" : `编辑任务 · r${editing.revision}`}</legend>
            <ScheduleEditor value={draft} onChange={setDraft} editing={editing !== null} />
            <div className={styles.taskRow}>
              <label>
                可用航点
                <select value={selected} onChange={(event) => {
                  setSelected(event.target.value);
                  setHighlight(event.target.value);
                }}>
                  <option value="">请选择</option>
                  {available.map((point) => <option key={point.waypoint_id}>{point.waypoint_id}</option>)}
                </select>
              </label>
              <button type="button" disabled={!waypoints.has(selected)} onClick={() => addWaypoint(selected)}>加入路线</button>
            </div>
            <ol aria-label="任务航点顺序" className={styles.route}>
              {waypointIds.map((id, index) => (
                <li key={`${index}:${id}`}>
                  <div className={styles.taskRow} onFocus={() => setHighlight(id)}>
                    <code>{id}</code>
                    <button type="button" aria-label={`上移 ${id}`} disabled={index === 0}
                      onClick={() => moveWaypoint(index, -1)}>上移</button>
                    <button type="button" aria-label={`下移 ${id}`} disabled={index === waypointIds.length - 1}
                      onClick={() => moveWaypoint(index, 1)}>下移</button>
                    <button type="button" aria-label={`移除 ${id}`}
                      onClick={() => setWaypointIds(waypointIds.filter((_, offset) => offset !== index))}>移除</button>
                  </div>
                </li>
              ))}
            </ol>
            {missingWaypoint && <p className={styles.warning}>路线含当前目录中不存在的航点，提交已锁定。</p>}
          </fieldset>
        </div>
        <footer className={styles.footer}>
          {message !== null && <p role="status">{message}</p>}
          {conflict && editing !== null && (
            <button type="button" disabled={pending} onClick={() => void openTask(editing.task_id)}>重载车端版本</button>
          )}
          <button type="button" disabled={pending || !workspace} onClick={newTask}>新建任务</button>
          <button type="button" className={styles.primary} disabled={!canSubmit}
            onClick={() => void submit()}>提交任务</button>
        </footer>
      </section>
    </div>
  );
}
