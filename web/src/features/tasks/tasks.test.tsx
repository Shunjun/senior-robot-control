import { act, cleanup, fireEvent, render, renderHook, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { useState } from "react";

import { CommandError, type RobotLink } from "../../api/client";
import { runtime } from "../../api/vehicleState.testUtils";
import type { LinkStatus, RobotSnapshot } from "../../api/protocol";
import { useWorkspaceMap } from "../map/useWorkspaceMap";

import { draftSchedule, NEW_TASK, ScheduleEditor, type TaskDraft } from "./ScheduleEditor";
import { getTaskDetails } from "../../api/workspace";
import { TaskList, type TaskWorkspaceData } from "./TaskList";

beforeEach(() => vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null));
afterEach(() => {
  cleanup();
  vi.restoreAllMocks();
});

const snapshot: RobotSnapshot = {
  vehicle_ui: runtime({capabilities:["task.names.v1"]}),
  robot_id: "robot-alpha",
  connected: true,
  localized: true,
  map_ref: { map_id: "map-1", reference_revision: 2 },
  home: null,
  task_revision: 7,
  runnable_task_ids: ["daily"],
  execution_status: "idle",
  collected_at: 1,
  stale: false,
  last_synced_at: 1,
  home_revision: 0,
};

function link(): RobotLink {
  return {
    connect: async () => undefined,
    disconnect: () => undefined,
    request: vi.fn(async () => ({
      request_type: "task.create",
      ok: true,
      status: "succeeded",
    })) as unknown as RobotLink["request"],
    subscribe: () => () => undefined,
    subscribeStatus: (_listener: (status: LinkStatus) => void) => () => undefined,
  };
}

describe("task schedules", () => {
  it("test_daily_interval_weekly_cron_roundtrip", () => {
    const changed: TaskDraft[] = [];
    function Editor() {
      const [draft, setDraft] = useState(NEW_TASK);
      return <ScheduleEditor value={draft} editing={false} onChange={(next) => {
        setDraft(next);
        changed.push(next);
      }} />;
    }
    render(<Editor />);
    fireEvent.change(screen.getByLabelText("任务名称"), { target: { value: "daily" } });
    expect(draftSchedule(changed.at(-1)!)).toEqual({ kind: "cron", expression: "0 9 * * *" });
    expect(changed.at(-1)?.timezone).toBe("Asia/Shanghai");

    fireEvent.change(screen.getByLabelText("计划类型"), { target: { value: "advanced" } });
    fireEvent.change(screen.getByLabelText("Cron 表达式"), { target: { value: "15 9 * * 1" } });
    expect(draftSchedule(changed.at(-1)!)).toEqual({ kind: "cron", expression: "15 9 * * 1" });
    expect(changed.at(-1)?.timezone).toBe("Asia/Shanghai");
  });

  it("submits a waypoint and edits each task with its own revision", async () => {
    const robotLink = link();
    const listed = {
      ...snapshot,
      tasks: [
        { task_id: "daily", revision: 4, enabled: true },
        { task_id: "night", revision: 9, enabled: false },
      ],
    };
    render(<TaskList snapshot={listed} link={robotLink} workspace={workspace} />);
    fireEvent.change(screen.getByLabelText("任务名称"), { target: { value: "evening" } });
    fireEvent.keyDown(screen.getByRole("button", { name: "添加地图航点 gate" }), { key: "Enter" });
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await screen.findByText("车端已确认保存任务。");
    const created = vi.mocked(robotLink.request).mock.calls[0]?.[1] as {
      body: {
        waypoint_ids: string[];
      };
    };
    expect(created.body.waypoint_ids).toEqual(["gate"]);

    fireEvent.click(screen.getByRole("switch", { name: "停用计划" }));
    await waitFor(() => expect(vi.mocked(robotLink.request).mock.calls).toHaveLength(2));
    fireEvent.click(screen.getByRole("switch", { name: "启用计划" }));
    await waitFor(() => expect(vi.mocked(robotLink.request).mock.calls).toHaveLength(3));
    fireEvent.click(screen.getAllByRole("button", { name: "删除" })[1]!);
    await waitFor(() => expect(vi.mocked(robotLink.request).mock.calls).toHaveLength(4));
    const calls = vi.mocked(robotLink.request).mock.calls;
    expect(calls[1]?.[0]).toBe("task.enable");
    expect(calls[1]?.[1]).toMatchObject({
      expected_revision: 4,
      body: { task_id: "daily", enabled: false },
    });
    expect(calls[2]?.[1]).toMatchObject({
      expected_revision: 9,
      body: { task_id: "night", enabled: true },
    });
    expect(calls[3]?.[0]).toBe("task.delete");
    expect(calls[3]?.[1]).toMatchObject({
      expected_revision: 9,
      body: { task_id: "night" },
    });
  });

  it("test_offline_edits_disabled", async () => {
    const stale = { ...snapshot, connected: false, stale: true };
    const robotLink = link();
    render(<TaskList snapshot={stale} link={robotLink} />);
    expect((screen.getByRole("button", { name: "提交任务" }) as HTMLButtonElement).disabled).toBe(true);
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await waitFor(() => expect(robotLink.request).not.toHaveBeenCalled());
  });
});

const task = { task_id: "daily", revision: 4, enabled: true };
const listedSnapshot = { ...snapshot, tasks: [task] };
const map = {
  map_ref: snapshot.map_ref!,
  origin: { x: 0, y: 0, yaw: 0 },
  resolution: 1,
  cells: []
};
const workspace: TaskWorkspaceData = {
  grid_id: "grid-1",
  map,
  waypoints: ["gate", "hall", "dock"].map((waypoint_id, x) => ({
    waypoint_id,
    x,
    y: 0,
    yaw: 0,
    status: "active",
    kind: "waypoint",
  })),
};
const taskDetails = {
  ...task,
  robot_id: snapshot.robot_id,
  map_ref: snapshot.map_ref,
  grid_id: workspace.grid_id,
  timezone: "Asia/Shanghai",
  schedule: { kind: "interval", every_seconds: 86400 },
  created_at: "2026-11-08T00:00:00+00:00",
};

function taskLink(waypointIds: string[]) {
  const robotLink = link();
  vi.mocked(robotLink.request).mockImplementation(async (type, payload) => {
    if (type === "task.get") {
      return { ok: true, data: { ...taskDetails, waypoint_ids: waypointIds } } as never;
    }
    if (type === "task.update") {
      waypointIds = (payload as {
        body: {
          waypoint_ids: string[];
        };
      }).body.waypoint_ids;
    }
    return { ok: true } as never;
  });
  return robotLink;
}

describe("ordered task route", () => {
  const openTask = () => fireEvent.click(screen.getByRole("button", { name: "编辑 daily" }));
  const route = () => Array.from(screen.getByRole("list", { name: "任务航点顺序" }).querySelectorAll("li"))
    .map((item) => item.querySelector("code")?.textContent);

  it("test_reorders_and_reopens_three_point_route", async () => {
    const robotLink = taskLink(["gate", "hall", "dock"]);
    render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
    openTask();
    await screen.findByRole("button", { name: "上移 dock" });
    fireEvent.click(screen.getByRole("button", { name: "上移 dock" }));
    expect(route()).toEqual(["gate", "dock", "hall"]);
    expect(screen.getByLabelText("任务航点预览").tagName).toBe("polyline");
    fireEvent.click(screen.getByRole("button", { name: "移除 hall" }));
    fireEvent.change(screen.getByLabelText("可用航点"), { target: { value: "hall" } });
    fireEvent.click(screen.getByRole("button", { name: "加入路线" }));
    expect(route()).toEqual(["gate", "dock", "hall"]);
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await screen.findByText("车端已确认保存任务。");
    expect(robotLink.request).toHaveBeenCalledWith("task.update", expect.objectContaining({
      expected_revision: 4,
      body: expect.objectContaining({ waypoint_ids: ["gate", "dock", "hall"], grid_id: workspace.grid_id }),
    }));
    openTask();
    await waitFor(() => expect(route()).toEqual(["gate", "dock", "hall"]));
  });

  it("test_reference_change_rejects_late_route_read", async () => {
    const robotLink = link();
    let resolveRead!: (value: never) => void;
    vi.mocked(robotLink.request).mockImplementation(() => new Promise((resolve) => {
      resolveRead = resolve;
    }));
    const view = render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
    openTask();
    await waitFor(() => expect(robotLink.request).toHaveBeenCalled());
    view.rerender(<TaskList snapshot={{
      ...listedSnapshot, map_ref: { map_id: "map-1", reference_revision: 3 },
    }} link={robotLink} workspace={workspace} />);
    await act(async () => resolveRead({ ok: true, data: { ...taskDetails, waypoint_ids: ["gate"] } } as never));
    await waitFor(() => expect(screen.queryByRole("button", { name: "移除 gate" })).toBeNull());
    expect((screen.getByRole("button", { name: "提交任务" }) as HTMLButtonElement).disabled).toBe(true);
  });

  it("retains the reordered draft on revision conflict until explicitly reloaded", async () => {
    const robotLink = taskLink(["gate", "hall"]);
    const fallback = vi.mocked(robotLink.request).getMockImplementation()!;
    vi.mocked(robotLink.request).mockImplementation(async (type, payload) => {
      if (type === "task.update") throw new CommandError("conflict", "revision changed");
      return fallback(type, payload);
    });
    render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
    openTask();
    await screen.findByRole("button", { name: "上移 hall" });
    fireEvent.click(screen.getByRole("button", { name: "上移 hall" }));
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    const reload = await screen.findByRole("button", { name: "重载车端版本" });
    expect(route()).toEqual(["hall", "gate"]);
    expect((screen.getByRole("button", { name: "提交任务" }) as HTMLButtonElement).disabled).toBe(true);
    fireEvent.click(reload);
    await waitFor(() => expect(route()).toEqual(["gate", "hall"]));
  });

  it("test_single_point_compatibility", async () => {
    const robotLink = taskLink(["gate"]);
    render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
    openTask();
    await screen.findByRole("button", { name: "移除 gate" });
    expect(route()).toEqual(["gate"]);
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await screen.findByText("车端已确认保存任务。");
    expect(robotLink.request).toHaveBeenCalledWith("task.update", expect.objectContaining({
      expected_revision: 4,
      body: expect.objectContaining({ waypoint_ids: ["gate"], created_at: taskDetails.created_at }),
    }));
  });
});

it("assembles current occupancy tiles and catalog before allowing route edits", async () => {
  const robotLink = link();
  const identity = { robot_id: snapshot.robot_id, map_ref: snapshot.map_ref, grid_id: "grid-1" };
  vi.mocked(robotLink.request).mockImplementation(async (type, payload) => {
    if (type === "map.manifest") return {
      data: {
        ...identity,
        width: 33,
        height: 2,
        resolution: 1,
        origin: map.origin,
        tile_size: 32,
      }
    } as never;
    if (type === "map.tile") {
      const x = (payload as {
        tile_x: number;
      }).tile_x;
      return {
        data: {
          ...identity,
          tile_x: x,
          tile_y: 0,
          width: x === 0 ? 32 : 1,
          height: 2,
          cells: x === 0 ? [...Array(32).fill(0), ...Array(32).fill(-1)] : [5, 75],
        }
      } as never;
    }
    return { data: { ...identity, waypoints: workspace.waypoints, next_cursor: null } } as never;
  });
  const { result } = renderHook(() => useWorkspaceMap(robotLink, snapshot, {
    enabled: true, catalog: true, refresh: 0,
  }));
  await waitFor(() => expect(result.current.waypoints?.map((waypoint) => waypoint.waypoint_id)).toEqual(["gate", "hall", "dock"]));
  const grid = result.current.map?.grid;
  expect(grid && Array.from(grid.values)).toEqual([
    ...Array(32).fill(0), 5, ...Array(32).fill(-1), 75,
  ]);
  expect(vi.mocked(robotLink.request).mock.calls.map(([type]) => type)).toEqual([
    "map.manifest", "map.tile", "map.tile", "waypoint.list",
  ]);
});

it("saves a picked point before exposing it as a selectable route waypoint", async () => {
  const robotLink = link();
  let confirm!: (response: never) => void;
  const picked = {
    waypoint_id: "new-point",
    x: 0,
    y: 0,
    yaw: 0,
    kind: "waypoint",
    status: "confirmed"
  };
  vi.mocked(robotLink.request).mockImplementation(async (type) => {
    if (type === "waypoint.create") return new Promise((resolve) => {
      confirm = resolve;
    });
    if (type === "waypoint.list") return {
      ok: true, data: {
        robot_id: snapshot.robot_id,
        map_ref: snapshot.map_ref,
        grid_id: workspace.grid_id,
        waypoints: [...workspace.waypoints, picked],
        next_cursor: null,
      }
    } as never;
    return { ok: false } as never;
  });
  render(<TaskList snapshot={listedSnapshot} link={robotLink}
    workspace={{ ...workspace, waypointWrites: true }} />);
  fireEvent.click(screen.getByRole("button", { name: "在地图上新建航点" }));
  const surface = screen.getByRole("application", { name: "地图" });
  const dispatchPointer = (type: string) => fireEvent(surface, Object.assign(
    new MouseEvent(type, { bubbles: true, clientX: 320, clientY: 210 }), { pointerId: 1 },
  ));
  dispatchPointer("pointerdown");
  dispatchPointer("pointerup");
  fireEvent.change(screen.getByLabelText("新航点编号"), { target: { value: "new-point" } });
  fireEvent.click(screen.getByRole("button", { name: "保存航点" }));
  expect(robotLink.request).toHaveBeenCalledWith("waypoint.create", expect.objectContaining({
    body: {
      ...snapshot.map_ref,
      grid_id: workspace.grid_id,
      waypoint_id: "new-point",
      x: 0,
      y: 0,
      yaw: 0
    },
  }));
  expect(screen.queryByRole("option", { name: "new-point" })).toBeNull();
  await act(async () => confirm({ ok: true } as never));
  await screen.findByRole("option", { name: "new-point" });
  expect(screen.getByRole("list", { name: "任务航点顺序" }).children).toHaveLength(0);
  fireEvent.click(screen.getByRole("button", { name: "加入路线" }));
  expect(screen.getByRole("list", { name: "任务航点顺序" }).textContent).toContain("new-point");
});

it("shows only vehicle-reported progress on the current live map", () => {
  const current: RobotSnapshot = {
    ...snapshot,
    execution_status: "running",
    tasks: [{task_id:"daily",name:"夜班巡检",revision:4,enabled:true}],
    map_generation: "grid-1",
    route_progress: {
      execution_id: "run-1",
      task_id: "daily",
      map_generation: "grid-1",
      status: "running",
      active_waypoint_index: 1,
    },
  };
  const robotLink = link();
  const view = render(<TaskList snapshot={current} link={robotLink} />);
  expect(screen.getByText(/夜班巡检 · 第 2 个航点/)).toBeTruthy();
  view.rerender(<TaskList snapshot={{ ...current, map_generation: "grid-2" }} link={robotLink} />);
  expect(screen.queryByText(/夜班巡检 · 第 2 个航点/)).toBeNull();
  view.rerender(<TaskList snapshot={{ ...current, stale: true }} link={robotLink} />);
  expect(screen.queryByText(/夜班巡检 · 第 2 个航点/)).toBeNull();
});

it("shows current vehicle and laser hits on task maps only for qualified live generations", () => {
  const robotLink = link();
  const current = {
    ...snapshot,
    map_generation: workspace.grid_id,
    robot_pose: { x: 1, y: 2, yaw: 0.2 },
    scan_points: [{ x: 2, y: 3 }]
  };
  const { container, rerender } = render(<TaskList snapshot={current} link={robotLink} workspace={workspace} />);
  expect(screen.queryByLabelText("实时激光命中点")).not.toBeNull();
  expect(container.querySelector('[data-marker="robot"]')).not.toBeNull();
  for (const overrides of [{ localized: false }, { connected: false }, { stale: true }, { map_generation: "other-grid" }, { map_ref: { map_id: "other", reference_revision: 2 } }]) {
    rerender(<TaskList snapshot={{ ...current, ...overrides }} link={robotLink} workspace={workspace} map={map} />);
    expect(screen.queryByLabelText("实时激光命中点")).toBeNull();
    expect(container.querySelector('[data-marker="robot"]')).toBeNull();
  }
  rerender(<TaskList snapshot={current} link={robotLink} map={map} />);
  expect(screen.queryByLabelText("实时激光命中点")).toBeNull();
  expect(container.querySelector('[data-marker="robot"]')).toBeNull();
  expect(screen.queryByText("车辆位置暂不可用")).not.toBeNull();
});


it('shows a selected task route directly and keeps editing order live on the map', async () => {
  const robotLink = taskLink(['gate', 'hall', 'dock']);
  const { container } = render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button', { name: '查看路线 daily' }));
  await screen.findByText('任务路线 · daily');
  expect(robotLink.request).toHaveBeenCalledWith('task.get', expect.objectContaining({ task_id: 'daily' }));
  expect(screen.getByRole('button', { name: '查看路线 daily' }).getAttribute('aria-pressed')).toBe('true');
  const preview = screen.getByLabelText('任务航点预览');
  const before = preview.getAttribute('points');
  expect(container.querySelectorAll('text[data-route-index]')).toHaveLength(3);
  expect(screen.getByText(/顺序连线.*不代表.*避障/)).toBeTruthy();
  fireEvent.click(screen.getByRole('button', { name: '编辑 daily' }));
  await screen.findByRole('button', { name: '上移 dock' });
  fireEvent.click(screen.getByRole('button', { name: '上移 dock' }));
  expect(screen.getByLabelText('任务航点预览').getAttribute('points')).not.toBe(before);
});

it.each(['missing', 'grid'] as const)('does not draw a %s task route on the wrong/current incomplete map', async reason => {
  const robotLink = taskLink(['gate', 'missing']);
  if (reason === 'grid') vi.mocked(robotLink.request).mockResolvedValue({ ok: true, data: {
    ...taskDetails, grid_id: 'old-grid', waypoint_ids: ['gate', 'hall']
  } } as never);
  render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button', { name: '查看路线 daily' }));
  await screen.findByText(reason === 'grid' ? /地图.*变化/ : /路线含.*不存在的航点/);
  expect(screen.queryByLabelText('任务航点预览')).toBeNull();
});

it.each(['daily', 'weekly', 'interval'] as const)('creates, saves and reopens a visual %s schedule', async mode => {
  const robotLink = link();
  let saved = { ...taskDetails, waypoint_ids: ['gate'] };
  vi.mocked(robotLink.request).mockImplementation(async (type, payload) => {
    if (type === 'task.get') return { ok: true, data: saved } as never;
    if (type === 'task.create' || type === 'task.update') saved = {
      ...saved, ...(payload as { body: typeof saved }).body
    };
    return { ok: true } as never;
  });
  const { rerender } = render(<TaskList snapshot={snapshot} link={robotLink} workspace={workspace} />);
  fireEvent.change(screen.getByLabelText('任务名称'), { target: { value: 'daily' } });
  fireEvent.change(screen.getByLabelText('计划类型'), { target: { value: mode } });
  if (mode === 'interval') {
    fireEvent.change(screen.getByLabelText('间隔数值'), { target: { value: '2' } });
    fireEvent.change(screen.getByLabelText('间隔单位'), { target: { value: 'hours' } });
  } else {
    fireEvent.change(screen.getByLabelText('执行时间'), { target: { value: '08:30' } });
    if (mode === 'weekly') fireEvent.click(screen.getByLabelText('周三'));
  }
  fireEvent.keyDown(screen.getByRole('button', { name: '添加地图航点 gate' }), { key: 'Enter' });
  fireEvent.click(screen.getByRole('button', { name: '提交任务' }));
  await screen.findByText('车端已确认保存任务。');
  const expected = mode === 'interval' ? { kind: 'interval', every_seconds: 7200 }
    : { kind: 'cron', expression: mode === 'daily' ? '30 8 * * *' : '30 8 * * 1,3' };
  expect(saved.schedule).toEqual(expected);
  rerender(<TaskList snapshot={{...listedSnapshot,tasks:[{task_id:saved.task_id,revision:4,enabled:true}]}} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button', { name: `编辑 ${saved.task_id}` }));
  await waitFor(() => expect((screen.getByLabelText('计划类型') as HTMLSelectElement).value).toBe(mode));
  if (mode === 'interval') expect((screen.getByLabelText('间隔数值') as HTMLInputElement).value).toBe('2');
  else expect((screen.getByLabelText('执行时间') as HTMLInputElement).value).toBe('08:30');
  fireEvent.change(screen.getByLabelText('时区'), { target: { value: 'Europe/Berlin' } });
  fireEvent.click(screen.getByRole('button', { name: '提交任务' }));
  await screen.findByText('车端已确认保存任务。');
  expect(saved.schedule).toEqual(expected);
  expect(saved.timezone).toBe('Europe/Berlin');
});


it('keeps an existing complex cron unchanged when saving other task fields', async () => {
  const robotLink = taskLink(['gate']);
  const expression = '*/15 9-17 * * 1-5';
  const fallback = vi.mocked(robotLink.request).getMockImplementation()!;
  vi.mocked(robotLink.request).mockImplementation(async (type, payload) => type === 'task.get'
    ? { ok: true, data: { ...taskDetails, waypoint_ids: ['gate'], schedule: { kind: 'cron', expression } } } as never
    : fallback(type, payload));
  render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button', { name: '编辑 daily' }));
  await screen.findByLabelText('Cron 表达式');
  expect((screen.getByLabelText('Cron 表达式') as HTMLInputElement).value).toBe(expression);
  fireEvent.change(screen.getByLabelText('时区'), { target: { value: 'Europe/Berlin' } });
  fireEvent.click(screen.getByRole('button', { name: '提交任务' }));
  await screen.findByText('车端已确认保存任务。');
  expect(robotLink.request).toHaveBeenCalledWith('task.update', expect.objectContaining({
    expected_revision: 4, body: expect.objectContaining({ task_id: 'daily', timezone: 'Europe/Berlin', schedule: { kind: 'cron', expression } })
  }));
});

it("accepts independent raster and reference versions, retiring drafts on reference change", () => {
  const robotLink = link();
  const state = { ...snapshot, map_generation: "reference-7", map_grid_id: workspace.grid_id };
  const page = render(<TaskList snapshot={state} link={robotLink} workspace={workspace} />);
  const input = screen.getByLabelText("任务名称") as HTMLInputElement;
  expect(input.closest("fieldset")?.disabled).toBe(false);
  fireEvent.change(input, { target: { value: "draft-current" } });
  page.rerender(<TaskList snapshot={{ ...state, map_generation: "reference-8", map_ref: { map_id: "another", reference_revision: 3 } }} link={robotLink} workspace={workspace} />);
  expect((screen.getByLabelText("任务名称") as HTMLInputElement).value).toBe("");
  expect((screen.getByLabelText("任务名称") as HTMLInputElement).closest("fieldset")?.disabled).toBe(true);
});

it.each([
  "failed",
  "needs_review",
  "running",
  "succeeded",
  "unknown",
  "mismatched",
  "timeout_failed",
] as const)(
  "recovers draft points using journal evidence: %s",
  async (scenario) => {
    const outcome = scenario === "timeout_failed" ? "failed" : scenario;
    const robotLink = link();
    const request = vi.mocked(robotLink.request);
    const journal = new Map<string, string>();
    const creations: string[] = [];
    let failedOperation: string | undefined;
    request.mockImplementation(async (type, payload) => {
      const operationId = (payload as { operation_id: string }).operation_id;
      if (type === "operation.query") {
        if (outcome === "unknown")
          throw new CommandError("operation_unknown", "not found");
        return {
          ok: true,
          operation_id: outcome === "mismatched" ? "another" : operationId,
          status: outcome === "mismatched" ? "failed" : outcome,
        } as never;
      }
      if (type === "waypoint.create") {
        creations.push(operationId);
        if (
          journal.has(operationId) &&
          journal.get(operationId) !== "succeeded"
        ) {
          const status = journal.get(operationId);
          throw new CommandError(status === "needs_review" ? "operation_unknown"
            : status === "running" ? "conflict" : "unavailable", "journal replay rejected");
        }
        if (journal.size === 1) {
          failedOperation = operationId;
          journal.set(operationId, outcome === "unknown" || outcome === "mismatched" ? "needs_review" : outcome);
          throw new CommandError(
            scenario === "timeout_failed" || outcome !== "failed" ? "timeout" : "unavailable",
            "action response has no terminal status",
          );
        }
        journal.set(operationId, "succeeded");
      }
      return { ok: true, status: "succeeded" } as never;
    });
    render(
      <TaskList
        snapshot={snapshot}
        link={robotLink}
        workspace={{ ...workspace, waypointWrites: true }}
      />,
    );
    fireEvent.change(screen.getByLabelText("任务名称"), {
      target: { value: "route-new" },
    });
    const stableTaskId=(screen.getByLabelText("内部编号") as HTMLInputElement).value;
    fireEvent.click(screen.getByRole("button", { name: "多点路线选取" }));
    const surface = screen.getByRole("application", { name: "地图" });
    fireEvent.keyDown(surface, { key: "Enter" });
    fireEvent.keyDown(surface, { key: "ArrowRight" });
    fireEvent.keyDown(surface, { key: "Enter" });
    fireEvent.click(screen.getByRole("button", { name: "完成选点" }));
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    if (outcome === "succeeded") {
      await screen.findByText("车端已确认保存任务。");
      expect(creations).toHaveLength(2);
      expect(
        request.mock.calls.filter((call) => call[0] === "task.create"),
      ).toHaveLength(1);
      return;
    }
    await screen.findByText(/已确认 1 个航点，任务尚未保存/);
    expect(
      request.mock.calls.filter((call) => call[0] === "task.create"),
    ).toHaveLength(0);
    expect(request).toHaveBeenCalledWith("operation.query", {
      operation_id: failedOperation,
      body: {},
    });
    expect(screen.getByLabelText("任务名称")).toHaveProperty(
      "value",
      "route-new",
    );
    expect(screen.getByLabelText("内部编号")).toHaveProperty("value",stableTaskId);
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    if (outcome === "failed") {
      await screen.findByText("车端已确认保存任务。");
      expect(creations[2]).not.toBe(failedOperation);
    } else {
      await screen.findByText(/结果未知.*原操作/);
      expect(creations[2]).toBe(failedOperation);
      expect(
        request.mock.calls.filter((call) => call[0] === "task.create"),
      ).toHaveLength(0);
    }
    if (outcome !== "failed") {
      expect(screen.getByRole("button", { name: "提交任务" })).toHaveProperty("disabled", false);
    }
    expect(creations).toHaveLength(3);
    expect(creations.filter((id) => id === creations[0])).toHaveLength(1);
  },
);


it('makes missing immediate execution and queue actions explicit without sending requests', () => {
  const robotLink=link();
  render(<TaskList snapshot={{...snapshot,tasks:[{task_id:'saved',revision:1,enabled:true}]}} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button',{name:'立即执行'}));
  expect(screen.getByText('暂未实现：立即执行已保存任务')).toBeTruthy();
  fireEvent.click(screen.getByRole('button',{name:'等待队列'}));
  expect(screen.getByText('暂未实现：FIFO 等待队列')).toBeTruthy();
  expect(robotLink.request).not.toHaveBeenCalled();
});

it('persists drag order and a subsequent keyboard-accessible adjustment from the same route', async () => {
  const robotLink=taskLink(['gate','hall','dock']);
  render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button',{name:'编辑 daily'}));
  await screen.findByRole('button',{name:'上移 dock'});
  const route=screen.getByRole('list',{name:'任务航点顺序'});
  const items=route.querySelectorAll('li');
  const values=new Map<string,string>();const dataTransfer={setData:(k:string,v:string)=>values.set(k,v),getData:(k:string)=>values.get(k)??''};
  fireEvent.dragStart(items[2]!,{dataTransfer});
  fireEvent.drop(items[0]!,{dataTransfer});
  expect([...route.querySelectorAll('code')].map(node=>node.textContent)).toEqual(['dock','gate','hall']);
  fireEvent.click(screen.getByRole('button',{name:'上移 hall'}));
  fireEvent.click(screen.getByRole('button',{name:'提交任务'}));
  await screen.findByText('车端已确认保存任务。');
  expect(robotLink.request).toHaveBeenCalledWith('task.update',expect.objectContaining({body:expect.objectContaining({waypoint_ids:['dock','hall','gate']})}));
});

it('keeps the saved-route preview read-only while retaining the current editable draft', async () => {
  const robotLink=taskLink(['gate','hall']);
  render(<TaskList snapshot={listedSnapshot} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button',{name:'查看路线 daily'}));
  const preview=await screen.findByRole('region',{name:'所选任务路线'});
  expect([...preview.querySelectorAll('li')].some(item=>item.draggable)).toBe(false);
});

it("N7 creates an independent stable ID then renames without changing the accepted plan", async () => {
  const robotLink = link();
  let stored: Record<string, unknown> = {};
  vi.mocked(robotLink.request).mockImplementation(async (type, payload) => {
    const q = payload as {body: Record<string, unknown>};
    if(type === 'task.get') return {ok:true,data:{robot_id:snapshot.robot_id,map_ref:snapshot.map_ref,grid_id:workspace.grid_id,...stored,revision:1}} as never;
    if(type === 'task.create' || type === 'task.update') stored = {...q.body};
    return {ok:true,status:'succeeded'} as never;
  });
  const page=render(<TaskList snapshot={snapshot} link={robotLink} workspace={workspace} />);
  fireEvent.change(screen.getByLabelText('任务名称'), {
    target: { value: ' \u0085东区巡检\u0085 ' },
  });
  const id=(screen.getByLabelText('内部编号') as HTMLInputElement).value;
  expect(id).toMatch(/^[0-9a-f-]{36}$/);
  fireEvent.change(screen.getByLabelText('计划类型'),{target:{value:'interval'}});
  fireEvent.keyDown(screen.getByRole('button',{name:'添加地图航点 gate'}),{key:'Enter'});
  fireEvent.click(screen.getByRole('button',{name:'提交任务'}));
  await screen.findByText('车端已确认保存任务。');
  expect(stored).toMatchObject({task_id:id,name:'东区巡检',waypoint_ids:['gate']});
  const original={...stored};
  page.rerender(<TaskList snapshot={{...snapshot,tasks:[{task_id:id,name:'东区巡检',revision:1,enabled:true}]}} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button',{name:`编辑 ${id}`}));
  await waitFor(()=>expect(screen.getByLabelText('任务名称')).toHaveProperty('value','东区巡检'));
  fireEvent.change(screen.getByLabelText('任务名称'), {
    target: { value: ' \u0085夜班巡检\uFEFF ' },
  });
  fireEvent.click(screen.getByRole('button',{name:'提交任务'}));
  await screen.findByText('车端已确认保存任务。');
  expect(stored).toEqual({...original,name:'夜班巡检'});
  expect(robotLink.request).toHaveBeenLastCalledWith('task.update',expect.objectContaining({expected_revision:1}));
  page.rerender(<TaskList snapshot={{...snapshot,tasks:[{task_id:id,name:'夜班巡检',revision:2,enabled:true}]}} link={robotLink} workspace={workspace} />);
  fireEvent.click(screen.getByRole('button',{name:`查看路线 ${id}`}));
  expect(await screen.findByRole('heading',{name:'任务路线 · 夜班巡检'})).toBeTruthy();
});

it("N9 locks names on an old empty vehicle and preserves non-name editing", async () => {
  const robotLink=link();
  vi.mocked(robotLink.request).mockResolvedValue({ok:true,data:{robot_id:snapshot.robot_id,map_ref:snapshot.map_ref,grid_id:workspace.grid_id,task_id:'daily',waypoint_ids:['gate'],schedule:{kind:'cron',expression:'0 9 * * *'},timezone:'Asia/Shanghai',enabled:true,revision:4,created_at:'2026-11-24T00:00:00Z'}} as never);
  const old={...snapshot,vehicle_ui:{...snapshot.vehicle_ui!,capabilities:[]},tasks:[{task_id:'daily',name:'ignored',revision:4,enabled:true}]};
  const page=render(<TaskList snapshot={old} link={robotLink} workspace={workspace} />);
  expect(screen.getByText(/车端需升级/)).toBeTruthy();
  expect(screen.queryByText('ignored')).toBeNull();
  expect(screen.getByLabelText('任务名称')).toHaveProperty('readOnly',true);
  expect(screen.getByRole('button',{name:'新建任务'})).toHaveProperty('disabled',true);
  expect(screen.getByRole('button',{name:'提交任务'})).toHaveProperty('disabled',true);
  fireEvent.click(screen.getByRole('button',{name:'编辑 daily'}));
  await waitFor(()=>expect(screen.getByLabelText('内部编号')).toHaveProperty('value','daily'));
  fireEvent.click(screen.getByRole('button',{name:'提交任务'}));
  await screen.findByText('车端已确认保存任务。');
  const body=(vi.mocked(robotLink.request).mock.calls.at(-1)?.[1] as {body:object}).body;
  expect(body).not.toHaveProperty('name');
  page.rerender(<TaskList snapshot={{...old,tasks:[]}} link={robotLink} workspace={workspace} />);
  expect(screen.getByText(/车端需升级/)).toBeTruthy();
  page.rerender(<TaskList snapshot={{...snapshot,tasks:[]}} link={robotLink} workspace={workspace} />);
  expect(screen.getByLabelText('任务名称')).toHaveProperty('readOnly',false);
});

it("N10 trims names preserving internal spaces and identifies duplicate names by ID", async () => {
  const robotLink=link();
  render(<TaskList snapshot={{...snapshot,tasks:[{task_id:'one',name:'东区  巡检',revision:1,enabled:true},{task_id:'two',name:'东区  巡检',revision:1,enabled:true}]}} link={robotLink} workspace={workspace} />);
  expect(screen.getAllByText('东区  巡检', {exact:true,normalizer:value=>value})).toHaveLength(2);
  expect(screen.getByRole('button',{name:'编辑 one'})).toBeTruthy();
  expect(screen.getByRole('button',{name:'编辑 two'})).toBeTruthy();
  fireEvent.click(screen.getAllByRole('button',{name:'删除'})[1]!);
  await screen.findByText('车端已确认任务变更。');
  expect(robotLink.request).toHaveBeenCalledWith('task.delete',expect.objectContaining({body:{task_id:'two'}}));
  fireEvent.keyDown(screen.getByRole('button',{name:'添加地图航点 gate'}),{key:'Enter'});
  for (const value of ['   ', '\uFEFF', '\u0085']) {
    fireEvent.change(screen.getByLabelText('任务名称'), { target: { value } });
    expect(screen.getByRole('button', { name: '提交任务' })).toHaveProperty('disabled', true);
  }
  fireEvent.change(screen.getByLabelText('任务名称'),{target:{value:'  东区  巡检  '}});
  fireEvent.click(screen.getByRole('button',{name:'提交任务'}));
  await screen.findByText('车端已确认保存任务。');
  expect(robotLink.request).toHaveBeenCalledWith('task.create',expect.objectContaining({body:expect.objectContaining({name:'东区  巡检'})}));
});

it("rejects FEFF-only and NEL-only names from task.get", async () => {
  const robotLink=link();
  for (const name of ['\uFEFF', '\u0085']) {
    const details = { ...taskDetails, waypoint_ids: ['gate'], name };
    vi.mocked(robotLink.request).mockResolvedValue({ ok: true, data: details } as never);
    await expect(getTaskDetails(robotLink, snapshot.robot_id, snapshot.map_ref!, 'daily')).rejects.toThrow();
  }
});
