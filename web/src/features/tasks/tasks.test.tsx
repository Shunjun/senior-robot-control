import { act, cleanup, fireEvent, render, renderHook, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { useState } from "react";

import { CommandError, type RobotLink } from "../../api/client";
import type { LinkStatus, RobotSnapshot } from "../../api/protocol";
import { useWorkspaceMap } from "../map/useWorkspaceMap";

import { draftSchedule, NEW_TASK, ScheduleEditor, type TaskDraft } from "./ScheduleEditor";
import { TaskList, type TaskWorkspaceData } from "./TaskList";

beforeEach(() => vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null));
afterEach(() => { cleanup(); vi.restoreAllMocks(); });

const snapshot: RobotSnapshot = {
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
    fireEvent.change(screen.getByLabelText("任务编号"), { target: { value: "daily" } });
    expect(draftSchedule(changed.at(-1)!)).toEqual({ kind: "interval", every_seconds: 86400 });
    expect(changed.at(-1)?.timezone).toBe("Asia/Shanghai");

    fireEvent.change(screen.getByLabelText("计划类型"), { target: { value: "cron" } });
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
    fireEvent.change(screen.getByLabelText("任务编号"), { target: { value: "evening" } });
    fireEvent.keyDown(screen.getByRole("button", { name: "添加地图航点 gate" }), { key: "Enter" });
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await screen.findByText("车端已确认保存任务。");
    const created = vi.mocked(robotLink.request).mock.calls[0]?.[1] as {
      body: { waypoint_ids: string[] };
    };
    expect(created.body.waypoint_ids).toEqual(["gate"]);

    fireEvent.click(screen.getByRole("button", { name: "暂停" }));
    await waitFor(() => expect(vi.mocked(robotLink.request).mock.calls).toHaveLength(2));
    fireEvent.click(screen.getByRole("button", { name: "启用" }));
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
const map = { map_ref: snapshot.map_ref!, origin: { x: 0, y: 0, yaw: 0 }, resolution: 1, cells: [] };
const workspace: TaskWorkspaceData = {
  grid_id: "grid-1",
  map,
  waypoints: ["gate", "hall", "dock"].map((waypoint_id, x) => ({
    waypoint_id, x, y: 0, yaw: 0, status: "active", kind: "waypoint",
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
      waypointIds = (payload as { body: { waypoint_ids: string[] } }).body.waypoint_ids;
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
    vi.mocked(robotLink.request).mockImplementation(() => new Promise((resolve) => { resolveRead = resolve; }));
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
    if (type === "map.manifest") return { data: {
      ...identity, width: 33, height: 2, resolution: 1, origin: map.origin, tile_size: 32,
    } } as never;
    if (type === "map.tile") {
      const x = (payload as { tile_x: number }).tile_x;
      return { data: {
        ...identity, tile_x: x, tile_y: 0, width: x === 0 ? 32 : 1, height: 2,
        cells: x === 0 ? [...Array(32).fill(0), ...Array(32).fill(-1)] : [5, 75],
      } } as never;
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
