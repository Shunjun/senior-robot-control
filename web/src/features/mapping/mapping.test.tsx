import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

import type { RobotLink } from "../../api/client";
import type { LinkStatus, RobotSnapshot } from "../../api/protocol";
import { MapManager } from "./MapManager";

afterEach(cleanup);

function snapshot(overrides: Partial<RobotSnapshot> = {}): RobotSnapshot {
  return {
    robot_id: "robot-alpha",
    connected: true,
    localized: true,
    map_ref: { map_id: "map-1", reference_revision: 2 },
    home: null,
    task_revision: 7,
    runnable_task_ids: [],
    execution_status: "idle",
    collected_at: 1,
    stale: false,
    last_synced_at: 1,
    home_revision: 0,
    ...overrides,
  };
}

function fakeLink() {
  let status: ((value: LinkStatus) => void) | null = null;
  const requests = vi.fn(async (_type: string, payload: unknown) => ({
    request_type: "map.clear", ok: true,
    operation_id: (payload as { operation_id: string }).operation_id, status: "succeeded",
  }));
  const link: RobotLink = {
    connect: async () => undefined,
    disconnect: () => undefined,
    request: requests as RobotLink["request"],
    subscribe: () => () => undefined,
    subscribeStatus: (listener) => {
      status = listener;
      return () => undefined;
    },
  };
  return { link, requests, open: () => status?.({ state: "open", robotId: "robot-alpha", closeCode: null }) };
}

describe("map lifecycle", () => {
  it("test_clear_requires_named_robot_confirmation", async () => {
    const { link, requests } = fakeLink();
    render(<MapManager robotId="robot-alpha" snapshot={snapshot()} link={link} />);
    fireEvent.click(screen.getByRole("button", { name: "清除地图" }));
    expect(requests).not.toHaveBeenCalled();
    fireEvent.change(screen.getByLabelText("输入机器人编号确认"), { target: { value: "robot-alpha" } });
    fireEvent.click(screen.getByRole("button", { name: "清除地图" }));
    await waitFor(() => expect(requests).toHaveBeenCalledWith("map.clear", expect.any(Object)));
  });

  it("test_import_waits_for_localization", () => {
    const { link } = fakeLink();
    render(<MapManager robotId="robot-alpha" snapshot={snapshot({ localized: false })} link={link} />);
    expect((screen.getByRole("button", { name: "导入地图包" }) as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByText(/完成定位后才可导入/)).toBeTruthy();
  });

  it("test_inflight_operation_recovered_after_reconnect", async () => {
    const { link, requests, open } = fakeLink();
    (requests as ReturnType<typeof vi.fn>)
      .mockResolvedValueOnce({
        request_type: "map.export",
        ok: true,
        operation_id: "op-export",
        status: "running",
      })
      .mockResolvedValueOnce({
        request_type: "operation.query",
        ok: true,
        operation_id: "op-export",
        status: "succeeded",
      });
    render(<MapManager robotId="robot-alpha" snapshot={snapshot()} link={link} />);
    fireEvent.change(screen.getByLabelText("地图包名称"), { target: { value: "backup.zip" } });
    fireEvent.click(screen.getByRole("button", { name: "导出地图包" }));
    await waitFor(() => expect(requests).toHaveBeenCalledTimes(1));
    open();
    await waitFor(() =>
      expect(requests).toHaveBeenLastCalledWith("operation.query", {
        operation_id: "op-export",
        body: {},
      }),
    );
  });
});

it("uses the live catalog for lifecycle operations and discards replies from the previous robot", async () => {
  const { ControlSurface } = await import("../devices/ControlSurface");
  const ref = (map_id: string) => ({ map_id, reference_revision: 1 });
  const row = (map_id: string, name: string) => ({ map_ref: ref(map_id), name,
    checkpoint_id: "a".repeat(32), home_id: `home-${map_id}`, version: 1, saved_at: 1000 });
  let data = { robot_id: "robot-alpha", entries: [row("a", "仓库"), row("b", "院子")],
    selection: { map_ref: ref("a"), generation: 7, state: "ready" },
    runtime_available: true, phase: "mapping" };
  let delayed: ((value: unknown) => void) | undefined;
  let delayNext = false;
  const request = vi.fn(async (type: string, payload: unknown): Promise<unknown> => {
    if (type === "map.catalog") {
      if (delayNext) { delayNext = false; return new Promise(resolve => { delayed = resolve; }); }
      return { request_type: type, ok: true, data };
    }
    const body = payload as { operation_id: string; body: { name?: string; map_id?: string; expected_generation: number } };
    if (type === "map.create") {
      expect(body.body.expected_generation).toBe(7);
      data = { ...data, entries: [...data.entries, row("c", body.body.name!)],
        selection: { map_ref: ref("c"), generation: 8, state: "ready" } };
    } else if (type === "map.activate") {
      expect(body.body).toEqual({ ...ref("a"), expected_generation: 8 });
      data = { ...data, selection: { map_ref: ref("a"), generation: 9, state: "pending_calibration" }, phase: "pending_calibration" };
    } else if (type === "map.delete") {
      expect(body.body).toEqual({ ...ref("b"), expected_generation: 9 });
      data = { ...data, entries: data.entries.filter(entry => entry.map_ref.map_id !== "b") };
    } else throw new Error("No map raster in this fixture");
    return { request_type: type, ok: true, operation_id: body.operation_id, status: "succeeded" };
  });
  const { link } = fakeLink();
  link.request = request as RobotLink["request"];
  const { rerender } = render(<ControlSurface robotId="robot-alpha" snapshot={snapshot({ mapping_protocol_version: 1 })} link={link} />);
  fireEvent.click(screen.getByRole("button", { name: "地图管理" }));
  await screen.findByText("仓库");
  fireEvent.change(screen.getByLabelText("新地图名称"), { target: { value: "车间" } });
  fireEvent.click(screen.getByRole("button", { name: "新建并开始建图" }));
  await screen.findByText("车间");
  const { within, act } = await import("@testing-library/react");
  fireEvent.click(within(screen.getByRole("group", { name: "仓库地图" })).getByRole("button", { name: "切换" }));
  await screen.findByText("等待校准车辆位置");
  fireEvent.click(within(screen.getByRole("group", { name: "院子地图" })).getByRole("button", { name: "删除" }));
  fireEvent.click(screen.getByRole("button", { name: "确认删除" }));
  await waitFor(() => expect(screen.queryByText("院子")).toBeNull());
  delayNext = true;
  fireEvent.click(screen.getByRole("button", { name: "刷新地图目录" }));
  await waitFor(() => expect(delayed).toBeDefined());
  const { link: other } = fakeLink();
  other.request = vi.fn(async () => ({ ok: true, request_type: "map.catalog", data: {
    ...data, robot_id: "robot-beta", entries: [row("d", "乙车地图")],
    selection: { map_ref: ref("d"), generation: 1, state: "ready" },
  } })) as RobotLink["request"];
  rerender(<ControlSurface robotId="robot-beta" snapshot={snapshot({ robot_id: "robot-beta", mapping_protocol_version: 1 })} link={other} />);
  await screen.findByText("乙车地图");
  await act(async () => { delayed!({ ok: true, request_type: "map.catalog", data }); });
  expect(screen.queryByText("仓库")).toBeNull();
  expect(screen.getByText("乙车地图")).toBeTruthy();
});
