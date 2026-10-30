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
    ...overrides,
  };
}

function fakeLink() {
  let status: ((value: LinkStatus) => void) | null = null;
  const link: RobotLink = {
    connect: async () => undefined,
    disconnect: () => undefined,
    request: vi.fn(async (_type: string, payload: unknown) => ({
      request_type: "map.clear",
      ok: true,
      operation_id: (payload as { operation_id: string }).operation_id,
      status: "succeeded",
    })) as unknown as RobotLink["request"],
    subscribe: () => () => undefined,
    subscribeStatus: (listener) => {
      status = listener;
      return () => undefined;
    },
  };
  return { link, open: () => status?.({ state: "open", robotId: "robot-alpha", closeCode: null }) };
}

describe("map lifecycle", () => {
  it("test_clear_requires_named_robot_confirmation", async () => {
    const { link } = fakeLink();
    render(<MapManager robotId="robot-alpha" snapshot={snapshot()} link={link} />);
    fireEvent.click(screen.getByRole("button", { name: "清除地图" }));
    expect(link.request).not.toHaveBeenCalled();
    fireEvent.change(screen.getByLabelText("输入机器人编号确认"), { target: { value: "robot-alpha" } });
    fireEvent.click(screen.getByRole("button", { name: "清除地图" }));
    await waitFor(() => expect(link.request).toHaveBeenCalledWith("map.clear", expect.any(Object)));
  });

  it("test_import_waits_for_localization", () => {
    const { link } = fakeLink();
    render(<MapManager robotId="robot-alpha" snapshot={snapshot({ localized: false })} link={link} />);
    expect((screen.getByRole("button", { name: "导入地图包" }) as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByText(/完成定位后才可导入/)).toBeTruthy();
  });

  it("test_inflight_operation_recovered_after_reconnect", async () => {
    const { link, open } = fakeLink();
    (link.request as ReturnType<typeof vi.fn>)
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
    await waitFor(() => expect(link.request).toHaveBeenCalledTimes(1));
    open();
    await waitFor(() =>
      expect(link.request).toHaveBeenLastCalledWith("operation.query", {
        operation_id: "op-export",
        body: {},
      }),
    );
  });
});
