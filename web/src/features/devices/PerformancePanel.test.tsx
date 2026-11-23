import { act, cleanup, render, screen, fireEvent } from "@testing-library/react";
import { afterEach, expect, it, vi } from "vitest";
import { parseInbound } from "../../api/protocol";
import { ControlSurface } from "./ControlSurface";
import type { RobotLink } from "../../api/client";

afterEach(() => { cleanup(); vi.useRealTimers(); });

it("shows received resource counters and expires an unchanged sample even while connected", async () => {
  vi.useFakeTimers();
  const frame = parseInbound(JSON.stringify({
    v: 1, robot_id: "robot", session_id: "cloud", connection_epoch: 1,
    request_id: "resources", seq: 1, expires_at: Date.now() / 1000 + 10, type: "snapshot",
    payload: { connected: true, localized: false, map_ref: null, home: null,
      task_revision: 0, runnable_task_ids: [], execution_status: "idle", collected_at: 1000,
      stale: false, last_synced_at: 1000, home_revision: 0,
      performance: { sequence: 1, sampled_at: -1000, interval_s: 1, scope: "procfs_visible",
        cpu_count: 4, cpu_percent: 25, memory_total_bytes: 1073741824, memory_used_bytes: 536870912,
        modules: [{ name: "video", status: "inactive", cpu_percent: null, rss_bytes: null, process_count: 0 }] },
    },
  }));
  if (frame?.type !== "snapshot") throw new Error("Resource snapshot did not parse");
  const reading = frame.payload.performance;
  expect(reading?.sequence).toBe(1);
  const request = vi.fn(async () => { throw new Error("Unexpected command"); });
  const link: RobotLink = { connect: async () => {}, disconnect: () => {},
    request,
    subscribe: () => () => {}, subscribeStatus: () => () => {} };
  const snapshot = { ...frame.payload, robot_id: "robot" };
  const { rerender } = render(<ControlSurface robotId="robot" link={link} snapshot={snapshot} />);
  expect(screen.getByText("25.0%")).toBeTruthy();
  expect(screen.getByText("512 MiB")).toBeTruthy();
  expect(screen.getByText("未启动")).toBeTruthy();
  act(() => vi.advanceTimersByTime(3501));
  expect(screen.getByText("样本已过期")).toBeTruthy();
  fireEvent.click(screen.getByRole("button", { name: "地图管理" }));
  await act(async () => { await Promise.resolve(); });
  expect(request).not.toHaveBeenCalled();
  rerender(<ControlSurface robotId="robot" link={link} snapshot={{ ...snapshot, connected: false }} />);
  expect(screen.getByText("离线 · 最后样本")).toBeTruthy();
});
