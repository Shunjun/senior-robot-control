import { act, cleanup, render, screen, fireEvent } from "@testing-library/react";
import { afterEach, expect, it, vi } from "vitest";
import { parseInbound } from "../../api/protocol";
import { ControlSurface } from "./ControlSurface";
import type { RobotSnapshot } from "../../api/protocol";
import { testPort } from "../../api/vehicleState.testUtils";
import type { RobotLink } from "../../api/client";

afterEach(() => { cleanup(); vi.useRealTimers(); vi.restoreAllMocks(); });

it("shows received resource counters and expires an unchanged sample even while connected", async () => {
  vi.useFakeTimers();
  vi.spyOn(HTMLCanvasElement.prototype,"getContext").mockReturnValue(null);
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
  const { rerender } = render(<ControlSurface robotId="robot" link={link} snapshot={snapshot} statusOpen />);
  fireEvent.click(screen.getByRole("button", { name:"性能" }));
  expect(screen.getByText("25.0%")).toBeTruthy();
  expect(screen.getByText("512 MiB")).toBeTruthy();
  expect(screen.getByText("未启动")).toBeTruthy();
  act(() => vi.advanceTimersByTime(3501));
  expect(screen.getByText("样本已过期")).toBeTruthy();
  rerender(<ControlSurface robotId="robot" link={link} snapshot={snapshot} statusOpen={false} />);
  rerender(<ControlSurface robotId="robot" link={link} snapshot={snapshot} statusOpen />);
  expect(screen.getByText("样本已过期")).toBeTruthy();
  expect(screen.queryByText("25.0%")).toBeNull();
  fireEvent.click(screen.getByRole("button", { name: "地图管理" }));
  await act(async () => { await Promise.resolve(); });
  expect(request).not.toHaveBeenCalled();
  rerender(<ControlSurface robotId="robot" link={link} snapshot={{ ...snapshot, connected: false }} statusOpen />);
  expect(screen.getByText("离线 · 最后样本")).toBeTruthy();
});

it('uses null then a new CPU sample, leaves absent roll/pitch unavailable, and expires the unchanged sequence', async () => {
  vi.useFakeTimers();
  vi.spyOn(HTMLCanvasElement.prototype,"getContext").mockReturnValue(null);
  const port=testPort();
  const link:RobotLink={vehicle:port,connect:async()=>{},disconnect(){},request:(async()=>{throw new Error('No fixture map');}) as RobotLink['request'],subscribe:()=>()=>{},subscribeStatus:()=>()=>{}};
  const snapshot:RobotSnapshot={robot_id:'robot',connected:true,localized:true,map_ref:{map_id:'map',reference_revision:1},map_generation:'reference',map_grid_id:'raster',home:null,home_revision:0,task_revision:0,runnable_task_ids:[],execution_status:'idle',collected_at:1,last_synced_at:1,stale:false,robot_pose:{x:0,y:0,yaw:.5},performance:{sequence:1,sampled_at:1,interval_s:1,scope:'procfs_visible',cpu_count:4,cpu_percent:null,memory_used_bytes:null,memory_total_bytes:null,modules:[]}};
  const page=render(<ControlSurface robotId="robot" link={link} snapshot={snapshot} statusOpen />);
  await act(async()=>{await Promise.resolve();});
  expect(screen.getByText('28.6°')).toBeTruthy();
  expect(screen.getByText('Roll 横滚').nextElementSibling?.textContent).toBe('不可用');
  expect(screen.getByText('Pitch 俯仰').nextElementSibling?.textContent).toBe('不可用');
  expect(screen.getByText('整机 CPU').nextElementSibling?.textContent).toBe('不可用');
  const next={...snapshot,performance:{...snapshot.performance!,sequence:2,cpu_percent:25}};
  page.rerender(<ControlSurface robotId="robot" link={link} snapshot={next} statusOpen />);
  expect(screen.getByText('25.0%')).toBeTruthy();
  act(()=>vi.advanceTimersByTime(3501));
  expect(screen.getByText('整机 CPU').nextElementSibling?.textContent).toBe('不可用');
  page.rerender(<ControlSurface robotId="robot" link={link} snapshot={next} statusOpen={false} />);
  page.rerender(<ControlSurface robotId="robot" link={link} snapshot={next} statusOpen />);
  expect(screen.queryByText('25.0%')).toBeNull();
});
