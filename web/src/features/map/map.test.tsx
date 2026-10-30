import { act, cleanup, fireEvent, render, screen, within } from "@testing-library/react";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { useState, type ReactElement } from "react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { CommandError, type RobotLink } from "../../api/client";
import type { LinkStatus, ResponsePayload, RobotSnapshot } from "../../api/protocol";
import { HomeControls } from "./HomeControls";
import {
  MapCanvas,
  fitView,
  gridToScreen,
  gridToWorld,
  screenToWorld,
  worldToGrid,
  worldToScreen,
  type MapView,
  type OccupancySnapshot,
  type Pose,
} from "./MapCanvas";
import { PosePicker, type MapGoal, type PoseHint } from "./PosePicker";

// jsdom has no PointerEvent, so testing-library would send a plain Event without coordinates.
if (typeof window.PointerEvent === "undefined") {
  class TestPointerEvent extends MouseEvent {
    readonly pointerId: number;
    constructor(type: string, init: PointerEventInit = {}) {
      super(type, init);
      this.pointerId = init.pointerId ?? 0;
    }
  }
  Object.defineProperty(window, "PointerEvent", { configurable: true, value: TestPointerEvent });
}

beforeEach(() => {
  // jsdom has no canvas; the drawing path is skipped, the SVG overlay and the maths are tested.
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null);
});

afterEach(cleanup);

// ---------------------------------------------------------------- fixtures

const MAP_REF = { map_id: "map-1", reference_revision: 2 };

const ROTATED: OccupancySnapshot = {
  map_ref: MAP_REF,
  origin: { x: 3, y: -2, yaw: Math.PI / 3 },
  resolution: 0.05,
  cells: [
    { cell: [10, 10], belief: 0.95, hit_count: 6, free_count: 0, change_source: "geometric_confirmation" },
    { cell: [11, 10], belief: 0.9, hit_count: 4, free_count: 0, change_source: "geometric_confirmation" },
    { cell: [12, 10], belief: 0.2, hit_count: 3, free_count: 5, change_source: "observed_free_space" },
    { cell: [13, 10], belief: 0.1, hit_count: 1, free_count: 7, change_source: "observed_free_space" },
    { cell: [14, 10], belief: 0.05, hit_count: 0, free_count: 9, change_source: "observed_free_space" },
  ],
};

function snapshot(overrides: Partial<RobotSnapshot> = {}): RobotSnapshot {
  return {
    robot_id: "robot-alpha",
    connected: true,
    localized: true,
    map_ref: MAP_REF,
    home: { waypoint_id: "home-1", map_ref: MAP_REF, status: "ready" },
    task_revision: 7,
    runnable_task_ids: [],
    execution_status: "idle",
    collected_at: 1_000,
    last_synced_at: 1_000,
    stale: false,
    ...overrides,
  };
}

class FakeLink implements RobotLink {
  sent: Array<{ type: string; payload: unknown }> = [];
  requestImpl: (type: string, payload: unknown) => Promise<unknown> = async (type) => ({
    request_type: type,
    ok: true,
    status: "succeeded",
  });
  async connect(): Promise<void> {}
  disconnect(): void {}
  request<T = ResponsePayload>(type: string, payload: unknown): Promise<T> {
    this.sent.push({ type, payload });
    return this.requestImpl(type, payload) as Promise<T>;
  }
  subscribe(): () => void {
    return () => undefined;
  }
  subscribeStatus(_listener: (status: LinkStatus) => void): () => void {
    return () => undefined;
  }
}

const flush = async (): Promise<void> => {
  await act(async () => {});
};

const VIEW: MapView = { center: { x: 3.4, y: -1.6 }, zoom: 120, width: 640, height: 420 };

function stubRect(element: HTMLElement): void {
  vi.spyOn(element, "getBoundingClientRect").mockReturnValue({
    left: 0,
    top: 0,
    right: 640,
    bottom: 420,
    width: 640,
    height: 420,
    x: 0,
    y: 0,
    toJSON: () => ({}),
  });
}

// ---------------------------------------------------------------- coordinates

describe("test_rotated_map_coordinates_roundtrip", () => {
  it("places grid cells by origin, resolution and yaw", () => {
    const quarter: OccupancySnapshot = { ...ROTATED, origin: { x: 1, y: 2, yaw: Math.PI / 2 }, resolution: 0.5 };
    // Two cells along grid x is one metre along the map's rotated x axis, which points at world +y.
    const world = gridToWorld(quarter, 2, 0);
    expect(world.x).toBeCloseTo(1);
    expect(world.y).toBeCloseTo(3);
    const grid = worldToGrid(quarter, world);
    expect(grid.x).toBeCloseTo(2);
    expect(grid.y).toBeCloseTo(0);
  });

  it("round-trips screen, world and grid on a rotated map", () => {
    for (const [sx, sy] of [
      [0, 0],
      [640, 420],
      [321.5, 77.25],
      [12, 400],
    ] as const) {
      const world = screenToWorld(VIEW, { x: sx, y: sy });
      const grid = worldToGrid(ROTATED, world);
      const back = gridToWorld(ROTATED, grid.x, grid.y);
      expect(back.x).toBeCloseTo(world.x, 9);
      expect(back.y).toBeCloseTo(world.y, 9);
      const screen = worldToScreen(VIEW, back);
      expect(screen.x).toBeCloseTo(sx, 6);
      expect(screen.y).toBeCloseTo(sy, 6);
    }
  });

  it("agrees with gridToScreen and flips the y axis for the screen", () => {
    const direct = gridToScreen(VIEW, ROTATED, 12.5, 10.5);
    const viaWorld = worldToScreen(VIEW, gridToWorld(ROTATED, 12.5, 10.5));
    expect(direct.x).toBeCloseTo(viaWorld.x);
    expect(direct.y).toBeCloseTo(viaWorld.y);
    // World +y is up on screen, so a larger y is a smaller screen y.
    expect(worldToScreen(VIEW, { x: 3.4, y: -1.0 }).y).toBeLessThan(
      worldToScreen(VIEW, { x: 3.4, y: -1.6 }).y,
    );
  });

  it("fits the occupied cells inside the viewport", () => {
    const view = fitView(ROTATED, 640, 420);
    for (const entry of ROTATED.cells) {
      const point = gridToScreen(view, ROTATED, entry.cell[0] + 0.5, entry.cell[1] + 0.5);
      expect(point.x).toBeGreaterThan(0);
      expect(point.x).toBeLessThan(640);
      expect(point.y).toBeGreaterThan(0);
      expect(point.y).toBeLessThan(420);
    }
  });

  it("turns a pick on the rotated map into a world pose, with heading from the drag", () => {
    const picks: Pose[] = [];
    render(<MapCanvas map={ROTATED} stale={false} view={VIEW} pickEnabled onPick={(p) => picks.push(p)} />);
    const surface = screen.getByRole("application", { name: "地图" });
    stubRect(surface);

    // Press at the view centre and drag straight up: heading is world +y (90 degrees).
    fireEvent.pointerDown(surface, { pointerId: 1, clientX: 320, clientY: 210 });
    fireEvent.pointerMove(surface, { pointerId: 1, clientX: 320, clientY: 110 });
    fireEvent.pointerUp(surface, { pointerId: 1, clientX: 320, clientY: 110 });

    expect(picks).toHaveLength(1);
    expect(picks[0]?.x).toBeCloseTo(VIEW.center.x);
    expect(picks[0]?.y).toBeCloseTo(VIEW.center.y);
    expect(picks[0]?.yaw).toBeCloseTo(Math.PI / 2);

    // Drag right: heading 0.
    fireEvent.pointerDown(surface, { pointerId: 2, clientX: 420, clientY: 210 });
    fireEvent.pointerMove(surface, { pointerId: 2, clientX: 520, clientY: 210 });
    fireEvent.pointerUp(surface, { pointerId: 2, clientX: 520, clientY: 210 });
    const expected = screenToWorld(VIEW, { x: 420, y: 210 });
    expect(picks[1]?.x).toBeCloseTo(expected.x);
    expect(picks[1]?.y).toBeCloseTo(expected.y);
    expect(picks[1]?.yaw).toBeCloseTo(0);
  });
});

// ---------------------------------------------------------------- canvas behaviour

describe("MapCanvas", () => {
  it("zooms and pans from buttons and the keyboard", () => {
    render(<MapCanvas map={ROTATED} stale={false} />);
    const surface = screen.getByRole("application", { name: "地图" });
    const zoom0 = Number(surface.getAttribute("data-zoom"));
    const x0 = Number(surface.getAttribute("data-center-x"));

    fireEvent.click(screen.getByRole("button", { name: "放大" }));
    expect(Number(surface.getAttribute("data-zoom"))).toBeGreaterThan(zoom0);
    fireEvent.click(screen.getByRole("button", { name: "缩小" }));
    expect(Number(surface.getAttribute("data-zoom"))).toBeCloseTo(zoom0);

    fireEvent.keyDown(surface, { key: "ArrowRight" });
    expect(Number(surface.getAttribute("data-center-x"))).toBeGreaterThan(x0);
    fireEvent.click(screen.getByRole("button", { name: "适应窗口" }));
    expect(Number(surface.getAttribute("data-center-x"))).toBeCloseTo(x0);
  });

  it("pans by dragging when picking is off", () => {
    render(<MapCanvas map={ROTATED} stale={false} view={VIEW} />);
    const surface = screen.getByRole("application", { name: "地图" });
    stubRect(surface);
    fireEvent.pointerDown(surface, { pointerId: 1, clientX: 300, clientY: 200 });
    fireEvent.pointerMove(surface, { pointerId: 1, clientX: 360, clientY: 200 });
    fireEvent.pointerUp(surface, { pointerId: 1, clientX: 360, clientY: 200 });
    // Dragging the map right moves the view centre left in the world.
    expect(Number(surface.getAttribute("data-center-x"))).toBeCloseTo(VIEW.center.x - 60 / VIEW.zoom);
  });

  it("compares raw and filtered occupancy", () => {
    render(<MapCanvas map={ROTATED} stale={false} />);
    // Raw: cells with any hit (4). Filtered: belief at or above 0.5 (2).
    const diagnostics = screen.getByRole("status", { name: "覆盖诊断" });
    expect(diagnostics.textContent).toContain("原始 4");
    expect(diagnostics.textContent).toContain("过滤 2");
    expect(diagnostics.textContent).toContain("被过滤掉 2");

    const surface = screen.getByRole("application", { name: "地图" });
    expect(surface.getAttribute("data-layer")).toBe("compare");
    fireEvent.click(screen.getByRole("radio", { name: "仅过滤层" }));
    expect(surface.getAttribute("data-layer")).toBe("filtered");
    fireEvent.click(screen.getByRole("radio", { name: "仅原始层" }));
    expect(surface.getAttribute("data-layer")).toBe("raw");
  });

  it("draws trajectory, safety zones, robot, home, goal and the draft", () => {
    render(
      <MapCanvas
        map={ROTATED}
        stale={false}
        view={VIEW}
        robot={{ x: 3.4, y: -1.6, yaw: 0 }}
        trajectory={[
          { x: 3.0, y: -2.0 },
          { x: 3.5, y: -1.5 },
        ]}
        zones={[
          {
            id: "z1",
            label: "禁行区 A",
            kind: "keep_out",
            points: [
              { x: 3, y: -2 },
              { x: 4, y: -2 },
              { x: 4, y: -1 },
            ],
          },
        ]}
        home={{ x: 3.1, y: -1.9, yaw: 0 }}
        goal={{ x: 3.6, y: -1.4, yaw: 1 }}
        draft={{ x: 3.2, y: -1.8, yaw: 0.5 }}
      />,
    );
    const overlay = screen.getByRole("img", { name: "地图叠加层" });
    expect(overlay.querySelector("polyline[data-layer='trajectory']")).not.toBeNull();
    expect(overlay.querySelector("polygon[data-zone='z1']")).not.toBeNull();
    expect(overlay.querySelector("[data-marker='robot']")).not.toBeNull();
    expect(overlay.querySelector("[data-marker='home']")).not.toBeNull();
    expect(overlay.querySelector("[data-marker='goal']")).not.toBeNull();
    expect(overlay.querySelector("[data-marker='draft']")).not.toBeNull();
    expect(within(overlay).getByText("禁行区 A")).toBeTruthy();
  });

  it("marks a cached map as not live and refuses picks on it", () => {
    const picks: Pose[] = [];
    render(<MapCanvas map={ROTATED} stale view={VIEW} pickEnabled onPick={(p) => picks.push(p)} />);
    expect(screen.getByText(/缓存地图/)).toBeTruthy();
    const surface = screen.getByRole("application", { name: "地图" });
    expect(surface.getAttribute("data-stale")).toBe("true");
    stubRect(surface);
    fireEvent.pointerDown(surface, { pointerId: 1, clientX: 320, clientY: 210 });
    fireEvent.pointerUp(surface, { pointerId: 1, clientX: 320, clientY: 210 });
    expect(picks).toEqual([]);
  });
});

// ---------------------------------------------------------------- goal gating

function Harness({
  snap,
  map,
  onGoal,
  onHint,
}: {
  snap: RobotSnapshot;
  map: OccupancySnapshot;
  onGoal: (goal: MapGoal) => void;
  onHint: (hint: PoseHint) => void;
}): ReactElement {
  const [draft, setDraft] = useState<Pose | null>(null);
  return (
    <>
      <MapCanvas map={map} stale={snap.stale} view={VIEW} pickEnabled draft={draft} onPick={setDraft} />
      <PosePicker snapshot={snap} map={map} draft={draft} onDraftChange={setDraft} onGoal={onGoal} onHint={onHint} />
    </>
  );
}

function setField(name: string, value: string): void {
  fireEvent.change(screen.getByLabelText(name), { target: { value } });
}

describe("PosePicker", () => {
  it("test_stale_map_blocks_goal", () => {
    const onGoal = vi.fn();
    render(<Harness snap={snapshot({ stale: true, connected: false })} map={ROTATED} onGoal={onGoal} onHint={vi.fn()} />);
    setField("X (m)", "3.2");
    setField("Y (m)", "-1.8");
    setField("朝向 (°)", "30");
    const confirm = screen.getByRole("button", { name: "确认目标" }) as HTMLButtonElement;
    expect(confirm.disabled).toBe(true);
    expect(screen.getByText(/缓存地图.*不能下达目标/)).toBeTruthy();
    fireEvent.click(confirm);
    expect(onGoal).not.toHaveBeenCalled();
  });

  it("blocks a goal when the map on screen is not the map the robot reports", () => {
    const onGoal = vi.fn();
    const other = { ...ROTATED, map_ref: { map_id: "map-1", reference_revision: 1 } };
    render(<Harness snap={snapshot()} map={other} onGoal={onGoal} onHint={vi.fn()} />);
    setField("X (m)", "3.2");
    setField("Y (m)", "-1.8");
    expect((screen.getByRole("button", { name: "确认目标" }) as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByText(/地图版本与车端不一致/)).toBeTruthy();
  });

  it("emits a goal that carries the map reference", () => {
    const onGoal = vi.fn();
    render(<Harness snap={snapshot()} map={ROTATED} onGoal={onGoal} onHint={vi.fn()} />);
    expect((screen.getByRole("button", { name: "确认目标" }) as HTMLButtonElement).disabled).toBe(true);
    setField("X (m)", "3.2");
    setField("Y (m)", "-1.8");
    setField("朝向 (°)", "90");
    fireEvent.click(screen.getByRole("button", { name: "确认目标" }));
    expect(onGoal).toHaveBeenCalledTimes(1);
    const goal = onGoal.mock.calls[0]?.[0] as MapGoal;
    expect(goal.map_ref).toEqual(MAP_REF);
    expect(goal.pose.x).toBeCloseTo(3.2);
    expect(goal.pose.y).toBeCloseTo(-1.8);
    expect(goal.pose.yaw).toBeCloseTo(Math.PI / 2);
  });

  it("fills the fields from a pick on the map", () => {
    render(<Harness snap={snapshot()} map={ROTATED} onGoal={vi.fn()} onHint={vi.fn()} />);
    const surface = screen.getByRole("application", { name: "地图" });
    stubRect(surface);
    fireEvent.pointerDown(surface, { pointerId: 1, clientX: 320, clientY: 210 });
    fireEvent.pointerMove(surface, { pointerId: 1, clientX: 320, clientY: 110 });
    fireEvent.pointerUp(surface, { pointerId: 1, clientX: 320, clientY: 110 });
    expect((screen.getByLabelText("X (m)") as HTMLInputElement).value).toBe(VIEW.center.x.toFixed(2));
    expect((screen.getByLabelText("朝向 (°)") as HTMLInputElement).value).toBe("90.0");
  });

  it("rejects non-numeric input instead of sending NaN", () => {
    const onGoal = vi.fn();
    render(<Harness snap={snapshot()} map={ROTATED} onGoal={onGoal} onHint={vi.fn()} />);
    setField("X (m)", "abc");
    setField("Y (m)", "1");
    expect((screen.getByRole("button", { name: "确认目标" }) as HTMLButtonElement).disabled).toBe(true);
  });

  it("offers a localization hint while not localized, with the map reference", () => {
    const onHint = vi.fn();
    const onGoal = vi.fn();
    render(<Harness snap={snapshot({ localized: false })} map={ROTATED} onGoal={onGoal} onHint={onHint} />);
    // Goals need a localized robot; a hint is how it becomes localized.
    setField("X (m)", "3.2");
    setField("Y (m)", "-1.8");
    expect((screen.getByRole("button", { name: "确认目标" }) as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByText(/定位未完成.*不能下达目标/)).toBeTruthy();

    fireEvent.click(screen.getByRole("radio", { name: "重定位提示" }));
    const submit = screen.getByRole("button", { name: "提交定位提示" }) as HTMLButtonElement;
    expect(submit.disabled).toBe(false);
    fireEvent.click(submit);
    expect(onGoal).not.toHaveBeenCalled();
    const hint = onHint.mock.calls[0]?.[0] as PoseHint;
    expect(hint.map_ref).toEqual(MAP_REF);
    expect(hint.source).toBe("manual");
    expect(hint.pose?.x).toBeCloseTo(3.2);
  });

  it("blocks a hint on a cached map", () => {
    const onHint = vi.fn();
    render(
      <Harness snap={snapshot({ stale: true, connected: false, localized: false })} map={ROTATED} onGoal={vi.fn()} onHint={onHint} />,
    );
    fireEvent.click(screen.getByRole("radio", { name: "重定位提示" }));
    setField("X (m)", "3.2");
    setField("Y (m)", "-1.8");
    const submit = screen.getByRole("button", { name: "提交定位提示" }) as HTMLButtonElement;
    expect(submit.disabled).toBe(true);
    fireEvent.click(submit);
    expect(onHint).not.toHaveBeenCalled();
  });
});

// ---------------------------------------------------------------- home and relocalization

describe("HomeControls", () => {
  it("test_home_unset_and_localization_pending_disable_navigation", async () => {
    const link = new FakeLink();
    const { rerender } = render(<HomeControls snapshot={snapshot({ home: null })} link={link} />);
    const start = () => screen.getByRole("button", { name: "开始导航" }) as HTMLButtonElement;
    const resume = () => screen.getByRole("button", { name: "继续导航" }) as HTMLButtonElement;

    expect(start().disabled).toBe(true);
    expect(resume().disabled).toBe(true);
    expect(screen.getByText(/Home 未设置/)).toBeTruthy();
    fireEvent.click(start());
    expect(link.sent).toEqual([]);

    rerender(<HomeControls snapshot={snapshot({ localized: false })} link={link} />);
    expect(start().disabled).toBe(true);
    expect(resume().disabled).toBe(true);
    expect(screen.getByText(/导航需要先完成定位/)).toBeTruthy();
    expect(screen.getByRole("status", { name: "定位状态" }).textContent).toContain("重新定位未完成");

    rerender(<HomeControls snapshot={snapshot({ stale: true, connected: false })} link={link} />);
    expect(start().disabled).toBe(true);
    expect(screen.getByText(/缓存快照/)).toBeTruthy();

    rerender(<HomeControls snapshot={snapshot()} link={link} />);
    expect(start().disabled).toBe(false);
    fireEvent.click(start());
    await flush();
    expect(link.sent).toEqual([{ type: "navigation.start", payload: {} }]);
  });

  it("blocks navigation when Home belongs to another map", () => {
    const home = { waypoint_id: "home-1", map_ref: { map_id: "map-9", reference_revision: 1 }, status: "ready" };
    render(<HomeControls snapshot={snapshot({ home })} link={new FakeLink()} />);
    expect((screen.getByRole("button", { name: "开始导航" }) as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByText(/Home 不属于当前地图/)).toBeTruthy();
  });

  it("keeps cancel available while navigation start is blocked", async () => {
    const link = new FakeLink();
    render(<HomeControls snapshot={snapshot({ home: null, localized: false })} link={link} />);
    const cancel = screen.getByRole("button", { name: "取消导航" }) as HTMLButtonElement;
    expect(cancel.disabled).toBe(false);
    fireEvent.click(cancel);
    await flush();
    expect(link.sent).toEqual([{ type: "navigation.cancel", payload: {} }]);
  });

  it("shows no success for a command the robot rejected", async () => {
    const link = new FakeLink();
    link.requestImpl = () => Promise.reject(new CommandError("safety_rejected", "no"));
    render(<HomeControls snapshot={snapshot()} link={link} />);
    fireEvent.click(screen.getByRole("button", { name: "开始导航" }));
    await flush();
    const alert = screen.getByRole("alert");
    expect(alert.textContent).toContain("安全保护拒绝");
    expect(screen.queryByText(/车端已应答/)).toBeNull();
  });

  it("sets Home through an operation, only when localized, and confirms only on ok", async () => {
    const link = new FakeLink();
    const { rerender } = render(<HomeControls snapshot={snapshot({ home: null, localized: false })} link={link} />);
    expect((screen.getByRole("button", { name: "设为 Home" }) as HTMLButtonElement).disabled).toBe(true);

    rerender(<HomeControls snapshot={snapshot({ home: null })} link={link} />);
    fireEvent.click(screen.getByRole("button", { name: "设为 Home" }));
    await flush();
    expect(link.sent).toHaveLength(1);
    expect(link.sent[0]?.type).toBe("home.set");
    const payload = link.sent[0]?.payload as { operation_id: string; body: unknown };
    expect(payload.operation_id).toMatch(/\S/);
    expect(payload.body).toEqual({});
    expect(screen.getByText(/车端已应答/)).toBeTruthy();
  });

  it("asks before replacing an existing Home", async () => {
    const link = new FakeLink();
    render(<HomeControls snapshot={snapshot()} link={link} />);
    fireEvent.click(screen.getByRole("button", { name: "设为 Home" }));
    expect(link.sent).toEqual([]);
    fireEvent.click(screen.getByRole("button", { name: "确认替换 Home" }));
    await flush();
    expect(link.sent.map((entry) => entry.type)).toEqual(["home.set"]);
  });

  it("shows Home and localization state, and a missing snapshot as unknown", () => {
    const { rerender } = render(<HomeControls snapshot={snapshot()} link={new FakeLink()} />);
    expect(screen.getByText(/home-1/)).toBeTruthy();
    expect(screen.getByRole("status", { name: "定位状态" }).textContent).toContain("已定位");
    rerender(<HomeControls snapshot={null} link={new FakeLink()} />);
    expect(screen.getByText(/等待车端快照/)).toBeTruthy();
    expect((screen.getByRole("button", { name: "开始导航" }) as HTMLButtonElement).disabled).toBe(true);
  });
});

// ---------------------------------------------------------------- layout

describe("map layout", () => {
  it("keeps the map usable in landscape and the controls large", () => {
    const css = readFileSync(resolve(__dirname, "Map.module.css"), "utf8");
    expect(css).toMatch(/@media[^{]*orientation:\s*landscape/);
    expect(css).toMatch(/touch-action:\s*none/);
    expect(css).toMatch(/min-height:\s*var\(--touch\)/);
    expect(css).toMatch(/:focus-visible/);
  });
});
