import { act, cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { useEffect, type ReactElement } from "react";
import { MemoryRouter } from "react-router-dom";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { App } from "../../App";
import {
  CommandError,
  type ControlApi,
  type RobotLink,
} from "../../api/client";
import type { LinkStatus, ResponsePayload, RobotSnapshot, RobotSummary } from "../../api/protocol";
import {
  ASSIST_SCALE,
  DriveController,
  DriveView,
  MAX_ANGULAR,
  MAX_LINEAR,
  type DriveEvent,
} from "./DriveView";
import { EmergencyStop } from "./EmergencyStop";

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

afterEach(() => {
  cleanup();
  vi.useRealTimers();
  Object.defineProperty(document, "visibilityState", { configurable: true, value: "visible" });
});

// ---------------------------------------------------------------- fakes

function snapshot(robotId: string, overrides: Partial<RobotSnapshot> = {}): RobotSnapshot {
  return {
    robot_id: robotId,
    connected: true,
    localized: true,
    map_ref: { map_id: "map-1", reference_revision: 2 },
    home: null,
    task_revision: 7,
    runnable_task_ids: [],
    execution_status: "idle",
    collected_at: 1_000,
    last_synced_at: 1_000,
    stale: false,
    ...overrides,
  };
}

interface Sent {
  type: string;
  payload: unknown;
  /** The robot the link pointed at when the frame left. */
  robot: string | null;
}

class FakeLink implements RobotLink {
  current: string | null = "robot-alpha";
  sent: Sent[] = [];
  requestImpl: (type: string, payload: unknown) => Promise<unknown> = async (type) => ({
    request_type: type,
    ok: true,
  });
  async connect(robotId: string): Promise<void> {
    this.current = robotId;
  }
  disconnect(): void {
    this.current = null;
  }
  request<T = ResponsePayload>(type: string, payload: unknown): Promise<T> {
    this.sent.push({ type, payload, robot: this.current });
    return this.requestImpl(type, payload) as Promise<T>;
  }
  subscribe(): () => void {
    return () => undefined;
  }
  subscribeStatus(_listener: (status: LinkStatus) => void): () => void {
    return () => undefined;
  }
  types(): string[] {
    return this.sent.map((entry) => entry.type);
  }
  velocities(): Array<{ vx: number; wz: number }> {
    return this.sent
      .filter((entry) => entry.type === "drive.velocity")
      .map((entry) => entry.payload as { vx: number; wz: number });
  }
}

const flush = async (): Promise<void> => {
  await act(async () => {});
};

const tick = async (ms: number): Promise<void> => {
  await act(async () => {
    await vi.advanceTimersByTimeAsync(ms);
  });
};

function stick(name: string): HTMLElement {
  const element = screen.getByRole("group", { name });
  vi.spyOn(element, "getBoundingClientRect").mockReturnValue({
    left: 0,
    top: 0,
    right: 200,
    bottom: 200,
    width: 200,
    height: 200,
    x: 0,
    y: 0,
    toJSON: () => ({}),
  });
  return element;
}

const FORWARD = "前进后退摇杆";
const TURN = "左转右转摇杆";

async function acquire(): Promise<void> {
  fireEvent.click(screen.getByRole("button", { name: "获取控制权" }));
  await flush();
}

async function mountDrive(link: FakeLink, snap: RobotSnapshot = snapshot("robot-alpha")) {
  const view = render(<DriveView robotId="robot-alpha" link={link} snapshot={snap} />);
  await acquire();
  return view;
}

/** Full forward: the top edge of a 200px stick. */
function pushForward(element: HTMLElement, pointerId = 1): void {
  fireEvent.pointerDown(element, { pointerId, clientX: 100, clientY: 0 });
}

// ---------------------------------------------------------------- stop paths

describe("hold-to-drive stops", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  it("test_pointer_cancel_and_blur_stop", async () => {
    const link = new FakeLink();
    await mountDrive(link);
    const forward = stick(FORWARD);

    // Cancel (the browser took the pointer, e.g. a system gesture).
    pushForward(forward);
    await flush();
    expect(link.velocities()[0]).toEqual({ vx: MAX_LINEAR, wz: 0 });
    fireEvent.pointerCancel(forward, { pointerId: 1 });
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");
    const afterCancel = link.sent.length;
    await tick(1_000);
    expect(link.sent).toHaveLength(afterCancel);

    // The window loses focus while held.
    pushForward(forward);
    await flush();
    fireEvent.blur(window);
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");
    const afterBlur = link.sent.length;
    await tick(1_000);
    expect(link.sent).toHaveLength(afterBlur);

    // The page is hidden while held.
    pushForward(forward);
    await flush();
    Object.defineProperty(document, "visibilityState", { configurable: true, value: "hidden" });
    fireEvent(document, new Event("visibilitychange"));
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");
    const afterHide = link.sent.length;
    await tick(1_000);
    expect(link.sent).toHaveLength(afterHide);
  });

  it("stops when the stick loses focus and when the pointer lifts", async () => {
    const link = new FakeLink();
    await mountDrive(link);
    const forward = stick(FORWARD);

    fireEvent.keyDown(forward, { key: "ArrowUp" });
    await flush();
    expect(link.velocities()).toHaveLength(1);
    fireEvent.blur(forward);
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");

    pushForward(forward);
    await flush();
    fireEvent.pointerUp(forward, { pointerId: 1 });
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");
    expect(link.types().filter((type) => type === "drive.stop")).toHaveLength(2);
  });

  it("stops when the view unmounts while held", async () => {
    const link = new FakeLink();
    const { unmount } = await mountDrive(link);
    pushForward(stick(FORWARD));
    await flush();
    unmount();
    expect(link.types().at(-1)).toBe("drive.stop");
    const after = link.sent.length;
    await tick(1_000);
    expect(link.sent).toHaveLength(after);
  });

  it("resends the held command every interval, as short-lived commands", async () => {
    const link = new FakeLink();
    await mountDrive(link);
    pushForward(stick(FORWARD));
    await flush();
    await tick(450);
    expect(link.velocities().length).toBeGreaterThanOrEqual(4);
    for (const velocity of link.velocities()) expect(velocity).toEqual({ vx: MAX_LINEAR, wz: 0 });
  });

  it("combines both thumbs and keeps moving when only one lifts", async () => {
    const link = new FakeLink();
    await mountDrive(link);
    const forward = stick(FORWARD);
    const turn = stick(TURN);

    pushForward(forward, 1);
    // Left edge of the turn stick: positive angular velocity (counter-clockwise).
    fireEvent.pointerDown(turn, { pointerId: 2, clientX: 0, clientY: 100 });
    await flush();
    expect(link.velocities().at(-1)).toEqual({ vx: MAX_LINEAR, wz: MAX_ANGULAR });

    fireEvent.pointerUp(turn, { pointerId: 2 });
    await flush();
    expect(link.velocities().at(-1)).toEqual({ vx: MAX_LINEAR, wz: 0 });
    expect(link.types()).not.toContain("drive.stop");

    fireEvent.pointerUp(forward, { pointerId: 1 });
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");
  });

  it("reports a stop the robot did not acknowledge", async () => {
    const link = new FakeLink();
    await mountDrive(link);
    pushForward(stick(FORWARD));
    await flush();
    link.requestImpl = async (type) => {
      if (type === "drive.stop") throw new CommandError("timeout", "no answer");
      return { request_type: type, ok: true };
    };
    fireEvent.pointerUp(stick(FORWARD), { pointerId: 1 });
    await flush();
    const alert = screen.getByRole("alert");
    expect(alert.textContent).toContain("停止未确认");
    expect(alert.textContent).toContain("急停");
  });
});

// ---------------------------------------------------------------- switching

describe("robot switching", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  /** Mirrors RobotPanel: the parent disconnects in a passive effect cleanup. */
  function Panel({ robotId, link }: { robotId: string; link: FakeLink }): ReactElement {
    useEffect(() => {
      void link.connect(robotId);
      return () => link.disconnect();
    }, [link, robotId]);
    return <DriveView robotId={robotId} link={link} snapshot={snapshot(robotId)} />;
  }

  it("test_switch_robot_stops_old_robot", async () => {
    const link = new FakeLink();
    const { rerender } = render(<Panel key="robot-alpha" robotId="robot-alpha" link={link} />);
    await acquire();
    fireEvent.keyDown(stick(FORWARD), { key: "ArrowUp" });
    await flush();
    expect(link.velocities()).toHaveLength(1);

    // The shell remounts the panel with a new key.
    rerender(<Panel key="robot-beta" robotId="robot-beta" link={link} />);
    await flush();

    const stops = link.sent.filter((entry) => entry.type === "drive.stop");
    expect(stops).toHaveLength(1);
    // The stop left while the socket still pointed at the old robot.
    expect(stops[0]?.robot).toBe("robot-alpha");
    const count = link.sent.length;
    await tick(1_000);
    expect(link.sent).toHaveLength(count);
    // The new robot starts without a lease: nothing is taken back on its behalf.
    expect(screen.getByRole("button", { name: "获取控制权" })).toBeTruthy();
    expect(link.types().filter((type) => type === "control.acquire")).toHaveLength(1);
  });

  it("stops the old robot when the same view is pointed at another robot", async () => {
    const link = new FakeLink();
    const { rerender } = render(
      <DriveView robotId="robot-alpha" link={link} snapshot={snapshot("robot-alpha")} />,
    );
    await acquire();
    fireEvent.keyDown(stick(FORWARD), { key: "ArrowUp" });
    await flush();

    rerender(<DriveView robotId="robot-beta" link={link} snapshot={snapshot("robot-beta")} />);
    await flush();

    const stops = link.sent.filter((entry) => entry.type === "drive.stop");
    expect(stops).toHaveLength(1);
    expect(stops[0]?.robot).toBe("robot-alpha");
    const count = link.sent.length;
    await tick(1_000);
    expect(link.sent).toHaveLength(count);
    expect(screen.getByRole("button", { name: "获取控制权" })).toBeTruthy();
  });
});

// ---------------------------------------------------------------- gating

describe("when driving is not allowed", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  it("sends nothing before the lease is held", async () => {
    const link = new FakeLink();
    render(<DriveView robotId="robot-alpha" link={link} snapshot={snapshot("robot-alpha")} />);
    pushForward(stick(FORWARD));
    await flush();
    expect(link.sent).toEqual([]);
    expect(screen.getByRole("group", { name: FORWARD }).getAttribute("aria-disabled")).toBe("true");
  });

  it("blocks and stops on a stale snapshot", async () => {
    const link = new FakeLink();
    const view = await mountDrive(link);
    pushForward(stick(FORWARD));
    await flush();
    view.rerender(
      <DriveView
        robotId="robot-alpha"
        link={link}
        snapshot={snapshot("robot-alpha", { stale: true, connected: false })}
      />,
    );
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");
    expect(screen.getByText(/缓存快照/)).toBeTruthy();
    const count = link.sent.length;
    pushForward(stick(FORWARD), 3);
    await flush();
    await tick(500);
    expect(link.sent).toHaveLength(count);
  });

  it("blocks when no snapshot has arrived", () => {
    const link = new FakeLink();
    render(<DriveView robotId="robot-alpha" link={link} snapshot={null} />);
    expect(screen.getByText(/等待车端快照/)).toBeTruthy();
    expect(
      (screen.getByRole("button", { name: "获取控制权" }) as HTMLButtonElement).disabled,
    ).toBe(true);
  });

  it("locks driving until the operator explicitly enters limited assisted mode", async () => {
    const link = new FakeLink();
    const lost = snapshot("robot-alpha", { localized: false });
    render(<DriveView robotId="robot-alpha" link={link} snapshot={lost} />);
    await acquire();
    pushForward(stick(FORWARD));
    await flush();
    expect(link.velocities()).toEqual([]);
    expect(screen.getByText(/定位未完成/)).toBeTruthy();

    const enter = screen.getByRole("button", { name: /进入定位辅助遥控/ });
    expect(enter.getAttribute("aria-pressed")).toBe("false");
    fireEvent.click(enter);
    expect(enter.getAttribute("aria-pressed")).toBe("true");
    expect(screen.getByText(/限速/)).toBeTruthy();

    pushForward(stick(FORWARD), 2);
    await flush();
    const { vx } = link.velocities()[0]!;
    expect(vx).toBeCloseTo(MAX_LINEAR * ASSIST_SCALE);
    expect(vx).toBeLessThan(MAX_LINEAR);
  });

  it("does not take the lease back after a command is rejected", async () => {
    const link = new FakeLink();
    await mountDrive(link);
    link.requestImpl = async (type) => {
      if (type === "drive.velocity") throw new CommandError("offline", "lease gone");
      return { request_type: type, ok: true };
    };
    pushForward(stick(FORWARD));
    await flush();
    await tick(1_000);
    expect(screen.getByRole("alert").textContent).toContain("离线");
    expect(screen.getByRole("button", { name: "获取控制权" })).toBeTruthy();
    expect(link.types().filter((type) => type === "control.acquire")).toHaveLength(1);
    expect(link.velocities()).toHaveLength(1);
  });

  it("releases the lease only when asked", async () => {
    const link = new FakeLink();
    await mountDrive(link);
    fireEvent.click(screen.getByRole("button", { name: "释放控制权" }));
    await flush();
    expect(link.types()).toEqual(["control.acquire", "control.release"]);
    expect(screen.getByRole("button", { name: "获取控制权" })).toBeTruthy();
  });
});

// ---------------------------------------------------------------- controller

describe("DriveController", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  it("sends at once, repeats, and stops once", async () => {
    const link = new FakeLink();
    const events: DriveEvent[] = [];
    const controller = new DriveController(link, { intervalMs: 100, onEvent: (e) => events.push(e) });
    controller.start({ vx: 0.2, wz: 0 });
    expect(link.velocities()).toEqual([{ vx: 0.2, wz: 0 }]);
    await tick(250);
    expect(link.velocities()).toHaveLength(3);
    controller.start({ vx: 0.1, wz: 0.3 });
    expect(link.velocities().at(-1)).toEqual({ vx: 0.1, wz: 0.3 });
    controller.stop("release");
    controller.stop("blur");
    await flush();
    expect(link.types().filter((type) => type === "drive.stop")).toHaveLength(1);
    expect(events).toEqual([{ type: "stopped", reason: "release" }]);
    expect(controller.holding).toBe(false);
  });

  it("sends nothing when stopped while idle", () => {
    const link = new FakeLink();
    const controller = new DriveController(link);
    controller.stop("unmount");
    expect(link.sent).toEqual([]);
  });

  it("refuses non-finite and oversized input", () => {
    const link = new FakeLink();
    const controller = new DriveController(link);
    controller.start({ vx: Number.NaN, wz: 0 });
    controller.start({ vx: MAX_LINEAR * 3, wz: 0 });
    expect(link.velocities()).toEqual([{ vx: MAX_LINEAR, wz: 0 }]);
  });
});

// ---------------------------------------------------------------- emergency stop

describe("EmergencyStop", () => {
  it("stays pressable while a request is in flight", async () => {
    const link = new FakeLink();
    link.requestImpl = () => new Promise(() => undefined);
    render(<EmergencyStop link={link} />);
    const button = screen.getByRole("button", { name: "急停" }) as HTMLButtonElement;
    fireEvent.click(button);
    expect(button.disabled).toBe(false);
    fireEvent.click(button);
    expect(link.types()).toEqual(["safety.estop", "safety.estop"]);
    expect(screen.queryByText(/急停已确认/)).toBeNull();
  });

  it("does not show success for a stop the robot has not acknowledged", async () => {
    const link = new FakeLink();
    link.requestImpl = () => Promise.reject(new CommandError("timeout", "no answer"));
    render(<EmergencyStop link={link} />);
    fireEvent.click(screen.getByRole("button", { name: "急停" }));
    const alert = await screen.findByRole("alert");
    expect(alert.textContent).toContain("急停未确认");
    expect(alert.textContent).toContain("物理急停");
    expect(screen.queryByText(/急停已确认/)).toBeNull();
  });

  it("confirms only after an ok response, and a later failed retry does not erase it", async () => {
    const link = new FakeLink();
    render(<EmergencyStop link={link} />);
    fireEvent.click(screen.getByRole("button", { name: "急停" }));
    await screen.findByText(/急停已确认/);
    expect(screen.queryByRole("alert")).toBeNull();

    link.requestImpl = () => Promise.reject(new CommandError("offline", "gone"));
    fireEvent.click(screen.getByRole("button", { name: "急停" }));
    await flush();
    expect(screen.getByText(/急停已确认/)).toBeTruthy();
  });

  it("is the only emergency stop in the app shell", async () => {
    const api: ControlApi = {
      login: async () => undefined,
      logout: async () => undefined,
      listRobots: async (): Promise<RobotSummary[]> => [
        { id: "robot-alpha", name: "Alpha 仓库车", online: true },
      ],
    };
    render(
      <MemoryRouter initialEntries={["/robots/robot-alpha"]}>
        <App api={api} link={new FakeLink()} />
      </MemoryRouter>,
    );
    await waitFor(() => expect(screen.getAllByRole("button", { name: "急停" })).toHaveLength(1));
    const panel = readFileSync(resolve(__dirname, "../devices/RobotPanel.tsx"), "utf8");
    expect(panel).not.toMatch(/function EmergencyStop/);
    expect(panel).toMatch(/drive\/EmergencyStop/);
  });
});

// ---------------------------------------------------------------- landscape and keyboard

describe("test_landscape_controls_accessible", () => {
  const styles = readFileSync(resolve(__dirname, "Drive.module.css"), "utf8");

  it("labels both thumbs' sticks and makes them keyboard operable", async () => {
    vi.useFakeTimers();
    const link = new FakeLink();
    await mountDrive(link);
    for (const name of [FORWARD, TURN]) {
      const element = screen.getByRole("group", { name });
      expect(element.getAttribute("tabindex")).toBe("0");
      const hintId = element.getAttribute("aria-describedby");
      expect(hintId).toBeTruthy();
      expect(document.getElementById(hintId as string)?.textContent).toMatch(/方向键/);
    }
    fireEvent.keyDown(stick(TURN), { key: "ArrowLeft" });
    await flush();
    expect(link.velocities().at(-1)).toEqual({ vx: 0, wz: MAX_ANGULAR });
    fireEvent.keyDown(stick(TURN), { key: "ArrowLeft", repeat: true });
    await flush();
    expect(link.velocities()).toHaveLength(1);
    fireEvent.keyUp(stick(TURN), { key: "ArrowLeft" });
    await flush();
    expect(link.types().at(-1)).toBe("drive.stop");
  });

  it("lays the sticks out for two thumbs in landscape and keeps targets large", () => {
    expect(styles).toMatch(/@media[^{]*orientation:\s*landscape/);
    expect(styles).toMatch(/touch-action:\s*none/);
    expect(styles).toMatch(/\.estopDock\s*\{[^}]*position:\s*fixed/);
    expect(styles).toMatch(/min-height:\s*var\(--touch\)/);
    expect(styles).toMatch(/:focus-visible/);
  });

  it("exposes the stop as a named button", () => {
    render(<EmergencyStop link={new FakeLink()} />);
    expect(screen.getByRole("button", { name: "急停" })).toBeTruthy();
  });
});
