import { StrictMode } from "react";
import {
  act,
  cleanup,
  fireEvent,
  render,
  screen,
} from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import { testPort } from "../../api/vehicleState.testUtils";
import { ManualController } from "./ManualController";
import { DriveView } from "./DriveView";
import { EmergencyStop } from "./EmergencyStop";
import type { RobotLink } from "../../api/client";

afterEach(() => {
  cleanup();
  vi.useRealTimers();
});
function link(port = testPort()): RobotLink {
  return {
    vehicle: port,
    connect: async () => {},
    disconnect() {},
    request: vi.fn(async () => ({ ok: true })) as RobotLink["request"],
    subscribe: () => () => {},
    subscribeStatus: () => () => {},
  };
}
describe("normalized manual ownership", () => {
  it("prepared authority stays still until a fresh press and sends normalized gear intent", async () => {
    vi.useFakeTimers();
    const sent: [string, unknown][] = [];
    const port = testPort(async (type, value) => {
      sent.push([type, value]);
      return { authority_id: "a", state: "prepared" };
    });
    const controller = new ManualController(port, () => {});
    await controller.prepare();
    await vi.advanceTimersByTimeAsync(10000);
    expect(sent.map((x) => x[0])).toEqual(["control.acquire"]);
    controller.begin(1, 0.5, "high");
    await Promise.resolve();
    expect(sent[1]).toEqual([
      "drive.intent",
      {
        authority_id: "a",
        gesture: 1,
        sequence: 1,
        linear: 1,
        turn: 0.5,
        gear: "high",
      },
    ]);
    controller.release();
    expect(sent.map((x) => x[0])).not.toContain("drive.velocity");
  });
  it("late acquire cannot revive a released press", async () => {
    let finish!: (value: unknown) => void;
    const sent: string[] = [];
    const port = testPort(async (type) => {
      sent.push(type);
      return type === "control.acquire"
        ? new Promise((r) => {
            finish = r;
          })
        : null;
    });
    const controller = new ManualController(port, () => {});
    const preparing = controller.prepare();
    controller.begin(1, 0, "low");
    controller.release();
    finish({ authority_id: "late", state: "prepared" });
    await preparing;
    controller.update(1, 0, "low");
    expect(sent).not.toContain("drive.intent");
    expect(controller.phase).toBe("none");
  });
  it("samples the latest held input despite an unresolved send and fences on release", async () => {
    vi.useFakeTimers();
    const intents: unknown[] = [];
    const port = testPort(async (type, value) => {
      if (type === "drive.intent") {
        intents.push(value);
        return new Promise<void>(() => {});
      }
      return { authority_id: "a", state: "prepared" };
    });
    const controller = new ManualController(port, () => {});
    await controller.prepare();
    controller.begin(0.1, 0, "low");
    controller.update(0.2, 0, "low");
    controller.update(0.9, 0, "low");
    expect(intents).toHaveLength(3);
    await vi.advanceTimersByTimeAsync(400);
    expect(intents).toHaveLength(7);
    expect(intents.slice(3)).toEqual(
      Array(4).fill(expect.objectContaining({ linear: 0.9 })),
    );
    controller.release();
    await vi.advanceTimersByTimeAsync(500);
    expect(intents).toHaveLength(7);
    controller.dispose();
  });
});
it("shows unavailable instead of enabling old velocity controls", () => {
  const old = link();
  delete (
    old as {
      vehicle?: unknown;
    }
  ).vehicle;
  render(<DriveView robotId="robot" link={old} snapshot={null} />);
  expect(
    screen
      .getByRole("group", { name: "驾驶摇杆" })
      .getAttribute("aria-disabled"),
  ).toBe("true");
  expect(
    (screen.getByRole("button", { name: "获取控制权" }) as HTMLButtonElement)
      .disabled,
  ).toBe(true);
});
it("a request acknowledgement does not claim confirmed emergency stop", async () => {
  const l = link();
  render(<EmergencyStop link={l} />);
  await act(async () =>
    fireEvent.click(screen.getByRole("button", { name: "急停" })),
  );
  expect(screen.getByText(/等待车端状态确认/)).toBeTruthy();
  expect(screen.queryByText("软件急停已锁存")).toBeNull();
});

it("actual controls stop on key release and restart with the same authority on a fresh press", async () => {
  const sent: [string, unknown][] = [];
  const port = testPort(async (type, value) => {
    sent.push([type, value]);
    return { authority_id: "a", state: "prepared" };
  });
  render(<DriveView robotId="robot" link={link(port)} snapshot={null} />);
  await act(async () =>
    fireEvent.click(screen.getByRole("button", { name: "获取控制权" })),
  );
  fireEvent.click(screen.getByRole("button", { name: "高速" }));
  const stick = screen.getByRole("group", { name: "驾驶摇杆" });
  await act(async () => fireEvent.keyDown(stick, { key: "ArrowUp" }));
  expect(sent.find((item) => item[0] === "drive.intent")?.[1]).toMatchObject({
    linear: 1,
    turn: 0,
    gear: "high",
  });
  await act(async () => fireEvent.keyUp(stick, { key: "ArrowUp" }));
  expect(sent.at(-1)).toEqual([
    "drive.stop",
    { authority_id: "a", gesture: 1 },
  ]);
  expect(screen.getByText("已准备 · 等待新按压")).toBeTruthy();
  await act(async () =>
    fireEvent.keyDown(stick, { key: "ArrowUp", repeat: true }),
  );
  expect(sent.at(-1)?.[0]).toBe("drive.stop");
  await act(async () => fireEvent.keyDown(stick, { key: "ArrowDown" }));
  expect(sent.at(-1)).toEqual([
    "drive.intent",
    expect.objectContaining({ authority_id: "a", gesture: 2, linear: -1 }),
  ]);
  expect(sent.filter(([type]) => type === "control.acquire")).toHaveLength(1);
  await act(async () =>
    fireEvent.click(screen.getByRole("button", { name: "释放控制权" })),
  );
  expect(sent.at(-1)).toEqual(["control.release", { authority_id: "a" }]);
  expect(screen.getByText("未获取控制权")).toBeTruthy();
});
it("sends emergency stop before fencing a held input, and requires owner permission for software reset", async () => {
  const sent: string[] = [];
  const port = testPort(async (type) => {
    sent.push(type);
    return { authority_id: "a", state: "prepared" };
  });
  const l = link(port);
  l.request = (async () => {
    sent.push("safety.estop");
    return { ok: true };
  }) as RobotLink["request"];
  render(
    <StrictMode>
      <DriveView robotId="robot" link={l} snapshot={null} />
      <EmergencyStop link={l} />
    </StrictMode>,
  );
  await act(async () =>
    fireEvent.click(screen.getByRole("button", { name: "获取控制权" })),
  );
  await act(async () =>
    fireEvent.keyDown(screen.getByRole("group", { name: "驾驶摇杆" }), {
      key: "ArrowUp",
    }),
  );
  await act(async () =>
    fireEvent.click(screen.getByRole("button", { name: "急停" })),
  );
  expect(sent.slice(-2)).toEqual(["safety.estop", "control.release"]);
  const old = port.getSnapshot().frame!;
  act(() =>
    port.update(
      {
        ...old,
        sequence: 2,
        mode: "estop",
        confirmed_estop_status: "software_latched",
        estop_revision: 8,
        allowed_actions: [],
      },
      true,
    ),
  );
  expect(
    (screen.getByRole("button", { name: "复位软件急停" }) as HTMLButtonElement)
      .disabled,
  ).toBe(true);
});

it("keeps stopped ownership idle without heartbeats or acquire and ignores old send failures", async () => {
  vi.useFakeTimers();
  const sent: [string, unknown][] = [];
  let rejectOld!: (error: Error) => void;
  const port = testPort(async (type, value) => {
    sent.push([type, value]);
    if (type === "drive.intent" && !rejectOld)
      return new Promise((_, reject) => {
        rejectOld = reject;
      });
    return { authority_id: "same", state: "prepared" };
  });
  const controller = new ManualController(port, () => {});
  await controller.prepare();
  controller.begin(1, 0, "low");
  controller.stopGesture();
  await vi.advanceTimersByTimeAsync(10000);
  expect(controller.phase).toBe("prepared");
  expect(sent.map(([type]) => type)).toEqual([
    "control.acquire",
    "drive.intent",
    "drive.stop",
  ]);
  expect(controller.begin(-1, 0, "low")).toBe(true);
  rejectOld(new Error("old send failure"));
  await Promise.resolve();
  expect(controller.phase).toBe("active");
  expect(sent.at(-1)).toEqual([
    "drive.intent",
    expect.objectContaining({ authority_id: "same", gesture: 2, sequence: 2 }),
  ]);
  controller.dispose();
});

it("requires a fresh press after stop acknowledgement without replaying a held pointer", async () => {
  const sent: [string, unknown][] = [];
  let finishStop!: () => void;
  const port = testPort(async (type, value) => {
    sent.push([type, value]);
    if (type === "drive.stop")
      return new Promise<void>((resolve) => {
        finishStop = resolve;
      });
    return { authority_id: "a", state: "prepared" };
  });
  render(<DriveView robotId="robot" link={link(port)} snapshot={null} />);
  await act(async () =>
    fireEvent.click(screen.getByRole("button", { name: "获取控制权" })),
  );
  const stick = screen.getByRole("group", { name: "驾驶摇杆" });
  await act(async () => fireEvent.keyDown(stick, { key: "ArrowUp" }));
  await act(async () => fireEvent.keyUp(stick, { key: "ArrowUp" }));
  await act(async () => fireEvent.pointerDown(stick, { pointerId: 7 }));
  await act(async () => finishStop());
  await act(async () => fireEvent.pointerMove(stick, { pointerId: 7 }));
  await act(async () =>
    fireEvent.keyDown(stick, { key: "ArrowUp", repeat: true }),
  );
  expect(sent.filter(([type]) => type === "drive.intent")).toHaveLength(1);
  await act(async () => fireEvent.keyDown(stick, { key: "ArrowDown" }));
  expect(sent.at(-1)).toEqual([
    "drive.intent",
    expect.objectContaining({ gesture: 2, linear: -1 }),
  ]);
});

it("a late stop receipt cannot restore released ownership and a failed stop revokes it", async () => {
  let finishStop!: () => void;
  const port = testPort(async (type) => {
    if (type === "drive.stop")
      return new Promise<void>((resolve) => {
        finishStop = resolve;
      });
    return { authority_id: "a", state: "prepared" };
  });
  const controller = new ManualController(port, () => {});
  await controller.prepare();
  controller.begin(1, 0, "low");
  controller.stopGesture();
  controller.release();
  finishStop();
  await Promise.resolve();
  expect(controller.phase).toBe("none");
  const failing = new ManualController(
    testPort(async (type) => {
      if (type === "drive.stop") throw new Error("stop failed");
      return { authority_id: "b", state: "prepared" };
    }),
    () => {},
  );
  await failing.prepare();
  failing.begin(1, 0, "low");
  failing.stopGesture();
  await new Promise((resolve) => setTimeout(resolve, 0));
  expect(failing.phase).toBe("none");
});

it("V7 presents a single authority action through preparing, prepared and release without inventing ownership", async () => {
  let confirm!: (value: unknown) => void;
  const sent: string[] = [];
  const port = testPort(async (type) => {
    sent.push(type);
    if (type === "control.acquire")
      return new Promise((resolve) => {
        confirm = resolve;
      });
    return null;
  });
  render(<DriveView robotId="robot" link={link(port)} snapshot={null} />);
  fireEvent.click(screen.getByRole("button", { name: "获取控制权" }));
  expect(screen.queryByRole("button", { name: "获取控制权" })).toBeNull();
  expect(screen.getByText("等待车端准备")).toBeTruthy();
  expect(screen.getAllByRole("button", { name: "释放控制权" })).toHaveLength(1);
  await act(async () => confirm({ authority_id: "v7", state: "prepared" }));
  expect(screen.getByText("已准备 · 等待新按压")).toBeTruthy();
  expect(sent).toEqual(["control.acquire"]);
  fireEvent.click(screen.getByRole("button", { name: "释放控制权" }));
  expect(screen.getByRole("button", { name: "获取控制权" })).toBeTruthy();
  expect(sent).toEqual(["control.acquire", "control.release"]);
});
