import { testPort, runtime, settingsFixture } from "../../api/vehicleState.testUtils";
import { VEHICLE_REQUEST_TYPES, VehicleTransport, type LearningStatus } from "../../api/vehicleState";
import {
  act,
  cleanup,
  fireEvent,
  render,
  screen,
  waitFor,
  within,
} from "@testing-library/react";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { MemoryRouter } from "react-router-dom";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { App } from "../../App";
import {
  ApiError,
  CommandError,
  HttpControlApi,
  InvalidCredentialsError,
  RobotClient,
  RateLimitedError,
  UnauthorizedError,
  type ControlApi,
  type RobotLink,
  type SocketLike,
} from "../../api/client";
import { createDemoBackend } from "../../api/demo";
import {
  ENVELOPE_FIELDS,
  ERROR_CODES,
  MAX_MESSAGE_BYTES,
  OPERATION_REQUEST_TYPES,
  REQUEST_TYPES,
  RESPONSE_REQUEST_TYPES,
  SIMPLE_REQUEST_TYPES,
  WORKSPACE_REQUEST_TYPES,
  SNAPSHOT_FIELDS,
  parseInbound,
  type LinkStatus,
  type ResponsePayload,
  type RobotSnapshot,
  type RobotSummary,
} from "../../api/protocol";

beforeEach(() => {
  vi.stubGlobal(
    "ResizeObserver",
    class {
      observe() {}
      disconnect() {}
      unobserve() {}
    },
  );
  // The map on the robot screen draws on a canvas, which jsdom does not provide.
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(null);
});

afterEach(cleanup);

// ---------------------------------------------------------------- fakes

const ALPHA: RobotSummary = {
  id: "robot-alpha",
  name: "Alpha 仓库车",
  online: true,
};
const BETA: RobotSummary = {
  id: "robot-beta",
  name: "Beta 装卸车",
  online: false,
};

function snapshot(
  robotId: string,
  overrides: Partial<RobotSnapshot> = {},
): RobotSnapshot {
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
    home_revision: 0,
    ...overrides,
  };
}

class FakeLink implements RobotLink {
  readonly supportsMotionChallenges = true;
  connects: string[] = [];
  disconnects = 0;
  requests: Array<{
    type: string;
    payload: unknown;
  }> = [];
  requestImpl: (type: string, payload: unknown) => Promise<unknown> = () =>
    Promise.reject(new CommandError("offline", "no link"));
  private snapshotListeners = new Set<(s: RobotSnapshot) => void>();
  private statusListeners = new Set<(s: LinkStatus) => void>();

  async connect(robotId: string): Promise<void> {
    this.connects.push(robotId);
    this.emitStatus({ state: "open", robotId, closeCode: null });
  }
  disconnect(): void {
    this.disconnects += 1;
  }
  request<T = ResponsePayload>(type: string, payload: unknown): Promise<T> {
    this.requests.push({ type, payload });
    return this.requestImpl(type, payload) as Promise<T>;
  }
  subscribe(listener: (s: RobotSnapshot) => void): () => void {
    this.snapshotListeners.add(listener);
    return () => this.snapshotListeners.delete(listener);
  }
  subscribeStatus(listener: (s: LinkStatus) => void): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }
  emit(value: RobotSnapshot): void {
    act(() => this.snapshotListeners.forEach((listener) => listener(value)));
  }
  emitStatus(value: LinkStatus): void {
    act(() => this.statusListeners.forEach((listener) => listener(value)));
  }
}

class FakeApi implements ControlApi {
  robots: RobotSummary[] = [ALPHA, BETA];
  failure: Error | null = null;
  loginFailure: Error | null = null;
  logins: string[] = [];
  loggedOut = 0;
  async login(password: string): Promise<void> {
    this.logins.push(password);
    if (this.loginFailure !== null) throw this.loginFailure;
  }
  async logout(): Promise<void> {
    this.loggedOut += 1;
  }
  async revokeRobot(robotId: string): Promise<void> {
    this.robots = this.robots.filter((robot) => robot.id !== robotId);
  }
  async createPairingCode() {
    return { code: "example", expires_in: 30 };
  }
  async listRobots(): Promise<RobotSummary[]> {
    if (this.failure !== null) throw this.failure;
    return this.robots;
  }
}

function mount(path: string, api: ControlApi, link: RobotLink, demo = false) {
  return render(
    <MemoryRouter initialEntries={[path]}>
      <App api={api} link={link} demo={demo} />
    </MemoryRouter>,
  );
}

async function openDevices(): Promise<void> {
  const trigger = (
    await screen.findAllByRole("button", { name: /^(选择车辆|切换车辆)$/ })
  )[0]!;
  if (!screen.queryByRole("navigation", { name: "设备列表" }))
    fireEvent.click(trigger);
}

// ---------------------------------------------------------------- switching

describe("robot switching", () => {
  it("test_switch_robot_drops_previous_state", async () => {
    const api = new FakeApi();
    const link = new FakeLink();
    mount("/robots/robot-alpha", api, link);
    await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
    link.emit(
      snapshot("robot-alpha", { execution_status: "patrolling-alpha" }),
    );
    expect(screen.getByText("车端运行状态尚未接通")).toBeTruthy();
    expect(screen.queryByText("patrolling-alpha")).toBeNull();

    await openDevices();
    fireEvent.click(screen.getByRole("link", { name: /Beta/ }));

    expect(screen.queryByText("patrolling-alpha")).toBeNull();
    await waitFor(() =>
      expect(link.connects).toEqual(["robot-alpha", "robot-beta"]),
    );
    expect(link.disconnects).toBeGreaterThanOrEqual(1);
    await screen.findByText("等待车端快照");

    // A late frame of the old robot must not repaint the new robot's panel.
    link.emit(
      snapshot("robot-alpha", { execution_status: "patrolling-alpha" }),
    );
    expect(screen.queryByText("patrolling-alpha")).toBeNull();

    link.emit(snapshot("robot-beta", { execution_status: "charging-beta" }));
    expect(screen.queryByText("等待车端快照")).toBeNull();
    expect(screen.getByText("车端运行状态尚未接通")).toBeTruthy();
  });
});

// ---------------------------------------------------------------- cache label

describe("snapshot freshness", () => {
  it("test_stale_cache_label", async () => {
    const link = new FakeLink();
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(link.connects).toHaveLength(1));

    link.emit(
      snapshot("robot-alpha", {
        stale: true,
        connected: false,
        last_synced_at: 1_700_000_000,
      }),
    );
    expect(screen.getByText("缓存快照 · 非实时")).toBeTruthy();
    expect(screen.getByText(/最后同步/)).toBeTruthy();
    expect(document.querySelector('[data-freshness="live"]')).toBeNull();

    link.emit(
      snapshot("robot-alpha", {
        stale: false,
        last_synced_at: Date.now() / 1000,
      }),
    );
    expect(document.querySelector('[data-freshness="live"]')).toBeTruthy();
    expect(screen.queryByText("缓存快照 · 非实时")).toBeNull();
  });
});

// ---------------------------------------------------------------- session

describe("session", () => {
  it("test_session_expired_returns_login", async () => {
    const api = new FakeApi();
    const link = new FakeLink();
    mount("/robots/robot-alpha", api, link);
    await waitFor(() => expect(link.connects).toHaveLength(1));
    link.emit(
      snapshot("robot-alpha", { execution_status: "patrolling-alpha" }),
    );

    api.failure = new UnauthorizedError("login required");
    await openDevices();
    fireEvent.click(screen.getByRole("button", { name: "刷新设备" }));

    await screen.findByRole("heading", { name: "登录控制台" });
    expect(screen.queryByRole("navigation", { name: "设备列表" })).toBeNull();
    expect(screen.queryByText("patrolling-alpha")).toBeNull();
    expect(link.disconnects).toBeGreaterThanOrEqual(1);
  });

  it("returns to login when the first request is unauthorized, and logs in", async () => {
    const api = new FakeApi();
    api.failure = new UnauthorizedError("login required");
    mount("/robots/robot-alpha", api, new FakeLink());
    await screen.findByRole("heading", { name: "登录控制台" });

    api.failure = null;
    fireEvent.change(screen.getByLabelText("管理员密码"), {
      target: { value: "s3cret" },
    });
    fireEvent.click(screen.getByRole("button", { name: "进入控制台" }));

    await openDevices();
    await screen.findByRole("navigation", { name: "设备列表" });
    expect(api.logins).toEqual(["s3cret"]);
  });

  it("shows a wrong password and stays on login", async () => {
    const api = new FakeApi();
    api.failure = new UnauthorizedError("login required");
    api.loginFailure = new InvalidCredentialsError("invalid credentials");
    mount("/robots", api, new FakeLink());
    await screen.findByRole("heading", { name: "登录控制台" });

    fireEvent.change(screen.getByLabelText("管理员密码"), {
      target: { value: "wrong" },
    });
    fireEvent.click(screen.getByRole("button", { name: "进入控制台" }));

    const alert = await screen.findByRole("alert");
    expect(alert.textContent).toContain("密码错误");
    expect(screen.getByRole("heading", { name: "登录控制台" })).toBeTruthy();
  });

  it("logs out and returns to login", async () => {
    const api = new FakeApi();
    mount("/robots", api, new FakeLink());
    await openDevices();
    await screen.findByRole("navigation", { name: "设备列表" });
    fireEvent.click(screen.getByRole("button", { name: "退出登录" }));
    await screen.findByRole("heading", { name: "登录控制台" });
    expect(api.loggedOut).toBe(1);
  });
});

// ---------------------------------------------------------------- device list

describe("device list", () => {
  it("shows the real online state of each robot", async () => {
    mount("/robots", new FakeApi(), new FakeLink());
    await openDevices();
    const nav = await screen.findByRole("navigation", { name: "设备列表" });
    const alpha = within(nav).getByRole("link", { name: /Alpha/ });
    const beta = within(nav).getByRole("link", { name: /Beta/ });
    expect(within(alpha).getByText("在线")).toBeTruthy();
    expect(within(beta).getByText("离线")).toBeTruthy();
  });

  it.each([
    { online: true, label: "在线" },
    { online: false, label: "离线" },
  ])(
    "updates from reconnecting to $label after the next robot list response",
    async ({ online, label }) => {
      const answers = [
        reply(200, {
          robots: [{ ...ALPHA, online: false, reconnecting: true }],
        }),
        reply(200, { robots: [{ ...ALPHA, online, reconnecting: false }] }),
      ];
      const api = new HttpControlApi(
        (async () => answers.shift()!) as typeof fetch,
        memoryStorage(),
      );
      mount("/robots", api, new FakeLink());

      await openDevices();
      const robot = await screen.findByRole("link", { name: /Alpha/ });
      expect(within(robot).getByText("重连中")).toBeTruthy();

      await openDevices();
      fireEvent.click(screen.getByRole("button", { name: "刷新设备" }));
      expect(await within(robot).findByText(label)).toBeTruthy();
    },
  );

  it("shows an empty state and a retryable error state", async () => {
    const api = new FakeApi();
    api.robots = [];
    mount("/robots", api, new FakeLink());
    await openDevices();
    await screen.findByText("尚未配对任何设备");

    api.failure = new Error("boom");
    await openDevices();
    fireEvent.click(screen.getByRole("button", { name: "刷新设备" }));
    const alert = await screen.findByRole("alert");
    expect(alert.textContent).toContain("设备列表加载失败");
  });

  it("tells the operator when the selected robot does not exist", async () => {
    mount("/robots/ghost", new FakeApi(), new FakeLink());
    await screen.findByText("没有这台设备");
  });
});

// ---------------------------------------------------------------- command errors

describe("command errors", () => {
  it("test_command_error_never_shown_success", async () => {
    const link = new FakeLink();
    link.requestImpl = () =>
      Promise.reject(new CommandError("offline", "robot offline"));
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(link.connects).toHaveLength(1));

    fireEvent.click(screen.getByRole("button", { name: "急停" }));

    const alert = await screen.findByRole("alert");
    expect(alert.textContent).toContain("急停未确认");
    expect(alert.textContent).toContain("离线");
    expect(link.requests).toEqual([{ type: "safety.estop", payload: {} }]);
    expect(screen.queryByText(/急停已确认/)).toBeNull();
  });

  it("only reports an e-stop as confirmed after an ok response", async () => {
    const link = new FakeLink();
    link.requestImpl = async () => ({ request_type: "safety.estop", ok: true });
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(link.connects).toHaveLength(1));

    fireEvent.click(screen.getByRole("button", { name: "急停" }));

    await screen.findByText(/等待车端状态确认/);
    expect(screen.queryByRole("alert")).toBeNull();
  });
});

// ---------------------------------------------------------------- demo

describe("demo data", () => {
  it("test_demo_label_always_visible", async () => {
    const api = new FakeApi();
    api.failure = new UnauthorizedError("login required");
    mount("/robots/robot-alpha", api, new FakeLink(), true);

    await screen.findByRole("heading", { name: "登录控制台" });
    expect(screen.getByLabelText("演示数据标记").textContent).toContain(
      "演示数据",
    );

    api.failure = null;
    fireEvent.change(screen.getByLabelText("管理员密码"), {
      target: { value: "x" },
    });
    fireEvent.click(screen.getByRole("button", { name: "进入控制台" }));
    await openDevices();
    await screen.findByRole("navigation", { name: "设备列表" });
    expect(screen.getByLabelText("演示数据标记").textContent).toContain(
      "演示数据",
    );

    await openDevices();
    fireEvent.click(screen.getByRole("link", { name: /Beta/ }));
    expect(screen.getByLabelText("演示数据标记").textContent).toContain(
      "演示数据",
    );
  });

  it("shows no demo label for real data", async () => {
    mount("/robots/robot-alpha", new FakeApi(), new FakeLink(), false);
    await openDevices();
    await screen.findByRole("navigation", { name: "设备列表" });
    expect(screen.queryByLabelText("演示数据标记")).toBeNull();
  });

  it("demo backend names its robots as demo and never drives a chassis", async () => {
    const { api, link } = createDemoBackend();
    await api.login("anything");
    const robots = await api.listRobots();
    expect(robots.length).toBeGreaterThan(0);
    for (const robot of robots) expect(robot.name).toContain("演示");
    await link.connect(robots[0]!.id);
    await expect(link.request("safety.estop", {})).rejects.toMatchObject({
      code: "safety_rejected",
    });
    link.disconnect();
  });
});

// ---------------------------------------------------------------- HttpControlApi

function memoryStorage(): Pick<Storage, "getItem" | "setItem" | "removeItem"> {
  const values = new Map<string, string>();
  return {
    getItem: (key) => values.get(key) ?? null,
    setItem: (key, value) => void values.set(key, value),
    removeItem: (key) => void values.delete(key),
  };
}

function reply(
  status: number,
  body?: unknown,
  headers: Record<string, string> = {},
): Response {
  return new Response(body === undefined ? null : JSON.stringify(body), {
    status,
    headers,
  });
}

describe("HttpControlApi", () => {
  it("logs in with the password, keeps the csrf token and sends it on logout", async () => {
    const calls: Array<{
      url: string;
      init: RequestInit;
    }> = [];
    const answers = [reply(200, { csrf_token: "csrf-1" }), reply(204)];
    const fetchImpl = vi.fn(
      async (url: RequestInfo | URL, init?: RequestInit) => {
        calls.push({ url: String(url), init: init ?? {} });
        return answers.shift()!;
      },
    );
    const api = new HttpControlApi(fetchImpl as typeof fetch, memoryStorage());

    await api.login("pw");
    await api.logout();

    expect(calls[0]).toMatchObject({ url: "/api/session" });
    expect(calls[0]!.init).toMatchObject({
      method: "POST",
      body: JSON.stringify({ password: "pw" }),
    });
    expect(calls[1]!.url).toBe("/api/session");
    expect(calls[1]!.init).toMatchObject({
      method: "DELETE",
      headers: { "X-CSRF-Token": "csrf-1" },
    });
  });

  it("maps 401, 429 and other login failures to distinct errors", async () => {
    const answers = [
      reply(401, { detail: "invalid credentials" }),
      reply(429, { detail: "too many attempts" }, { "Retry-After": "42" }),
      reply(403, { detail: "origin not allowed" }),
    ];
    const api = new HttpControlApi(
      (async () => answers.shift()!) as typeof fetch,
      memoryStorage(),
    );
    await expect(api.login("x")).rejects.toBeInstanceOf(
      InvalidCredentialsError,
    );
    await expect(api.login("x")).rejects.toMatchObject({
      retryAfterSeconds: 42,
    });
    await expect(api.login("x")).rejects.toBeInstanceOf(ApiError);
  });

  it("lists robots, and reports a 401 as an unauthorized session", async () => {
    const answers = [
      reply(200, { robots: [{ id: "r1", name: "A", online: true }] }),
      reply(401, { detail: "login required" }),
      reply(200, { robots: [{ id: "r1", name: "A" }] }),
    ];
    const api = new HttpControlApi(
      (async () => answers.shift()!) as typeof fetch,
      memoryStorage(),
    );
    await expect(api.listRobots()).resolves.toEqual([
      { id: "r1", name: "A", online: true },
    ]);
    await expect(api.listRobots()).rejects.toBeInstanceOf(UnauthorizedError);
    await expect(api.listRobots()).rejects.toBeInstanceOf(ApiError);
  });
});

// ---------------------------------------------------------------- RobotClient

class FakeSocket implements SocketLike {
  static instances: FakeSocket[] = [];
  readyState = 0;
  bufferedAmount = 0;
  sent: string[] = [];
  closedWith: number | undefined;
  onopen: (() => void) | null = null;
  onmessage: ((event: { data: unknown }) => void) | null = null;
  onclose: ((event: { code: number }) => void) | null = null;
  onerror: (() => void) | null = null;
  constructor(readonly url: string) {
    FakeSocket.instances.push(this);
  }
  send(data: string): void {
    this.sent.push(data);
  }
  close(code?: number): void {
    this.closedWith = code;
    this.readyState = 3;
  }
  open(): void {
    this.readyState = 1;
    this.onopen?.();
    const robot = new URL(this.url, "http://local").searchParams.get(
      "robot_id",
    );
    this.connection(3, robot ?? "robot-alpha");
  }
  connection(epoch: number, robot = "robot-alpha"): void {
    this.receive({
      v: 1,
      type: "control.connection",
      robot_id: robot,
      connection_epoch: epoch,
    });
  }
  receive(frame: unknown): void {
    this.onmessage?.({
      data: typeof frame === "string" ? frame : JSON.stringify(frame),
    });
  }
  serverClose(code: number): void {
    this.readyState = 3;
    this.onclose?.({ code });
  }
  lastSent(): Record<string, unknown> {
    return JSON.parse(this.sent[this.sent.length - 1] ?? "null") as Record<
      string,
      unknown
    >;
  }
}

function frame(
  type: string,
  robotId: string,
  payload: unknown,
  requestId = "r1",
) {
  return {
    v: 1,
    robot_id: robotId,
    session_id: "agent",
    connection_epoch: 3,
    request_id: requestId,
    seq: 1,
    expires_at: Date.now() / 1000 + 30,
    type,
    payload,
  };
}

const wirePayload = {
  connected: true,
  localized: true,
  map_ref: { map_id: "map-1", reference_revision: 2 },
  home: null,
  task_revision: 7,
  runnable_task_ids: ["t1"],
  execution_status: "idle",
  collected_at: 1_000,
  stale: false,
  last_synced_at: 2_000,
};

/** The epoch the cloud puts on a snapshot frame while an agent is connected. */
const AGENT_EPOCH = 3;

async function connected(
  options: {
    staleAfterMs?: number;
    requestTimeoutMs?: number;
    snapshot?: boolean;
  } = {},
) {
  FakeSocket.instances = [];
  const { snapshot: deliver = true, ...clientOptions } = options;
  const client = new RobotClient({
    socketFactory: (url) => new FakeSocket(url),
    ...clientOptions,
  });
  const pending = client.connect("robot-alpha");
  const socket = FakeSocket.instances[0]!;
  socket.open();
  await pending;
  if (deliver) socket.receive(frame("snapshot", "robot-alpha", wirePayload));
  return { client, socket };
}

describe("RobotClient", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });
  afterEach(() => {
    vi.useRealTimers();
  });

  it("opens the control socket for one robot", async () => {
    const { socket } = await connected();
    expect(socket.url).toMatch(/\/ws\/control\?robot_id=robot-alpha$/);
  });

  it("sends a v1 envelope and resolves only on an ok response", async () => {
    const { client, socket } = await connected();
    const result = client.request<ResponsePayload>("safety.estop", {});
    const sent = socket.lastSent();
    expect(Object.keys(sent).sort()).toEqual(
      [...ENVELOPE_FIELDS, "payload", "type"].sort(),
    );
    expect(sent).toMatchObject({
      v: 1,
      robot_id: "robot-alpha",
      type: "safety.estop",
      payload: {},
    });
    expect(Number(sent["expires_at"])).toBeGreaterThan(Date.now() / 1000);
    socket.receive(
      frame(
        "response",
        "robot-alpha",
        { request_type: "safety.estop", ok: true },
        String(sent["request_id"]),
      ),
    );
    await expect(result).resolves.toMatchObject({ ok: true });
  });

  it("rejects an ok=false response with its error code", async () => {
    const { client, socket } = await connected();
    const result = client.request("drive.stop", {});
    const id = String(socket.lastSent()["request_id"]);
    socket.receive(
      frame(
        "response",
        "robot-alpha",
        { request_type: "drive.stop", ok: false, error: "conflict" },
        id,
      ),
    );
    await expect(result).rejects.toMatchObject({ code: "conflict" });
  });

  it("numbers requests with an increasing seq", async () => {
    const { client, socket } = await connected();
    void client.request("drive.stop", {}).catch(() => undefined);
    const first = Number(socket.lastSent()["seq"]);
    void client.request("drive.stop", {}).catch(() => undefined);
    expect(Number(socket.lastSent()["seq"])).toBeGreaterThan(first);
    client.disconnect();
  });

  it("refuses unknown request types and an unopened link", async () => {
    const idle = new RobotClient({
      socketFactory: (url) => new FakeSocket(url),
    });
    await expect(idle.request("safety.estop", {})).rejects.toMatchObject({
      code: "offline",
    });
    const { client } = await connected();
    await expect(client.request("not.a.type", {})).rejects.toMatchObject({
      code: "validation",
    });
  });

  it("times out an unanswered request instead of waiting forever", async () => {
    const { client } = await connected({ requestTimeoutMs: 2_000 });
    const result = client.request("drive.stop", {});
    const assertion = expect(result).rejects.toMatchObject({ code: "timeout" });
    await vi.advanceTimersByTimeAsync(2_001);
    await assertion;
  });

  it("rejects pending requests when disconnected", async () => {
    const { client } = await connected();
    const result = client.request("drive.stop", {});
    client.disconnect();
    await expect(result).rejects.toMatchObject({ code: "offline" });
  });

  it("publishes snapshots as live, then marks them stale and not connected", async () => {
    const { client, socket } = await connected({ staleAfterMs: 5_000 });
    const seen: RobotSnapshot[] = [];
    client.subscribe((value) => seen.push(value));

    socket.receive(
      frame("snapshot", "robot-alpha", {
        ...wirePayload,
        last_synced_at: 4_321,
      }),
    );
    expect(seen).toHaveLength(1);
    expect(seen[0]).toMatchObject({
      robot_id: "robot-alpha",
      stale: false,
      connected: true,
    });
    expect(seen[0]!.last_synced_at).toBe(4_321);

    await vi.advanceTimersByTimeAsync(5_001);
    expect(seen).toHaveLength(2);
    expect(seen[1]).toMatchObject({ stale: true, connected: false });
    expect(seen[1]!.last_synced_at).toBe(seen[0]!.last_synced_at);
  });

  it("keeps the cloud's stale flag and sync time on a cached snapshot", async () => {
    const { client, socket } = await connected({ snapshot: false });
    const seen: RobotSnapshot[] = [];
    client.subscribe((value) => seen.push(value));
    socket.receive(
      frame("snapshot", "robot-alpha", {
        ...wirePayload,
        connected: false,
        stale: true,
        last_synced_at: 1_700_000_000,
      }),
    );
    expect(seen).toHaveLength(1);
    expect(seen[0]).toMatchObject({
      stale: true,
      connected: false,
      last_synced_at: 1_700_000_000,
    });
    await vi.advanceTimersByTimeAsync(10_000);
    expect(seen).toHaveLength(1);
  });

  it("ignores a snapshot that does not say when the cloud last synced it", async () => {
    const { client, socket } = await connected({ snapshot: false });
    const seen: RobotSnapshot[] = [];
    client.subscribe((value) => seen.push(value));
    const { stale: _stale, last_synced_at: _synced, ...bare } = wirePayload;
    socket.receive(frame("snapshot", "robot-alpha", bare));
    expect(seen).toHaveLength(0);
    expect(client.vehicle.getSnapshot().manual_connected).toBe(true);
  });

  it("sends a command with the independent connection epoch", async () => {
    const { client, socket } = await connected({ snapshot: false });
    socket.connection(3);
    socket.receive(frame("snapshot", "robot-alpha", wirePayload));
    void client.request("drive.stop", {}).catch(() => undefined);
    expect(socket.lastSent()["connection_epoch"]).toBe(AGENT_EPOCH);
    socket.connection(4);
    socket.receive({
      ...frame("snapshot", "robot-alpha", wirePayload),
      connection_epoch: 4,
    });
    void client.request("drive.stop", {}).catch(() => undefined);
    expect(socket.lastSent()["connection_epoch"]).toBe(4);
    client.disconnect();
  });

  it("sends urgent commands before connection identity and waits for a real response", async () => {
    const { client, socket } = await connected({ snapshot: false });
    socket.connection(0);
    await expect(client.request("control.acquire", {})).rejects.toMatchObject({
      code: "offline",
    });
    const estop = client.request("safety.estop", {});
    const sent = socket.lastSent();
    expect(sent).toMatchObject({ type: "safety.estop", connection_epoch: 0 });
    socket.receive(
      frame(
        "response",
        "robot-alpha",
        { request_type: "safety.estop", ok: true },
        String(sent.request_id),
      ),
    );
    await expect(estop).resolves.toMatchObject({ ok: true });
    const stop = client.request("drive.stop", {});
    expect(socket.lastSent()).toMatchObject({
      type: "drive.stop",
      connection_epoch: 0,
    });
    client.disconnect();
    await expect(stop).rejects.toMatchObject({ code: "offline" });
  });

  it("allows an urgent stop on a stale snapshot while rejecting ordinary commands", async () => {
    const { client, socket } = await connected({ snapshot: false });
    socket.receive(
      frame("snapshot", "robot-alpha", {
        ...wirePayload,
        connected: false,
        stale: true,
      }),
    );
    await expect(client.request("home.set", {})).rejects.toMatchObject({
      code: "offline",
    });
    const stop = client.request("drive.stop", {});
    expect(socket.lastSent()["type"]).toBe("drive.stop");
    client.disconnect();
    await expect(stop).rejects.toMatchObject({ code: "offline" });
  });

  it("still permits urgent stop after the cached snapshot times out", async () => {
    const { client, socket } = await connected({ staleAfterMs: 5_000 });
    await vi.advanceTimersByTimeAsync(5_001);
    await expect(client.request("home.set", {})).rejects.toMatchObject({
      code: "offline",
    });
    const stop = client.request("drive.stop", {});
    expect(socket.lastSent()["type"]).toBe("drive.stop");
    client.disconnect();
    await expect(stop).rejects.toMatchObject({ code: "offline" });
  });

  it("marks the last snapshot stale when the socket closes", async () => {
    const { client, socket } = await connected({ snapshot: false });
    const seen: RobotSnapshot[] = [];
    const statuses: LinkStatus[] = [];
    client.subscribe((value) => seen.push(value));
    client.subscribeStatus((value) => statuses.push(value));
    socket.receive(frame("snapshot", "robot-alpha", wirePayload));
    socket.serverClose(1008);
    expect(seen.at(-1)).toMatchObject({ stale: true, connected: false });
    expect(statuses.at(-1)).toEqual({
      state: "closed",
      robotId: "robot-alpha",
      closeCode: 1008,
    });
  });

  it("drops frames for another robot and malformed frames", async () => {
    const { client, socket } = await connected({ snapshot: false });
    const seen: RobotSnapshot[] = [];
    client.subscribe((value) => seen.push(value));
    socket.receive(frame("snapshot", "robot-beta", wirePayload));
    socket.receive("{not json");
    socket.receive(
      frame("snapshot", "robot-alpha", { ...wirePayload, task_revision: "7" }),
    );
    socket.receive({ ...frame("snapshot", "robot-alpha", wirePayload), v: 2 });
    expect(seen).toHaveLength(0);
  });

  it("ignores frames of the previous robot after switching", async () => {
    const { client, socket: first } = await connected({ snapshot: false });
    const seen: RobotSnapshot[] = [];
    client.subscribe((value) => seen.push(value));
    const next = client.connect("robot-beta");
    const second = FakeSocket.instances[1]!;
    expect(first.closedWith).toBeDefined();
    first.receive(frame("snapshot", "robot-alpha", wirePayload));
    second.open();
    await next;
    expect(seen).toHaveLength(0);
    second.receive(frame("snapshot", "robot-beta", wirePayload));
    expect(seen.map((value) => value.robot_id)).toEqual(["robot-beta"]);
  });

  it("rejects connect when the socket closes before opening", async () => {
    FakeSocket.instances = [];
    const client = new RobotClient({
      socketFactory: (url) => new FakeSocket(url),
    });
    const pending = client.connect("robot-alpha");
    const assertion = expect(pending).rejects.toMatchObject({
      code: "offline",
    });
    FakeSocket.instances[0]!.serverClose(1008);
    await assertion;
  });
});

// ---------------------------------------------------------------- cached snapshot in the page

describe("cached snapshot delivered by the cloud", () => {
  it("test_stale_snapshot_from_the_cloud_is_labeled_cache_in_the_page", async () => {
    FakeSocket.instances = [];
    const link = new RobotClient({
      socketFactory: (url) => new FakeSocket(url),
    });
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(FakeSocket.instances).toHaveLength(1));
    const socket = FakeSocket.instances[0]!;
    act(() => socket.open());

    act(() =>
      socket.receive(
        frame("snapshot", "robot-alpha", {
          ...wirePayload,
          connected: false,
          stale: true,
          execution_status: "cached-idle",
          last_synced_at: 1_700_000_000,
        }),
      ),
    );

    expect(screen.getByText("缓存快照 · 非实时")).toBeTruthy();
    expect(document.querySelector('[data-freshness="live"]')).toBeNull();
    expect(screen.queryByText("cached-idle")).toBeNull();
    expect(screen.getByText(/最后同步/).textContent).toContain(
      new Date(1_700_000_000 * 1000).toLocaleTimeString("zh-CN", {
        hour12: false,
      }),
    );
    link.disconnect();
  });

  it("test_command_uses_the_epoch_the_cloud_delivered", async () => {
    FakeSocket.instances = [];
    const link = new RobotClient({
      socketFactory: (url) => new FakeSocket(url),
    });
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(FakeSocket.instances).toHaveLength(1));
    const socket = FakeSocket.instances[0]!;
    act(() => socket.open());
    act(() => socket.connection(9));
    act(() =>
      socket.receive({
        ...frame("snapshot", "robot-alpha", {
          ...wirePayload,
          last_synced_at: Date.now() / 1000,
        }),
        connection_epoch: 9,
      }),
    );

    fireEvent.click(screen.getByRole("button", { name: "急停" }));

    await waitFor(() => {
      const commands = socket.sent
        .map((raw) => JSON.parse(raw) as Record<string, unknown>)
        .filter((frame) => frame.type === "safety.estop");
      expect(commands).toHaveLength(1);
      expect(commands[0]).toMatchObject({
        type: "safety.estop",
        connection_epoch: 9,
      });
    });
    link.disconnect();
  });
});

// ---------------------------------------------------------------- protocol parity

function pythonClass(source: string, name: string): string {
  const start = source.indexOf(`class ${name}(`);
  if (start < 0) throw new Error(`class ${name} not found`);
  const rest = source.slice(start + 1);
  const next = rest.search(/\nclass |\n\n\n\S/);
  return rest.slice(0, next < 0 ? undefined : next);
}

function quoted(text: string): string[] {
  return [...text.matchAll(/"([^"]+)"/g)].map((match) => match[1]!);
}

function literalOf(text: string): string[] {
  const match = /Literal\[([^\]]*)\]/.exec(text);
  if (match === null) throw new Error("no Literal found");
  return quoted(match[1]!);
}

function fieldsOf(body: string): string[] {
  return [...body.matchAll(/^ {4}(\w+): (.*)$/gm)]
    .filter((match) => !/=\s*PrivateAttr\(/.test(match[2]!))
    .map((match) => match[1]!);
}

describe("protocol parity with src/senior_control/protocol.py", () => {
  const root = resolve(import.meta.dirname, "../../../..");
  const protocol = readFileSync(
    resolve(root, "contracts/src/senior_contracts/wire.py"),
    "utf8",
  );
  const errors = readFileSync(
    resolve(root, "contracts/src/senior_contracts/errors.py"),
    "utf8",
  );

  it("mirrors the request types", () => {
    const simple = literalOf(pythonClass(protocol, "SimpleEnvelope"));
    const operations = quoted(
      /OPERATION_TYPES = \(([^)]*)\)/.exec(protocol)![1]!,
    );
    expect([...SIMPLE_REQUEST_TYPES]).toEqual(simple);
    expect([...OPERATION_REQUEST_TYPES]).toEqual(operations);
    const workspace = literalOf(
      /WorkspaceRequest = Literal\[[^\]]+\]/.exec(protocol)![0],
    );
    expect([...WORKSPACE_REQUEST_TYPES]).toEqual(workspace);
    expect([...REQUEST_TYPES]).toEqual([
      ...simple,
      ...operations,
      ...workspace,
    ]);
  });

  it("mirrors the response request types and error codes", () => {
    const responseBody = pythonClass(protocol, "ResponsePayload");
    const requests = literalOf(responseBody);
    expect(
      [
        ...new Set([...RESPONSE_REQUEST_TYPES, ...VEHICLE_REQUEST_TYPES]),
      ].sort(),
    ).toEqual([...requests].sort());
    for (const requestType of requests) {
      expect(
        parseInbound(
          JSON.stringify(
            frame("response", "robot-alpha", {
              request_type: requestType,
              ok: false,
              error: "unavailable",
            }),
          ),
        )?.type,
      ).toBe("response");
    }
    const codes = [
      ...pythonClass(errors, "ErrorCode").matchAll(/= "(\w+)"/g),
    ].map((m) => m[1]!);
    expect([...ERROR_CODES]).toEqual(codes);
  });

  it("mirrors the envelope and snapshot fields and the size bound", () => {
    expect([...ENVELOPE_FIELDS]).toEqual(
      fieldsOf(pythonClass(protocol, "EnvelopeBase")),
    );
    // The UI extension has its own decoder; the two owner fields are relay-only
    // and are cleared before browser delivery (test_manual_wire covers that boundary).
    expect([
      ...SNAPSHOT_FIELDS,
      "vehicle_ui",
      "manual_controller_generation",
      "manual_agent_epoch",
    ]).toEqual(fieldsOf(pythonClass(protocol, "SnapshotPayload")));
    expect(MAX_MESSAGE_BYTES).toBe(
      Number(/MAX_MESSAGE_BYTES = (\d+)/.exec(protocol)![1]),
    );
  });
});

describe("remote drive admission", () => {
  it("refuses the retired velocity route without sending it", async () => {
    const { client, socket } = await connected();
    const attempt = client.request("drive.velocity", { vx: 0.2, wz: 0 });
    const rejection = attempt.catch((error: unknown) => error);
    try {
      expect(socket.sent).toHaveLength(0);
      expect(await rejection).toMatchObject({ code: "validation" });
    } finally {
      client.disconnect();
    }
  });

  it("does not enable the joystick before control authority is acquired", async () => {
    FakeSocket.instances = [];
    const link = new RobotClient({
      socketFactory: (url) => new FakeSocket(url),
    });
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(FakeSocket.instances).toHaveLength(1));
    const socket = FakeSocket.instances[0]!;
    act(() => socket.open());
    act(() => socket.receive(frame("snapshot", "robot-alpha", wirePayload)));
    expect(
      screen
        .getByRole("group", { name: "驾驶摇杆" })
        .getAttribute("aria-disabled"),
    ).toBe("true");
  });

  it("marks an existing live view stale when the control link closes", async () => {
    const link = new FakeLink();
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
    link.emit(snapshot("robot-alpha"));
    expect(document.querySelector('[data-freshness="live"]')).toBeTruthy();
    link.emitStatus({
      state: "closed",
      robotId: "robot-alpha",
      closeCode: 1006,
    });
    expect(document.querySelector('[data-freshness="live"]')).toBeNull();
    expect(screen.getByText("缓存快照 · 非实时")).toBeTruthy();
  });
});

it("does not repaint a current robot with an older agent epoch after an offline frame", async () => {
  const { client, socket } = await connected({ snapshot: false });
  const seen: RobotSnapshot[] = [];
  client.subscribe((value) => seen.push(value));
  try {
    socket.connection(4);
    socket.receive({
      ...frame("snapshot", "robot-alpha", {
        ...wirePayload,
        execution_status: "new",
      }),
      connection_epoch: 4,
    });
    socket.receive({
      ...frame("snapshot", "robot-alpha", {
        ...wirePayload,
        execution_status: "old",
      }),
      connection_epoch: 3,
    });
    expect(seen.map((value) => value.execution_status)).toEqual(["new"]);
    socket.connection(0);
    socket.receive({
      ...frame("snapshot", "robot-alpha", {
        ...wirePayload,
        connected: false,
        stale: true,
        execution_status: "offline",
      }),
      connection_epoch: 0,
    });
    socket.receive({
      ...frame("snapshot", "robot-alpha", {
        ...wirePayload,
        execution_status: "old",
      }),
      connection_epoch: 3,
    });
    socket.receive({
      ...frame("snapshot", "robot-alpha", {
        ...wirePayload,
        execution_status: "old-again",
      }),
      connection_epoch: 4,
    });
    expect(seen.map((value) => value.execution_status)).toEqual([
      "new",
      "offline",
    ]);
  } finally {
    client.disconnect();
  }
});

describe("device pairing", () => {
  it("posts a pairing code request with the authenticated session and CSRF", async () => {
    const fetchImpl = vi
      .fn()
      .mockResolvedValueOnce(
        new Response(JSON.stringify({ csrf_token: "csrf" })),
      )
      .mockResolvedValueOnce(
        new Response(JSON.stringify({ code: "pair-once", expires_in: 30 }), {
          status: 201,
        }),
      );
    const api = new HttpControlApi(fetchImpl, null);
    await api.login("password");
    await expect(api.createPairingCode()).resolves.toEqual({
      code: "pair-once",
      expires_in: 30,
    });
    expect(fetchImpl).toHaveBeenLastCalledWith(
      "/api/pairing-codes",
      expect.objectContaining({
        method: "POST",
        credentials: "same-origin",
        headers: { "X-CSRF-Token": "csrf" },
      }),
    );
  });

  it.each([401, 403, 429, 201])(
    "handles HTTP %i without accepting invalid data",
    async (status) => {
      const api = new HttpControlApi(
        vi
          .fn()
          .mockResolvedValue(
            new Response(
              status === 201
                ? JSON.stringify({ code: "example", expires_in: 0.5 })
                : "",
              { status, headers: { "Retry-After": "12" } },
            ),
          ),
        null,
      );
      const result = api.createPairingCode();
      if (status === 401)
        await expect(result).rejects.toBeInstanceOf(UnauthorizedError);
      else if (status === 429)
        await expect(result).rejects.toMatchObject({
          name: "RateLimitedError",
          retryAfterSeconds: 12,
        });
      else
        await expect(result).rejects.toMatchObject({
          name: "ApiError",
          status,
        });
    },
  );

  it("keeps a new login's CSRF token when an older list refresh answers 401", async () => {
    let resolveRefresh!: (response: Response) => void;
    const staleRefresh = new Promise<Response>((resolve) => {
      resolveRefresh = resolve;
    });
    const fetchImpl = vi
      .fn()
      .mockResolvedValueOnce(
        new Response(JSON.stringify({ csrf_token: "old-csrf" })),
      )
      .mockReturnValueOnce(staleRefresh)
      .mockResolvedValueOnce(
        new Response(JSON.stringify({ csrf_token: "new-csrf" })),
      )
      .mockResolvedValueOnce(
        new Response(JSON.stringify({ code: "pair-once", expires_in: 30 }), {
          status: 201,
        }),
      );
    const api = new HttpControlApi(fetchImpl, null);
    await api.login("password");
    const outdated = api.listRobots();
    await api.login("password");
    resolveRefresh(new Response("", { status: 401 }));
    await expect(outdated).rejects.toBeInstanceOf(UnauthorizedError);
    await api.createPairingCode();
    expect(fetchImpl).toHaveBeenLastCalledWith(
      "/api/pairing-codes",
      expect.objectContaining({
        headers: { "X-CSRF-Token": "new-csrf" },
      }),
    );
  });

  it("offers pairing on an empty list and ignores repeated and closed requests", async () => {
    const api = new FakeApi();
    api.robots = [];
    let finish!: (value: { code: string; expires_in: number }) => void;
    const createPairingCode = vi.fn(
      () =>
        new Promise<{
          code: string;
          expires_in: number;
        }>((resolve) => {
          finish = resolve;
        }),
    );
    mount("/robots", Object.assign(api, { createPairingCode }), new FakeLink());
    await openDevices();
    await screen.findByText("尚未配对任何设备");
    fireEvent.click(screen.getAllByRole("button", { name: "配对新车" })[0]!);
    const generate = screen.getByRole("button", { name: "生成配对码" });
    fireEvent.click(generate);
    fireEvent.click(generate);
    expect(createPairingCode).toHaveBeenCalledTimes(1);
    fireEvent.click(screen.getByRole("button", { name: "关闭配对" }));
    await act(async () => finish({ code: "hidden-code", expires_in: 30 }));
    fireEvent.click(screen.getAllByRole("button", { name: "配对新车" })[0]!);
    expect(screen.queryByText("hidden-code")).toBeNull();
  });

  it("clears expired and logged-out codes", async () => {
    const createPairingCode = vi
      .fn()
      .mockResolvedValueOnce({ code: "first-code", expires_in: 1 })
      .mockResolvedValueOnce({ code: "second-code", expires_in: 60 });
    mount(
      "/robots",
      Object.assign(new FakeApi(), { createPairingCode }),
      new FakeLink(),
    );
    fireEvent.click(await screen.findByRole("button", { name: "配对新车" }));
    vi.useFakeTimers();
    try {
      await act(async () =>
        fireEvent.click(screen.getByRole("button", { name: "生成配对码" })),
      );
      expect(screen.getByText("first-code")).toBeTruthy();
      const command = screen.getByText(
        /python -m vehicle_manager.agent --pair/,
      ).textContent;
      expect(command).toBe(
        `python -m vehicle_manager.agent --pair --cloud-url 'wss://${window.location.host}/ws/agent' --device-name '巡检车' --credentials-file "$PWD/device-credentials.json"`,
      );
      await act(async () => vi.advanceTimersByTimeAsync(1_000));
      expect(screen.queryByText("first-code")).toBeNull();
      await act(async () =>
        fireEvent.click(screen.getByRole("button", { name: "重新生成配对码" })),
      );
      expect(screen.getByText("second-code")).toBeTruthy();
      await act(async () =>
        fireEvent.click(screen.getByRole("button", { name: "退出登录" })),
      );
      expect(screen.queryByText("second-code")).toBeNull();
    } finally {
      vi.useRealTimers();
    }
  });

  it("clears the code while logout is pending and submits logout only once", async () => {
    const createPairingCode = vi
      .fn()
      .mockResolvedValue({ code: "pending-code", expires_in: 30 });
    let finishLogout!: () => void;
    const logout = vi.fn(
      () =>
        new Promise<void>((resolve) => {
          finishLogout = resolve;
        }),
    );
    mount(
      "/robots",
      Object.assign(new FakeApi(), { createPairingCode, logout }),
      new FakeLink(),
    );
    fireEvent.click(await screen.findByRole("button", { name: "配对新车" }));
    fireEvent.click(screen.getByRole("button", { name: "生成配对码" }));
    await screen.findByText("pending-code");
    const button = screen.getByRole("button", { name: "退出登录" });
    fireEvent.click(button);
    fireEvent.click(button);
    expect(logout).toHaveBeenCalledTimes(1);
    expect(screen.queryByText("pending-code")).toBeNull();
    await act(async () => finishLogout());
    expect(await screen.findByLabelText("管理员密码")).toBeTruthy();
  });

  it("clears a pairing code on return from a suspended page", async () => {
    const createPairingCode = vi
      .fn()
      .mockResolvedValue({ code: "sleep-code", expires_in: 30 });
    mount(
      "/robots",
      Object.assign(new FakeApi(), { createPairingCode }),
      new FakeLink(),
    );
    fireEvent.click(await screen.findByRole("button", { name: "配对新车" }));
    vi.useFakeTimers();
    try {
      await act(async () =>
        fireEvent.click(screen.getByRole("button", { name: "生成配对码" })),
      );
      expect(screen.getByText("sleep-code")).toBeTruthy();
      vi.setSystemTime(Date.now() + 31_000);
      fireEvent(document, new Event("visibilitychange"));
      expect(screen.queryByText("sleep-code")).toBeNull();
      expect(
        screen.getByRole("button", { name: "重新生成配对码" }),
      ).toBeTruthy();
    } finally {
      vi.useRealTimers();
    }
  });

  it("allows retry after a rate limit and redirects on expired auth", async () => {
    const createPairingCode = vi
      .fn()
      .mockRejectedValueOnce(new RateLimitedError("limited", 12))
      .mockRejectedValueOnce(new UnauthorizedError("login required"));
    mount(
      "/robots",
      Object.assign(new FakeApi(), { createPairingCode }),
      new FakeLink(),
    );
    fireEvent.click(await screen.findByRole("button", { name: "配对新车" }));
    fireEvent.click(screen.getByRole("button", { name: "生成配对码" }));
    expect(await screen.findByText(/12 秒/)).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "生成配对码" }));
    expect(await screen.findByLabelText("管理员密码")).toBeTruthy();
  });

  it("offers reauthentication when the session cookie is present but CSRF is unavailable", async () => {
    const createPairingCode = vi
      .fn()
      .mockRejectedValue(new ApiError("csrf token required", 403));
    const api = Object.assign(new FakeApi(), { createPairingCode });
    mount("/robots", api, new FakeLink());
    fireEvent.click(await screen.findByRole("button", { name: "配对新车" }));
    fireEvent.click(screen.getByRole("button", { name: "生成配对码" }));
    fireEvent.click(await screen.findByRole("button", { name: "重新登录" }));
    expect(await screen.findByLabelText("管理员密码")).toBeTruthy();
    expect(api.loggedOut).toBe(0);
  });

  it("hides real pairing in demo mode", async () => {
    const demo = createDemoBackend();
    mount("/robots", demo.api, demo.link, true);
    await openDevices();
    await screen.findByText("演示 · 仓库车 A");
    expect(screen.queryByRole("button", { name: "配对新车" })).toBeNull();
  });
});

describe("operator workspace", () => {
  async function openWorkspace() {
    const link = new FakeLink();
    link.requestImpl = async () => ({ ok: true });
    Object.assign(link, {
      vehicle: testPort(async (type, payload) => {
        link.requests.push({ type, payload });
        return { authority_id: "prepared", state: "prepared" };
      }),
    });
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
    link.emit(snapshot("robot-alpha"));
    return link;
  }

  it("test_keeps_manual_controls_mounted_on_primary_view_switch", async () => {
    await openWorkspace();
    expect(screen.queryByRole("region", { name: "实时视频" })).toBeNull();
    const stick = screen.getByRole("group", { name: "驾驶摇杆" });
    const stop = screen.getByRole("button", { name: "获取控制权" });
    fireEvent.click(screen.getByRole("switch", { name: "摄像头" }));
    fireEvent.click(screen.getByRole("button", { name: "交换主次画面" }));
    expect(screen.getByRole("region", { name: "实时视频" })).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "交换主次画面" }));
    fireEvent.click(screen.getByRole("switch", { name: "摄像头" }));
    expect(screen.queryByRole("region", { name: "实时视频" })).toBeNull();
    expect(screen.getByRole("group", { name: "驾驶摇杆" })).toBe(stick);
    expect(screen.getByRole("button", { name: "获取控制权" })).toBe(stop);
  });

  it("test_leaving_manual_revokes_gesture_without_rearm", async () => {
    const link = await openWorkspace();
    await act(async () =>
      fireEvent.click(screen.getByRole("button", { name: "获取控制权" })),
    );
    const stick = screen.getByRole("group", { name: "驾驶摇杆" });
    fireEvent.keyDown(stick, { key: "ArrowUp" });
    await act(async () =>
      fireEvent.click(screen.getByRole("button", { name: "导航与定位" })),
    );
    expect(
      link.requests.filter(({ type }) => type === "drive.stop"),
    ).toHaveLength(0);
    expect(
      link.requests.filter(({ type }) => type === "control.release"),
    ).toHaveLength(1);
    expect(screen.getByText("未获取控制")).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "手动驾驶" }));
    expect(screen.getByRole("button", { name: "获取控制权" })).toBeTruthy();
    fireEvent.keyDown(stick, { key: "ArrowUp" });
    expect(
      link.requests.filter(({ type }) => type === "drive.intent"),
    ).toHaveLength(1);
  });
});

it("keeps an exported map read-only and requires task runtime before cancellation", async () => {
  const link = new FakeLink();
  const current = snapshot("robot-alpha");
  const identity = {
    robot_id: current.robot_id,
    map_ref: current.map_ref,
    grid_id: "export-1",
  };
  link.requestImpl = async (type) => {
    if (type === "map.manifest")
      return {
        data: {
          ...identity,
          width: 1,
          height: 1,
          resolution: 1,
          origin: { x: 0, y: 0, yaw: 0 },
          tile_size: 32,
          display_only: true,
        },
      };
    if (type === "map.tile")
      return {
        data: {
          ...identity,
          tile_x: 0,
          tile_y: 0,
          width: 1,
          height: 1,
          cells: [0],
        },
      };
    return { ok: true };
  };
  mount("/robots/robot-alpha", new FakeApi(), link);
  await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
  link.emit(current);
  fireEvent.click(screen.getByRole("button", { name: "导航与定位" }));
  await screen.findByText("地图导出副本 · 仅供查看");
  expect(
    screen
      .getByRole("application", { name: "地图" })
      .getAttribute("data-picking"),
  ).toBe("false");
  for (const name of ["设为 Home", "返航", "确认目标"]) {
    expect(
      (screen.getByRole("button", { name }) as HTMLButtonElement).disabled,
    ).toBe(true);
  }
  expect(
    (screen.getByRole("button", { name: "取消导航" }) as HTMLButtonElement)
      .disabled,
  ).toBe(true);
  fireEvent.click(screen.getByRole("button", { name: "任务管理" }));
  await screen.findByText("地图导出副本 · 仅供查看");
  expect(
    (screen.getByText("提交任务").closest("button") as HTMLButtonElement)
      .disabled,
  ).toBe(true);
  expect(link.requests.some(({ type }) => type === "waypoint.list")).toBe(
    false,
  );
});

it("sends an eligible goal and return with the current map fence", async () => {
  const link = new FakeLink();
  const base = snapshot("robot-alpha");
  const current = {
    ...base,
    home: { waypoint_id: "home", status: "confirmed", map_ref: base.map_ref! },
  };
  const identity = {
    robot_id: current.robot_id,
    map_ref: current.map_ref,
    grid_id: "live-grid",
  };
  link.requestImpl = async (type) => {
    if (type === "map.manifest")
      return {
        data: {
          ...identity,
          width: 1,
          height: 1,
          resolution: 1,
          origin: { x: 0, y: 0, yaw: 0 },
          tile_size: 32,
          navigation_ready: true,
          waypoint_writes: true,
        },
      };
    if (type === "map.tile")
      return {
        data: {
          ...identity,
          tile_x: 0,
          tile_y: 0,
          width: 1,
          height: 1,
          cells: [0],
        },
      };
    return { ok: true };
  };
  mount("/robots/robot-alpha", new FakeApi(), link);
  await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
  link.emit(current);
  fireEvent.click(screen.getByRole("button", { name: "导航与定位" }));
  await waitFor(() =>
    expect(
      (screen.getByRole("button", { name: "返航" }) as HTMLButtonElement)
        .disabled,
    ).toBe(false),
  );
  fireEvent.change(screen.getByLabelText("X (m)"), {
    target: { value: "0.5" },
  });
  fireEvent.change(screen.getByLabelText("Y (m)"), {
    target: { value: "0.5" },
  });
  fireEvent.click(screen.getByRole("button", { name: "确认目标" }));
  await screen.findByText(/车端已受理目标/);
  fireEvent.click(screen.getByRole("button", { name: "返航" }));
  await screen.findByText(/车端已受理返航/);
  expect(link.requests).toEqual(
    expect.arrayContaining([
      {
        type: "navigation.goal",
        payload: expect.objectContaining({
          body: {
            ...current.map_ref,
            grid_id: "live-grid",
            x: 0.5,
            y: 0.5,
            yaw: 0,
          },
        }),
      },
      {
        type: "navigation.return_home",
        payload: expect.objectContaining({
          body: { ...current.map_ref, grid_id: "live-grid" },
        }),
      },
    ]),
  );
});

it("validates optional snapshot navigation readiness", () => {
  expect(
    parseInbound(JSON.stringify(frame("snapshot", "robot-alpha", wirePayload)))
      ?.payload,
  ).not.toHaveProperty("navigation_ready");
  for (const navigation_ready of [false, true]) {
    expect(
      parseInbound(
        JSON.stringify(
          frame("snapshot", "robot-alpha", {
            ...wirePayload,
            navigation_ready,
          }),
        ),
      )?.payload,
    ).toHaveProperty("navigation_ready", navigation_ready);
  }
  for (const navigation_ready of [null, "true", 1]) {
    expect(
      parseInbound(
        JSON.stringify(
          frame("snapshot", "robot-alpha", {
            ...wirePayload,
            navigation_ready,
          }),
        ),
      ),
    ).toBeNull();
  }
});

it("updates navigation readiness from snapshots without reloading the map", async () => {
  const link = new FakeLink();
  const base = snapshot("robot-alpha");
  const current = {
    ...base,
    map_generation: "live-grid",
    home: { waypoint_id: "home", status: "confirmed", map_ref: base.map_ref! },
  };
  const identity = {
    robot_id: current.robot_id,
    map_ref: current.map_ref,
    grid_id: current.map_generation,
  };
  link.requestImpl = async (type) => {
    if (type === "map.manifest")
      return {
        data: {
          ...identity,
          width: 1,
          height: 1,
          resolution: 1,
          origin: { x: 0, y: 0, yaw: 0 },
          tile_size: 32,
          navigation_ready: true,
        },
      };
    if (type === "map.tile")
      return {
        data: {
          ...identity,
          tile_x: 0,
          tile_y: 0,
          width: 1,
          height: 1,
          cells: [0],
        },
      };
    return { ok: true };
  };
  mount("/robots/robot-alpha", new FakeApi(), link);
  await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
  link.emit({ ...current, navigation_ready: false });
  fireEvent.click(screen.getByRole("button", { name: "导航与定位" }));
  await waitFor(() =>
    expect(
      screen
        .getByRole("application", { name: "地图" })
        .getAttribute("data-picking"),
    ).toBe("true"),
  );
  const returnHome = () =>
    screen.getByRole("button", { name: "返航" }) as HTMLButtonElement;
  expect(returnHome().disabled).toBe(true);
  link.emit({ ...current, navigation_ready: true });
  expect(returnHome().disabled).toBe(false);
  fireEvent.change(screen.getByLabelText("X (m)"), {
    target: { value: "0.5" },
  });
  fireEvent.change(screen.getByLabelText("Y (m)"), {
    target: { value: "0.5" },
  });
  const goal = () =>
    screen.getByRole("button", { name: "确认目标" }) as HTMLButtonElement;
  expect(goal().disabled).toBe(false);
  link.emit({
    ...current,
    navigation_ready: true,
    route_progress: {
      execution_id: "route-1",
      task_id: null,
      map_generation: "live-grid",
      status: "cancel_pending",
      active_waypoint_index: 0,
    },
  });
  expect(returnHome().disabled).toBe(true);
  expect(goal().disabled).toBe(true);
  expect(
    (screen.getByRole("button", { name: "取消导航" }) as HTMLButtonElement)
      .disabled,
  ).toBe(true);
  link.emit({ ...current, navigation_ready: false });
  expect(returnHome().disabled).toBe(true);
  expect(link.requests.map(({ type }) => type)).toEqual([
    "map.manifest",
    "map.tile",
  ]);
});

it("validates bounded finite live map telemetry and defaults legacy snapshots", () => {
  const parse = (payload: Record<string, unknown>) =>
    parseInbound(
      JSON.stringify(
        frame("snapshot", "robot-alpha", { ...wirePayload, ...payload }),
      ),
    );
  expect(parse({})?.payload).toMatchObject({
    robot_pose: null,
    scan_points: [],
  });
  expect(parse({ robot_pose: null, scan_points: null })?.payload).toMatchObject(
    { robot_pose: null, scan_points: [] },
  );
  const pose = { x: 1, y: -2, yaw: 0.4 };
  const points = Array.from({ length: 96 }, (_, x) => ({ x, y: 2 }));
  expect(
    parse({ robot_pose: pose, scan_points: points })?.payload,
  ).toMatchObject({ robot_pose: pose, scan_points: points });
  for (const invalid of [
    { robot_pose: { ...pose, yaw: null } },
    { scan_points: [...points, { x: 0, y: 0 }] },
    { scan_points: [{ x: 0, y: "1" }] },
    { robot_pose: [] },
  ]) {
    expect(parse(invalid)).toBeNull();
  }
  expect(
    parseInbound(
      JSON.stringify(
        frame("snapshot", "robot-alpha", { ...wirePayload, robot_pose: pose }),
      ).replace('"yaw":0.4', '"yaw":1e400'),
    ),
  ).toBeNull();
});

it.each([false, true])(
  "shows only telemetry qualified for the displayed map (export copy: %s)",
  async (displayOnly) => {
    const link = new FakeLink();
    const current = snapshot("robot-alpha", {
      map_generation: "live-grid",
      robot_pose: { x: 0.2, y: 0.4, yaw: 0 },
      scan_points: [{ x: 0.5, y: 0.5 }],
    });
    const identity = {
      robot_id: current.robot_id,
      map_ref: current.map_ref,
      grid_id: "live-grid",
    };
    link.requestImpl = async (type) => {
      if (type === "map.manifest")
        return {
          data: {
            ...identity,
            width: 1,
            height: 1,
            resolution: 1,
            origin: { x: 0, y: 0, yaw: 0 },
            tile_size: 32,
            display_only: displayOnly,
          },
        };
      if (type === "map.tile")
        return {
          data: {
            ...identity,
            tile_x: 0,
            tile_y: 0,
            width: 1,
            height: 1,
            cells: [0],
          },
        };
      return { ok: true };
    };
    const { container } = mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
    link.emit(current);
    fireEvent.click(screen.getByRole("button", { name: "导航与定位" }));
    if (displayOnly) {
      await screen.findByText("地图导出副本 · 仅供查看");
      expect(screen.queryByLabelText("实时激光命中点")).toBeNull();
      expect(container.querySelector('[data-marker="robot"]')).toBeNull();
      expect(screen.queryByText("车辆位置暂不可用")).not.toBeNull();
      return;
    }
    await waitFor(() =>
      expect(screen.queryByLabelText("实时激光命中点")).not.toBeNull(),
    );
    const first = container
      .querySelector('[data-marker="robot"]')!
      .getAttribute("transform");
    link.emit({ ...current, robot_pose: { x: 0.6, y: 0.8, yaw: Math.PI / 2 } });
    expect(
      container
        .querySelector('[data-marker="robot"]')!
        .getAttribute("transform"),
    ).not.toBe(first);
    const { map_generation: _generation, ...legacy } = current;
    link.emit(legacy);
    expect(screen.queryByLabelText("实时激光命中点")).toBeNull();
    expect(container.querySelector('[data-marker="robot"]')).toBeNull();
    for (const overrides of [
      { localized: false },
      { stale: true },
      { connected: false },
      { map_generation: "other-grid" },
      { map_ref: { map_id: "other", reference_revision: 2 } },
    ]) {
      link.emit({ ...current, ...overrides });
      expect(screen.queryByLabelText("实时激光命中点")).toBeNull();
      expect(container.querySelector('[data-marker="robot"]')).toBeNull();
    }
  },
);

it("validates optional navigation block diagnostics", () => {
  const parse = (navigation_block_reason?: unknown) =>
    parseInbound(
      JSON.stringify(
        frame("snapshot", "robot-alpha", {
          ...wirePayload,
          navigation_block_reason,
        }),
      ),
    );
  expect(parse()?.payload).toMatchObject({ navigation_block_reason: null });
  expect(parse(null)?.payload).toMatchObject({ navigation_block_reason: null });
  expect(parse("planner state unavailable")?.payload).toMatchObject({
    navigation_block_reason: "planner state unavailable",
  });
  expect(parse("x".repeat(256))).not.toBeNull();
  for (const value of ["", "x".repeat(257), 1, false, {}])
    expect(parse(value)).toBeNull();
});

it("explains the vehicle's concrete navigation blocker", async () => {
  const link = new FakeLink();
  mount("/robots/robot-alpha", new FakeApi(), link);
  await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
  link.emit(snapshot("robot-alpha", { navigation_ready: false }));
  fireEvent.click(screen.getByRole("button", { name: "导航与定位" }));
  expect(
    screen.queryByText("车端未报告已验证的导航能力，无法开始、继续或返航。"),
  ).not.toBeNull();
  for (const [reason, explanation] of [
    [
      "map or localization generation changed",
      "地图或定位代次已变化，请等待重新定位。",
    ],
    ["map or localization unavailable", "当前地图或定位尚未就绪。"],
    ["planner state unavailable", "路径规划器或运动控制器尚未就绪。"],
    [
      "safety status unavailable (scan stale)",
      "传感器安全检查尚未通过：scan stale。",
    ],
    [
      "heartbeat or control authority unavailable",
      "车辆心跳或控制权限尚未就绪。",
    ],
    ["navigation unavailable", "车端导航尚未就绪。"],
  ] as const) {
    link.emit(
      snapshot("robot-alpha", {
        navigation_ready: false,
        navigation_block_reason: reason,
      }),
    );
    expect(screen.queryByText(`无法导航：${explanation}`)).not.toBeNull();
    expect(
      screen.queryByText("车端未报告已验证的导航能力，无法开始、继续或返航。"),
    ).toBeNull();
  }
});

// ---------------------------------------------------------------- revoke pairing

describe("device unpairing", () => {
  it("deletes with session, CSRF and an encoded robot ID", async () => {
    const fetchImpl = vi
      .fn()
      .mockResolvedValueOnce(reply(200, { csrf_token: "csrf-delete" }))
      .mockResolvedValueOnce(reply(204))
      .mockResolvedValueOnce(reply(401));
    const api = new HttpControlApi(fetchImpl, null);
    await api.login("pw");
    await api.revokeRobot("robot/a b");
    expect(fetchImpl).toHaveBeenLastCalledWith(
      "/api/robots/robot%2Fa%20b",
      expect.objectContaining({
        method: "DELETE",
        credentials: "same-origin",
        headers: { "X-CSRF-Token": "csrf-delete" },
      }),
    );
    await expect(api.revokeRobot("robot-alpha")).rejects.toBeInstanceOf(
      UnauthorizedError,
    );
  });

  it("confirms credential invalidation, prevents duplicates and exits the removed robot", async () => {
    let finish!: () => void;
    const revokeRobot = vi.fn(
      () =>
        new Promise<void>((resolve) => {
          finish = resolve;
        }),
    );
    const api = Object.assign(new FakeApi(), { revokeRobot });
    const link = new FakeLink();
    mount("/robots/robot-alpha", api, link);
    await waitFor(() => expect(link.connects).toHaveLength(1));
    fireEvent.click(screen.getByRole("button", { name: "切换车辆" }));
    fireEvent.click(
      screen.getByRole("button", { name: "删除 Alpha 仓库车 的配对" }),
    );
    expect(screen.getByText(/车端凭据将失效/)).toBeTruthy();
    expect(revokeRobot).not.toHaveBeenCalled();
    const confirm = screen.getByRole("button", { name: "确认删除配对" });
    fireEvent.click(confirm);
    fireEvent.click(confirm);
    expect(revokeRobot).toHaveBeenCalledExactlyOnceWith("robot-alpha");
    expect((confirm as HTMLButtonElement).disabled).toBe(true);
    await act(async () => finish());
    await screen.findByRole("heading", { name: "选择车辆，开始工作" });
    expect(screen.queryByRole("link", { name: /Alpha/ })).toBeNull();
    await openDevices();
    expect(screen.getByRole("link", { name: /Beta/ })).toBeTruthy();
    expect(link.disconnects).toBeGreaterThanOrEqual(1);
  });

  it("keeps the device after failure and allows retry or cancellation", async () => {
    const revokeRobot = vi.fn().mockRejectedValue(new ApiError("failed", 500));
    mount(
      "/robots",
      Object.assign(new FakeApi(), { revokeRobot }),
      new FakeLink(),
    );
    await openDevices();
    await screen.findByRole("link", { name: /Alpha/ });
    fireEvent.click(
      screen.getByRole("button", { name: "删除 Alpha 仓库车 的配对" }),
    );
    fireEvent.click(screen.getByRole("button", { name: "确认删除配对" }));
    await screen.findByText(/删除配对失败/);
    expect(screen.getByRole("link", { name: /Alpha/ })).toBeTruthy();
    expect(
      (
        screen.getByRole("button", {
          name: "确认删除配对",
        }) as HTMLButtonElement
      ).disabled,
    ).toBe(false);
    fireEvent.click(screen.getByRole("button", { name: "取消" }));
    expect(screen.queryByRole("button", { name: "确认删除配对" })).toBeNull();
  });

  it("keeps another selected robot and rejects an older refresh that still contains the deleted one", async () => {
    const api = new FakeApi();
    const link = new FakeLink();
    const listRobots = vi.spyOn(api, "listRobots");
    mount("/robots/robot-alpha", api, link);
    await waitFor(() => expect(link.connects).toHaveLength(1));
    let finishRefresh!: (robots: RobotSummary[]) => void;
    listRobots.mockImplementationOnce(
      () =>
        new Promise((resolve) => {
          finishRefresh = resolve;
        }),
    );
    fireEvent.click(screen.getByRole("button", { name: "切换车辆" }));
    await openDevices();
    fireEvent.click(screen.getByRole("button", { name: "刷新设备" }));
    fireEvent.click(
      screen.getByRole("button", { name: "删除 Beta 装卸车 的配对" }),
    );
    fireEvent.click(screen.getByRole("button", { name: "确认删除配对" }));
    await waitFor(() =>
      expect(screen.queryByRole("link", { name: /Beta/ })).toBeNull(),
    );
    await act(async () => finishRefresh([ALPHA, BETA]));
    expect(screen.queryByRole("link", { name: /Beta/ })).toBeNull();
    expect(link.connects).toEqual(["robot-alpha"]);
    expect(
      screen.getByRole("button", { name: "切换车辆" }).textContent,
    ).toContain(ALPHA.name);
  });

  it("hides deletion in demo mode", async () => {
    mount("/robots", new FakeApi(), new FakeLink(), true);
    await openDevices();
    await screen.findByRole("link", { name: /Alpha/ });
    expect(screen.queryByRole("button", { name: /删除.*配对/ })).toBeNull();
  });

  it("returns to login when revocation discovers an expired session", async () => {
    const revokeRobot = vi
      .fn()
      .mockRejectedValue(new UnauthorizedError("expired"));
    mount(
      "/robots",
      Object.assign(new FakeApi(), { revokeRobot }),
      new FakeLink(),
    );
    await openDevices();
    await screen.findByRole("link", { name: /Alpha/ });
    fireEvent.click(
      screen.getByRole("button", { name: "删除 Alpha 仓库车 的配对" }),
    );
    fireEvent.click(screen.getByRole("button", { name: "确认删除配对" }));
    await screen.findByRole("heading", { name: "登录控制台" });
  });
});

it("refreshes grid data within one reference without losing drafts or accepting late tiles", async () => {
  const parse = (map_grid_id?: unknown) =>
    parseInbound(
      JSON.stringify(
        frame("snapshot", "robot-alpha", { ...wirePayload, map_grid_id }),
      ),
    );
  expect(parse()?.payload).not.toHaveProperty("map_grid_id");
  expect(parse("grid-1")?.payload).toHaveProperty("map_grid_id", "grid-1");
  for (const invalid of [null, "", "x".repeat(129), 1])
    expect(parse(invalid)).toBeNull();
  const link = new FakeLink();
  const current = snapshot("robot-alpha", {
    map_generation: "reference-1",
    map_grid_id: "grid-1",
    navigation_ready: true,
    robot_pose: { x: 0.2, y: 0.4, yaw: 0 },
    home: {
      waypoint_id: "home",
      status: "confirmed",
      map_ref: { map_id: "map-1", reference_revision: 2 },
    },
  });
  let activeGrid = "grid-1";
  let firstTile = true;
  let releaseOld!: (value: unknown) => void;
  let releaseGoal!: (value: unknown) => void;
  const oldTile = new Promise((resolve) => {
    releaseOld = resolve;
  });
  const goalReply = new Promise((resolve) => {
    releaseGoal = resolve;
  });
  const identity = (grid_id: string) => ({
    robot_id: current.robot_id,
    map_ref: current.map_ref,
    grid_id,
  });
  const tile = (grid_id: string) => ({
    data: {
      ...identity(grid_id),
      tile_x: 0,
      tile_y: 0,
      width: 1,
      height: 1,
      cells: [0],
    },
  });
  link.requestImpl = async (type, payload) => {
    if (type === "map.manifest")
      return {
        data: {
          ...identity(activeGrid),
          width: 1,
          height: 1,
          resolution: 1,
          origin: { x: 0, y: 0, yaw: 0 },
          tile_size: 32,
          navigation_ready: true,
        },
      };
    if (type === "map.tile") {
      const requested = (payload as { grid_id: string }).grid_id;
      if (requested === "grid-1" && !firstTile) return oldTile;
      firstTile = false;
      return tile(requested);
    }
    if (type === "navigation.goal") return goalReply;
    return { ok: true };
  };
  const { container } = mount("/robots/robot-alpha", new FakeApi(), link);
  await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
  link.emit(current);
  fireEvent.click(screen.getByRole("button", { name: "导航与定位" }));
  await waitFor(() =>
    expect(container.querySelector('[data-marker="robot"]')).not.toBeNull(),
  );
  fireEvent.change(screen.getByLabelText("X (m)"), {
    target: { value: "0.5" },
  });
  fireEvent.change(screen.getByLabelText("Y (m)"), {
    target: { value: "0.5" },
  });
  const confirm = () =>
    screen.getByRole("button", { name: "确认目标" }) as HTMLButtonElement;
  fireEvent.click(confirm());
  expect(confirm().disabled).toBe(true);
  fireEvent.click(screen.getByRole("button", { name: "刷新地图" }));
  await waitFor(() =>
    expect(
      link.requests.filter(({ type }) => type === "map.tile"),
    ).toHaveLength(2),
  );
  activeGrid = "grid-2";
  link.emit({ ...current, map_grid_id: "grid-2" });
  // Existing workspace reads serialize per link: retire the old response before the new cut loads.
  expect(confirm().disabled).toBe(true);
  await act(async () => {
    releaseOld(tile("grid-1"));
    await oldTile;
  });
  await waitFor(() =>
    expect(
      link.requests.filter(({ type }) => type === "map.tile"),
    ).toHaveLength(3),
  );
  await waitFor(() =>
    expect(container.querySelector('[data-marker="robot"]')).not.toBeNull(),
  );
  expect((screen.getByLabelText("X (m)") as HTMLInputElement).value).toBe(
    "0.5",
  );
  expect(confirm().disabled).toBe(true);
  expect(confirm().disabled).toBe(true);
  await act(async () => {
    releaseGoal({ ok: true });
    await goalReply;
  });
  expect(confirm().disabled).toBe(false);
  expect((screen.getByLabelText("X (m)") as HTMLInputElement).value).toBe(
    "0.5",
  );
  fireEvent.click(confirm());
  await waitFor(() =>
    expect(
      link.requests.filter(({ type }) => type === "navigation.goal"),
    ).toHaveLength(2),
  );
  expect(
    link.requests.filter(({ type }) => type === "navigation.goal")[1]?.payload,
  ).toMatchObject({ body: { grid_id: "grid-2", x: 0.5, y: 0.5 } });
});

it("V1/V2 starts with a central empty page and closes the anchored picker by outside, Escape and selection", async () => {
  const link = new FakeLink();
  mount("/robots", new FakeApi(), link);
  await screen.findByRole("heading", { name: "选择车辆，开始工作" });
  expect(screen.queryByRole("navigation", { name: "设备列表" })).toBeNull();
  expect(link.connects).toEqual([]);
  expect(screen.queryByRole("group", { name: "驾驶摇杆" })).toBeNull();
  await openDevices();
  fireEvent.pointerDown(document.body);
  expect(screen.queryByRole("navigation", { name: "设备列表" })).toBeNull();
  await openDevices();
  fireEvent.keyDown(document, { key: "Escape" });
  expect(screen.queryByRole("navigation", { name: "设备列表" })).toBeNull();
  expect(document.activeElement).toBe(
    screen.getAllByRole("button", { name: "选择车辆" })[0],
  );
  await openDevices();
  fireEvent.click(screen.getByRole("link", { name: /Beta/ }));
  await waitFor(() => expect(link.connects).toEqual(["robot-beta"]));
  expect(screen.queryByRole("navigation", { name: "设备列表" })).toBeNull();
  await openDevices();
  fireEvent.click(screen.getByRole("button", { name: "取消选择" }));
  await screen.findByRole("heading", { name: "选择车辆，开始工作" });
  expect(link.disconnects).toBeGreaterThan(0);
});

it("V4/V6 keeps connection and first snapshot states in a single viewport without old article or sync wrappers", async () => {
  const link = new FakeLink();
  let opened!: () => void;
  link.connect = async (id) => {
    link.connects.push(id);
    await new Promise<void>((resolve) => {
      opened = resolve;
    });
  };
  const { container } = mount("/robots/robot-alpha", new FakeApi(), link);
  await screen.findByRole("heading", { name: "正在建立控制通道…" });
  const viewport = container.querySelector("[data-freshness]");
  expect(screen.queryByRole("group", { name: "驾驶摇杆" })).toBeNull();
  await act(async () => opened());
  await screen.findByRole("heading", { name: "等待车端快照" });
  expect(container.querySelector("[data-freshness]")).toBe(viewport);
  expect(screen.queryByRole("button", { name: "取消等待" })).toBeNull();
  link.emit(snapshot("robot-alpha"));
  expect(container.querySelector("[data-freshness]")).toBe(viewport);
  expect(container.querySelector("article[data-workspace]")).toBeNull();
  expect(screen.queryByText(/最后同步/)).toBeNull();
  fireEvent.click(screen.getByRole("button", { name: "设置" }));
  expect(screen.getByRole("navigation", { name: "设置分类" })).toBeTruthy();
  fireEvent.click(screen.getByRole("button", { name: "状态与性能" }));
  expect(screen.getByText("车辆诊断与固定回波校准")).toBeTruthy();
});

it("keeps the approved login reveal, busy guard and successful device initialization on the real authentication chain", async () => {
  const api = new FakeApi();
  api.failure = new UnauthorizedError("login required");
  let finish!: () => void;
  api.login = vi.fn(async (password) => {
    api.logins.push(password);
    await new Promise<void>((resolve) => {
      finish = resolve;
    });
  });
  mount("/robots", api, new FakeLink());
  await screen.findByRole("heading", { name: "登录控制台" });
  const password = screen.getByLabelText("管理员密码") as HTMLInputElement;
  fireEvent.change(password, { target: { value: "never-log-this" } });
  fireEvent.click(screen.getByRole("button", { name: "显示密码" }));
  expect(password.type).toBe("text");
  expect(password.value).toBe("never-log-this");
  expect(api.logins).toEqual([]);
  fireEvent.click(screen.getByRole("button", { name: "隐藏密码" }));
  expect(password.type).toBe("password");
  fireEvent.submit(password.closest("form")!);
  fireEvent.submit(password.closest("form")!);
  expect(api.logins).toHaveLength(1);
  expect(
    (screen.getByRole("button", { name: "验证中…" }) as HTMLButtonElement)
      .disabled,
  ).toBe(true);
  api.failure = null;
  await act(async () => finish());
  await screen.findByRole("heading", { name: "选择车辆，开始工作" });
  expect(password.value).toBe("");
  await openDevices();
  expect(screen.getByRole("link", { name: /Alpha/ })).toBeTruthy();
});

it('V24 retains the uncertain calibration request across settings categories and the logs page', async () => {
  const status: LearningStatus = {
    run_id:'b'.repeat(32), active_revision:1,
    active_model:{attempt_id:'c'.repeat(32),model_id:'d'.repeat(32),point_count:72},
    attempt_id:'c'.repeat(32),phase:'completed',reason:'classified_body_returns',
    motion_complete:true,capture_complete:true,compute_complete:true,durable:true,
    can_start:true,cannot_start_reason:null
  };
  let starts=0, original=''; const queries:unknown[]=[];
  const port = new VehicleTransport(async (type,payload,requestId) => {
    if(type==='settings.get') return settingsFixture;
    if(type==='learning.start'){starts++;original=requestId!;throw new Error('timeout');}
    if(type==='learning.status'){queries.push(payload);return status;}
    throw new Error('Unexpected fixture request');
  });
  port.update(runtime({capabilities:['runtime.v1','settings.v1','learning.v1']}),true);
  const link=Object.assign(new FakeLink(),{vehicle:port});
  mount('/robots/robot-alpha',new FakeApi(),link);
  await waitFor(()=>expect(link.connects).toHaveLength(1));
  link.emit(snapshot('robot-alpha'));
  fireEvent.click(screen.getByRole('button',{name:'设置'}));
  fireEvent.click(screen.getByRole('button',{name:'状态与性能'}));
  fireEvent.click(screen.getByText('车辆诊断与固定回波校准'));
  const start=screen.getByRole('button',{name:'重新校准'});
  await waitFor(()=>expect((start as HTMLButtonElement).disabled).toBe(false));
  await act(async()=>fireEvent.click(start));
  expect(starts).toBe(1);
  fireEvent.click(screen.getByRole('button',{name:'驾驶操控'}));
  fireEvent.click(screen.getByRole('button',{name:'日志'}));
  fireEvent.click(screen.getByRole('button',{name:'设置'}));
  fireEvent.click(screen.getByRole('button',{name:'状态与性能'}));
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'查询本次结果'})));
  expect(queries.at(-1)).toEqual({start_request_id:original});
  expect(starts).toBe(1);
});
