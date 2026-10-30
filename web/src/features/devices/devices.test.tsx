import { act, cleanup, fireEvent, render, screen, waitFor, within } from "@testing-library/react";
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
  SNAPSHOT_FIELDS,
  type LinkStatus,
  type ResponsePayload,
  type RobotSnapshot,
  type RobotSummary,
} from "../../api/protocol";

afterEach(cleanup);

// ---------------------------------------------------------------- fakes

const ALPHA: RobotSummary = { id: "robot-alpha", name: "Alpha 仓库车", online: true };
const BETA: RobotSummary = { id: "robot-beta", name: "Beta 装卸车", online: false };

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

class FakeLink implements RobotLink {
  connects: string[] = [];
  disconnects = 0;
  requests: Array<{ type: string; payload: unknown }> = [];
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

// ---------------------------------------------------------------- switching

describe("robot switching", () => {
  it("test_switch_robot_drops_previous_state", async () => {
    const api = new FakeApi();
    const link = new FakeLink();
    mount("/robots/robot-alpha", api, link);
    await waitFor(() => expect(link.connects).toEqual(["robot-alpha"]));
    link.emit(snapshot("robot-alpha", { execution_status: "patrolling-alpha" }));
    expect(screen.getByText("patrolling-alpha")).toBeTruthy();

    fireEvent.click(screen.getByRole("link", { name: /Beta/ }));

    expect(screen.queryByText("patrolling-alpha")).toBeNull();
    await waitFor(() => expect(link.connects).toEqual(["robot-alpha", "robot-beta"]));
    expect(link.disconnects).toBeGreaterThanOrEqual(1);
    await screen.findByText("等待车端快照");

    // A late frame of the old robot must not repaint the new robot's panel.
    link.emit(snapshot("robot-alpha", { execution_status: "patrolling-alpha" }));
    expect(screen.queryByText("patrolling-alpha")).toBeNull();

    link.emit(snapshot("robot-beta", { execution_status: "charging-beta" }));
    expect(screen.getByText("charging-beta")).toBeTruthy();
  });
});

// ---------------------------------------------------------------- cache label

describe("snapshot freshness", () => {
  it("test_stale_cache_label", async () => {
    const link = new FakeLink();
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(link.connects).toHaveLength(1));

    link.emit(
      snapshot("robot-alpha", { stale: true, connected: false, last_synced_at: 1_700_000_000 }),
    );
    expect(screen.getByText("缓存快照 · 非实时")).toBeTruthy();
    expect(screen.getByText(/最后同步/)).toBeTruthy();
    expect(screen.queryByText("实时快照")).toBeNull();

    link.emit(snapshot("robot-alpha", { stale: false, last_synced_at: Date.now() / 1000 }));
    expect(screen.getByText("实时快照")).toBeTruthy();
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
    link.emit(snapshot("robot-alpha", { execution_status: "patrolling-alpha" }));

    api.failure = new UnauthorizedError("login required");
    fireEvent.click(screen.getByRole("button", { name: "刷新设备" }));

    await screen.findByRole("heading", { name: "操作员登录" });
    expect(screen.queryByRole("navigation", { name: "设备列表" })).toBeNull();
    expect(screen.queryByText("patrolling-alpha")).toBeNull();
    expect(link.disconnects).toBeGreaterThanOrEqual(1);
  });

  it("returns to login when the first request is unauthorized, and logs in", async () => {
    const api = new FakeApi();
    api.failure = new UnauthorizedError("login required");
    mount("/robots/robot-alpha", api, new FakeLink());
    await screen.findByRole("heading", { name: "操作员登录" });

    api.failure = null;
    fireEvent.change(screen.getByLabelText("管理员密码"), { target: { value: "s3cret" } });
    fireEvent.click(screen.getByRole("button", { name: "进入控制台" }));

    await screen.findByRole("navigation", { name: "设备列表" });
    expect(api.logins).toEqual(["s3cret"]);
  });

  it("shows a wrong password and stays on login", async () => {
    const api = new FakeApi();
    api.failure = new UnauthorizedError("login required");
    api.loginFailure = new InvalidCredentialsError("invalid credentials");
    mount("/robots", api, new FakeLink());
    await screen.findByRole("heading", { name: "操作员登录" });

    fireEvent.change(screen.getByLabelText("管理员密码"), { target: { value: "wrong" } });
    fireEvent.click(screen.getByRole("button", { name: "进入控制台" }));

    const alert = await screen.findByRole("alert");
    expect(alert.textContent).toContain("密码错误");
    expect(screen.getByRole("heading", { name: "操作员登录" })).toBeTruthy();
  });

  it("logs out and returns to login", async () => {
    const api = new FakeApi();
    mount("/robots", api, new FakeLink());
    await screen.findByRole("navigation", { name: "设备列表" });
    fireEvent.click(screen.getByRole("button", { name: "退出登录" }));
    await screen.findByRole("heading", { name: "操作员登录" });
    expect(api.loggedOut).toBe(1);
  });
});

// ---------------------------------------------------------------- device list

describe("device list", () => {
  it("shows the real online state of each robot", async () => {
    mount("/robots", new FakeApi(), new FakeLink());
    const nav = await screen.findByRole("navigation", { name: "设备列表" });
    const alpha = within(nav).getByRole("link", { name: /Alpha/ });
    const beta = within(nav).getByRole("link", { name: /Beta/ });
    expect(within(alpha).getByText("在线")).toBeTruthy();
    expect(within(beta).getByText("离线")).toBeTruthy();
  });

  it("shows an empty state and a retryable error state", async () => {
    const api = new FakeApi();
    api.robots = [];
    mount("/robots", api, new FakeLink());
    await screen.findByText("尚未配对任何设备");

    api.failure = new Error("boom");
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
    link.requestImpl = () => Promise.reject(new CommandError("offline", "robot offline"));
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

    await screen.findByText(/急停已确认/);
    expect(screen.queryByRole("alert")).toBeNull();
  });
});

// ---------------------------------------------------------------- demo

describe("demo data", () => {
  it("test_demo_label_always_visible", async () => {
    const api = new FakeApi();
    api.failure = new UnauthorizedError("login required");
    mount("/robots/robot-alpha", api, new FakeLink(), true);

    await screen.findByRole("heading", { name: "操作员登录" });
    expect(screen.getByLabelText("演示数据标记").textContent).toContain("演示数据");

    api.failure = null;
    fireEvent.change(screen.getByLabelText("管理员密码"), { target: { value: "x" } });
    fireEvent.click(screen.getByRole("button", { name: "进入控制台" }));
    await screen.findByRole("navigation", { name: "设备列表" });
    expect(screen.getByLabelText("演示数据标记").textContent).toContain("演示数据");

    fireEvent.click(screen.getByRole("link", { name: /Beta/ }));
    expect(screen.getByLabelText("演示数据标记").textContent).toContain("演示数据");
  });

  it("shows no demo label for real data", async () => {
    mount("/robots/robot-alpha", new FakeApi(), new FakeLink(), false);
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

function reply(status: number, body?: unknown, headers: Record<string, string> = {}): Response {
  return new Response(body === undefined ? null : JSON.stringify(body), { status, headers });
}

describe("HttpControlApi", () => {
  it("logs in with the password, keeps the csrf token and sends it on logout", async () => {
    const calls: Array<{ url: string; init: RequestInit }> = [];
    const answers = [reply(200, { csrf_token: "csrf-1" }), reply(204)];
    const fetchImpl = vi.fn(async (url: RequestInfo | URL, init?: RequestInit) => {
      calls.push({ url: String(url), init: init ?? {} });
      return answers.shift()!;
    });
    const api = new HttpControlApi(fetchImpl as typeof fetch, memoryStorage());

    await api.login("pw");
    await api.logout();

    expect(calls[0]).toMatchObject({ url: "/api/session" });
    expect(calls[0]!.init).toMatchObject({ method: "POST", body: JSON.stringify({ password: "pw" }) });
    expect(calls[1]!.url).toBe("/api/session");
    expect(calls[1]!.init).toMatchObject({ method: "DELETE", headers: { "X-CSRF-Token": "csrf-1" } });
  });

  it("maps 401, 429 and other login failures to distinct errors", async () => {
    const answers = [
      reply(401, { detail: "invalid credentials" }),
      reply(429, { detail: "too many attempts" }, { "Retry-After": "42" }),
      reply(403, { detail: "origin not allowed" }),
    ];
    const api = new HttpControlApi((async () => answers.shift()!) as typeof fetch, memoryStorage());
    await expect(api.login("x")).rejects.toBeInstanceOf(InvalidCredentialsError);
    await expect(api.login("x")).rejects.toMatchObject({ retryAfterSeconds: 42 });
    await expect(api.login("x")).rejects.toBeInstanceOf(ApiError);
  });

  it("lists robots, and reports a 401 as an unauthorized session", async () => {
    const answers = [
      reply(200, { robots: [{ id: "r1", name: "A", online: true }] }),
      reply(401, { detail: "login required" }),
      reply(200, { robots: [{ id: "r1", name: "A" }] }),
    ];
    const api = new HttpControlApi((async () => answers.shift()!) as typeof fetch, memoryStorage());
    await expect(api.listRobots()).resolves.toEqual([{ id: "r1", name: "A", online: true }]);
    await expect(api.listRobots()).rejects.toBeInstanceOf(UnauthorizedError);
    await expect(api.listRobots()).rejects.toBeInstanceOf(ApiError);
  });
});

// ---------------------------------------------------------------- RobotClient

class FakeSocket implements SocketLike {
  static instances: FakeSocket[] = [];
  readyState = 0;
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
  }
  receive(frame: unknown): void {
    this.onmessage?.({ data: typeof frame === "string" ? frame : JSON.stringify(frame) });
  }
  serverClose(code: number): void {
    this.readyState = 3;
    this.onclose?.({ code });
  }
  lastSent(): Record<string, unknown> {
    return JSON.parse(this.sent[this.sent.length - 1] ?? "null") as Record<string, unknown>;
  }
}

function frame(type: string, robotId: string, payload: unknown, requestId = "r1") {
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
  options: { staleAfterMs?: number; requestTimeoutMs?: number; snapshot?: boolean } = {},
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
    expect(Object.keys(sent).sort()).toEqual([...ENVELOPE_FIELDS, "payload", "type"].sort());
    expect(sent).toMatchObject({ v: 1, robot_id: "robot-alpha", type: "safety.estop", payload: {} });
    expect(Number(sent["expires_at"])).toBeGreaterThan(Date.now() / 1000);
    socket.receive(
      frame("response", "robot-alpha", { request_type: "safety.estop", ok: true }, String(sent["request_id"])),
    );
    await expect(result).resolves.toMatchObject({ ok: true });
  });

  it("rejects an ok=false response with its error code", async () => {
    const { client, socket } = await connected();
    const result = client.request("navigation.start", {});
    const id = String(socket.lastSent()["request_id"]);
    socket.receive(
      frame("response", "robot-alpha", { request_type: "navigation.start", ok: false, error: "conflict" }, id),
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
    const idle = new RobotClient({ socketFactory: (url) => new FakeSocket(url) });
    await expect(idle.request("safety.estop", {})).rejects.toMatchObject({ code: "offline" });
    const { client } = await connected();
    await expect(client.request("not.a.type", {})).rejects.toMatchObject({ code: "validation" });
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

    socket.receive(frame("snapshot", "robot-alpha", { ...wirePayload, last_synced_at: 4_321 }));
    expect(seen).toHaveLength(1);
    expect(seen[0]).toMatchObject({ robot_id: "robot-alpha", stale: false, connected: true });
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
    expect(seen[0]).toMatchObject({ stale: true, connected: false, last_synced_at: 1_700_000_000 });
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
    await expect(client.request("drive.stop", {})).rejects.toMatchObject({ code: "offline" });
  });

  it("sends a command with the epoch the snapshot frame delivered", async () => {
    const { client, socket } = await connected({ snapshot: false });
    socket.receive(frame("snapshot", "robot-alpha", wirePayload));
    void client.request("drive.stop", {}).catch(() => undefined);
    expect(socket.lastSent()["connection_epoch"]).toBe(AGENT_EPOCH);
    socket.receive({ ...frame("snapshot", "robot-alpha", wirePayload), connection_epoch: 4 });
    void client.request("drive.stop", {}).catch(() => undefined);
    expect(socket.lastSent()["connection_epoch"]).toBe(4);
    client.disconnect();
  });

  it("sends no command before any snapshot has delivered an epoch", async () => {
    const { client, socket } = await connected({ snapshot: false });
    await expect(client.request("safety.estop", {})).rejects.toMatchObject({ code: "offline" });
    expect(socket.sent).toEqual([]);
  });

  it("sends no command on a stale cache or when the agent is offline", async () => {
    const { client, socket } = await connected({ snapshot: false });
    socket.receive(
      frame("snapshot", "robot-alpha", { ...wirePayload, connected: false, stale: true }),
    );
    await expect(client.request("drive.stop", {})).rejects.toMatchObject({ code: "offline" });

    socket.receive({
      ...frame("snapshot", "robot-alpha", { ...wirePayload, connected: false, stale: true }),
      connection_epoch: 0,
    });
    await expect(client.request("drive.stop", {})).rejects.toMatchObject({ code: "offline" });
    expect(socket.sent).toEqual([]);
  });

  it("stops sending commands once the cached snapshot times out", async () => {
    const { client, socket } = await connected({ staleAfterMs: 5_000 });
    await vi.advanceTimersByTimeAsync(5_001);
    await expect(client.request("drive.stop", {})).rejects.toMatchObject({ code: "offline" });
    expect(socket.sent).toEqual([]);
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
    expect(statuses.at(-1)).toEqual({ state: "closed", robotId: "robot-alpha", closeCode: 1008 });
  });

  it("drops frames for another robot and malformed frames", async () => {
    const { client, socket } = await connected({ snapshot: false });
    const seen: RobotSnapshot[] = [];
    client.subscribe((value) => seen.push(value));
    socket.receive(frame("snapshot", "robot-beta", wirePayload));
    socket.receive("{not json");
    socket.receive(frame("snapshot", "robot-alpha", { ...wirePayload, task_revision: "7" }));
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
    const client = new RobotClient({ socketFactory: (url) => new FakeSocket(url) });
    const pending = client.connect("robot-alpha");
    const assertion = expect(pending).rejects.toMatchObject({ code: "offline" });
    FakeSocket.instances[0]!.serverClose(1008);
    await assertion;
  });
});

// ---------------------------------------------------------------- cached snapshot in the page

describe("cached snapshot delivered by the cloud", () => {
  it("test_stale_snapshot_from_the_cloud_is_labeled_cache_in_the_page", async () => {
    FakeSocket.instances = [];
    const link = new RobotClient({ socketFactory: (url) => new FakeSocket(url) });
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
    expect(screen.queryByText("实时快照")).toBeNull();
    expect(screen.getByText("cached-idle")).toBeTruthy();
    expect(screen.getByText(/最后同步/).textContent).toContain(
      new Date(1_700_000_000 * 1000).toLocaleTimeString("zh-CN", { hour12: false }),
    );
    link.disconnect();
  });

  it("test_command_uses_the_epoch_the_cloud_delivered", async () => {
    FakeSocket.instances = [];
    const link = new RobotClient({ socketFactory: (url) => new FakeSocket(url) });
    mount("/robots/robot-alpha", new FakeApi(), link);
    await waitFor(() => expect(FakeSocket.instances).toHaveLength(1));
    const socket = FakeSocket.instances[0]!;
    act(() => socket.open());
    act(() =>
      socket.receive({
        ...frame("snapshot", "robot-alpha", { ...wirePayload, last_synced_at: Date.now() / 1000 }),
        connection_epoch: 9,
      }),
    );

    fireEvent.click(screen.getByRole("button", { name: "急停" }));

    await waitFor(() => expect(socket.sent).toHaveLength(1));
    expect(socket.lastSent()).toMatchObject({ type: "safety.estop", connection_epoch: 9 });
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
  return [...body.matchAll(/^ {4}(\w+): /gm)].map((match) => match[1]!);
}

describe("protocol parity with src/senior_control/protocol.py", () => {
  const root = resolve(import.meta.dirname, "../../../..");
  const protocol = readFileSync(resolve(root, "src/senior_control/protocol.py"), "utf8");
  const errors = readFileSync(resolve(root, "src/senior_control/errors.py"), "utf8");

  it("mirrors the request types", () => {
    const simple = literalOf(pythonClass(protocol, "SimpleEnvelope"));
    const operations = quoted(/OPERATION_TYPES = \(([^)]*)\)/.exec(protocol)![1]!);
    expect([...SIMPLE_REQUEST_TYPES]).toEqual(simple);
    expect([...OPERATION_REQUEST_TYPES]).toEqual(operations);
    expect([...REQUEST_TYPES]).toEqual(["drive.velocity", ...simple, ...operations]);
  });

  it("mirrors the response request types and error codes", () => {
    const responseBody = pythonClass(protocol, "ResponsePayload");
    expect([...RESPONSE_REQUEST_TYPES]).toEqual(literalOf(responseBody));
    const codes = [...pythonClass(errors, "ErrorCode").matchAll(/= "(\w+)"/g)].map((m) => m[1]!);
    expect([...ERROR_CODES]).toEqual(codes);
  });

  it("mirrors the envelope and snapshot fields and the size bound", () => {
    expect([...ENVELOPE_FIELDS]).toEqual(fieldsOf(pythonClass(protocol, "EnvelopeBase")));
    expect([...SNAPSHOT_FIELDS]).toEqual(fieldsOf(pythonClass(protocol, "SnapshotPayload")));
    expect(MAX_MESSAGE_BYTES).toBe(Number(/MAX_MESSAGE_BYTES = (\d+)/.exec(protocol)![1]));
  });
});
