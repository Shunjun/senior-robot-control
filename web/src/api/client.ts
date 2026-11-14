import { BrowserVehicleLogs } from "./logs";
import { IntentBackpressureError, VehicleTransport, VEHICLE_REQUEST_TYPES, type VehiclePort, type VehicleLogPort } from "./vehicleState";
import {
  MAX_MESSAGE_BYTES,
  PROTOCOL_VERSION,
  isRequestType,
  isWorkspaceRequestType,
  parseInbound,
  parseVideoInbound,
  type EnvelopeBase,
  type ErrorCode,
  type LinkStatus,
  type ResponsePayload,
  type RobotSnapshot,
  type RobotSummary,
  type SnapshotEnvelope,
  type VideoSignalType,
  type VideoSignalEnvelope,
} from "./protocol";

// ---------------------------------------------------------------- errors

/** HTTP 401: the session is missing or expired. The console returns to login. */
export class UnauthorizedError extends Error {
  override name = "UnauthorizedError";
}

/** `POST /api/session` answered 401: the password is wrong. */
export class InvalidCredentialsError extends Error {
  override name = "InvalidCredentialsError";
}

/** HTTP 429 with `Retry-After`. */
export class RateLimitedError extends Error {
  override name = "RateLimitedError";
  constructor(
    message: string,
    readonly retryAfterSeconds: number | null,
  ) {
    super(message);
  }
}

export class ApiError extends Error {
  override name = "ApiError";
  constructor(
    message: string,
    readonly status: number,
  ) {
    super(message);
  }
}

/** A command that did not succeed. A resolved `request` always means the robot said ok. */
export class CommandError extends Error {
  override name = "CommandError";
  constructor(
    readonly code: ErrorCode,
    message: string,
  ) {
    super(message);
  }
}

const COMMAND_ERROR_TEXT: Record<ErrorCode, string> = {
  validation: "指令不合法或不属于这台设备",
  conflict: "指令与当前状态冲突，或设备通道已满",
  offline: "车辆离线或控制通道未连接",
  unavailable: "车端能力暂不可用",
  localization_unavailable: "定位不可用",
  safety_rejected: "安全保护拒绝了该指令",
  timeout: "指令已过期或车端未在时限内应答",
  operation_unknown: "操作结果未知，需人工核查",
};

export function describeCommandError(code: ErrorCode): string {
  return COMMAND_ERROR_TEXT[code];
}

// ---------------------------------------------------------------- HTTP

export interface PairingCode {
  code: string;
  expires_in: number;
}

export interface ControlApi {
  login(password: string): Promise<void>;
  logout(): Promise<void>;
  listRobots(): Promise<RobotSummary[]>;
  revokeRobot(robotId: string): Promise<void>;
  createPairingCode(signal?: AbortSignal): Promise<PairingCode>;
}

const CSRF_HEADER = "X-CSRF-Token";
const CSRF_STORAGE_KEY = "senior-control.csrf";

async function detail(response: Response): Promise<string> {
  try {
    const body: unknown = await response.json();
    if (typeof body === "object" && body !== null && "detail" in body) {
      const value = (body as {
        detail: unknown;
      }).detail;
      if (typeof value === "string") return value;
    }
  } catch {
    // The body is not JSON; the status line is all there is.
  }
  return response.statusText || `HTTP ${response.status}`;
}

function parseRobots(body: unknown): RobotSummary[] {
  const list = typeof body === "object" && body !== null ? (body as {
    robots?: unknown;
  }).robots : null;
  if (!Array.isArray(list)) throw new ApiError("unexpected robot list", 200);
  return list.map((row: unknown) => {
    const item = row as Partial<RobotSummary> | null;
    if (
      item === null ||
      typeof item !== "object" ||
      typeof item.id !== "string" ||
      typeof item.name !== "string" ||
      typeof item.online !== "boolean"
    ) {
      throw new ApiError("unexpected robot list", 200);
    }
    const robot = { id: item.id, name: item.name, online: item.online };
    //Compatible: Older relays omit reconnecting.
    return typeof item.reconnecting === "boolean" ? { ...robot, reconnecting: item.reconnecting } : robot;
  });
}

type SessionStorageLike = Pick<Storage, "getItem" | "setItem" | "removeItem">;

function defaultStorage(): SessionStorageLike | null {
  return typeof sessionStorage === "undefined" ? null : sessionStorage;
}

/**
 * The cloud's single-admin session API. The session cookie is HttpOnly and never visible here;
 * the CSRF token comes back in the login body and rides on every write.
 */
export class HttpControlApi implements ControlApi {
  private csrf: string | null;

  constructor(
    private readonly fetchImpl: typeof fetch = (input, init) => fetch(input, init),
    private readonly storage: SessionStorageLike | null = defaultStorage(),
  ) {
    this.csrf = storage?.getItem(CSRF_STORAGE_KEY) ?? null;
  }

  async login(password: string): Promise<void> {
    const response = await this.fetchImpl("/api/session", {
      method: "POST",
      credentials: "same-origin",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ password }),
    });
    if (response.status === 401) throw new InvalidCredentialsError("invalid credentials");
    if (response.status === 429) {
      const wait = Number(response.headers.get("Retry-After"));
      throw new RateLimitedError("too many attempts", Number.isFinite(wait) && wait > 0 ? wait : null);
    }
    if (!response.ok) throw new ApiError(await detail(response), response.status);
    const body = (await response.json()) as {
      csrf_token?: unknown;
    };
    if (typeof body.csrf_token !== "string" || body.csrf_token === "") {
      throw new ApiError("login response has no csrf token", response.status);
    }
    this.setCsrf(body.csrf_token);
  }

  async logout(): Promise<void> {
    const headers: Record<string, string> = {};
    if (this.csrf !== null) headers[CSRF_HEADER] = this.csrf;
    const response = await this.fetchImpl("/api/session", {
      method: "DELETE",
      credentials: "same-origin",
      headers,
    });
    if (response.status === 401) {
      this.setCsrf(null);
      throw new UnauthorizedError("login required");
    }
    if (!response.ok) throw new ApiError(await detail(response), response.status);
    this.setCsrf(null);
  }

  async listRobots(): Promise<RobotSummary[]> {
    const csrf = this.csrf;
    const response = await this.fetchImpl("/api/robots", { credentials: "same-origin" });
    if (response.status === 401) {
      if (this.csrf === csrf) this.setCsrf(null);
      throw new UnauthorizedError("login required");
    }
    if (!response.ok) throw new ApiError(await detail(response), response.status);
    return parseRobots(await response.json());
  }

  async revokeRobot(robotId: string): Promise<void> {
    const csrf = this.csrf;
    const response = await this.fetchImpl(`/api/robots/${encodeURIComponent(robotId)}`, {
      method: "DELETE",
      credentials: "same-origin",
      headers: csrf === null ? {} : { [CSRF_HEADER]: csrf },
    });
    if (response.status === 401) {
      if (this.csrf === csrf) this.setCsrf(null);
      throw new UnauthorizedError("login required");
    }
    if (!response.ok) throw new ApiError(await detail(response), response.status);
  }

  async createPairingCode(signal?: AbortSignal): Promise<PairingCode> {
    const csrf = this.csrf;
    const response = await this.fetchImpl("/api/pairing-codes", {
      method: "POST",
      credentials: "same-origin",
      headers: csrf === null ? {} : { [CSRF_HEADER]: csrf },
      signal: signal ?? null,
    });
    if (response.status === 401) {
      if (!signal?.aborted && this.csrf === csrf) this.setCsrf(null);
      throw new UnauthorizedError("login required");
    }
    if (response.status === 429) {
      const wait = Number(response.headers.get("Retry-After"));
      throw new RateLimitedError("too many pairing codes", Number.isFinite(wait) && wait > 0 ? wait : null);
    }
    if (!response.ok) throw new ApiError("pairing request failed", response.status);

    let body: unknown;
    try {
      body = await response.json();
    } catch {
      throw new ApiError("unexpected pairing response", response.status);
    }
    const item = body as Partial<PairingCode> | null;
    if (
      item === null || typeof item !== "object" ||
      typeof item.code !== "string" || item.code.trim() === "" ||
      typeof item.expires_in !== "number" || !Number.isSafeInteger(item.expires_in) ||
      item.expires_in <= 0
    ) {
      throw new ApiError("unexpected pairing response", response.status);
    }
    return { code: item.code, expires_in: item.expires_in };
  }

  private setCsrf(value: string | null): void {
    this.csrf = value;
    if (value === null) this.storage?.removeItem(CSRF_STORAGE_KEY);
    else this.storage?.setItem(CSRF_STORAGE_KEY, value);
  }
}

// ---------------------------------------------------------------- control socket

export interface RobotLink {
  readonly vehicle?: VehiclePort;
  connect(robotId: string): Promise<void>;
  disconnect(): void;
  request<T = ResponsePayload>(type: string, payload: unknown): Promise<T>;
  sendVideoSignal?(type: VideoSignalType, payload: {
    sdp: string;
  } | {
    candidate: string;
  } | Record<string, never>): void;
  subscribeVideo?(listener: (frame: VideoSignalEnvelope) => void): () => void;
  subscribe(listener: (snapshot: RobotSnapshot) => void): () => void;
  subscribeStatus(listener: (status: LinkStatus) => void): () => void;
}
export interface SocketLike {
  readyState: number;
  bufferedAmount: number;
  send(data: string): void;
  close(code?: number): void;
  onopen: (() => void) | null;
  onmessage: ((event: {
    data: unknown;
  }) => void) | null;
  onclose: ((event: {
    code: number;
  }) => void) | null;
  onerror: (() => void) | null;
}
export interface RobotClientOptions {
  socketFactory?: (url: string) => SocketLike;
  staleAfterMs?: number;
  requestTimeoutMs?: number;
  /** Inject only the separately authenticated log connection once its route exists. */
  logs?: VehicleLogPort;
}
interface Pending {
  type: string;
  sessionId: string;
  epoch: number;
  seq: number;
  sentAt: number;
  resolve: (value: ResponsePayload) => void;
  reject: (error: CommandError) => void;
  timer: ReturnType<typeof setTimeout>;
}
const SOCKET_OPEN = 1;
const CLOSE_NORMAL = 1000;
const COMMAND_TTL_SECONDS = 5;
function controlUrl(robotId: string): string {
  return `${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws/control?robot_id=${encodeURIComponent(robotId)}`;
}
function openBrowserSocket(url: string): SocketLike {
  return new WebSocket(url) as unknown as SocketLike;
}
export class RobotClient implements RobotLink {
  readonly vehicle: VehicleTransport;
  private readonly browserLogs: BrowserVehicleLogs;
  private readonly socketFactory: (url: string) => SocketLike;
  private readonly staleAfterMs: number;
  private readonly requestTimeoutMs: number;
  private readonly snapshotListeners = new Set<(snapshot: RobotSnapshot) => void>();
  private readonly statusListeners = new Set<(status: LinkStatus) => void>();
  private readonly videoListeners = new Set<(frame: VideoSignalEnvelope) => void>();
  private readonly pending = new Map<string, Pending>();
  private generation = 0;
  private socket: SocketLike | null = null;
  private robotId: string | null = null;
  private sessionId = "";
  private epoch = 0;
  private epochFrontier = 0;
  private retiredEpoch = 0;
  private seq = 0;
  private latest: RobotSnapshot | null = null;
  private staleTimer: ReturnType<typeof setTimeout> | null = null;
  private abortConnect: (() => void) | null = null;
  constructor(options: RobotClientOptions = {}) {
    this.socketFactory = options.socketFactory ?? openBrowserSocket;
    this.staleAfterMs = options.staleAfterMs ?? 5000;
    this.requestTimeoutMs = options.requestTimeoutMs ?? 5000;
    this.browserLogs = new BrowserVehicleLogs(() => this.robotId, this.socketFactory);
    this.vehicle = new VehicleTransport(async (type, payload) => {
      const response = await this.request(type, payload);
      return response.data ?? null;
    }, options.logs ?? this.browserLogs);
  }
  connect(robotId: string): Promise<void> {
    this.teardown();
    if (this.robotId !== robotId) this.vehicle.reset();
    const generation = ++this.generation;
    this.robotId = robotId;
    this.sessionId = `web-${crypto.randomUUID()}`;
    this.epoch = 0;
    this.epochFrontier = 0;
    this.retiredEpoch = 0;
    this.seq = 0;
    this.setStatus({ state: "connecting", robotId, closeCode: null });

    return new Promise<void>((resolve, reject) => {
      let settled = false;
      const fail = (message: string) => {
        if (!settled) {
          settled = true;
          reject(new CommandError("offline", message));
        }
      };
      this.abortConnect = () => fail("connection replaced");
      let socket: SocketLike;
      try {
        socket = this.socketFactory(controlUrl(robotId));
      } catch {
        this.setStatus({ state: "closed", robotId, closeCode: null });
        fail("control socket could not be opened");
        return;
      }
      this.socket = socket;
      socket.onopen = () => {
        if (generation !== this.generation) return;
        settled = true;
        this.abortConnect = null;
        this.setStatus({ state: "open", robotId, closeCode: null });
        resolve();
      };
      socket.onmessage = (event) => {
        if (generation === this.generation) this.receive(event.data);
      };
      socket.onerror = () => {
        // A close always follows; it carries the code.
      };
      socket.onclose = (event) => {
        if (generation !== this.generation) return;
        this.socket = null;
        this.onClosed(event.code);
        fail("control socket closed before opening");
      };
    });
  }

  disconnect(): void {
    this.teardown();
    this.robotId = null;
    this.latest = null;
    this.setStatus({ state: "idle", robotId: null, closeCode: null });
  }


  request<T = ResponsePayload>(type: string, payload: unknown): Promise<T> {
    if (type === "drive.velocity" || type === "drive.challenge" || (!isRequestType(type) && !VEHICLE_REQUEST_TYPES.includes(type))) return Promise.reject(new CommandError("validation", "unsupported request type"));
    const urgent = type === "safety.estop" || type === "drive.stop";
    const safety = urgent || type === "control.release";
    const socket = this.socket, robotId = this.robotId;
    if (socket === null || robotId === null || socket.readyState !== SOCKET_OPEN) return Promise.reject(new CommandError("offline", "control socket is not open"));
    if (!urgent && (this.epoch < 1 || this.latest === null || (!safety && (this.latest.stale || !this.latest.connected)))) return Promise.reject(new CommandError("offline", "no live vehicle snapshot"));
    const required = type.startsWith("settings.") ? "settings.v1" : type === "task.execution.command" ? "task.execution.v1" : type === "drive.intent" || type === "control.acquire" ? "motion.intent.v1" : type === "safety.estop.reset" || type === "safety.fault.recover" ? "runtime.v1" : null;
    const vehicle = this.vehicle.getSnapshot();
    if (required && (!vehicle.connected || !vehicle.available || !vehicle.frame?.capabilities.includes(required))) return Promise.reject(new CommandError("unavailable", "vehicle capability is not connected"));
    if (type === "drive.intent" && socket.bufferedAmount > 0) return Promise.reject(new IntentBackpressureError());
    const requestId = crypto.randomUUID();
    const envelope = {
      v: PROTOCOL_VERSION,
      robot_id: robotId,
      session_id: this.sessionId,
      connection_epoch: this.epoch,
      request_id: requestId,
      seq: ++this.seq,
      expires_at: Date.now() / 1000 + COMMAND_TTL_SECONDS,
      type,
      payload
    };
    const text = JSON.stringify(envelope);
    if (new TextEncoder().encode(text).length > MAX_MESSAGE_BYTES) return Promise.reject(new CommandError("validation", "request is too large"));
    return new Promise<T>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(requestId);
        reject(new CommandError("timeout", "request result unconfirmed"));
      }, this.requestTimeoutMs);
      this.pending.set(requestId, {
        type,
        sessionId: this.sessionId,
        epoch: this.epoch,
        seq: envelope.seq,
        sentAt: performance.now(),
        resolve: value => resolve(value as T),
        reject,
        timer
      });
      try {
        socket.send(text);
      } catch {
        clearTimeout(timer);
        this.pending.delete(requestId);
        reject(new CommandError("offline", "socket rejected frame"));
      } finally {
        if (type === "safety.estop") this.vehicle.fenceManual();
      }
    });
  }
  sendVideoSignal(
    type: VideoSignalType,
    payload: {
      sdp: string;
    } | {
      candidate: string;
    } | Record<string, never>,
  ): void {
    const socket = this.socket;
    const robotId = this.robotId;
    if (socket === null || robotId === null || socket.readyState !== SOCKET_OPEN || this.epoch < 1) {
      throw new CommandError("offline", "control socket is not open for video signaling");
    }
    const envelope: EnvelopeBase & {
      type: VideoSignalType;
      payload: typeof payload;
    } = {
      v: PROTOCOL_VERSION,
      robot_id: robotId,
      session_id: this.sessionId,
      connection_epoch: this.epoch,
      request_id: crypto.randomUUID(),
      seq: ++this.seq,
      expires_at: Date.now() / 1000 + COMMAND_TTL_SECONDS,
      type,
      payload,
    };
    const text = JSON.stringify(envelope);
    if (text.length > MAX_MESSAGE_BYTES) {
      throw new CommandError("validation", "video signaling frame is too large");
    }
    try {
      socket.send(text);
    } catch {
      // A media signaling failure must not tear down the control socket or its drive slot.
      throw new CommandError("offline", "control socket rejected the video signaling frame");
    }
  }


  subscribe(listener: (snapshot: RobotSnapshot) => void): () => void {
    this.snapshotListeners.add(listener);
    return () => this.snapshotListeners.delete(listener);
  }
  subscribeVideo(listener: (frame: VideoSignalEnvelope) => void): () => void {
    this.videoListeners.add(listener);
    return () => this.videoListeners.delete(listener);
  }
  subscribeStatus(listener: (status: LinkStatus) => void): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }
  private teardown(): void {
    this.browserLogs.close();
    this.generation++;
    this.vehicle.setConnection(false);
    const socket = this.socket;
    this.socket = null;
    if (socket) {
      socket.onopen = socket.onmessage = socket.onclose = socket.onerror = null;
      socket.close(CLOSE_NORMAL);
    }
    this.clearStaleTimer();
    this.latest = null;
    this.abortConnect?.();
    this.abortConnect = null;
    this.failPending();
  }
  private failPending(): void {
    for (const entry of this.pending.values()) {
      clearTimeout(entry.timer);
      entry.reject(new CommandError("offline", "control socket closed"));
    }
    this.pending.clear();
  }
  private onClosed(code: number): void {
    this.vehicle.setConnection(false);
    this.failPending();
    this.markStale();
    this.setStatus({ state: "closed", robotId: this.robotId, closeCode: code });
  }
  private receive(raw: unknown): void {
    const envelope = parseInbound(raw);
    if (envelope === null) {
      const video = parseVideoInbound(raw);
      if (video && video.robot_id === this.robotId) this.videoListeners.forEach(l => l(video));
      return;
    }
    if (envelope.robot_id !== this.robotId) return;
    if (envelope.type === "snapshot") {
      this.onSnapshot(envelope);
      return;
    }
    const entry = this.pending.get(envelope.request_id);
    if (!entry || entry.type !== envelope.payload.request_type) return;
    const strict = isWorkspaceRequestType(entry.type) || VEHICLE_REQUEST_TYPES.includes(entry.type) || entry.type === "control.acquire" || entry.type === "control.release";
    if (strict && (entry.sessionId !== envelope.session_id || entry.epoch !== envelope.connection_epoch || entry.seq !== envelope.seq)) return;
    this.pending.delete(envelope.request_id);
    clearTimeout(entry.timer);
    this.vehicle.setConnection(this.vehicle.getSnapshot().connected, Math.max(0, performance.now() - entry.sentAt));
    if (strict && entry.epoch !== this.epoch) {
      entry.reject(new CommandError("offline", "connection replaced"));
      return;
    }
    if (envelope.payload.ok) entry.resolve(envelope.payload);
    else entry.reject(new CommandError(envelope.payload.error ?? "validation", describeCommandError(envelope.payload.error ?? "validation")));
  }
  private onSnapshot(envelope: SnapshotEnvelope): void {
    // The cloud sends its cache with the epoch of the connected agent (0 when none is), and the
    // cache's own `stale` and `last_synced_at`. Nothing here is the browser's receipt time.
    const incomingEpoch = envelope.connection_epoch;
    if (incomingEpoch > 0 && (incomingEpoch < this.epochFrontier || incomingEpoch <= this.retiredEpoch)) return;
    if (incomingEpoch === 0) this.retiredEpoch = Math.max(this.retiredEpoch, this.epochFrontier);
    else this.epochFrontier = Math.max(this.epochFrontier, incomingEpoch);
    const payload = incomingEpoch === 0
      ? { ...envelope.payload, stale: true, connected: false }
      : envelope.payload;
    // Retire only browser manual ownership before switching the request epoch.
    // Tasks belong to the car and continue independently of this input lifetime.
    if (incomingEpoch !== this.epoch) this.vehicle.fenceManual();
    this.epoch = envelope.connection_epoch;
    this.latest = { robot_id: envelope.robot_id, ...payload };
    this.clearStaleTimer();
    if (!this.latest.stale) {
      this.staleTimer = setTimeout(() => this.markStale(), this.staleAfterMs);
    }
    if (this.latest.vehicle_ui !== undefined) this.vehicle.update(this.latest.vehicle_ui, !this.latest.stale && this.latest.connected);
    else this.vehicle.setUnsupportedConnection(!this.latest.stale && this.latest.connected);
    this.snapshotListeners.forEach((listener) => listener(this.latest as RobotSnapshot));
  }

  /** A cached copy never claims the robot is connected (same rule as the cloud SnapshotCache). */
  private markStale(): void {
    this.clearStaleTimer();
    const current = this.latest;
    if (current === null || current.stale) return;
    this.vehicle.setConnection(false);
    this.latest = { ...current, connected: false, stale: true };
    if (this.latest.vehicle_ui !== undefined) this.vehicle.update(this.latest.vehicle_ui, !this.latest.stale && this.latest.connected);
    else this.vehicle.setUnsupportedConnection(!this.latest.stale && this.latest.connected);
    this.snapshotListeners.forEach((listener) => listener(this.latest as RobotSnapshot));
  }


  private clearStaleTimer(): void {
    if (this.staleTimer !== null) clearTimeout(this.staleTimer);
    this.staleTimer = null;
  }
  private setStatus(status: LinkStatus): void {
    this.statusListeners.forEach(l => l(status));
  }
}
