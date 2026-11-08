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

/** A sample has no current server challenge; it is discarded rather than queued. */
export class MotionNotReadyError extends CommandError {
  override name = "MotionNotReadyError";
  constructor() {
    super("safety_rejected", "no current motion challenge");
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
  createPairingCode(signal?: AbortSignal): Promise<PairingCode>;
}

const CSRF_HEADER = "X-CSRF-Token";
const CSRF_STORAGE_KEY = "senior-control.csrf";

async function detail(response: Response): Promise<string> {
  try {
    const body: unknown = await response.json();
    if (typeof body === "object" && body !== null && "detail" in body) {
      const value = (body as { detail: unknown }).detail;
      if (typeof value === "string") return value;
    }
  } catch {
    // The body is not JSON; the status line is all there is.
  }
  return response.statusText || `HTTP ${response.status}`;
}

function parseRobots(body: unknown): RobotSummary[] {
  const list = typeof body === "object" && body !== null ? (body as { robots?: unknown }).robots : null;
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
    const body = (await response.json()) as { csrf_token?: unknown };
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

export type DriveFeedback =
  | { status: "applied"; latencyMs: number; vx: number; wz: number; limited: boolean }
  | { status: "blocked"; reason: string | null }
  | { status: "superseded" | "expired" };

interface AwaitedDriveFeedback {
  sentAt: number;
  epoch: number;
  seq: number;
  gesture: number;
  timer: ReturnType<typeof setTimeout>;
}

export interface RobotLink {
  /** Whether this link implements the challenge-based motion protocol. */
  readonly supportsMotionChallenges?: boolean;
  readonly timedMotion?: boolean;
  subscribeDriveFeedback?(listener: (feedback: DriveFeedback) => void): () => void;
  subscribeMotionRevoked?(listener: (authorityLost: boolean) => void): () => void;
  connect(robotId: string): Promise<void>;
  disconnect(): void;
  /** Resolves with the robot's response only when it says ok; every failure rejects. */
  request<T = ResponsePayload>(type: string, payload: unknown): Promise<T>;
  /** Video signaling uses the existing socket envelope but never enters the command queue. */
  sendVideoSignal?(type: VideoSignalType, payload: { sdp: string } | { candidate: string } | Record<string, never>): void;
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
  onmessage: ((event: { data: unknown }) => void) | null;
  onclose: ((event: { code: number }) => void) | null;
  onerror: (() => void) | null;
}

export interface RobotClientOptions {
  socketFactory?: (url: string) => SocketLike;
  /**
   * A live snapshot with no newer frame after this long is shown as a cached copy. Matches the
   * cloud cache. Frames the cloud already marks stale are never made live by this timer.
   */
  staleAfterMs?: number;
  requestTimeoutMs?: number;
}

const SOCKET_OPEN = 1;
const CLOSE_NORMAL = 1000;
const DEFAULT_STALE_AFTER_MS = 5_000;
const DEFAULT_REQUEST_TIMEOUT_MS = 5_000;
const COMMAND_TTL_SECONDS = 5;

function controlUrl(robotId: string): string {
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  return `${scheme}//${location.host}/ws/control?robot_id=${encodeURIComponent(robotId)}`;
}

function openBrowserSocket(url: string): SocketLike {
  // The DOM event types are narrower than SocketLike's; the handlers only read `data` and `code`.
  return new WebSocket(url) as unknown as SocketLike;
}

interface Pending {
  type: string;
  sessionId: string;
  epoch: number;
  seq: number;
  resolve: (value: ResponsePayload) => void;
  reject: (error: CommandError) => void;
  timer: ReturnType<typeof setTimeout>;
}

export class RobotClient implements RobotLink {
  readonly supportsMotionChallenges = true;
  private readonly socketFactory: (url: string) => SocketLike;
  private readonly staleAfterMs: number;
  private readonly requestTimeoutMs: number;
  private readonly snapshotListeners = new Set<(snapshot: RobotSnapshot) => void>();
  private readonly statusListeners = new Set<(status: LinkStatus) => void>();
  private readonly videoListeners = new Set<(frame: VideoSignalEnvelope) => void>();
  private readonly pending = new Map<string, Pending>();
  private readonly motionListeners = new Set<(authorityLost: boolean) => void>();
  private readonly driveFeedbackListeners = new Set<(feedback: DriveFeedback) => void>();
  private readonly awaitedDriveFeedback = new Map<string, AwaitedDriveFeedback>();
  private latestFeedbackSequence = 0;
  private motionLease = false;
  private motionVersion = 0;
  private challenge: { id: string; requestedAt: number } | null = null;
  private challengePending: symbol | null = null;
  private timingEpoch = 0;
  private timingTimer: ReturnType<typeof setInterval> | null = null;
  private timingProbe: { nonce: string; sent: number; wall: number } | null = null;
  private timingConfirmation: { nonce: string; t4: number } | null = null;
  private readyAt: number | null = null;
  private gestureCounter = 0;
  private activeGesture: number | null = null;

  get timedMotion(): boolean {
    return this.timingEpoch > 0 && this.timingEpoch === this.epoch;
  }

  private generation = 0;
  private socket: SocketLike | null = null;
  private robotId: string | null = null;
  private sessionId = "";
  /** The agent epoch the last snapshot frame delivered. 0 means none: no agent owns epoch 0. */
  private epoch = 0;
  private epochFrontier = 0;
  private retiredEpoch = 0;
  private seq = 0;
  private latest: RobotSnapshot | null = null;
  private staleTimer: ReturnType<typeof setTimeout> | null = null;
  private abortConnect: (() => void) | null = null;

  constructor(options: RobotClientOptions = {}) {
    this.socketFactory = options.socketFactory ?? openBrowserSocket;
    this.staleAfterMs = options.staleAfterMs ?? DEFAULT_STALE_AFTER_MS;
    this.requestTimeoutMs = options.requestTimeoutMs ?? DEFAULT_REQUEST_TIMEOUT_MS;
  }

  connect(robotId: string): Promise<void> {
    this.teardown();
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
    if (!isRequestType(type)) {
      return Promise.reject(new CommandError("validation", `unsupported request type ${type}`));
    }
    const timed = this.timedMotion;
    const stoppingGesture = type === "drive.stop" ? this.activeGesture : null;
    if (type === "drive.stop" || type === "safety.estop" || type.startsWith("control.")) {
      this.activeGesture = null;
    }
    const urgent = type === "drive.stop" || type === "safety.estop";
    const safety = urgent || type === "control.release";
    const authorityLost = type === "safety.estop" || type === "control.release";
    const replacingLease = type === "control.acquire" && this.motionLease;
    if (safety || type === "control.acquire") this.invalidateMotion(authorityLost || type === "control.acquire");
    if (replacingLease) this.notifyMotionRevoked(true);
    const socket = this.socket;
    const robotId = this.robotId;
    if (socket === null || robotId === null || socket.readyState !== SOCKET_OPEN) {
      if (safety) this.notifyMotionRevoked(authorityLost);
      return Promise.reject(new CommandError("offline", "control socket is not open"));
    }
    if (!urgent && (this.latest === null || this.epoch < 1 || ((this.latest.stale || !this.latest.connected) && !safety))) {
      if (safety) this.notifyMotionRevoked(authorityLost);
      return Promise.reject(new CommandError("offline", "no live snapshot from the vehicle, so no agent epoch to use"));
    }
    if (type === "control.acquire" && timed && !this.driveClockReady()) {
      return Promise.reject(new CommandError("unavailable", "motion clock is not ready"));
    }
    let wirePayload = stoppingGesture === null ? payload : { gesture: stoppingGesture };
    if (type === "drive.velocity") {
      if (this.challenge !== null && performance.now() - this.challenge.requestedAt >= 250) {
        this.challenge = null;
      }
      if (!this.motionLease || socket.bufferedAmount > 0 || (!timed && this.challenge === null)) {
        this.prefetchChallenge();
        return Promise.reject(new MotionNotReadyError());
      }
      if (typeof payload !== "object" || payload === null || Array.isArray(payload)) {
        return Promise.reject(new CommandError("validation", "invalid velocity"));
      }
      const input = payload as Record<string, unknown>;
      if (typeof input.vx !== "number" || !Number.isFinite(input.vx) ||
          typeof input.wz !== "number" || !Number.isFinite(input.wz) ||
          Object.keys(input).some((key) => key !== "vx" && key !== "wz")) {
        return Promise.reject(new CommandError("validation", "invalid velocity"));
      }
      if (timed) {
        if (!this.driveClockReady()) {
          this.loseDriveClock();
          return Promise.reject(new MotionNotReadyError());
        }
        this.activeGesture ??= ++this.gestureCounter;
        wirePayload = {
          vx: input.vx, wz: input.wz, gesture: this.activeGesture, sampled_at: performance.now() / 1000,
        };
      } else {
        //Compatible: older agents require a one-time cloud challenge per sample.
        wirePayload = { vx: input.vx, wz: input.wz, challenge_id: this.challenge?.id };
        this.challenge = null;
      }
    }
    const requestId = crypto.randomUUID();
    const version = this.motionVersion;
    const envelope: EnvelopeBase & { type: string; payload: unknown } = {
      v: PROTOCOL_VERSION,
      robot_id: robotId,
      session_id: this.sessionId,
      connection_epoch: this.epoch,
      request_id: requestId,
      seq: ++this.seq,
      expires_at: Date.now() / 1000 + COMMAND_TTL_SECONDS,
      type,
      payload: wirePayload,
    };
    const text = JSON.stringify(envelope);
    if (text.length > MAX_MESSAGE_BYTES) {
      return Promise.reject(new CommandError("validation", "request is too large"));
    }
    let sent = false;
    const response = new Promise<T>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(requestId);
        reject(timed && type === "drive.velocity" ? new MotionNotReadyError() :
          new CommandError("timeout", "robot did not answer in time"));
      }, this.requestTimeoutMs);
      this.pending.set(requestId, {
        type, sessionId: envelope.session_id, epoch: envelope.connection_epoch, seq: envelope.seq,
        resolve: (value) => resolve(value as T), reject, timer,
      });
      try {
        socket.send(text);
        sent = true;
        if (timed && type === "drive.velocity" && this.activeGesture !== null) {
          this.awaitDriveFeedback(requestId, envelope.seq, this.activeGesture);
        }
      } catch {
        clearTimeout(timer);
        this.pending.delete(requestId);
        reject(new CommandError("offline", "control socket rejected the frame"));
      }
    });
    if (!sent) {
      if (safety || type === "drive.velocity") {
        this.invalidateMotion(true);
        this.notifyMotionRevoked(true);
      }
      return response;
    }
    if (safety) this.notifyMotionRevoked(authorityLost);
    if (type === "drive.stop" || type === "drive.velocity") this.prefetchChallenge();
    if (type !== "control.acquire") return response;
    return response.then((value) => {
      if (version !== this.motionVersion || socket !== this.socket || this.epoch !== envelope.connection_epoch) {
        throw new CommandError("conflict", "control authority changed before acknowledgement");
      }
      this.motionLease = true;
      this.prefetchChallenge();
      return value;
    });
  }

  private invalidateMotion(authorityLost: boolean): void {
    this.motionVersion += 1;
    this.clearAwaitedDriveFeedback();
    this.challenge = null;
    this.challengePending = null;
    if (authorityLost) this.motionLease = false;
    for (const [id, pending] of this.pending) {
      if (pending.type !== "drive.challenge") continue;
      clearTimeout(pending.timer);
      this.pending.delete(id);
      pending.reject(new CommandError("offline", "motion challenge revoked"));
    }
  }

  private notifyMotionRevoked(authorityLost: boolean): void {
    this.motionListeners.forEach((listener) => listener(authorityLost));
  }

  private prefetchChallenge(): void {
    if (this.timedMotion || !this.motionLease || this.challenge !== null || this.challengePending !== null) return;
    const attempt = Symbol("challenge");
    const version = this.motionVersion;
    const requestedAt = performance.now();
    this.challengePending = attempt;
    void this.request<ResponsePayload>("drive.challenge", {}).then(
      (response) => {
        if (this.challengePending !== attempt || version !== this.motionVersion || !this.motionLease) return;
        this.challengePending = null;
        if (typeof response.challenge_id === "string" && typeof response.server_deadline === "number") {
          this.challenge = { id: response.challenge_id, requestedAt };
        }
      },
      () => {
        if (this.challengePending !== attempt || version !== this.motionVersion) return;
        this.challengePending = null;
        this.invalidateMotion(true);
        this.notifyMotionRevoked(true);
      },
    );
  }

  sendVideoSignal(
    type: VideoSignalType,
    payload: { sdp: string } | { candidate: string } | Record<string, never>,
  ): void {
    const socket = this.socket;
    const robotId = this.robotId;
    if (socket === null || robotId === null || socket.readyState !== SOCKET_OPEN || this.epoch < 1) {
      throw new CommandError("offline", "control socket is not open for video signaling");
    }
    const envelope: EnvelopeBase & { type: VideoSignalType; payload: typeof payload } = {
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

  subscribeMotionRevoked(listener: (authorityLost: boolean) => void): () => void {
    this.motionListeners.add(listener);
    return () => this.motionListeners.delete(listener);
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

  // ------------------------------------------------------------ internals

  /** Drop the socket, timers, the previous robot's snapshot and every waiting request. */
  private teardown(): void {
    this.clearDriveClock();
    this.generation += 1;
    this.invalidateMotion(true);
    this.notifyMotionRevoked(true);
    const socket = this.socket;
    this.socket = null;
    if (socket !== null) {
      socket.onopen = socket.onmessage = socket.onclose = socket.onerror = null;
      socket.close(CLOSE_NORMAL);
    }
    this.clearStaleTimer();
    this.latest = null;
    this.abortConnect?.();
    this.abortConnect = null;
    this.failPending("control socket closed");
  }

  private failPending(message: string): void {
    for (const entry of this.pending.values()) {
      clearTimeout(entry.timer);
      entry.reject(new CommandError("offline", message));
    }
    this.pending.clear();
  }

  private onClosed(code: number): void {
    this.clearDriveClock();
    this.invalidateMotion(true);
    this.notifyMotionRevoked(true);
    this.clearStaleTimer();
    this.failPending("control socket closed");
    this.markStale();
    this.setStatus({ state: "closed", robotId: this.robotId, closeCode: code });
  }

  subscribeDriveFeedback(listener: (feedback: DriveFeedback) => void): () => void {
    this.driveFeedbackListeners.add(listener);
    return () => this.driveFeedbackListeners.delete(listener);
  }

  private clearAwaitedDriveFeedback(): void {
    for (const pending of this.awaitedDriveFeedback.values()) clearTimeout(pending.timer);
    this.awaitedDriveFeedback.clear();
  }

  private reportDriveFeedback(seq: number, feedback: DriveFeedback): void {
    if (seq <= this.latestFeedbackSequence) return;
    this.latestFeedbackSequence = seq;
    this.driveFeedbackListeners.forEach((listener) => listener(feedback));
  }

  private awaitDriveFeedback(requestId: string, seq: number, gesture: number): void {
    const sentAt = performance.now();
    const timer = setTimeout(() => {
      this.awaitedDriveFeedback.delete(requestId);
      this.reportDriveFeedback(seq, { status: "expired" });
    }, 2_000);
    this.awaitedDriveFeedback.set(requestId, { sentAt, epoch: this.epoch, seq, gesture, timer });
    if (this.awaitedDriveFeedback.size > 32) {
      const oldest = this.awaitedDriveFeedback.keys().next().value;
      if (oldest !== undefined) {
        clearTimeout(this.awaitedDriveFeedback.get(oldest)?.timer);
        this.awaitedDriveFeedback.delete(oldest);
      }
    }
  }

  private receiveDriveFeedback(frame: Record<string, unknown>): void {
    const requestId = frame.request_id;
    if (frame.robot_id !== this.robotId || frame.session_id !== this.sessionId ||
        frame.connection_epoch !== this.epoch || typeof requestId !== "string") return;
    const pending = this.awaitedDriveFeedback.get(requestId);
    if (pending === undefined || pending.epoch !== frame.connection_epoch ||
        pending.seq !== frame.seq || pending.gesture !== frame.gesture ||
        pending.gesture !== this.activeGesture) return;
    const latencyMs = performance.now() - pending.sentAt;
    if (latencyMs < 0 || latencyMs >= 2_000) return;
    let feedback: DriveFeedback;
    if (frame.status === "applied") {
      if (typeof frame.vx !== "number" || typeof frame.wz !== "number" ||
          !Number.isFinite(frame.vx) || !Number.isFinite(frame.wz) ||
          typeof frame.limited !== "boolean") return;
      feedback = { status: "applied", vx: frame.vx, wz: frame.wz, limited: frame.limited, latencyMs };
    } else if (frame.status === "blocked") {
      feedback = {
        status: "blocked", reason: typeof frame.reason === "string" ? frame.reason.slice(0, 128) : null,
      };
    } else if (frame.status === "superseded" || frame.status === "expired") {
      feedback = { status: frame.status };
    } else return;
    clearTimeout(pending.timer);
    this.awaitedDriveFeedback.delete(requestId);
    this.reportDriveFeedback(pending.seq, feedback);
  }

  private driveClockReady(): boolean {
    return this.readyAt !== null && performance.now() - this.readyAt < 1_500;
  }

  private loseDriveClock(): void {
    this.readyAt = null;
    if (!this.motionLease) return;
    this.invalidateMotion(true);
    this.notifyMotionRevoked(true);
  }

  private clearDriveClock(): void {
    if (this.timingTimer !== null) clearInterval(this.timingTimer);
    this.timingTimer = null;
    this.timingEpoch = 0;
    this.readyAt = null;
    this.timingProbe = null;
    this.timingConfirmation = null;
    this.activeGesture = null;
    this.latestFeedbackSequence = 0;
  }

  private probeDriveClock(): void {
    const socket = this.socket;
    if (!this.driveClockReady()) this.loseDriveClock();
    if (socket === null || socket.readyState !== SOCKET_OPEN || socket.bufferedAmount > 0) return;
    if (this.timingProbe !== null && performance.now() - this.timingProbe.sent < 1_000) return;
    if (this.timingProbe !== null) this.timingConfirmation = null;
    const nonce = crypto.randomUUID();
    const sent = performance.now();
    this.timingProbe = { nonce, sent, wall: Date.now() };
    try {
      socket.send(JSON.stringify({
        type: "drive.clock", nonce, t1: sent / 1000, confirmation: this.timingConfirmation,
      }));
    } catch {
      this.loseDriveClock();
    }
  }

  private receiveDriveClock(raw: unknown): boolean {
    if (typeof raw !== "string" || raw.length > MAX_MESSAGE_BYTES) return false;
    let frame: Record<string, unknown>;
    try {
      frame = JSON.parse(raw) as Record<string, unknown>;
    } catch {
      return false;
    }
    if (frame === null || typeof frame !== "object") return false;
    if (frame.type === "drive.feedback") {
      this.receiveDriveFeedback(frame);
      return true;
    }
    if (frame.type === "drive.timing") {
      if (frame.robot_id !== this.robotId || typeof frame.connection_epoch !== "number" ||
          !Number.isSafeInteger(frame.connection_epoch) || frame.connection_epoch < this.epochFrontier ||
          frame.connection_epoch <= this.timingEpoch) return true;
      this.loseDriveClock();
      this.timingEpoch = frame.connection_epoch;
      this.timingProbe = null;
      this.timingConfirmation = null;
      if (this.timingTimer === null) {
        this.timingTimer = setInterval(() => this.probeDriveClock(), 500);
      }
      this.probeDriveClock();
      return true;
    }
    if (frame.type !== "drive.clock.reply") return false;
    const pending = this.timingProbe;
    if (pending === null || frame.nonce !== pending.nonce) return true;
    this.timingProbe = null;
    const now = performance.now();
    const elapsed = now - pending.sent;
    if (elapsed < 0 || elapsed > 1_000 || Math.abs(Date.now() - pending.wall - elapsed) > 50 ||
        typeof frame.t2 !== "number" || typeof frame.t3 !== "number" ||
        !Number.isFinite(frame.t2) || !Number.isFinite(frame.t3) || frame.t3 < frame.t2) {
      this.timingConfirmation = null;
      this.loseDriveClock();
      return true;
    }
    this.timingConfirmation = { nonce: pending.nonce, t4: now / 1000 };
    if (frame.ready === true && frame.connection_epoch === this.timingEpoch) this.readyAt = now;
    else this.loseDriveClock();
    return true;
  }

  private receive(raw: unknown): void {
    if (this.receiveDriveClock(raw)) return;
    const envelope = parseInbound(raw);
    if (envelope === null) {
      const video = parseVideoInbound(raw);
      if (video !== null && video.robot_id === this.robotId) {
        this.videoListeners.forEach((listener) => listener(video));
      }
      return;
    }
    if (envelope.robot_id !== this.robotId) return;
    if (envelope.type === "snapshot") {
      this.onSnapshot(envelope);
      return;
    }
    const entry = this.pending.get(envelope.request_id);
    if (entry === undefined || entry.type !== envelope.payload.request_type) return;
    const workspace = isWorkspaceRequestType(entry.type);
    if ((entry.type === "control.acquire" || entry.type === "drive.challenge" || workspace) &&
        (entry.sessionId !== envelope.session_id || entry.epoch !== envelope.connection_epoch || entry.seq !== envelope.seq)) return;
    this.pending.delete(envelope.request_id);
    clearTimeout(entry.timer);
    const { payload } = envelope;
    if (workspace && (entry.epoch !== this.epoch || this.latest === null || this.latest.stale || !this.latest.connected)) {
      entry.reject(new CommandError("offline", "workspace response belongs to an expired connection"));
    } else if (payload.ok) entry.resolve(payload);
    else {
      const code = payload.error ?? "validation";
      entry.reject(new CommandError(code, describeCommandError(code)));
    }
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
    const lostLiveState = (payload.stale || !payload.connected) &&
      (this.latest === null || (!this.latest.stale && this.latest.connected));
    if ((this.epoch > 0 && this.epoch !== envelope.connection_epoch) || lostLiveState) {
      this.invalidateMotion(true);
      this.notifyMotionRevoked(true);
    }
    this.epoch = envelope.connection_epoch;
    this.latest = { robot_id: envelope.robot_id, ...payload };
    this.clearStaleTimer();
    if (!this.latest.stale) {
      this.staleTimer = setTimeout(() => this.markStale(), this.staleAfterMs);
    }
    this.snapshotListeners.forEach((listener) => listener(this.latest as RobotSnapshot));
  }

  /** A cached copy never claims the robot is connected (same rule as the cloud SnapshotCache). */
  private markStale(): void {
    this.clearStaleTimer();
    const current = this.latest;
    if (current === null || current.stale) return;
    this.invalidateMotion(true);
    this.notifyMotionRevoked(true);
    this.latest = { ...current, connected: false, stale: true };
    this.snapshotListeners.forEach((listener) => listener(this.latest as RobotSnapshot));
  }

  private clearStaleTimer(): void {
    if (this.staleTimer !== null) clearTimeout(this.staleTimer);
    this.staleTimer = null;
  }

  private setStatus(status: LinkStatus): void {
    this.statusListeners.forEach((listener) => listener(status));
  }
}
