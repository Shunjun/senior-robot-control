// Mirror of src/senior_control/protocol.py and src/senior_control/errors.py (wire format v1).
// The lists below are hand-mirrored; devices.test.tsx parses the Python sources and fails when
// a request type, error code, envelope field or snapshot field drifts apart.

export const PROTOCOL_VERSION = 1;
export const MAX_MESSAGE_BYTES = 65536;

export const ERROR_CODES = [
  "validation",
  "conflict",
  "offline",
  "localization_unavailable",
  "safety_rejected",
  "timeout",
  "operation_unknown",
] as const;
export type ErrorCode = (typeof ERROR_CODES)[number];

export const SIMPLE_REQUEST_TYPES = [
  "control.acquire",
  "control.release",
  "drive.stop",
  "safety.estop",
  "safety.reset",
  "navigation.start",
  "navigation.cancel",
  "navigation.resume",
] as const;

export const OPERATION_REQUEST_TYPES = [
  "task.create",
  "task.update",
  "task.delete",
  "task.enable",
  "home.set",
  "map.export",
  "map.import",
  "map.clear",
  "operation.query",
] as const;

/** Frames a browser may originate for the agent, excluding video signaling. */
export const REQUEST_TYPES = [
  "drive.velocity",
  ...SIMPLE_REQUEST_TYPES,
  ...OPERATION_REQUEST_TYPES,
] as const;
export type RequestType = (typeof REQUEST_TYPES)[number];

export const RESPONSE_REQUEST_TYPES = [
  ...OPERATION_REQUEST_TYPES,
  "control.acquire",
  "control.release",
  "drive.velocity",
  "drive.stop",
  "safety.estop",
  "safety.reset",
  "navigation.start",
  "navigation.cancel",
  "navigation.resume",
  "video.offer",
  "video.answer",
  "video.candidate",
  "video.close",
] as const;

export const ENVELOPE_FIELDS = [
  "v",
  "robot_id",
  "session_id",
  "connection_epoch",
  "request_id",
  "seq",
  "expires_at",
] as const;

export const SNAPSHOT_FIELDS = [
  "connected",
  "localized",
  "map_ref",
  "home",
  "task_revision",
  "runnable_task_ids",
  "execution_status",
  "collected_at",
  "stale",
  "last_synced_at",
] as const;

export const RESPONSE_STATUSES = [
  "accepted",
  "dispatching",
  "running",
  "succeeded",
  "failed",
  "needs_review",
] as const;
export type ResponseStatus = (typeof RESPONSE_STATUSES)[number];

export const VIDEO_SIGNAL_TYPES = [
  "video.offer",
  "video.answer",
  "video.candidate",
  "video.close",
] as const;
export type VideoSignalType = (typeof VIDEO_SIGNAL_TYPES)[number];
export type VideoSignalEnvelope = EnvelopeBase &
  (
    | { type: "video.offer" | "video.answer"; payload: { sdp: string } }
    | { type: "video.candidate"; payload: { candidate: string } }
    | { type: "video.close"; payload: Record<string, never> }
  );

export interface EnvelopeBase {
  v: 1;
  robot_id: string;
  session_id: string;
  connection_epoch: number;
  request_id: string;
  seq: number;
  expires_at: number;
}

export interface MapRef {
  map_id: string;
  reference_revision: number;
}

export interface HomeSummary {
  waypoint_id: string;
  map_ref: MapRef;
  status: string;
}

/**
 * SnapshotPayload: the robot's own view of itself. On the control socket the cloud always adds
 * `stale` and `last_synced_at` from its cache; the console never substitutes its own clock.
 */
export interface SnapshotPayload {
  connected: boolean;
  localized: boolean;
  map_ref: MapRef | null;
  home: HomeSummary | null;
  task_revision: number;
  runnable_task_ids: string[];
  execution_status: string;
  collected_at: number;
  stale: boolean;
  last_synced_at: number;
}

export interface ResponsePayload {
  request_type: (typeof RESPONSE_REQUEST_TYPES)[number];
  ok: boolean;
  error?: ErrorCode | null;
  operation_id?: string | null;
  status?: ResponseStatus | null;
}

export interface SnapshotEnvelope extends EnvelopeBase {
  type: "snapshot";
  payload: SnapshotPayload;
}

export interface ResponseEnvelope extends EnvelopeBase {
  type: "response";
  payload: ResponsePayload;
}

export type InboundEnvelope = SnapshotEnvelope | ResponseEnvelope;

/**
 * A snapshot as the console shows it. It mirrors the cloud's CloudSnapshot: the robot's
 * snapshot plus `last_synced_at` and `stale`. A stale snapshot is a cached copy, never live
 * state, and `connected` is already false there.
 */
export interface RobotSnapshot extends SnapshotPayload {
  robot_id: string;
}

/** One row of `GET /api/robots`. */
export interface RobotSummary {
  id: string;
  name: string;
  online: boolean;
}

export interface LinkStatus {
  state: "idle" | "connecting" | "open" | "closed";
  robotId: string | null;
  closeCode: number | null;
}

// ---------------------------------------------------------------- parsing

type Json = Record<string, unknown>;

function isRecord(value: unknown): value is Json {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function isText(value: unknown, max: number): value is string {
  return typeof value === "string" && value.length >= 1 && value.length <= max;
}

function isCount(value: unknown): value is number {
  return typeof value === "number" && Number.isInteger(value) && value >= 0;
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

function parseMapRef(value: unknown): MapRef | null | undefined {
  if (value === null) return null;
  if (!isRecord(value) || !isText(value["map_id"], 128) || !isCount(value["reference_revision"])) {
    return undefined;
  }
  return { map_id: value["map_id"], reference_revision: value["reference_revision"] };
}

function parseHome(value: unknown): HomeSummary | null | undefined {
  if (value === null) return null;
  if (!isRecord(value) || !isText(value["waypoint_id"], 128) || !isText(value["status"], 128)) {
    return undefined;
  }
  const mapRef = parseMapRef(value["map_ref"]);
  if (mapRef === undefined || mapRef === null) return undefined;
  return { waypoint_id: value["waypoint_id"], map_ref: mapRef, status: value["status"] };
}

function parseSnapshotPayload(value: unknown): SnapshotPayload | null {
  if (!isRecord(value)) return null;
  const mapRef = parseMapRef(value["map_ref"]);
  const home = parseHome(value["home"]);
  const runnable = value["runnable_task_ids"];
  if (
    typeof value["connected"] !== "boolean" ||
    typeof value["localized"] !== "boolean" ||
    mapRef === undefined ||
    home === undefined ||
    !isCount(value["task_revision"]) ||
    !Array.isArray(runnable) ||
    runnable.length > 200 ||
    !runnable.every((id) => isText(id, 128)) ||
    !isText(value["execution_status"], 64) ||
    !isFiniteNumber(value["collected_at"]) ||
    typeof value["stale"] !== "boolean" ||
    !isFiniteNumber(value["last_synced_at"])
  ) {
    return null;
  }
  return {
    connected: value["connected"],
    localized: value["localized"],
    map_ref: mapRef,
    home,
    task_revision: value["task_revision"],
    runnable_task_ids: runnable as string[],
    execution_status: value["execution_status"],
    collected_at: value["collected_at"],
    stale: value["stale"],
    last_synced_at: value["last_synced_at"],
  };
}

function parseResponsePayload(value: unknown): ResponsePayload | null {
  if (!isRecord(value) || typeof value["ok"] !== "boolean") return null;
  const requestType = value["request_type"];
  if (!(RESPONSE_REQUEST_TYPES as readonly unknown[]).includes(requestType)) return null;
  const error = value["error"] ?? null;
  if (error !== null && !(ERROR_CODES as readonly unknown[]).includes(error)) return null;
  // Same rule as ResponsePayload.consistent_result: success and error cannot agree.
  if (value["ok"] === (error !== null)) return null;
  const status = value["status"] ?? null;
  if (status !== null && !(RESPONSE_STATUSES as readonly unknown[]).includes(status)) return null;
  const operationId = value["operation_id"] ?? null;
  if (operationId !== null && typeof operationId !== "string") return null;
  return {
    request_type: requestType as ResponsePayload["request_type"],
    ok: value["ok"],
    error: error as ErrorCode | null,
    operation_id: operationId,
    status: status as ResponseStatus | null,
  };
}

/** Validate one inbound text frame. Anything malformed, oversized or of another type is null. */
export function parseInbound(raw: unknown): InboundEnvelope | null {
  if (typeof raw !== "string" || raw.length > MAX_MESSAGE_BYTES) return null;
  let data: unknown;
  try {
    data = JSON.parse(raw);
  } catch {
    return null;
  }
  if (!isRecord(data) || data["v"] !== PROTOCOL_VERSION) return null;
  if (
    !isText(data["robot_id"], 128) ||
    !isText(data["session_id"], 128) ||
    !isCount(data["connection_epoch"]) ||
    !isText(data["request_id"], 128) ||
    !isCount(data["seq"]) ||
    !isFiniteNumber(data["expires_at"])
  ) {
    return null;
  }
  const base: EnvelopeBase = {
    v: PROTOCOL_VERSION,
    robot_id: data["robot_id"],
    session_id: data["session_id"],
    connection_epoch: data["connection_epoch"],
    request_id: data["request_id"],
    seq: data["seq"],
    expires_at: data["expires_at"],
  };
  if (data["type"] === "snapshot") {
    const payload = parseSnapshotPayload(data["payload"]);
    return payload === null ? null : { ...base, type: "snapshot", payload };
  }
  if (data["type"] === "response") {
    const payload = parseResponsePayload(data["payload"]);
    return payload === null ? null : { ...base, type: "response", payload };
  }
  return null;
}

/** Parse only the four existing signaling frame shapes; command parsing remains unchanged. */
export function parseVideoInbound(raw: unknown): VideoSignalEnvelope | null {
  if (typeof raw !== "string" || raw.length > MAX_MESSAGE_BYTES) return null;
  let data: unknown;
  try {
    data = JSON.parse(raw);
  } catch {
    return null;
  }
  if (!isRecord(data) || data["v"] !== PROTOCOL_VERSION || !isRecord(data["payload"])) return null;
  if (
    !isText(data["robot_id"], 128) ||
    !isText(data["session_id"], 128) ||
    !isCount(data["connection_epoch"]) ||
    !isText(data["request_id"], 128) ||
    !isCount(data["seq"]) ||
    !isFiniteNumber(data["expires_at"])
  ) return null;
  const base: EnvelopeBase = {
    v: PROTOCOL_VERSION,
    robot_id: data["robot_id"],
    session_id: data["session_id"],
    connection_epoch: data["connection_epoch"],
    request_id: data["request_id"],
    seq: data["seq"],
    expires_at: data["expires_at"],
  };
  const payload = data["payload"];
  if ((data["type"] === "video.offer" || data["type"] === "video.answer") && isText(payload["sdp"], 16_384)) {
    return { ...base, type: data["type"], payload: { sdp: payload["sdp"] } };
  }
  if (
    data["type"] === "video.candidate" &&
    isText(payload["candidate"], 512) &&
    payload["candidate"].startsWith("candidate:")
  ) {
    return { ...base, type: data["type"], payload: { candidate: payload["candidate"] } };
  }
  if (data["type"] === "video.close" && Object.keys(payload).length === 0) {
    return { ...base, type: data["type"], payload: {} };
  }
  return null;
}

export function isRequestType(value: string): value is RequestType {
  return (REQUEST_TYPES as readonly string[]).includes(value);
}
