import { decodeVehicleFrame, VEHICLE_REQUEST_TYPES, type VehicleFrame, type VehicleRequestType } from "./vehicleState";
// Mirror of src/senior_control/protocol.py and src/senior_control/errors.py (wire format v1).
// The lists below are hand-mirrored; devices.test.tsx parses the Python sources and fails when
// a request type, error code, envelope field or snapshot field drifts apart.

export const PROTOCOL_VERSION = 1;
export const MAX_MESSAGE_BYTES = 65536;

export const ERROR_CODES = [
  "validation",
  "conflict",
  "offline",
  "unavailable",
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
  "navigation.goal",
  "navigation.return_home",
  "waypoint.create",
  "map.export",
  "map.import",
  "map.clear",
  "operation.query",
] as const;

export const WORKSPACE_REQUEST_TYPES = [
  "task.get",
  "waypoint.list",
  "map.manifest",
  "map.tile",
] as const;
export type WorkspaceRequestType = (typeof WORKSPACE_REQUEST_TYPES)[number];

export function isWorkspaceRequestType(type: string): type is WorkspaceRequestType {
  return (WORKSPACE_REQUEST_TYPES as readonly string[]).includes(type);
}

/** Frames a browser may originate for the agent, excluding video signaling. */
export const REQUEST_TYPES = [
  ...SIMPLE_REQUEST_TYPES,
  ...OPERATION_REQUEST_TYPES,
  ...WORKSPACE_REQUEST_TYPES,
] as const;
export type RequestType = (typeof REQUEST_TYPES)[number];

export const RESPONSE_REQUEST_TYPES = [
  "task.execution.command",
  ...OPERATION_REQUEST_TYPES,
  "control.acquire",
  "control.release",
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
  ...WORKSPACE_REQUEST_TYPES,
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
  "tasks",
  "home_revision",
  "map_generation",
  "route_progress",
  "navigation_ready",
  "robot_pose",
  "navigation_block_reason",
  "scan_points",
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
    | {
      type: "video.offer" | "video.answer";
      payload: {
        sdp: string;
      };
    }
    | {
      type: "video.candidate";
      payload: {
        candidate: string;
      };
    }
    | {
      type: "video.close";
      payload: Record<string, never>;
    }
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
export interface TaskSummary {
  task_id: string;
  revision: number;
  enabled: boolean;
}

const ROUTE_STATUSES = [
  "starting", "running", "succeeded", "failed", "cancel_pending", "canceled",
] as const;

export interface RouteProgress {
  execution_id: string;
  task_id: string | null;
  map_generation: string;
  status: (typeof ROUTE_STATUSES)[number];
  active_waypoint_index: number | null;
}

export interface MapPoint {
  x: number;
  y: number;
}

export interface RobotPose extends MapPoint {
  yaw: number;
}

export interface SnapshotPayload {
  //Compatible: legacy map/video snapshots omit the UI extension and never enable intent driving.
  vehicle_ui?: VehicleFrame;
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
  /** Missing on older frames. The parser fills an empty list. */
  tasks?: TaskSummary[];
  /**
   * `home_edit.revision`. Missing on an older frame means 0, the same answer as a store
   * with no row.
   */
  home_revision: number;
  //Compatible: legacy snapshots omit the live reference generation.
  map_generation?: string;
  route_progress?: RouteProgress;
  navigation_ready?: boolean;
  //Compatible: older snapshots omit live map telemetry.
  robot_pose?: RobotPose | null;
  //Compatible: older vehicles do not report the navigation blocker.
  navigation_block_reason?: string | null;
  scan_points?: MapPoint[];
}

export interface ResponsePayload {
  request_type: (typeof RESPONSE_REQUEST_TYPES)[number] | VehicleRequestType;
  ok: boolean;
  error?: ErrorCode | null;
  operation_id?: string | null;
  status?: ResponseStatus | null;
  data?: Record<string, unknown>;
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
  reconnecting?: boolean;
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

function parseRouteProgress(value: unknown): RouteProgress | null {
  if (!isRecord(value) || !isText(value["execution_id"], 128) ||
    !isText(value["map_generation"], 128)) return null;
  const task = value["task_id"];
  const status = ROUTE_STATUSES.find((candidate) => candidate === value["status"]);
  const index = value["active_waypoint_index"];
  if ((task !== null && !isText(task, 128)) || status === undefined ||
    (index !== null && (!isCount(index) || index >= 64))) return null;
  return {
    execution_id: value["execution_id"],
    task_id: task,
    map_generation: value["map_generation"],
    status,
    active_waypoint_index: index,
  };
}

function parseRobotPose(value: unknown): RobotPose | null | undefined {
  //Compatible: missing or null telemetry means there is no qualified live pose.
  if (value === undefined || value === null) return null;
  if (!isRecord(value) || !isFiniteNumber(value["x"]) || !isFiniteNumber(value["y"]) ||
    !isFiniteNumber(value["yaw"])) return undefined;
  return { x: value["x"], y: value["y"], yaw: value["yaw"] };
}

function parseScanPoints(value: unknown): MapPoint[] | null {
  //Compatible: legacy snapshots may omit or null the scan overlay.
  if (value === undefined || value === null) return [];
  if (!Array.isArray(value) || value.length > 96) return null;
  const points: MapPoint[] = [];
  for (const point of value) {
    if (!isRecord(point) || !isFiniteNumber(point["x"]) || !isFiniteNumber(point["y"])) return null;
    points.push({ x: point["x"], y: point["y"] });
  }
  return points;
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
  const robotPose = parseRobotPose(value["robot_pose"]);
  const scanPoints = parseScanPoints(value["scan_points"]);
  if (robotPose === undefined || scanPoints === null) return null;
  const tasks = parseTasks(value["tasks"]);
  if (tasks === null) return null;
  const homeRevision = value["home_revision"] === undefined ? 0 : value["home_revision"];
  if (!isCount(homeRevision)) return null;
  const navigationBlockReason = value["navigation_block_reason"] ?? null;
  if (navigationBlockReason !== null && !isText(navigationBlockReason, 256)) return null;
  const navigationReady = value["navigation_ready"];
  if (navigationReady !== undefined && typeof navigationReady !== "boolean") return null;
  const mapGeneration = value["map_generation"];
  const route = value["route_progress"] === undefined ? undefined :
    parseRouteProgress(value["route_progress"]);
  if (route === null) return null;
  if (mapGeneration !== undefined && !isText(mapGeneration, 128)) return null;
  let vehicle: VehicleFrame | undefined;
  if (value["vehicle_ui"] !== undefined) {
    try {
      vehicle = decodeVehicleFrame(value["vehicle_ui"]);
    } catch {
      return null;
    }
  }
  return {
    ...(vehicle === undefined ? {} : { vehicle_ui: vehicle }),
    ...(mapGeneration === undefined ? {} : { map_generation: mapGeneration }),
    ...(navigationReady === undefined ? {} : { navigation_ready: navigationReady }),
    ...(route === undefined || route.map_generation !== mapGeneration ? {} : { route_progress: route }),
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
    tasks,
    home_revision: homeRevision,
    robot_pose: robotPose,
    navigation_block_reason: navigationBlockReason,
    scan_points: scanPoints,
  };
}

function parseTasks(value: unknown): TaskSummary[] | null {
  if (value === undefined) return [];
  if (!Array.isArray(value) || value.length > 200) return null;
  const tasks: TaskSummary[] = [];
  for (const item of value) {
    if (!isRecord(item) || !isText(item["task_id"], 128) || !isCount(item["revision"])) return null;
    if (typeof item["enabled"] !== "boolean") return null;
    tasks.push({ task_id: item["task_id"], revision: item["revision"], enabled: item["enabled"] });
  }
  return tasks;
}

/** Matches the car's TaskNavigationState, including unique action identities. */
function isTaskNavigationState(value: unknown): boolean {
  if (!isRecord(value)) return false;
  const keys = ["execution_id", "phase", "epoch", "completed", "remaining"];
  if (Object.keys(value).length !== keys.length || keys.some(key => !(key in value))) return false;
  if (!isText(value["execution_id"], 128)) return false;
  const phase = value["phase"];
  if (typeof phase !== "string" || !["starting", "running", "revoking", "paused", "succeeded", "canceled", "failed"].includes(phase)) return false;
  if (!isCount(value["epoch"]) || !Number.isSafeInteger(value["epoch"])) return false;
  const completed = value["completed"], remaining = value["remaining"];
  if (!Array.isArray(completed) || !Array.isArray(remaining)) return false;
  const actions: unknown[] = [...completed, ...remaining];
  return actions.every(id => isText(id, 256)) && new Set(actions).size === actions.length;
}

function parseResponsePayload(value: unknown): ResponsePayload | null {
  if (!isRecord(value) || typeof value["ok"] !== "boolean") return null;
  const requestType = value["request_type"];
  if (!(RESPONSE_REQUEST_TYPES as readonly unknown[]).includes(requestType) && !(VEHICLE_REQUEST_TYPES as readonly unknown[]).includes(requestType)) return null;
  const error = value["error"] ?? null;
  if (error !== null && !(ERROR_CODES as readonly unknown[]).includes(error)) return null;
  // Same rule as ResponsePayload.consistent_result: success and error cannot agree.
  if (value["ok"] === (error !== null)) return null;
  const status = value["status"] ?? null;
  if (status !== null && !(RESPONSE_STATUSES as readonly unknown[]).includes(status)) return null;
  const operationId = value["operation_id"] ?? null;
  if (operationId !== null && typeof operationId !== "string") return null;
  const data = value["data"];
  if (typeof requestType !== "string") return null;
  const typed = VEHICLE_REQUEST_TYPES.includes(requestType) || requestType === "control.acquire" || requestType === "control.release";
  const expectsData = (isWorkspaceRequestType(requestType) || requestType.startsWith("settings.") || requestType === "control.acquire") && value["ok"];
  //Compatible: older navigation and non-task operation.query replies omit lifecycle data.
  const optionalTask = requestType === "navigation.goal" || requestType === "navigation.return_home" || requestType === "operation.query";
  if (optionalTask) {
    if (data !== undefined && data !== null && (!value["ok"] || !isTaskNavigationState(data))) return null;
  } else if (expectsData ? (!isRecord(data) && !(requestType === "settings.result" && data === null)) : data !== undefined && data !== null && (!typed || !isRecord(data))) return null;
  return {
    ...(isRecord(data) ? { data } : {}),
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
