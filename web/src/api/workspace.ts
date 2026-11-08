import { CommandError, type RobotLink } from "./client";
import type { MapRef, ResponsePayload, WorkspaceRequestType } from "./protocol";

const TILE_SIZE = 32;
const MAX_GRID_SIDE = 65_536;
const MAX_GRID_CELLS = 16_777_216;

interface WorkspaceIdentity {
  robot_id: string;
  map_ref: MapRef;
  grid_id: string;
}

interface MapPose {
  x: number;
  y: number;
  yaw: number;
}

export interface MapManifest extends WorkspaceIdentity {
  width: number;
  height: number;
  resolution: number;
  origin: MapPose;
  tile_size: 32;
  display_only?: boolean;
}

export interface MapTile extends WorkspaceIdentity {
  tile_x: number;
  tile_y: number;
  width: number;
  height: number;
  cells: number[];
}

export interface WorkspaceWaypoint extends MapPose {
  waypoint_id: string;
  kind: "waypoint" | "home";
  status: string;
}

export interface WaypointPage extends WorkspaceIdentity {
  waypoints: WorkspaceWaypoint[];
  next_cursor: string | null;
}

export type TaskSchedule =
  | { kind: "interval"; every_seconds: number }
  | { kind: "cron"; expression: string };

export interface TaskDetails extends WorkspaceIdentity {
  task_id: string;
  waypoint_ids: string[];
  schedule: TaskSchedule;
  timezone: string;
  enabled: boolean;
  revision: number;
  created_at: string;
}

const queuedReads = new WeakMap<RobotLink, Promise<void>>();

function valid(condition: unknown): asserts condition {
  if (!condition) throw new CommandError("validation", "workspace data is invalid");
}

function isObject(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function isText(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0;
}

function isNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

function isInteger(value: unknown, min: number, max = Number.MAX_SAFE_INTEGER): value is number {
  return isNumber(value) && Number.isSafeInteger(value) && value >= min && value <= max;
}

function isPose(value: unknown): value is MapPose {
  return isObject(value) && isNumber(value["x"]) && isNumber(value["y"]) && isNumber(value["yaw"]);
}

function read(
  link: RobotLink,
  robotId: string,
  mapRef: MapRef,
  type: WorkspaceRequestType,
  fields: Record<string, unknown> = {},
): Promise<WorkspaceIdentity & Record<string, unknown>> {
  const next = (queuedReads.get(link) ?? Promise.resolve()).then(async () => {
    const response = await link.request<ResponsePayload>(type, { map_ref: mapRef, ...fields });
    const data = response.data;
    valid(isObject(data));
    const ref = data["map_ref"];
    if (data["robot_id"] !== robotId || !isObject(ref) ||
        ref["map_id"] !== mapRef.map_id || ref["reference_revision"] !== mapRef.reference_revision) {
      throw new CommandError("conflict", "workspace response is from another map or vehicle");
    }
    const gridId = data["grid_id"];
    valid(isText(gridId) && gridId.length <= 128);
    return { ...data, robot_id: robotId, map_ref: mapRef, grid_id: gridId };
  });
  queuedReads.set(link, next.then(() => undefined, () => undefined));
  return next;
}

export async function getMapManifest(
  link: RobotLink, robotId: string, mapRef: MapRef,
): Promise<MapManifest> {
  const data = await read(link, robotId, mapRef, "map.manifest");
  const { width, height, resolution, origin, tile_size, display_only } = data;
  valid(isInteger(width, 1, MAX_GRID_SIDE) && isInteger(height, 1, MAX_GRID_SIDE) &&
    width * height <= MAX_GRID_CELLS && isNumber(resolution) && resolution > 0 &&
    isPose(origin) && tile_size === TILE_SIZE &&
    (display_only === undefined || typeof display_only === "boolean"));
  return { ...data, width, height, resolution, origin, tile_size, display_only: display_only ?? false };
}

export async function getMapTile(
  link: RobotLink, robotId: string, mapRef: MapRef, gridId: string, tileX: number, tileY: number,
): Promise<MapTile> {
  const data = await read(link, robotId, mapRef, "map.tile", {
    grid_id: gridId, tile_x: tileX, tile_y: tileY,
  });
  const { width, height, cells } = data;
  if (data.grid_id !== gridId || data["tile_x"] !== tileX || data["tile_y"] !== tileY) {
    throw new CommandError("conflict", "map grid changed");
  }
  valid(isInteger(width, 1, TILE_SIZE) && isInteger(height, 1, TILE_SIZE) &&
    Array.isArray(cells) && cells.length === width * height &&
    cells.every((cell: unknown) => isInteger(cell, -1, 100)));
  return { ...data, tile_x: tileX, tile_y: tileY, width, height, cells };
}

function isWaypoint(value: unknown): value is WorkspaceWaypoint {
  return isObject(value) && isPose(value) && isText(value["waypoint_id"]) &&
    (value["kind"] === "home" || value["kind"] === "waypoint") && isText(value["status"]);
}

export async function listWaypoints(
  link: RobotLink, robotId: string, mapRef: MapRef, cursor: string | null = null,
): Promise<WaypointPage> {
  const data = await read(link, robotId, mapRef, "waypoint.list", { cursor });
  const { waypoints, next_cursor } = data;
  valid(Array.isArray(waypoints) && waypoints.length <= 32 && waypoints.every(isWaypoint) &&
    (next_cursor === null || (isText(next_cursor) && next_cursor !== cursor)));
  return { ...data, waypoints, next_cursor };
}

function isSchedule(value: unknown): value is TaskSchedule {
  if (!isObject(value)) return false;
  if (value["kind"] === "interval") return isInteger(value["every_seconds"], 1);
  return value["kind"] === "cron" && isText(value["expression"]);
}

export async function getTaskDetails(
  link: RobotLink, robotId: string, mapRef: MapRef, taskId: string,
): Promise<TaskDetails> {
  const data = await read(link, robotId, mapRef, "task.get", { task_id: taskId });
  const { waypoint_ids, schedule, timezone, enabled, revision, created_at } = data;
  valid(data["task_id"] === taskId && Array.isArray(waypoint_ids) && waypoint_ids.length > 0 &&
    waypoint_ids.every(isText) && isSchedule(schedule) && isText(timezone) &&
    typeof enabled === "boolean" && isInteger(revision, 0) && isText(created_at) &&
    Number.isFinite(Date.parse(created_at)));
  return { ...data, task_id: taskId, waypoint_ids, schedule, timezone, enabled, revision, created_at };
}
