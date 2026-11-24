import { CommandError, type RobotLink } from "./client";
import type { MapRef, ResponsePayload } from "./protocol";
import { workspaceRequest } from "./workspace";

export type MapPhase = "idle" | "loading" | "mapping" | "saving" | "matching" | "pending_calibration" | "unavailable" | "failed";
export type MapCommand = "map.create" | "map.activate" | "map.delete" | "map.calibrate" | "map.home";
export interface SavedMap {
  map_ref: MapRef;
  name: string;
  checkpoint_id: string | null;
  home_id: string | null;
  version: number;
  saved_at: number | null;
}
export interface MappingCatalog {
  robot_id: string;
  entries: SavedMap[];
  selection: { map_ref: MapRef | null; generation: number; state: "none" | "matching" | "ready" | "pending_calibration" };
  runtime_available: boolean;
  phase: MapPhase;
}
const PHASES = new Set<string>(["idle", "loading", "mapping", "saving", "matching", "pending_calibration", "unavailable", "failed"]);
const STATES = new Set<string>(["none", "matching", "ready", "pending_calibration"]);
const record = (value: unknown): value is Record<string, unknown> => typeof value === "object" && value !== null && !Array.isArray(value);
const count = (value: unknown): value is number => typeof value === "number" && Number.isSafeInteger(value) && value >= 0;
const text = (value: unknown, limit: number): value is string => typeof value === "string" && value.trim().length > 0 && value.length <= limit;
function valid(value: unknown): asserts value {
  if (!value) throw new CommandError("validation", "地图目录响应无效");
}
function reference(value: unknown): MapRef {
  valid(record(value) && text(value["map_id"], 128) && count(value["reference_revision"]));
  return { map_id: value["map_id"] as string, reference_revision: value["reference_revision"] as number };
}
export async function readMappingCatalog(link: RobotLink, robotId: string): Promise<MappingCatalog> {
  const response = await workspaceRequest(link, "map.catalog", {});
  if (!response.ok) throw new CommandError(response.error ?? "unavailable", "地图目录不可用");
  const data = response.data;
  valid(record(data));
  if (data["robot_id"] !== robotId) throw new CommandError("conflict", "地图目录来自另一台设备");
  const rows = data["entries"], selected = data["selection"];
  valid(Array.isArray(rows) && rows.length <= 64 && record(selected));
  valid(typeof data["runtime_available"] === "boolean" && typeof data["phase"] === "string" && PHASES.has(data["phase"]));
  valid(count(selected["generation"]) && typeof selected["state"] === "string" && STATES.has(selected["state"]));
  const ids = new Set<string>();
  const entries = rows.map((row: unknown): SavedMap => {
    valid(record(row));
    const ref = reference(row["map_ref"]);
    valid(!ids.has(ref.map_id)); ids.add(ref.map_id);
    valid(text(row["name"], 64) && count(row["version"]));
    valid(row["saved_at"] === null || typeof row["saved_at"] === "number" && Number.isFinite(row["saved_at"]));
    valid(row["checkpoint_id"] === null || text(row["checkpoint_id"], 128));
    valid(row["home_id"] === null || text(row["home_id"], 128));
    return { map_ref: ref, name: row["name"], checkpoint_id: row["checkpoint_id"], home_id: row["home_id"], version: row["version"], saved_at: row["saved_at"] } as SavedMap;
  });
  return { robot_id: robotId, entries, selection: {
    map_ref: selected["map_ref"] === null ? null : reference(selected["map_ref"]),
    generation: selected["generation"], state: selected["state"] as MappingCatalog["selection"]["state"],
  }, runtime_available: data["runtime_available"], phase: data["phase"] as MapPhase } as MappingCatalog;
}
export async function sendMapCommand(link: RobotLink, kind: MapCommand, operationId: string, body: Record<string, unknown>): Promise<ResponsePayload> {
  const response = await link.request<ResponsePayload>(kind, { operation_id: operationId, body });
  if (!response.ok) throw new CommandError(response.error ?? "operation_unknown", "地图操作未确认");
  if (response.operation_id !== operationId || response.status == null) throw new CommandError("operation_unknown", "地图操作回执身份不符");
  return response;
}
