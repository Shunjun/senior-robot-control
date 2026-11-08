import { useEffect, useState } from "react";

import type { RobotLink } from "../../api/client";
import type { MapRef, RobotSnapshot } from "../../api/protocol";
import { getMapManifest, getMapTile, listWaypoints, type WorkspaceWaypoint } from "../../api/workspace";
import type { OccupancySnapshot } from "./MapCanvas";

interface MapResult {
  map?: OccupancySnapshot;
  gridId?: string;
  displayOnly?: boolean;
  waypointWrites?: boolean;
  navigationReady?: boolean;
  waypoints?: WorkspaceWaypoint[];
  message?: string;
}

interface LoadOptions {
  enabled: boolean;
  catalog: boolean;
  refresh: number;
}

export async function listCurrentWaypoints(
  link: RobotLink, robotId: string, ref: MapRef, gridId: string, signal: AbortSignal,
): Promise<WorkspaceWaypoint[]> {
  const waypoints: WorkspaceWaypoint[] = [];
  const cursors = new Set<string>();
  let cursor: string | null = null;
  do {
    signal.throwIfAborted();
    const page = await listWaypoints(link, robotId, ref, cursor);
    signal.throwIfAborted();
    if (page.grid_id !== gridId || (page.next_cursor !== null && cursors.has(page.next_cursor))) {
      throw new Error("航点目录已变化");
    }
    waypoints.push(...page.waypoints);
    cursor = page.next_cursor;
    if (cursor !== null) cursors.add(cursor);
  } while (cursor !== null);
  return waypoints;
}

export function useWorkspaceMap(link: RobotLink, snapshot: RobotSnapshot, { enabled, catalog, refresh }: LoadOptions): MapResult {
  const { robot_id: robotId, map_ref: ref, stale, connected } = snapshot;
  const mapId = ref?.map_id;
  const revision = ref?.reference_revision;
  const scope = JSON.stringify([robotId, mapId, revision]);
  const [result, setResult] = useState<{ scope: string; link: RobotLink; value: MapResult } | null>(null);

  useEffect(() => {
    if (stale || !connected) {
      setResult(null);
      return;
    }
    if (!enabled || mapId === undefined || revision === undefined) return;
    const mapRef = { map_id: mapId, reference_revision: revision };
    const reads = new AbortController();
    const publish = (value: MapResult) => {
      if (!reads.signal.aborted) setResult({ scope, link, value });
    };

    async function load(): Promise<void> {
      let map: OccupancySnapshot | undefined;
      publish({ message: "正在读取当前地图…" });
      try {
        const manifest = await getMapManifest(link, robotId, mapRef);
        if (reads.signal.aborted) return;
        const { width, height, resolution, origin, tile_size: size, grid_id: gridId } = manifest;
        const values = new Int8Array(width * height);
        for (let y = 0; y < height; y += size) {
          for (let x = 0; x < width; x += size) {
            const tile = await getMapTile(link, robotId, mapRef, gridId, x / size, y / size);
            if (reads.signal.aborted) return;
            if (tile.width !== Math.min(size, width - x) || tile.height !== Math.min(size, height - y)) {
              throw new Error("地图图块尺寸与清单不一致");
            }
            for (let row = 0; row < tile.height; row += 1) {
              values.set(tile.cells.slice(row * tile.width, (row + 1) * tile.width), (y + row) * width + x);
            }
          }
        }

        map = { map_ref: mapRef, resolution, origin, cells: [], grid: { width, height, values } };
        if (manifest.display_only === true) {
          publish({ map, gridId, displayOnly: true, message: "地图导出副本 · 仅供查看" });
          return;
        }
        const waypointWrites = "waypoint_writes" in manifest && manifest.waypoint_writes === true;
        const navigationReady = "navigation_ready" in manifest && manifest.navigation_ready === true;
        publish({ map, gridId, waypointWrites, navigationReady });
        if (!catalog) return;

        const waypoints = await listCurrentWaypoints(link, robotId, mapRef, gridId, reads.signal);
        publish({ map, gridId, waypointWrites, navigationReady, waypoints });
      } catch {
        if (map === undefined) publish({ message: "地图不可用 · 车端未提供当前引用的栅格" });
        else publish({ map, message: "航点目录不可用 · 请重新打开任务场景" });
      }
    }
    void load();
    return () => reads.abort();
  }, [link, robotId, mapId, revision, scope, stale, connected, enabled, catalog, refresh]);

  if (stale || !connected) return { message: "地图不可用 · 车辆未实时连接" };
  if (ref === null) return { message: "地图不可用 · 当前没有地图引用" };
  return result?.scope === scope && result.link === link ? result.value : { message: "正在读取当前地图…" };
}
