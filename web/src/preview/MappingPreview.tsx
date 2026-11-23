/** Temporary original-position preview. It never creates a network transport. */
import { useEffect, useState, type ReactElement } from "react";
import type { RobotLink } from "../api/client";
import type { RobotSnapshot } from "../api/protocol";
import { ControlSurface } from "../features/devices/ControlSurface";
import { PerformancePanel } from "../features/devices/PerformancePanel";
import { MapCatalog, type MapCatalogEntry } from "../features/mapping/MapCatalog";

const link: RobotLink = {
  connect: async () => {}, disconnect: () => {},
  request: async <T,>(type: string): Promise<T> => {
    const identity = { robot_id: "preview", map_ref: { map_id: "warehouse", reference_revision: 1 }, grid_id: "preview-grid" };
    if (type === "map.manifest") return { ok: true, data: { ...identity, width: 32, height: 24, resolution: .25, origin: { x: 0, y: 0, yaw: 0 }, tile_size: 32, display_only: true } } as T;
    if (type === "map.tile") return { ok: true, data: { ...identity, tile_x: 0, tile_y: 0, width: 32, height: 24,
      cells: Array.from({ length: 32 * 24 }, (_, i) => { const x = i % 32, y = Math.floor(i / 32); return x === 3 || x === 28 || y === 3 || y === 20 || x > 10 && x < 14 && y > 9 && y < 15 ? 100 : x > 3 && x < 28 && y > 3 && y < 20 ? 0 : -1; }) } } as T;
    throw new Error("视觉预览不发送指令");
  },
  subscribe: () => () => {}, subscribeStatus: () => () => {},
};
const snapshot: RobotSnapshot = {
  robot_id: "preview", connected: true, stale: false, localized: false,
  map_ref: { map_id: "warehouse", reference_revision: 1 }, home: null,
  task_revision: 0, runnable_task_ids: [], execution_status: "idle",
  collected_at: 0, last_synced_at: 0, home_revision: 0,
};

export function MappingPreview(): ReactElement {
  const [sequence, setSequence] = useState(1);
  useEffect(() => { const timer = setInterval(() => setSequence(value => value + 1), 1000); return () => clearInterval(timer); }, []);
  const [entries, setEntries] = useState<MapCatalogEntry[]>([
    { map_id: "warehouse", name: "仓库一层", saved_at: "刚刚自动保存" },
    { map_id: "yard", name: "室外装卸区", saved_at: "今天 14:32" },
  ]);
  const [active, setActive] = useState<string | null>("warehouse");
  const [pending, setPending] = useState(false);
  const [offline, setOffline] = useState(false);
  const performance = {
    sequence, sampled_at: 0, interval_s: 1, scope: "procfs_visible" as const,
    cpu_count: 4, cpu_percent: 72.4, memory_total_bytes: 8 * 1073741824,
    memory_used_bytes: 2.3 * 1073741824,
    modules: [
      { name: "video" as const, status: "running" as const, cpu_percent: 18.1, rss_bytes: 131072000, process_count: 3 },
      { name: "navigation" as const, status: "inactive" as const, cpu_percent: null, rss_bytes: null, process_count: 0 },
      { name: "mapping" as const, status: "running" as const, cpu_percent: 11.8, rss_bytes: 94371840, process_count: 1 },
    ],
  };
  return <main style={{ maxWidth: 1400, margin: "auto", padding: 20 }}>
    <p>交互预览 · 模拟数据 · 不连接车辆</p>
    <button onClick={() => setOffline(value => !value)}>预览{offline ? "在线" : "离线"}状态</button>
    <button onClick={() => setPending(value => !value)}>预览{pending ? "正常建图" : "待校准"}</button>
    <ControlSurface robotId="preview" link={link} snapshot={{ ...snapshot, connected: !offline }}
      performancePreview={<PerformancePanel live={!offline} reading={performance} />}
      mappingPreview={<MapCatalog entries={entries} activeId={active} status={pending ? "pending_calibration" : "mapping"} busy={offline}
        onCreate={name => { const id = crypto.randomUUID(); setEntries(rows => [...rows, { map_id: id, name, saved_at: null }]); setActive(id); }}
        onActivate={id => { setActive(id); setPending(true); }}
        onDelete={id => { setEntries(rows => rows.filter(row => row.map_id !== id)); if (active === id) setActive(null); }} />}
    />
  </main>;
}
