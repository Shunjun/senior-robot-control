import { useState, type ReactElement } from "react";
import styles from "./Mapping.module.css";

export interface MapCatalogEntry {
  map_id: string;
  name: string;
  saved_at: string | null;
}
interface MapCatalogProps {
  entries: readonly MapCatalogEntry[];
  activeId: string | null;
  status: "mapping" | "saving" | "matching" | "pending_calibration";
  busy: boolean;
  onCreate: (name: string) => void;
  onActivate: (id: string) => void;
  onDelete: (id: string) => void;
}

export function MapCatalog({ entries, activeId, status, busy, onCreate, onActivate, onDelete }: MapCatalogProps): ReactElement {
  const [name, setName] = useState("");
  const [deleting, setDeleting] = useState<string | null>(null);
  const labels = { mapping: "持续建图 · 自动保存", saving: "正在保存最新地图", matching: "正在匹配车辆位置", pending_calibration: "等待校准车辆位置" };
  return <section className={styles.panel} aria-label="活动地图管理">
    <header><p className={styles.kicker}>ACTIVE MAP</p><h3>地图管理</h3></header>
    <p role="status">{labels[status]}</p>
    <p className={styles.note}>手动驾驶时也会更新地图，切换视频或地图视图不影响建图。</p>
    {status === "pending_calibration" && <p className={styles.warning}>Home 标记仅作为待校准参考。请在“导航与定位”中校准车辆位置，匹配通过后继续建图与导航。</p>}
    <div className={styles.catalog}>{entries.map(entry => <div className={styles.catalogRow} key={entry.map_id}>
      <div><strong>{entry.name}</strong><p className={styles.note}>{entry.map_id === activeId ? "当前地图" : "已保存"} · {entry.saved_at ?? "等待首次保存"}</p></div>
      <div className={styles.actions}>
        {entry.map_id !== activeId && <button disabled={busy} onClick={() => onActivate(entry.map_id)}>切换</button>}
        <button disabled={busy} onClick={() => setDeleting(entry.map_id)}>删除</button>
      </div>
    </div>)}</div>
    <label>新地图名称<input maxLength={64} value={name} onChange={event => setName(event.target.value)} /></label>
    <div className={styles.actions}><button disabled={busy || name.trim().length === 0} onClick={() => { onCreate(name.trim()); setName(""); }}>新建并开始建图</button></div>
    <p className={styles.note}>新地图的 Home 默认设为开始建图时的车辆位置。每张地图单独保存 Home。</p>
    {deleting !== null && <div role="alertdialog" aria-label="删除地图确认" className={styles.warning}>
      <p>删除“{entries.find(entry => entry.map_id === deleting)?.name}”？{deleting === activeId ? "当前建图会话将先停止。" : ""}此操作不可恢复。</p>
      <div className={styles.actions}><button disabled={busy} onClick={() => { onDelete(deleting); setDeleting(null); }}>确认删除</button><button onClick={() => setDeleting(null)}>取消</button></div>
    </div>}
  </section>;
}
