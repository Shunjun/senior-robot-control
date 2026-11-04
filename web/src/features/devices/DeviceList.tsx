import type { CSSProperties, ReactElement } from "react";
import { NavLink } from "react-router-dom";

import type { RobotSummary } from "../../api/protocol";
import styles from "./Devices.module.css";

export interface RobotListState {
  status: "loading" | "ready";
  robots: RobotSummary[];
  /** The last refresh failed. The robots shown are the last known list, online flags unverified. */
  failed: boolean;
}

export interface DeviceListProps {
  list: RobotListState;
  selectedId: string | null;
  onRefresh: () => void;
}

function Presence({ robot, unknown }: { robot: RobotSummary; unknown: boolean }): ReactElement {
  if (unknown) {
    return <span className={`${styles.presence} ${styles.presenceUnknown}`}>状态未知</span>;
  }
  return robot.online ? (
    <span className={`${styles.presence} ${styles.presenceOnline}`}>在线</span>
  ) : (
    <span className={`${styles.presence} ${styles.presenceOffline}`}>离线</span>
  );
}

export function DeviceList({ list, selectedId, onRefresh }: DeviceListProps): ReactElement {
  const empty = list.status === "ready" && !list.failed && list.robots.length === 0;
  return (
    <nav className={styles.deviceList} aria-label="设备列表">
      <div className={styles.listHead}>
        <h2 className={styles.listTitle}>
          设备 <span className={styles.count}>{String(list.robots.length).padStart(2, "0")}</span>
        </h2>
        <button type="button" className={styles.ghost} onClick={onRefresh}>
          刷新设备
        </button>
      </div>
      {list.failed && (
        <p className={styles.failure} role="alert">
          设备列表加载失败，显示的是上一次的列表，在线状态未经核实。
        </p>
      )}
      {list.status === "loading" && !list.failed && (
        <p className={styles.quiet} role="status">
          正在读取设备…
        </p>
      )}
      {empty && (
        <div className={styles.empty}>
          <p className={styles.emptyTitle}>尚未配对任何设备</p>
          <p className={styles.quiet}>在车端使用配对码完成配对后，设备会出现在这里。</p>
        </div>
      )}
      <ul className={styles.items}>
        {list.robots.map((robot, index) => (
          <li key={robot.id} style={{ "--i": index } as CSSProperties}>
            <NavLink
              to={`/robots/${encodeURIComponent(robot.id)}`}
              className={({ isActive }) =>
                isActive || robot.id === selectedId ? `${styles.item} ${styles.itemActive}` : styles.item
              }
            >
              <span className={styles.itemName}>{robot.name}</span>
              <span className={styles.itemId}>{robot.id}</span>
              <Presence robot={robot} unknown={list.failed} />
            </NavLink>
          </li>
        ))}
      </ul>
    </nav>
  );
}
