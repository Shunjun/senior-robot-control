import { useRef, useState, type ReactElement } from "react";
import { NavLink } from "react-router-dom";

import { X, CircleDot, RefreshCw } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { IconButton } from "../../components/ui/IconButton";

import type { RobotSummary } from "../../api/protocol";
import { PairingPanel, type PairingPanelProps } from "./PairingPanel";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  presence: [
    "devices-presence inline-flex items-center gap-1 text-[10px]"
  ].join(" "),
  presenceUnknown: [
    "devices-presence-unknown text-muted"
  ].join(" "),
  presenceOnline: [
    "devices-presence-online text-online"
  ].join(" "),
  presenceOffline: [
    "devices-presence-offline text-muted"
  ].join(" "),
  deviceList: [
    "devices-device-list pt-0 pr-0 pb-0 pl-0"
  ].join(" "),
  listHead: [
    "devices-list-head flex items-center justify-between gap-2.5 mb-[5px] [&>button]:border-0 [&>button]:border-none",
    "[&>button]:border-current [&>button]:bg-none [&>button]:bg-transparent [&>button]:pt-1 [&>button]:pr-[7px] [&>button]:pb-1 [&>button]:pl-[7px] [&>button]:text-quiet"
  ].join(" "),
  listTitle: [
    "devices-list-title mt-0 mr-0 mb-0 ml-0 text-[13px] font-semibold"
  ].join(" "),
  count: [
    "devices-count text-[10px] text-muted"
  ].join(" "),
  listActions: [
    "devices-list-actions flex items-center gap-2 [&_button]:border-0 [&_button]:border-none [&_button]:border-current [&_button]:bg-none",
    "[&_button]:bg-transparent [&_button]:pt-1 [&_button]:pr-[7px] [&_button]:pb-1 [&_button]:pl-[7px] [&_button]:text-quiet"
  ].join(" "),
  quiet: [
    "devices-quiet mt-0 mr-0 mb-0 ml-0 text-muted text-[11px] leading-[1.8]"
  ].join(" "),
  failure: [
    "devices-failure mt-2 mr-0 mb-2 ml-0 pt-2.5 pr-2.5 pb-2.5",
    "pl-2.5 text-alert-ink text-[11px] bg-none bg-red-wash rounded-md"
  ].join(" "),
  empty: [
    "devices-empty pt-5 pr-0 pb-5 pl-0"
  ].join(" "),
  emptyTitle: [
    "devices-empty-title text-[16px] font-[450]"
  ].join(" "),
  items: [
    "devices-items list-none pt-0 pr-0 pb-0 pl-0 mt-4 mr-0",
    "mb-4 ml-0 grid gap-1.5"
  ].join(" "),
  item: [
    "devices-item flex items-center gap-[11px] pt-3 pr-3 pb-3 pl-3",
    "border border-solid border-devices-item-border-color rounded-control bg-none bg-app-empty-symbol-background-color no-underline [&::after]:[content:\"→\"]",
    "[&::after]:ml-auto"
  ].join(" "),
  itemActive: "devices-item-active [&&]:bg-devices-item-active-background-color [&&]:border-devices-item-active-border-color [&&::after]:content-['✓'] [&&::after]:text-amber",
  vehicleMark: [
    "devices-vehicle-mark flex-none text-devices-vehicle-mark-color"
  ].join(" "),
  vehicleIdentity: [
    "devices-vehicle-identity flex-1 min-w-0"
  ].join(" "),
  itemName: [
    "devices-item-name block text-[12px] font-medium"
  ].join(" "),
  itemId: [
    "devices-item-id block mt-1 text-[10px] text-ink-dim [overflow-wrap:anywhere]"
  ].join(" "),
  unpairConfirm: [
    "devices-unpair-confirm pt-2.5 pr-2.5 pb-2.5 pl-2.5 bg-none bg-devices-unpair-confirm-background-color rounded-md",
    "text-[11px] [&_p]:text-[11px]"
  ].join(" "),
  unpairActions: [
    "devices-unpair-actions flex items-center gap-2.5"
  ].join(" "),
  removePairing: [
    "devices-remove-pairing pt-1 pr-[7px] pb-1 pl-[7px] bg-none bg-transparent border-0",
    "border-none border-current text-quiet text-[10px] cursor-pointer"
  ].join(" "),
  deviceActions: [
    "devices-device-actions flex items-center gap-2.5 [&>button:first-child]:flex-1"
  ].join(" "),
  selectionActions: [
    "devices-selection-actions flex items-center gap-2.5 justify-between mt-3 [&_button]:pt-1 [&_button]:pr-[7px]",
    "[&_button]:pb-1 [&_button]:pl-[7px] [&_button]:bg-none [&_button]:bg-transparent [&_button]:border-0 [&_button]:border-none [&_button]:border-current [&_button]:text-quiet",
    "[&_button]:text-[10px]"
  ].join(" "),
} as const;

export interface RobotListState {
  status: "loading" | "ready";
  robots: RobotSummary[];
  /** The last refresh failed. The robots shown are the last known list, online flags unverified. */
  failed: boolean;
}

export interface DeviceListProps {
  onSelect?: () => void;
  onClose?: () => void;
  onPair?: (() => void) | undefined;
  onClearSelection?: (() => void) | undefined;
  list: RobotListState;
  selectedId: string | null;
  onRefresh: () => void;
  pairing: PairingPanelProps | null;
  onRevoke: ((robotId: string) => Promise<void>) | null;
}

function Presence({
  robot,
  unknown,
}: {
  robot: RobotSummary;
  unknown: boolean;
}): ReactElement {
  if (unknown) {
    return (
      <span className={`${ui.presence} ${ui.presenceUnknown}`}>
        状态未知
      </span>
    );
  }
  if (robot.reconnecting) {
    return (
      <span className={`${ui.presence} ${ui.presenceUnknown}`}>
        重连中
      </span>
    );
  }
  return robot.online ? (
    <span className={`${ui.presence} ${ui.presenceOnline}`}>在线</span>
  ) : (
    <span className={`${ui.presence} ${ui.presenceOffline}`}>离线</span>
  );
}

export function DeviceList({
  list,
  selectedId,
  onRefresh,
  pairing,
  onRevoke,
  onSelect,
  onClose,
  onPair,
  onClearSelection,
}: DeviceListProps): ReactElement {
  const [confirmId, setConfirmId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [failure, setFailure] = useState(false);
  const pending = useRef(false);

  async function revoke(robotId: string): Promise<void> {
    if (pending.current || onRevoke === null) return;
    pending.current = true;
    setBusy(true);
    setFailure(false);
    try {
      await onRevoke(robotId);
      setConfirmId(null);
    } catch {
      setFailure(true);
    } finally {
      pending.current = false;
      setBusy(false);
    }
  }
  const selectedRobot = list.robots.find((robot) => robot.id === selectedId);
  const empty =
    list.status === "ready" && !list.failed && list.robots.length === 0;
  return (
    <nav className={ui.deviceList} aria-label="设备列表">
      <div className={ui.listHead}>
        <h2 className={ui.listTitle}>
          选择车辆{" "}
          <span className={ui.count}>
            {String(list.robots.length).padStart(2, "0")}
          </span>
        </h2>
        <div className={ui.listActions}>
          {onClose && (
            <IconButton label="关闭设备列表" onClick={onClose}>
              <X size={16} />
            </IconButton>
          )}
        </div>
      </div>
      <p className={ui.quiet}>选择已配对车辆进入控制</p>
      {pairing !== null && <PairingPanel {...pairing} />}
      {list.failed && (
        <p className={ui.failure} role="alert">
          设备列表加载失败，显示的是上一次的列表，在线状态未经核实。
        </p>
      )}
      {list.status === "loading" && !list.failed && (
        <p className={ui.quiet} role="status">
          正在读取设备…
        </p>
      )}
      {empty && (
        <div className={ui.empty}>
          <p className={ui.emptyTitle}>尚未配对任何设备</p>
          <p className={ui.quiet}>
            在车端使用配对码完成配对后，设备会出现在这里。
          </p>
        </div>
      )}
      <ul className={ui.items}>
        {list.robots.map((robot) => (
          <li key={robot.id}>
            <NavLink
              onClick={onSelect}
              to={`/robots/${encodeURIComponent(robot.id)}`}
              className={({ isActive }) =>
                isActive || robot.id === selectedId
                  ? `${ui.item} ${ui.itemActive}`
                  : ui.item
              }
            >
              <CircleDot className={ui.vehicleMark} size={22} />
              <span className={ui.vehicleIdentity}>
                <strong className={ui.itemName}>{robot.name}</strong>
                <small className={ui.itemId}>
                  {robot.id} · <Presence robot={robot} unknown={list.failed} />
                </small>
              </span>
            </NavLink>
          </li>
        ))}
      </ul>
      <div className={ui.deviceActions}>
        {onPair && (
          <Button variant="primary" aria-label="配对新车" onClick={onPair}>
            ＋ 配对新车
          </Button>
        )}
        <IconButton label="刷新设备" onClick={onRefresh}>
          <RefreshCw size={14} />
        </IconButton>
      </div>
      <div className={ui.selectionActions}>
        {onRevoke !== null && (
          <button
            type="button"
            className={ui.removePairing}
            disabled={busy || selectedRobot === undefined}
            aria-label={selectedRobot ? `删除 ${selectedRobot.name} 的配对` : "解除当前车辆配对"}
            onClick={() => {
              if (!selectedRobot) return;
              setFailure(false);
              setConfirmId(selectedRobot.id);
            }}
          >
            解除当前车辆配对…
          </button>
        )}
        <Button disabled={!selectedRobot || !onClearSelection} onClick={onClearSelection}>
          取消选择
        </Button>
      </div>
      {onRevoke !== null && selectedRobot !== undefined && confirmId === selectedRobot.id && (
        <div className={ui.unpairConfirm}>
          <p>
            确定删除「{selectedRobot.name}」的配对？车端凭据将失效，现有连接将断开，需重新配对后才能连接。
          </p>
          {failure && (
            <p role="alert" className={ui.failure}>
              删除配对失败，请检查网络或会话后重试。
            </p>
          )}
          <div className={ui.unpairActions}>
            <Button
              disabled={busy}
              aria-label="确认删除配对"
              onClick={() => void revoke(selectedRobot.id)}
            >
              {busy ? "正在删除…" : "确认删除配对"}
            </Button>
            <Button disabled={busy} onClick={() => setConfirmId(null)}>
              取消
            </Button>
          </div>
        </div>
      )}
    </nav>
  );
}
