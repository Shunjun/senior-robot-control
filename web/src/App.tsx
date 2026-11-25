import {
  useCallback,
  useEffect,
  useRef,
  useState,
  type ReactElement,
} from "react";
import {
  Navigate,
  Route,
  Routes,
  useLocation,
  useNavigate,
  useParams,
} from "react-router-dom";

import { ChevronDown, CircleDot, ArrowLeftRight } from "lucide-react";
import vehicleMarker from "./assets/vehicle-marker.svg";
import { Button } from "./components/ui/Button";
import { SegmentedControl } from "./components/ui/SegmentedControl";
import { EmergencyStop } from "./features/drive/EmergencyStop";
import { useWorkspaceView } from "./features/devices/useWorkspaceView";
import { SpeedUnitContext } from "./display/speedUnits";

import {
  UnauthorizedError,
  type ControlApi,
  type RobotLink,
} from "./api/client";
import type { RobotSummary } from "./api/protocol";
import { DeviceList, type RobotListState } from "./features/devices/DeviceList";
import {
  PairingPanel,
  type PairingPanelProps,
} from "./features/devices/PairingPanel";
import { Login } from "./features/devices/Login";
import { VehicleSettings } from "./features/settings/VehicleSettings";
import { RobotPanel } from "./features/devices/RobotPanel";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  demo: [
    "app-demo pt-2 pr-2 pb-2 pl-2 bg-none bg-app-demo-background-color text-white"
  ].join(" "),
  demoTag: [
    "app-demo-tag font-bold mr-3"
  ].join(" "),
  placeholder: [
    "app-placeholder flex flex-col items-center justify-center text-center h-[var(--viewport-height)] min-h-[var(--viewport-min)]",
    "text-ink-soft border border-solid border-line rounded-scene bg-none bg-app-placeholder-background-color pt-8",
    "pr-8 pb-8 pl-8 [background:linear-gradient(135deg,_var(--color-tone-26282ba8),_var(--color-tone-1b1e2299))] [&_h2]:text-3xl [&_h2]:font-[450] [&_h2]:mt-3.5 [&_h2]:mr-0",
    "[&_h2]:mb-3.5 [&_h2]:ml-0 [&_h2]:tracking-[-0.5px] [&_p]:text-sm [&_p]:leading-[2] [&_p]:mt-3.5 [&_p]:mr-0 [&_p]:mb-[27px]",
    "[&_p]:ml-0 max-[460.001px]:[&_h2]:text-2xl max-[460.001px]:pt-5 max-[460.001px]:pr-5 max-[460.001px]:pb-5 max-[460.001px]:pl-5 max-[460.001px]:[&_p]:text-xs"
  ].join(" "),
  emptySymbol: [
    "app-empty-symbol grid place-items-center w-[92px] h-[92px] mt-0 mr-auto mb-[27px]",
    "ml-auto border border-solid border-app-empty-symbol-border-color bg-none bg-app-empty-symbol-background-color rounded-symbol shadow-app-empty-symbol",
    "[&_img]:opacity-[0.85] max-[460.001px]:w-[80px] max-[460.001px]:h-[80px]"
  ].join(" "),
  emptyEnglish: [
    "app-empty-english font-normal text-xs leading-[normal] font-mono tracking-[2px] text-app-empty-english-color"
  ].join(" "),
  emptyActions: [
    "app-empty-actions flex justify-center gap-2.5 [&_button]:min-w-[125px] [&_button]:pt-3 [&_button]:pr-4.5 [&_button]:pb-3",
    "[&_button]:pl-4.5 max-[460.001px]:[&_button]:min-w-[112px] max-[460.001px]:[&_button]:pt-[11px] max-[460.001px]:[&_button]:pr-3.5 max-[460.001px]:[&_button]:pb-[11px] max-[460.001px]:[&_button]:pl-3.5"
  ].join(" "),
  emptyFootnote: [
    "app-empty-footnote text-xs text-app-empty-footnote-color mt-[29px] max-[460.001px]:text-xs"
  ].join(" "),
  shell: [
    "app-shell relative flex flex-col gap-3.5 leading-[normal] max-[760.001px]:gap-2.5 max-[460.001px]:[&[data-workspace=false]_.app-device-trigger]:[grid-column:2/5]",
    "max-[460.001px]:[&[data-workspace=false]_.app-device-trigger]:justify-self-end",
    // Recovery notices must reserve space above the settings header actions.
    "[&[data-page=settings]:has(.drive-recovery-actions)_.app-header]:h-auto [&[data-page=settings]:has(.drive-recovery-actions)_.app-header]:flex-wrap [&[data-page=settings]_.drive-recovery-actions]:static",
    "max-[460.001px]:[&[data-page=settings]:has(.drive-recovery-actions)_.drive-estop-dock]:contents max-[460.001px]:[&[data-page=settings]_.drive-recovery-actions]:col-span-full max-[460.001px]:[&[data-page=settings]_.drive-recovery-actions]:row-start-3 max-[460.001px]:[&[data-page=settings]_.drive-recovery-actions]:w-full",
    "max-[460.001px]:[&[data-page=settings]:has(.drive-recovery-actions)_.drive-estop]:col-start-4 max-[460.001px]:[&[data-page=settings]:has(.drive-recovery-actions)_.drive-estop]:row-span-2"
  ].join(" "),
  header: [
    "app-header flex items-center gap-5 h-[46px] flex-none relative z-[30]",
    "[&>div[role=group]]:bg-none [&>div[role=group]]:bg-transparent [&>div[role=group]]:pt-0 [&>div[role=group]]:pr-0 [&>div[role=group]]:pb-0 [&>div[role=group]]:pl-0 [&>div[role=group]]:w-[max-content] [&>div[role=group]]:justify-self-start",
    "[&>div[role=group]_button]:text-xs [&>div[role=group]_button]:pt-[9px] [&>div[role=group]_button]:pr-[13px] [&>div[role=group]_button]:pb-[9px] [&>div[role=group]_button]:pl-[13px] max-[760.001px]:gap-[7px] max-[760.001px]:flex-wrap max-[760.001px]:h-auto",
    "max-[760.001px]:min-h-[44px] max-[760.001px]:[&>button]:text-xs max-[460.001px]:grid max-[460.001px]:grid-cols-[auto_minmax(0,_1fr)_auto_auto] max-[460.001px]:gap-[7px] max-[460.001px]:[&>div[role=group]]:[grid-row:2] max-[460.001px]:[&>div[role=group]]:[grid-column:1/3] max-[460.001px]:[&>div[role=group]_button]:text-xs",
    "max-[460.001px]:[&>div[role=group]_button]:min-h-0 max-[460.001px]:[&>div[role=group]_button]:pt-[7px] max-[460.001px]:[&>div[role=group]_button]:pr-[7px] max-[460.001px]:[&>div[role=group]_button]:pb-[7px] max-[460.001px]:[&>div[role=group]_button]:pl-[7px] max-[460.001px]:[&>.app-grow]:hidden max-[460.001px]:[&>button[aria-label=切换速度单位]]:[grid-row:1] max-[460.001px]:[&>button[aria-label=切换速度单位]]:[grid-column:3]",
    "max-[460.001px]:[&>button[aria-label=切换速度单位]]:min-h-0 max-[460.001px]:[&>button[aria-label=切换速度单位]]:text-xs max-[460.001px]:[&>button[aria-label=切换速度单位]]:pt-[5px] max-[460.001px]:[&>button[aria-label=切换速度单位]]:pr-[9px] max-[460.001px]:[&>button[aria-label=切换速度单位]]:pb-[5px] max-[460.001px]:[&>button[aria-label=切换速度单位]]:pl-[9px] max-[460.001px]:[&>button[aria-label=切换速度单位]]:leading-[1.2] max-[460.001px]:[&>button[aria-expanded]]:[grid-row:2]",
    "max-[460.001px]:[&>button[aria-expanded]]:[grid-column:3] max-[460.001px]:[&>button[aria-expanded]]:min-h-0 max-[460.001px]:[&>button[aria-expanded]]:text-xs max-[460.001px]:[&>button[aria-expanded]]:pt-1.5 max-[460.001px]:[&>button[aria-expanded]]:pr-2 max-[460.001px]:[&>button[aria-expanded]]:pb-1.5 max-[460.001px]:[&>button[aria-expanded]]:pl-2 max-[460.001px]:[&>div:has(>button[aria-label=急停])]:[grid-row:1/3]",
    "max-[460.001px]:[&>div:has(>button[aria-label=急停])]:[grid-column:4] [&>button[aria-label=切换速度单位]]:text-xs [&>button[aria-label=切换速度单位]]:pt-2 [&>button[aria-label=切换速度单位]]:pr-2.5 [&>button[aria-label=切换速度单位]]:pb-2 [&>button[aria-label=切换速度单位]]:pl-2.5 [&>button[aria-label=切换速度单位]>span]:flex [&>button[aria-label=切换速度单位]>span]:items-center",
    "[&>button[aria-label=切换速度单位]>span]:gap-1 [&>button[aria-label=切换速度单位]_small]:flex [&>button[aria-label=切换速度单位]_small]:items-center [&>button[aria-label=切换速度单位]_small]:gap-[3px] [&>button[aria-label=切换速度单位]_small]:text-app-header-color [&>button[aria-label=切换速度单位]_small]:text-xs [&>button[aria-expanded]]:text-sm max-[460.001px]:[&>button[aria-label=切换速度单位]>span]:block",
    "max-[460.001px]:[&>button[aria-label=切换速度单位]_small]:mt-px max-[460.001px]:[&>button[aria-label=切换速度单位]_small]:text-xs [&>button[aria-label=切换速度单位]_small_svg]:w-[10px] [&>button[aria-label=切换速度单位]_small_svg]:h-[10px]"
  ].join(" "),
  brand: [
    "app-brand flex items-center gap-[9px] whitespace-nowrap pr-5 border-r [border-right-style:solid]",
    "border-r-line [&_svg]:w-[29px] [&_svg]:h-[29px] [&_svg]:text-amber max-[760.001px]:[&_svg]:w-[23px] max-[760.001px]:[&_svg]:h-[23px] max-[460.001px]:pr-0 max-[460.001px]:border-0",
    "max-[460.001px]:border-none max-[460.001px]:border-current"
  ].join(" "),
  brandName: [
    "app-brand-name text-xl tracking-[2px] font-semibold max-[760.001px]:text-base max-[460.001px]:tracking-[1px]"
  ].join(" "),
  deviceTrigger: [
    "app-device-trigger flex-none [&_button]:text-xs [&_button]:pt-1 [&_button]:pr-[7px] [&_button]:pb-1 [&_button]:pl-[7px] [&_button]:bg-none",
    "[&_button]:bg-transparent [&_button]:border-0 [&_button]:border-none [&_button]:border-current max-[760.001px]:[&_button]:text-xs max-[760.001px]:[&_button]:pt-1.5 max-[760.001px]:[&_button]:pr-1.5 max-[760.001px]:[&_button]:pb-1.5",
    "max-[760.001px]:[&_button]:pl-1.5 max-[460.001px]:min-w-0 max-[460.001px]:[&_button]:max-w-full max-[460.001px]:[&_button>span:first-child]:overflow-hidden max-[460.001px]:[&_button>span:first-child]:text-ellipsis max-[460.001px]:[&_button>span:first-child]:whitespace-nowrap"
  ].join(" "),
  srOnly: [
    "app-sr-only absolute w-[1px] h-[1px] pt-0 pr-0 pb-0 pl-0",
    "overflow-hidden [clip-path:inset(50%)] whitespace-nowrap"
  ].join(" "),
  grow: [
    "app-grow flex-1"
  ].join(" "),
  notice: [
    "app-notice text-alert-ink bg-none bg-red-wash pt-3 pr-3 pb-3 pl-3",
    "rounded-md"
  ].join(" "),
  side: [
    "app-side absolute z-[60] top-14 left-40 w-[328px] max-h-[calc(var(--viewport-height)_-_96px)] overflow-auto",
    "bg-none bg-app-side-background-color border border-solid border-line rounded-card shadow-app-side pt-4.5",
    "pr-4.5 pb-4.5 pl-4.5 max-[760.001px]:left-1 max-[760.001px]:top-22 max-[760.001px]:max-w-[calc(100%_-_8px)] max-[460.001px]:left-0 max-[460.001px]:right-0",
    "max-[460.001px]:top-20.5 max-[460.001px]:w-auto"
  ].join(" "),
  main: [
    "app-main relative min-w-0 rounded-scene overflow-hidden"
  ].join(" "),
  footer: [
    "app-footer text-xs text-app-footer-color flex justify-between gap-3 max-[760.001px]:text-xs max-[460.001px]:[&_span:last-child]:hidden"
  ].join(" "),
  app: [
    "app-app min-h-svh pt-5 pr-6 pb-4.5 pl-6 max-[760.001px]:pt-3 max-[760.001px]:pr-3",
    "max-[760.001px]:pb-3 max-[760.001px]:pl-3"
  ].join(" "),
  splash: [
    "app-splash grid place-items-center min-h-[80dvh]"
  ].join(" "),
} as const;

const ROBOT_POLL_MS = 5_000;

export interface AppProps {
  api: ControlApi;
  link: RobotLink;
  /** The data comes from the demo backend. The label is shown on every screen. */
  demo?: boolean;
}

type Auth = "checking" | "out" | "in";

function DemoLabel(): ReactElement {
  return (
    <div className={ui.demo} role="note" aria-label="演示数据标记">
      <span className={ui.demoTag}>演示数据</span>
      <span>非真实设备，所有状态均为模拟，不会驱动底盘</span>
    </div>
  );
}

interface ShellProps {
  list: RobotListState;
  link: RobotLink;
  onRefresh: () => void;
  onLinkLost: () => void;
  onLogout: () => void;
  loggingOut: boolean;
  notice: string | null;
  pairing: PairingPanelProps | null;
  onRevoke: ((robotId: string) => Promise<void>) | null;
}

function Shell({
  list,
  link,
  onRefresh,
  onLinkLost,
  onLogout,
  loggingOut,
  notice,
  pairing,
  onRevoke,
}: ShellProps): ReactElement {
  const { robotId } = useParams();
  const navigate = useNavigate();
  const currentRobotId = useRef(robotId);
  currentRobotId.current = robotId;
  const [openDeviceList, setOpenDeviceList] = useState<string | null>(null);
  const devicesOpen = openDeviceList === (robotId ?? "empty");
  const workspace = useWorkspaceView();
  const [pairOpen, setPairOpen] = useState(false);
  const menu = useRef<HTMLElement>(null);
  const scene = useRef<HTMLElement>(null);
  const trigger = useRef<HTMLDivElement>(null);
  const closeDevices = useCallback(() => setOpenDeviceList(null), []);
  useEffect(() => {
    if (!devicesOpen) return;
    const outside = (event: PointerEvent) => {
      if (
        !menu.current?.contains(event.target as Node) &&
        !trigger.current?.contains(event.target as Node)
      )
        closeDevices();
    };
    const escape = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        closeDevices();
        trigger.current?.querySelector("button")?.focus();
      }
    };
    menu.current?.querySelector<HTMLElement>("a, button")?.focus();
    document.addEventListener("pointerdown", outside);
    document.addEventListener("keydown", escape);
    return () => {
      document.removeEventListener("pointerdown", outside);
      document.removeEventListener("keydown", escape);
    };
  }, [devicesOpen, closeDevices]);
  const selected: RobotSummary | undefined = list.robots.find(
    (robot) => robot.id === robotId,
  );

  let main: ReactElement;
  if (robotId === undefined) {
    main = (
      <section className={ui.placeholder}>
        <div className={ui.emptySymbol}>
          <img src={vehicleMarker} width="46" height="54" alt="" />
        </div>
        <span className={ui.emptyEnglish}>WHEEL / READY WHEN YOU ARE</span>
        <h2>还没有选择车辆</h2>
        <p>
          选择已配对的车辆进入控制，
          <br />
          或生成配对码，添加一辆新车。
        </p>
        <div className={ui.emptyActions}>
          <Button variant="primary" onClick={() => setOpenDeviceList("empty")}>
            选择车辆
          </Button>
          {pairing && (
            <Button onClick={() => setPairOpen(true)}>配对新车</Button>
          )}
        </div>
        <div className={ui.emptyFootnote}>
          {list.failed
            ? "设备列表暂未更新"
            : `已有 ${list.robots.length} 辆已配对车辆`}
        </div>
      </section>
    );
  } else if (selected !== undefined) {
    // The key remounts the panel, so nothing of the previous robot survives a switch.
    main = (
      <RobotPanel
        key={selected.id}
        robot={selected}
        link={link}
        onLinkLost={onLinkLost}
        onLogout={onLogout}
        loggingOut={loggingOut}
        page={workspace.page}
        onPageChange={workspace.setPage}
        statusOpen={workspace.statusOpen}
        onStatusClose={() => workspace.setStatusOpen(false)}
      />
    );
  } else if (list.status === "loading") {
    main = (
      <section className={ui.placeholder} role="status">
        <p>正在读取设备列表…</p>
      </section>
    );
  } else {
    main = (
      <section className={ui.placeholder}>
        <h2>没有这台设备</h2>
        <p>设备编号 {robotId} 不在当前账户的设备列表中，可能已被撤销。</p>
      </section>
    );
  }

  if (!selected && workspace.page === "settings") {
    main = (
      <section className="relative h-[var(--viewport-height)] min-h-[var(--viewport-min)]">
        <VehicleSettings
          enabled={false}
          onBack={() => workspace.setPage("control")}
          onLogout={onLogout}
          loggingOut={loggingOut}
        />
      </section>
    );
  }

  return (
    <SpeedUnitContext.Provider value={workspace.unit}>
      <div className={ui.shell} data-workspace={selected !== undefined} data-page={workspace.page}>
        <header className={`${ui.header} ${!selected ? "max-[460.001px]:grid-rows-[34.59375px_30px_28px]" : ""}`}>
          <div className={ui.brand}>
            <svg viewBox="0 0 36 36" fill="none" aria-hidden="true">
              <circle cx="18" cy="18" r="15" stroke="#eeeade" strokeWidth="2" />
              <circle
                cx="18"
                cy="18"
                r="8"
                stroke="#ff7532"
                strokeWidth="3"
                strokeDasharray="9 5"
              />
              <circle cx="18" cy="18" r="2" fill="#eeeade" />
            </svg>
            <span className={ui.brandName}>WHEEL</span>
          </div>
          <div ref={trigger} className={`${ui.deviceTrigger} ${!selected ? "max-[460.001px]:row-start-3" : ""}`}>
            <Button
              aria-label={selected ? "切换车辆" : "选择车辆"}
              aria-expanded={devicesOpen}
              aria-controls="device-switcher"
              onClick={() =>
                setOpenDeviceList(devicesOpen ? null : (robotId ?? "empty"))
              }
            >
              <span>{selected?.name ?? "选择车辆"}</span>
              <ChevronDown size={16} />
              <span className={ui.srOnly}>
                {selected ? "切换车辆" : "选择设备"}
              </span>
            </Button>
          </div>
          {selected && (
            <SegmentedControl
              label="页面视图"
              value={workspace.page}
              disabled={!selected}
              onChange={workspace.setPage}
              options={[
                { value: "control", label: "控制" },
                { value: "logs", label: "日志" },
                { value: "settings", label: "设置" },
              ]}
            />
          )}
          {!selected && <Button onClick={() => workspace.setPage("settings")}>设置</Button>}
          <div className={ui.grow} />
          {selected && (
            <Button
              aria-label="切换速度单位"
              disabled={!selected}
              onClick={() =>
                workspace.setUnit(workspace.unit === "m/s" ? "km/h" : "m/s")
              }
            >
              <span>
                {workspace.unit}
                <small>
                  <ArrowLeftRight size={10} />
                  {workspace.unit === "m/s" ? "km/h" : "m/s"}
                </small>
              </span>
            </Button>
          )}
          {selected && (
            <Button
              disabled={!selected || workspace.page !== "control"}
              aria-expanded={workspace.statusOpen}
              onClick={() => workspace.setStatusOpen(!workspace.statusOpen)}
            >
              <CircleDot size={12} />
              状态
            </Button>
          )}
          {selected && <EmergencyStop key={selected.id} link={link} compact />}
        </header>
        {notice !== null && (
          <p className={ui.notice} role="alert">
            {notice}
          </p>
        )}
        {devicesOpen && (
          <aside ref={menu} id="device-switcher" className={ui.side}>
            <DeviceList
              list={list}
              selectedId={robotId ?? null}
              onRefresh={onRefresh}
              pairing={null}
              onPair={
                pairing
                  ? () => {
                      closeDevices();
                      setPairOpen(true);
                    }
                  : undefined
              }
              onClearSelection={
                selected
                  ? () => {
                      closeDevices();
                      navigate("/robots");
                    }
                  : undefined
              }
              onSelect={closeDevices}
              onClose={() => {
                closeDevices();
                trigger.current?.querySelector("button")?.focus();
              }}
              onRevoke={
                onRevoke === null
                  ? null
                  : async (id) => {
                      await onRevoke(id);
                      if (currentRobotId.current === id)
                        navigate("/robots", { replace: true });
                    }
              }
            />
          </aside>
        )}
        {pairing && pairOpen && (
          <PairingPanel
            {...pairing}
            initiallyOpen
            overlayTarget={scene.current}
            onClose={() => {
              setPairOpen(false);
              trigger.current?.querySelector("button")?.focus();
            }}
          />
        )}
        <main ref={scene} className={ui.main}>
          {main}
        </main>
        <footer className={ui.footer}>
          <span>WHEEL · 车辆控制工作区</span>
          <span className="max-[460.001px]:hidden">地图连线表示任务顺序，不代表实际规划路径</span>{" "}
        </footer>
      </div>
    </SpeedUnitContext.Provider>
  );
}

export function App({ api, link, demo = false }: AppProps): ReactElement {
  const [auth, setAuth] = useState<Auth>("checking");
  const [list, setList] = useState<RobotListState>({
    status: "loading",
    robots: [],
    failed: false,
  });
  const [loggingOut, setLoggingOut] = useState(false);
  const [notice, setNotice] = useState<string | null>(null);
  const latestLoad = useRef(0);
  const logoutActive = useRef(false);
  const location = useLocation();

  /** Every call that can learn the session is gone goes through here: a 401 means login. */
  const refresh = useCallback(async (): Promise<void> => {
    if (logoutActive.current) return;
    const ticket = ++latestLoad.current;
    try {
      const robots = await api.listRobots();
      if (ticket !== latestLoad.current) return;
      setList({ status: "ready", robots, failed: false });
      setAuth("in");
    } catch (error) {
      if (ticket !== latestLoad.current) return;
      if (error instanceof UnauthorizedError) {
        setList({ status: "loading", robots: [], failed: false });
        setAuth("out");
        return;
      }
      setList((previous) => ({ ...previous, status: "ready", failed: true }));
      setAuth((current) => (current === "checking" ? "in" : current));
    }
  }, [api]);

  useEffect(() => {
    void refresh();
  }, [refresh]);

  useEffect(() => {
    if (auth !== "in") return;
    const timer = setInterval(() => void refresh(), ROBOT_POLL_MS);
    return () => clearInterval(timer);
  }, [auth, refresh]);

  const expireSession = useCallback(() => {
    latestLoad.current += 1;
    setList({ status: "loading", robots: [], failed: false });
    setAuth("out");
  }, []);

  const logout = useCallback(async () => {
    if (logoutActive.current) return;
    logoutActive.current = true;
    latestLoad.current += 1;
    setLoggingOut(true);
    try {
      await api.logout();
    } catch (error) {
      // A 401 means the session was already gone, which is what the operator asked for.
      if (!(error instanceof UnauthorizedError)) {
        setNotice("退出登录失败，会话仍然有效。请重试。");
        return;
      }
    } finally {
      logoutActive.current = false;
      setLoggingOut(false);
    }
    setNotice(null);
    expireSession();
  }, [api, expireSession]);

  const revokeRobot = useCallback(
    async (robotId: string): Promise<void> => {
      try {
        await api.revokeRobot(robotId);
      } catch (error) {
        if (error instanceof UnauthorizedError) expireSession();
        throw error;
      }
      // A list request started before deletion must not restore the removed device.
      latestLoad.current += 1;
      setList((previous) => ({
        ...previous,
        robots: previous.robots.filter((robot) => robot.id !== robotId),
      }));
    },
    [api, expireSession],
  );

  const from = (location.state as { from?: unknown } | null)?.from;
  const returnTo =
    typeof from === "string" && from.startsWith("/robots") ? from : "/robots";

  const guarded = (element: ReactElement): ReactElement =>
    auth === "in" ? (
      element
    ) : (
      <Navigate to="/login" replace state={{ from: location.pathname }} />
    );

  const shell = (
    <Shell
      list={list}
      link={link}
      onRefresh={() => void refresh()}
      onLinkLost={() => void refresh()}
      onLogout={() => void logout()}
      loggingOut={loggingOut}
      notice={notice}
      onRevoke={demo || loggingOut ? null : revokeRobot}
      pairing={
        demo || loggingOut
          ? null
          : {
              createCode: (signal) => api.createPairingCode(signal),
              onReauthenticate: expireSession,
            }
      }
    />
  );

  return (
    <div className={ui.app}>
      {demo && <DemoLabel />}
      {auth === "checking" ? (
        <div className={ui.splash} role="status">
          正在校验会话…
        </div>
      ) : (
        <Routes>
          <Route
            path="/login"
            element={
              auth === "in" ? (
                <Navigate to={returnTo} replace />
              ) : (
                <Login api={api} onLoggedIn={refresh} />
              )
            }
          />
          <Route path="/robots" element={guarded(shell)} />
          <Route path="/robots/:robotId" element={guarded(shell)} />
          <Route path="*" element={<Navigate to="/robots" replace />} />
        </Routes>
      )}
    </div>
  );
}
