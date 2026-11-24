import { useCallback, useEffect, useRef, useState, type ReactElement } from "react";
import { Navigate, Route, Routes, useLocation, useNavigate, useParams } from "react-router-dom";

import { CircleDot, ChevronDown, Activity, ArrowLeftRight } from "lucide-react";
import { Button } from "./components/ui/Button";
import { SegmentedControl } from "./components/ui/SegmentedControl";
import { EmergencyStop } from "./features/drive/EmergencyStop";
import { useWorkspaceView } from "./features/devices/useWorkspaceView";
import { SpeedUnitContext } from "./display/speedUnits";
import styles from "./App.module.css";
import { UnauthorizedError, type ControlApi, type RobotLink } from "./api/client";
import type { RobotSummary } from "./api/protocol";
import { DeviceList, type RobotListState } from "./features/devices/DeviceList";
import type { PairingPanelProps } from "./features/devices/PairingPanel";
import { Login } from "./features/devices/Login";
import { RobotPanel } from "./features/devices/RobotPanel";

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
    <div className={styles.demo} role="note" aria-label="演示数据标记">
      <span className={styles.demoTag}>演示数据</span>
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

function Shell({ list, link, onRefresh, onLinkLost, onLogout, loggingOut, notice, pairing, onRevoke }: ShellProps): ReactElement {
  const { robotId } = useParams();
  const navigate = useNavigate();
  const currentRobotId = useRef(robotId);
  currentRobotId.current = robotId;
  const [openDeviceList, setOpenDeviceList] = useState<string | null>(null);
  const devicesOpen = openDeviceList === (robotId ?? "empty");
  const workspace = useWorkspaceView();
  const selected: RobotSummary | undefined = list.robots.find((robot) => robot.id === robotId);

  let main: ReactElement;
  if (robotId === undefined) {
    main = (
      <section className={styles.placeholder}>
        <CircleDot size={52} aria-hidden="true" /><h2>选择一台设备</h2>
        <p>选择已配对车辆或添加设备，建立真实控制通道。</p>
      </section>
    );
  } else if (selected !== undefined) {
    // The key remounts the panel, so nothing of the previous robot survives a switch.
    main = <RobotPanel key={selected.id} robot={selected} link={link} onLinkLost={onLinkLost} page={workspace.page} onPageChange={workspace.setPage} statusOpen={workspace.statusOpen} />;
  } else if (list.status === "loading") {
    main = (
      <section className={styles.placeholder} role="status">
        <p>正在读取设备列表…</p>
      </section>
    );
  } else {
    main = (
      <section className={styles.placeholder}>
        <h2>没有这台设备</h2>
        <p>设备编号 {robotId} 不在当前账户的设备列表中，可能已被撤销。</p>
      </section>
    );
  }

  return (
    <SpeedUnitContext.Provider value={workspace.unit}><div className={styles.shell} data-workspace={selected !== undefined}>
      <header className={styles.header}>
        <div className={styles.brand}><CircleDot aria-hidden="true" /><span className={styles.brandName}>WHEEL</span></div>
        <Button aria-label={selected ? "切换车辆" : "选择车辆"} aria-expanded={devicesOpen} aria-controls="device-switcher" onClick={() => setOpenDeviceList(devicesOpen ? null : robotId ?? 'empty')}>
          <span>{selected?.name ?? '选择车辆'}</span><ChevronDown size={16} /><span className={styles.srOnly}>{selected ? '切换车辆' : '选择设备'}</span>
        </Button>
        <SegmentedControl label="页面视图" value={workspace.page} disabled={!selected} onChange={workspace.setPage} options={[{value:'control',label:'控制'},{value:'logs',label:'日志'},{value:'settings',label:'设置'}]} />
        <div className={styles.grow} />
        <Button aria-label="切换速度单位" disabled={!selected} onClick={() => workspace.setUnit(workspace.unit === 'm/s' ? 'km/h' : 'm/s')}>{workspace.unit}<ArrowLeftRight size={16} /></Button>
        <Button disabled={!selected} aria-expanded={workspace.statusOpen} onClick={() => workspace.setStatusOpen(!workspace.statusOpen)}><Activity size={16} />状态</Button>
        {selected && <EmergencyStop key={selected.id} link={link} compact />}
        <button type="button" className={styles.logout} disabled={loggingOut} onClick={onLogout}>
          退出登录
        </button>
      </header>
      {notice !== null && (
        <p className={styles.notice} role="alert">
          {notice}
        </p>
      )}
      <aside id="device-switcher" className={styles.side} data-open={devicesOpen || selected === undefined}>
        <DeviceList list={list} selectedId={robotId ?? null} onRefresh={onRefresh} pairing={pairing}
          onRevoke={onRevoke === null ? null : async (id) => {
            await onRevoke(id);
            if (currentRobotId.current === id) navigate("/robots", { replace: true });
          }} />
      </aside>
      <main className={styles.main}>{main}</main>
    </div></SpeedUnitContext.Provider>
  );
}

export function App({ api, link, demo = false }: AppProps): ReactElement {
  const [auth, setAuth] = useState<Auth>("checking");
  const [list, setList] = useState<RobotListState>({ status: "loading", robots: [], failed: false });
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

  const revokeRobot = useCallback(async (robotId: string): Promise<void> => {
    try {
      await api.revokeRobot(robotId);
    } catch (error) {
      if (error instanceof UnauthorizedError) expireSession();
      throw error;
    }
    // A list request started before deletion must not restore the removed device.
    latestLoad.current += 1;
    setList((previous) => ({ ...previous, robots: previous.robots.filter((robot) => robot.id !== robotId) }));
  }, [api, expireSession]);

  const from = (location.state as { from?: unknown } | null)?.from;
  const returnTo = typeof from === "string" && from.startsWith("/robots") ? from : "/robots";

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
      pairing={demo || loggingOut ? null : {
        createCode: (signal) => api.createPairingCode(signal),
        onReauthenticate: expireSession,
      }}
    />
  );

  return (
    <div className={styles.app}>
      {demo && <DemoLabel />}
      {auth === "checking" ? (
        <div className={styles.splash} role="status">
          <span className={styles.splashBar} aria-hidden="true" />
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
