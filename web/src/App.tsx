import { useCallback, useEffect, useRef, useState, type ReactElement } from "react";
import { Navigate, Route, Routes, useLocation, useParams } from "react-router-dom";

import styles from "./App.module.css";
import { UnauthorizedError, type ControlApi, type RobotLink } from "./api/client";
import type { RobotSummary } from "./api/protocol";
import { DeviceList, type RobotListState } from "./features/devices/DeviceList";
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
  notice: string | null;
}

function Shell({ list, link, onRefresh, onLinkLost, onLogout, notice }: ShellProps): ReactElement {
  const { robotId } = useParams();
  const selected: RobotSummary | undefined = list.robots.find((robot) => robot.id === robotId);

  let main: ReactElement;
  if (robotId === undefined) {
    main = (
      <section className={styles.placeholder}>
        <h2>选择一台设备</h2>
        <p>从左侧设备列表选择车辆，查看状态并建立控制通道。</p>
      </section>
    );
  } else if (selected !== undefined) {
    // The key remounts the panel, so nothing of the previous robot survives a switch.
    main = <RobotPanel key={selected.id} robot={selected} link={link} onLinkLost={onLinkLost} />;
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
    <div className={styles.shell}>
      <header className={styles.header}>
        <div className={styles.brand}>
          <span className={styles.mark} aria-hidden="true" />
          <span className={styles.brandName}>车辆控制台</span>
          <span className={styles.brandTag}>SENIOR · CTRL</span>
        </div>
        <button type="button" className={styles.logout} onClick={onLogout}>
          退出登录
        </button>
      </header>
      {notice !== null && (
        <p className={styles.notice} role="alert">
          {notice}
        </p>
      )}
      <aside className={styles.side}>
        <DeviceList list={list} selectedId={robotId ?? null} onRefresh={onRefresh} />
      </aside>
      <main className={styles.main}>{main}</main>
    </div>
  );
}

export function App({ api, link, demo = false }: AppProps): ReactElement {
  const [auth, setAuth] = useState<Auth>("checking");
  const [list, setList] = useState<RobotListState>({ status: "loading", robots: [], failed: false });
  const [notice, setNotice] = useState<string | null>(null);
  const latestLoad = useRef(0);
  const location = useLocation();

  /** Every call that can learn the session is gone goes through here: a 401 means login. */
  const refresh = useCallback(async (): Promise<void> => {
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

  const logout = useCallback(async () => {
    try {
      await api.logout();
    } catch (error) {
      // A 401 means the session was already gone, which is what the operator asked for.
      if (!(error instanceof UnauthorizedError)) {
        setNotice("退出登录失败，会话仍然有效。请重试。");
        return;
      }
    }
    setNotice(null);
    latestLoad.current += 1;
    setList({ status: "loading", robots: [], failed: false });
    setAuth("out");
  }, [api]);

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
      notice={notice}
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
