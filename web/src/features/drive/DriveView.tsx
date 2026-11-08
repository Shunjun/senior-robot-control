import { useEffect, useLayoutEffect, useRef, useState, type ReactElement } from "react";

import { CommandError, MotionNotReadyError, describeCommandError, type DriveFeedback, type RobotLink } from "../../api/client";
import type { ErrorCode, RobotSnapshot } from "../../api/protocol";
import styles from "./Drive.module.css";
import { Joystick, type ReleaseReason } from "./Joystick";

// ---------------------------------------------------------------- limits

/** Full stick deflection. The vehicle's own safety layer still clamps and limits. */
export const MAX_LINEAR = 0.5;
export const MAX_ANGULAR = 1.0;
/** Share of full speed in limited, localization-assisted driving. */
export const ASSIST_SCALE = 0.3;

/** A held command is resent this often. The vehicle stops by itself if they stop arriving. */
const RESEND_MS = 100;
/** Unanswered velocity requests allowed at once, so a slow link is not flooded. */
const MAX_IN_FLIGHT = 2;

// ---------------------------------------------------------------- controller

export interface DriveInput {
  /** Forward speed, m/s. */
  vx: number;
  /** Turn rate, rad/s, positive counter-clockwise. */
  wz: number;
}

export type StopReason =
  | ReleaseReason
  | "unmount"
  | "disabled"
  | "rejected"
  | "invalid"
  | "release_lease";

export type DriveEvent =
  | { type: "stopped"; reason: StopReason }
  | { type: "command_failed"; code: ErrorCode | null; message: string }
  | { type: "stop_unconfirmed"; reason: StopReason; message: string };

export interface DriveControllerOptions {
  intervalMs?: number;
  onEvent?: (event: DriveEvent) => void;
}

function describeFailure(error: unknown): { code: ErrorCode | null; message: string } {
  if (error instanceof CommandError) {
    return { code: error.code, message: describeCommandError(error.code) };
  }
  return { code: null, message: "无法发送指令" };
}

function limit(value: number, max: number): number {
  // `|| 0` turns -0 into 0 so the wire never carries a negative zero.
  return Math.max(-max, Math.min(max, value)) || 0;
}

/**
 * Hold-to-drive. `start` keeps sending the current input on a short interval, so every command
 * is short-lived; `stop` ends it with `drive.stop`. Nothing here takes a lease: the operator
 * does that explicitly, and a rejected command only stops.
 */
export class DriveController {
  private readonly intervalMs: number;
  private readonly onEvent: (event: DriveEvent) => void;
  private input: DriveInput | null = null;
  private timer: ReturnType<typeof setInterval> | null = null;
  private inFlight = 0;
  /** Bumped by every stop, so answers to an older hold cannot stop or fail a newer one. */
  private generation = 0;

  constructor(
    private readonly link: RobotLink,
    options: DriveControllerOptions = {},
  ) {
    this.intervalMs = options.intervalMs ?? RESEND_MS;
    this.onEvent = options.onEvent ?? (() => undefined);
  }

  get holding(): boolean {
    return this.input !== null;
  }

  start(input: DriveInput): void {
    if (!Number.isFinite(input.vx) || !Number.isFinite(input.wz)) {
      this.stop("invalid");
      return;
    }
    const next = { vx: limit(input.vx, MAX_LINEAR), wz: limit(input.wz, MAX_ANGULAR) };
    const previous = this.input;
    this.input = next;
    if (this.timer === null) {
      this.timer = setInterval(() => this.tick(), this.intervalMs);
      this.send();
    } else if (!this.link.timedMotion &&
        (previous === null || previous.vx !== next.vx || previous.wz !== next.wz)) {
      //Compatible: older links send a changed input immediately.
      this.send();
    }
  }

  stop(reason: StopReason): void {
    if (this.input === null) return;
    this.input = null;
    this.generation += 1;
    if (this.timer !== null) clearInterval(this.timer);
    this.timer = null;
    this.link.request("drive.stop", {}).then(
      () => this.onEvent({ type: "stopped", reason }),
      (error: unknown) =>
        this.onEvent({ type: "stop_unconfirmed", reason, message: describeFailure(error).message }),
    );
  }

  private tick(): void {
    if (this.link.timedMotion || this.inFlight < MAX_IN_FLIGHT) this.send();
  }

  private send(): void {
    const input = this.input;
    if (input === null) return;
    const generation = this.generation;
    this.inFlight += 1;
    this.link.request("drive.velocity", { vx: input.vx, wz: input.wz }).then(
      () => {
        this.inFlight -= 1;
      },
      (error: unknown) => {
        this.inFlight -= 1;
        if (generation !== this.generation || error instanceof MotionNotReadyError) return;
        this.onEvent({ type: "command_failed", ...describeFailure(error) });
        this.stop("rejected");
      },
    );
  }
}

// ---------------------------------------------------------------- view

export interface DriveViewProps {
  robotId: string;
  link: RobotLink;
  /** The robot's snapshot. Null until the first one arrives. */
  snapshot: RobotSnapshot | null;
  active?: boolean;
  compact?: boolean;
}

type Lease = "none" | "pending" | "held" | "releasing";

interface Held {
  linear: number | null;
  angular: number | null;
}

const NOT_HELD: Held = { linear: null, angular: null };
const STILL: DriveInput = { vx: 0, wz: 0 };

function lockReason(snapshot: RobotSnapshot | null): string | null {
  if (snapshot === null) return "等待车端快照，收到之前不能遥控。";
  if (snapshot.stale) return "缓存快照 · 非实时，遥控已锁定。";
  if (!snapshot.connected) return "车端未连接，遥控已锁定。";
  return null;
}

function speed(value: number): string {
  return value.toFixed(2);
}

export function DriveView({ robotId, link, snapshot, active = true, compact = false }: DriveViewProps): ReactElement {
  const [lease, setLease] = useState<Lease>("none");
  const leaseRef = useRef<Lease>("none");
  function setAuthority(value: Lease): void {
    leaseRef.current = value;
    setLease(value);
  }
  const [stopFeedback, setStopFeedback] = useState<string | null>(null);
  const [released, setReleased] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [assist, setAssist] = useState(false);
  const [readout, setReadout] = useState<DriveInput>(STILL);
  const [feedback, setFeedback] = useState<DriveFeedback | null>(null);
  const [lastOutput, setLastOutput] = useState<Extract<DriveFeedback, { status: "applied" }> | null>(null);
  const [feedbackStale, setFeedbackStale] = useState(false);
  const [gestureReset, setGestureReset] = useState(0);
  const controller = useRef<DriveController | null>(null);
  const held = useRef<Held>(NOT_HELD);
  /** Bumped per robot, so a late answer for the previous robot cannot touch this one. */
  const session = useRef(0);

  const lock = lockReason(snapshot);
  const liveSnapshot = lock === null ? snapshot : null;
  const live = liveSnapshot !== null;
  const localized = liveSnapshot?.localized === true;
  const limited = live && !localized;
  const unlocked = live && (localized || assist);
  const supportsMotionChallenges = link.supportsMotionChallenges === true;
  const canDrive = active && unlocked && lease === "held" && supportsMotionChallenges;

  // Layout, not passive: the stop for the old robot must leave before the shell's own cleanup
  // disconnects the link, and a parent's passive cleanup runs before a child's.
  useLayoutEffect(() => {
    session.current += 1;
    held.current = NOT_HELD;
    setAuthority("none");
    setError(null);
    setStopFeedback(null);
    setReleased(false);
    setAssist(false);
    setReadout(STILL);
    const next = new DriveController(link, {
      onEvent: (event) => {
        if (controller.current !== next) return;
        if (event.type === "stopped" && !next.holding) {
          setStopFeedback("停车指令已确认。");
        } else if (event.type === "command_failed") {
          // The lease is not taken back. The operator acquires it again on purpose.
          setError(`车端没有接受遥控指令：${event.message}。`);
          setAuthority("none");
        } else if (event.type === "stop_unconfirmed" && !next.holding) {
          setStopFeedback(null);
          setError(`停止未确认：${event.message}。请立即按下急停。`);
        }
      },
    });
    controller.current = next;
    return () => {
      session.current += 1;
      next.stop("unmount");
      if (leaseRef.current === "held" || leaseRef.current === "pending") {
        void link.request("control.release", {}).catch(() => undefined);
      }
      controller.current = null;
    };
  }, [link, robotId]);

  useLayoutEffect(() => {
    return link.subscribeMotionRevoked?.((authorityLost) => {
      setGestureReset((previous) => previous + 1);
      held.current = NOT_HELD;
      controller.current?.stop("disabled");
      setReadout(STILL);
      if (authorityLost && leaseRef.current !== "releasing") {
        session.current += 1;
        setAuthority("none");
        setFeedback(null);
        setLastOutput(null);
      }
    });
  }, [link]);

  useLayoutEffect(() => {
    if (live) return;
    session.current += 1;
    setAuthority("none");
    held.current = NOT_HELD;
    controller.current?.stop("disabled");
    setReadout(STILL);
  }, [live]);

  useLayoutEffect(() => {
    if (!active) revoke.current();
  }, [active]);

  const revoke = useRef<() => void>(() => undefined);
  revoke.current = () => {
    if (leaseRef.current === "held" || leaseRef.current === "pending") void giveUp();
  };

  useEffect(() => {
    const onBlur = () => revoke.current();
    const onVisibility = () => {
      if (document.visibilityState === "hidden") revoke.current();
    };
    window.addEventListener("blur", onBlur);
    window.addEventListener("pagehide", onBlur);
    document.addEventListener("visibilitychange", onVisibility);
    return () => {
      window.removeEventListener("blur", onBlur);
      window.removeEventListener("pagehide", onBlur);
      document.removeEventListener("visibilitychange", onVisibility);
    };
  }, []);

  useEffect(() => {
    setFeedback(null);
    setLastOutput(null);
    setFeedbackStale(false);
    return link.subscribeDriveFeedback?.((result) => {
      setFeedback(result);
      if (result.status === "applied") {
        setLastOutput(result);
        setFeedbackStale(false);
      }
    });
  }, [link, robotId, live]);

  useEffect(() => {
    if (lastOutput === null) return;
    const staleTimer = setTimeout(() => setFeedbackStale(true), 2_000);
    return () => clearTimeout(staleTimer);
  }, [lastOutput]);

  useEffect(() => {
    // Localization came back: the limited mode ends and cannot be resumed by accident later.
    if (localized) setAssist(false);
  }, [localized]);

  useEffect(() => {
    if (canDrive) return;
    held.current = NOT_HELD;
    controller.current?.stop("disabled");
    setReadout(STILL);
  }, [canDrive]);

  function drive(): void {
    const { linear, angular } = held.current;
    if (linear === null && angular === null) return;
    const scale = limited ? ASSIST_SCALE : 1;
    const input = {
      vx: (linear ?? 0) * MAX_LINEAR * scale || 0,
      wz: (angular ?? 0) * MAX_ANGULAR * scale || 0,
    };
    setReadout(input);
    controller.current?.start(input);
  }

  function hold(axis: keyof Held, value: number): void {
    if (!canDrive) return;
    if (held.current.linear === null && held.current.angular === null) {
      setError(null);
      setStopFeedback(null);
    }
    held.current = { ...held.current, [axis]: value };
    drive();
  }

  function release(axis: keyof Held, reason: ReleaseReason): void {
    if (reason !== "release") {
      held.current = NOT_HELD;
      setGestureReset((previous) => previous + 1);
      controller.current?.stop(reason);
      setReadout(STILL);
      return;
    }
    held.current = { ...held.current, [axis]: null };
    if (held.current.linear === null && held.current.angular === null) {
      controller.current?.stop(reason);
      setReadout(STILL);
    } else {
      drive();
    }
  }

  async function acquire(): Promise<void> {
    const mine = session.current;
    setAuthority("pending");
    setError(null);
    setReleased(false);
    try {
      await link.request("control.acquire", {});
      if (mine === session.current) setAuthority("held");
    } catch (failure) {
      if (mine !== session.current) return;
      setAuthority("none");
      setError(`没有拿到控制权：${describeFailure(failure).message}。`);
    }
  }

  async function giveUp(): Promise<void> {
    const mine = ++session.current;
    if (controller.current?.holding) {
      setStopFeedback("停车指令等待确认。");
      controller.current.stop("release_lease");
    }
    held.current = NOT_HELD;
    setGestureReset((previous) => previous + 1);
    setAuthority("releasing");
    try {
      await link.request("control.release", {});
      if (mine === session.current) setReleased(true);
    } catch (failure) {
      if (mine === session.current) {
        setError(`释放控制权未确认：${describeFailure(failure).message}。`);
      }
    } finally {
      if (mine === session.current) setAuthority("none");
    }
  }

  function park(): void {
    held.current = NOT_HELD;
    setGestureReset((previous) => previous + 1);
    setReadout(STILL);
    setStopFeedback("停车指令等待确认。");
    if (controller.current?.holding) {
      controller.current.stop("release");
    } else {
      const mine = session.current;
      void link.request("drive.stop", {}).then(
        () => {
          if (mine === session.current && !controller.current?.holding) setStopFeedback("停车指令已确认。");
        },
        (failure: unknown) => {
          if (mine !== session.current || controller.current?.holding) return;
          setStopFeedback(null);
          setError(`停止未确认：${describeFailure(failure).message}。`);
        },
      );
    }
  }

  const authorityFeedback = [
    stopFeedback,
    lease === "releasing" ? "控制权释放等待确认。" : released ? "控制权已释放。" : null,
  ].filter(Boolean).join(" ");
  const status = (
    <>
      {authorityFeedback !== "" && <p className={styles.quiet} role="status">{authorityFeedback}</p>}
      {error !== null && <p className={styles.failure} role="alert">{error}</p>}
    </>
  );
  if (!active) return status;

  return (
    <section className={styles.drive} aria-label="遥控" data-compact={compact} data-locked={!unlocked}>
      <header className={styles.driveHead}>
        <div className={styles.telemetry}>
          <p className={styles.eyebrow}>遥控 / {robotId}</p>
          <p className={styles.readout}>
            请求 vx {speed(readout.vx)} m/s · wz {speed(readout.wz)} rad/s
          </p>
          <p className={styles.readout}>
            控制反馈延迟 {lastOutput === null ? "无样本" : `${Math.round(lastOutput.latencyMs)} ms`}
            {lastOutput !== null && feedbackStale && " · 样本已过期"}
            {lastOutput !== null && (
              <> · 输出 {speed(lastOutput.vx)} m/s / {speed(lastOutput.wz)} rad/s{lastOutput.limited && " · 已限幅"}</>
            )}
            {feedback?.status === "blocked" && ` · 安全门拦截${feedback.reason ? `（${feedback.reason}）` : ""}`}
            {feedback?.status === "superseded" && " · 指令已替换"}
            {feedback?.status === "expired" && " · 指令超时"}
            {feedback?.status === "unconfirmed" && " · 未收到输出回执"}

          </p>
        </div>
        <div className={styles.leaseGroup}>
          <button type="button" className={styles.ghost} onClick={park}>停车</button>
          {lease === "held" ? (
            <>
              <span className={styles.leaseTag}>已持有控制权</span>
              <button type="button" className={styles.ghost} onClick={() => void giveUp()}>
                释放控制权
              </button>
            </>
          ) : (
            <button
              type="button"
              className={styles.primary}
              disabled={!live || lease === "pending" || lease === "releasing"}
              onClick={() => void acquire()}
            >
              获取控制权
            </button>
          )}
          {limited && (
            <button
              type="button"
              className={styles.ghost}
              aria-pressed={assist}
              onClick={() => setAssist(!assist)}
            >
              {assist ? "退出定位辅助遥控" : "进入定位辅助遥控"}
            </button>
          )}
        </div>
      </header>

      <div className={styles.pads} key={robotId}>
        <Joystick
          label="前进后退摇杆"
          resetToken={gestureReset}
          axis="vertical"
          hint="按住拖动，或用方向键 ↑ ↓ 前进 / 后退，松开即停"
          disabled={!canDrive}
          onChange={(value) => hold("linear", value)}
          onRelease={(reason) => release("linear", reason)}
        />
        <Joystick
          label="左转右转摇杆"
          resetToken={gestureReset}
          axis="horizontal"
          hint="按住拖动，或用方向键 ← → 左转 / 右转，松开即停"
          disabled={!canDrive}
          onChange={(value) => hold("angular", value)}
          onRelease={(reason) => release("angular", reason)}
        />
      </div>
      <div className={styles.driveNotes}>
        {lock !== null && <p className={styles.lock}>{lock}</p>}
        {live && !supportsMotionChallenges && (
          <p className={styles.lock}>遥控暂不可用：等待车端安全许可。</p>
        )}
        {live && !localized && !assist && (
          <p className={styles.lock}>定位未完成，遥控已锁定。进入定位辅助遥控后以低速行驶。</p>
        )}
        {limited && assist && (
          <p className={styles.assistNote}>
            定位辅助遥控已开启，限速 {Math.round(ASSIST_SCALE * 100)}%：只用于把车移到能重新定位的位置。
          </p>
        )}
        {unlocked && lease !== "held" && (
          <p className={styles.quiet}>先获取控制权，再按住摇杆。松手、切走页面或失去焦点都会停车。</p>
        )}
        {status}
      </div>
    </section>
  );
}
