import { useRef, useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import styles from "./Drive.module.css";

export interface EmergencyStopProps {
  link: RobotLink;
}

interface StopState {
  /** The robot has answered ok to at least one emergency stop. */
  acknowledged: boolean;
  /** Presses that have no answer yet. */
  inFlight: number;
  /** Why the latest press failed. Only shown while nothing has been acknowledged. */
  failure: string | null;
}

/**
 * The one emergency stop. It stays pressable while a request is in flight: a second press is
 * the operator's retry. "Confirmed" appears only after the robot answered ok; a timeout, an
 * error or silence is reported as not confirmed so the operator reaches for the physical stop.
 */
export function EmergencyStop({ link }: EmergencyStopProps): ReactElement {
  const [state, setState] = useState<StopState>({ acknowledged: false, inFlight: 0, failure: null });
  const latest = useRef(0);

  async function press(): Promise<void> {
    const mine = ++latest.current;
    setState((current) => ({ ...current, inFlight: current.inFlight + 1, failure: null }));
    try {
      await link.request("safety.estop", {});
      setState((current) => ({
        acknowledged: true,
        inFlight: current.inFlight - 1,
        failure: null,
      }));
    } catch (error) {
      const reason =
        error instanceof CommandError ? describeCommandError(error.code) : "无法发送指令";
      setState((current) => ({
        ...current,
        inFlight: current.inFlight - 1,
        // An older press that fails after a newer one was sent must not speak for it.
        failure: mine === latest.current ? reason : current.failure,
      }));
    }
  }

  const { acknowledged, inFlight, failure } = state;
  return (
    <div className={styles.estopDock}>
      {!acknowledged && failure !== null && (
        <p className={styles.estopFailure} role="alert">
          急停未确认：{failure}。请立即使用车上的物理急停。
        </p>
      )}
      {acknowledged && (
        <p className={styles.estopConfirmed} role="status">
          急停已确认：车端已应答
        </p>
      )}
      <button type="button" className={styles.estop} aria-label="急停" onClick={() => void press()}>
        <span aria-hidden="true" className={styles.estopCap}>
          急停
        </span>
        <span aria-hidden="true" className={styles.estopTag}>
          {inFlight > 0 ? "发送中" : "E-STOP"}
        </span>
      </button>
    </div>
  );
}
