import { useEffect, useId, useRef, useState, type ReactElement } from "react";

import { ApiError, RateLimitedError, UnauthorizedError, type PairingCode } from "../../api/client";
import styles from "./Devices.module.css";

export interface PairingPanelProps {
  createCode: (signal: AbortSignal) => Promise<PairingCode>;
  onReauthenticate: () => void;
}

interface IssuedCode {
  code: string;
  steadyDeadline: number;
  wallDeadline: number;
}

interface PairingFailure {
  message: string;
  needsLogin: boolean;
}

function pairingError(error: unknown): PairingFailure {
  if (error instanceof RateLimitedError) {
    return {
      message: error.retryAfterSeconds === null
        ? "配对码请求过多，请稍后重试。"
        : `配对码请求过多，请在 ${error.retryAfterSeconds} 秒后重试。`,
      needsLogin: false,
    };
  }
  if (error instanceof ApiError && error.status === 403) {
    return { message: "当前会话无法生成配对码，请重新登录。", needsLogin: true };
  }
  return { message: "无法生成配对码，请检查网络后重试。", needsLogin: false };
}

export function PairingPanel({ createCode, onReauthenticate }: PairingPanelProps): ReactElement {
  const panelId = useId();
  const headingId = useId();
  const request = useRef<AbortController | null>(null);
  const [open, setOpen] = useState(false);
  const [busy, setBusy] = useState(false);
  const [issued, setIssued] = useState<IssuedCode | null>(null);
  const [remaining, setRemaining] = useState(0);
  const [expired, setExpired] = useState(false);
  const [error, setError] = useState<PairingFailure | null>(null);

  useEffect(() => () => request.current?.abort(), []);

  useEffect(() => {
    if (issued === null) return;
    function updateRemaining(): void {
      if (issued === null) return;
      const milliseconds = Math.min(
        issued.steadyDeadline - performance.now(),
        issued.wallDeadline - Date.now(),
      );
      const seconds = Math.max(0, Math.ceil(milliseconds / 1000));
      setRemaining(seconds);
      if (seconds === 0) {
        setIssued(null);
        setExpired(true);
      }
    }
    const timer = setInterval(updateRemaining, 250);
    document.addEventListener("visibilitychange", updateRemaining);
    return () => {
      clearInterval(timer);
      document.removeEventListener("visibilitychange", updateRemaining);
    };
  }, [issued]);

  function toggle(): void {
    if (open) {
      request.current?.abort();
      request.current = null;
      setBusy(false);
      setIssued(null);
      setExpired(false);
      setError(null);
    }
    setOpen(!open);
  }

  async function generate(): Promise<void> {
    if (request.current !== null) return;
    const controller = new AbortController();
    request.current = controller;
    setBusy(true);
    setError(null);
    try {
      const result = await createCode(controller.signal);
      if (controller.signal.aborted) return;
      setRemaining(result.expires_in);
      const duration = result.expires_in * 1000;
      setIssued({
        code: result.code,
        steadyDeadline: performance.now() + duration,
        wallDeadline: Date.now() + duration,
      });
      setExpired(false);
    } catch (cause) {
      if (controller.signal.aborted) return;
      if (cause instanceof UnauthorizedError) onReauthenticate();
      else setError(pairingError(cause));
    } finally {
      if (request.current === controller && !controller.signal.aborted) {
        request.current = null;
        setBusy(false);
      }
    }
  }

  return (
    <div className={styles.pairing}>
      <button type="button" className={styles.ghost} aria-expanded={open} aria-controls={panelId} onClick={toggle}>
        {open ? "关闭配对" : "添加设备"}
      </button>
      {open && (
        <section id={panelId} className={styles.pairingPanel} aria-labelledby={headingId}>
          <h3 id={headingId} className={styles.pairingHeading}>设备配对</h3>
          {error !== null && <p role="alert" className={styles.failure}>{error.message}</p>}
          {expired && <p role="status" className={styles.quiet}>配对码已过期。</p>}
          {error?.needsLogin ? (
            <button type="button" className={styles.primary} onClick={onReauthenticate}>重新登录</button>
          ) : issued === null ? (
            <button type="button" className={styles.primary} disabled={busy} onClick={() => void generate()}>
              {busy ? "正在生成…" : expired ? "重新生成配对码" : "生成配对码"}
            </button>
          ) : (
            <>
              <div className={styles.pairingReceipt}>
                <code aria-label="配对码" className={styles.pairingCode}>{issued.code}</code>
                <span className={styles.quiet}>剩余 {remaining} 秒</span>
              </div>
              <p className={styles.quiet}>在车端可写的目标目录运行以下命令，并在提示符中输入配对码。</p>
              <code className={styles.pairingCommand}>
                {`python -m senior_control.agent.main --pair --cloud-url 'wss://${window.location.host}/ws/agent' --device-name '巡检车' --credentials-file "$PWD/device-credentials.json"`}
              </code>
              <p className={styles.quiet}>配对完成后可刷新列表。启动车端代理时使用同一凭据文件，连接后设备显示为在线。</p>
            </>
          )}
        </section>
      )}
    </div>
  );
}
