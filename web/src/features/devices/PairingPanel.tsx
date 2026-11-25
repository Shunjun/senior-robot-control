import { useEffect, useId, useRef, useState, type ReactElement } from "react";

import {
  ApiError,
  RateLimitedError,
  UnauthorizedError,
  type PairingCode,
} from "../../api/client";
import { createPortal } from "react-dom";
import { X } from "lucide-react";
import { IconButton } from "../../components/ui/IconButton";
import { Button } from "../../components/ui/Button";
import styles from "./Devices.module.css";

export interface PairingPanelProps {
  initiallyOpen?: boolean;
  onClose?: () => void;
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
      message:
        error.retryAfterSeconds === null
          ? "配对码请求过多，请稍后重试。"
          : `配对码请求过多，请在 ${error.retryAfterSeconds} 秒后重试。`,
      needsLogin: false,
    };
  }
  if (error instanceof ApiError && error.status === 403) {
    return {
      message: "当前会话无法生成配对码，请重新登录。",
      needsLogin: true,
    };
  }
  return { message: "无法生成配对码，请检查网络后重试。", needsLogin: false };
}

export function PairingPanel({
  createCode,
  onReauthenticate,
  initiallyOpen = false,
  onClose,
}: PairingPanelProps): ReactElement {
  const panelId = useId();
  const headingId = useId();
  const request = useRef<AbortController | null>(null);
  const [open, setOpen] = useState(initiallyOpen);
  const [busy, setBusy] = useState(false);
  const [issued, setIssued] = useState<IssuedCode | null>(null);
  const [remaining, setRemaining] = useState(0);
  const [copyStatus, setCopyStatus] = useState<string | null>(null);
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

  const command = `python -m vehicle_manager.agent --pair --cloud-url 'wss://${window.location.host}/ws/agent' --device-name '巡检车' --credentials-file "$PWD/device-credentials.json"`;
  const dialog = useRef<HTMLElement>(null);
  useEffect(() => {
    if (!open) return;
    const previous = document.activeElement as HTMLElement | null;
    dialog.current?.querySelector<HTMLElement>("button")?.focus();
    const keys = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        event.preventDefault();
        close();
      }
      if (event.key === "Tab") {
        const nodes = dialog.current?.querySelectorAll<HTMLElement>(
          'button:not(:disabled), [tabindex="0"]',
        );
        if (!nodes?.length) return;
        if (event.shiftKey && document.activeElement === nodes[0]) {
          event.preventDefault();
          nodes[nodes.length - 1]?.focus();
        } else if (
          !event.shiftKey &&
          document.activeElement === nodes[nodes.length - 1]
        ) {
          event.preventDefault();
          nodes[0]?.focus();
        }
      }
    };
    document.addEventListener("keydown", keys);
    return () => {
      document.removeEventListener("keydown", keys);
      previous?.focus();
    };
  }, [open]);
  function close(): void {
    request.current?.abort();
    request.current = null;
    setBusy(false);
    setIssued(null);
    setOpen(false);
    onClose?.();
  }
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

  async function copy(value: string): Promise<void> {
    try {
      await navigator.clipboard.writeText(value);
      setCopyStatus("已复制");
    } catch {
      setCopyStatus("无法自动复制，请选中文本手动复制。");
    }
  }

  return (
    <div className={styles.pairing}>
      {!initiallyOpen && (
        <Button
          type="button"
          variant="outline"
          aria-expanded={open}
          aria-controls={panelId}
          onClick={toggle}
        >
          {open ? "关闭配对" : "添加设备"}
        </Button>
      )}
      {open &&
        createPortal(
          <div
            className={styles.modal}
            onPointerDown={(event) => {
              if (event.target === event.currentTarget) close();
            }}
          >
            <section
              ref={dialog}
              role="dialog"
              aria-modal="true"
              id={panelId}
              className={styles.pairingPanel}
              aria-labelledby={headingId}
            >
              <div className={styles.listHead}>
                <h3 id={headingId} className={styles.pairingHeading}>
                  设备配对
                </h3>
                <IconButton label="关闭配对" onClick={close}>
                  <X size={16} />
                </IconButton>
              </div>
              <div className={styles.pairSteps}>
                <span>01 网页生成配对码</span>
                <span>02 车端输入</span>
                <span>03 刷新设备</span>
              </div>
              {!issued && (
                <p className={styles.quiet}>
                  先在这里生成一次性配对码，再到车端完成配对。
                </p>
              )}
              {error !== null && (
                <p role="alert" className={styles.failure}>
                  {error.message}
                </p>
              )}
              {expired && (
                <p role="status" className={styles.quiet}>
                  配对码已过期。
                </p>
              )}
              {error?.needsLogin ? (
                <Button
                  type="button"
                  variant="primary"
                  onClick={onReauthenticate}
                >
                  重新登录
                </Button>
              ) : issued === null ? (
                <Button
                  type="button"
                  variant="primary"
                  disabled={busy}
                  onClick={() => void generate()}
                >
                  {busy
                    ? "正在生成…"
                    : expired
                      ? "重新生成配对码"
                      : "生成配对码"}
                </Button>
              ) : (
                <>
                  <div className={styles.pairingReceipt}>
                    <code aria-label="配对码" className={styles.pairingCode}>
                      {issued.code}
                    </code>
                    <span className={styles.quiet}>剩余 {remaining} 秒</span>
                    <Button
                      type="button"
                      onClick={() => void copy(issued.code)}
                    >
                      复制配对码
                    </Button>
                  </div>
                  <p className={styles.quiet}>
                    在车端可写的目标目录运行以下命令，并在提示符中输入配对码。
                  </p>
                  <code className={styles.pairingCommand}>{command}</code>
                  <Button onClick={() => void copy(command)}>复制命令</Button>
                  {copyStatus && <p role="status">{copyStatus}</p>}
                  <p className={styles.quiet}>
                    配对完成后可刷新列表。启动车端代理时使用同一凭据文件，连接后设备显示为在线。
                  </p>
                </>
              )}
            </section>
          </div>,
          document.body,
        )}
    </div>
  );
}
