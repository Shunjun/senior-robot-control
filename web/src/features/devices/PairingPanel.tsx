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

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  modal: [
    "devices-modal absolute inset-0 z-[40] bg-none bg-devices-modal-background-color grid place-items-center",
    "pt-5 pr-5 pb-5 pl-5 [body>&]:fixed"
  ].join(" "),
  pairingPanel: [
    "devices-pairing-panel w-full max-w-[510px] max-h-[calc(100%_-_30px)] overflow-auto bg-none bg-card border",
    "border-solid border-line rounded-card pt-[19px] pr-[19px] pb-[19px] pl-[19px] shadow-ui-card",
    "backdrop-blur-[15px] [&_p]:mt-[11px] [&_p]:mr-0 [&_p]:mb-[11px] [&_p]:ml-0 [&_p]:leading-[1.9] [&>button]:w-full [&_.devices-pair-initial-hint]:text-xs",
    "[&_.devices-pair-initial-hint]:mt-[15px] [&_.devices-pair-initial-hint]:text-muted"
  ].join(" "),
  listHead: [
    "devices-list-head flex items-center justify-between gap-2.5 mb-[5px] [&>button]:border-0 [&>button]:border-none",
    "[&>button]:border-current [&>button]:bg-none [&>button]:bg-transparent [&>button]:pt-1 [&>button]:pr-[7px] [&>button]:pb-1 [&>button]:pl-[7px] [&>button]:text-quiet"
  ].join(" "),
  pairingHeading: [
    "devices-pairing-heading text-base font-medium mt-0 mr-0 mb-3 ml-0"
  ].join(" "),
  pairSteps: [
    "devices-pair-steps flex justify-between gap-2.5 text-devices-pair-steps-color text-xs mb-5 [&_[data-active=true]]:text-devices-pair-steps-color-2",
    "max-[460.001px]:text-xs max-[460.001px]:gap-[5px]"
  ].join(" "),
  quiet: [
    "devices-quiet mt-0 mr-0 mb-0 ml-0 text-muted text-xs leading-[1.8]"
  ].join(" "),
  pairError: [
    "devices-pair-error text-devices-pair-error-color text-xs min-h-[22px] mt-2 mr-0 mb-2 ml-0"
  ].join(" "),
  pairingReceipt: [
    "devices-pairing-receipt flex justify-between gap-3 items-center bg-none bg-devices-pairing-receipt-background-color border",
    "border-solid border-devices-pairing-receipt-border-color pt-4 pr-4 pb-4 pl-4 rounded-lg mt-4.5",
    "mr-0 mb-2 ml-0 [&_button]:text-xs [&_button]:pt-[7px] [&_button]:pr-2.5 [&_button]:pb-[7px] [&_button]:pl-2.5",
    "max-[460.001px]:pt-[13px] max-[460.001px]:pr-[9px] max-[460.001px]:pb-[13px] max-[460.001px]:pl-[9px]"
  ].join(" "),
  pairingCode: [
    "devices-pairing-code font-normal text-2xl leading-[normal] font-code tracking-[3px] text-devices-pairing-code-color select-all",
    "max-[460.001px]:text-xl max-[460.001px]:tracking-[1.5px]"
  ].join(" "),
  remaining: [
    "devices-remaining text-devices-remaining-color text-xs"
  ].join(" "),
  pairingCommand: [
    "devices-pairing-command block whitespace-pre-wrap [overflow-wrap:anywhere] bg-none bg-devices-pairing-command-background-color border border-solid",
    "border-devices-pairing-command-border-color rounded-md pt-3 pr-3 pb-3 pl-3 mt-2.5 mr-0",
    "mb-2.5 ml-0 font-normal text-xs leading-[1.8] font-code select-all"
  ].join(" "),
  pairInitialHint: [
    "devices-pair-initial-hint"
  ].join(" "),
} as const;

export interface PairingPanelProps {
  initiallyOpen?: boolean;
  overlayTarget?: Element | null | undefined;
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
  overlayTarget,
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
    <>
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
            className={ui.modal}
            onPointerDown={(event) => {
              if (event.target === event.currentTarget) close();
            }}
          >
            <section
              ref={dialog}
              role="dialog"
              aria-modal="true"
              id={panelId}
              className={ui.pairingPanel}
              aria-labelledby={headingId}
            >
              <div className={ui.listHead}>
                <h3 id={headingId} className={ui.pairingHeading}>
                  {issued ? "设备配对" : "添加设备"}
                </h3>
                <IconButton label="关闭配对" onClick={close}>
                  <X size={16} />
                </IconButton>
              </div>
              <div className={ui.pairSteps}>
                <span data-active={!issued}>01 网页生成配对码</span>
                <span data-active={issued !== null}>02 车端输入</span>
                <span>03 刷新设备</span>
              </div>
              {!issued && (
                <p className={ui.quiet}>
                  先在这里生成一次性配对码，再到车端完成配对。
                </p>
              )}
              {!issued && <div role={error ? "alert" : undefined} className={ui.pairError}>{error?.message}</div>}
              {expired && (
                <p role="status" className={ui.quiet}>
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
                  <div className={ui.pairingReceipt}>
                    <code aria-label="配对码" className={ui.pairingCode}>
                      {issued.code}
                    </code>
                    <Button
                      type="button"
                      aria-label="复制配对码"
                      onClick={() => void copy(issued.code)}
                    >
                      复制
                    </Button>
                  </div>
                  <div className={ui.remaining}>剩余 {remaining} 秒</div>
                  <p className={ui.quiet}>
                    在车端可写的目标目录运行以下命令，并在提示符中输入配对码。
                  </p>
                  <code className={ui.pairingCommand}>{command}</code>
                  <Button onClick={() => void copy(command)}>复制命令</Button>
                  {copyStatus && <p role="status">{copyStatus}</p>}
                  <p className={ui.quiet}>
                    配对完成后可刷新列表。启动车端代理时使用同一凭据文件，连接后设备显示为在线。
                  </p>
                </>
              )}
              {!issued && <p className={ui.pairInitialHint}>配对码由云端签发，以返回的有效期为准。</p>}
            </section>
          </div>,
          overlayTarget ?? document.body,
        )}
    </>
  );
}
