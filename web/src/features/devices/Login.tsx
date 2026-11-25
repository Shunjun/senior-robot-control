import { useId, useState, type FormEvent, type ReactElement } from "react";

import {
  ApiError,
  InvalidCredentialsError,
  RateLimitedError,
  type ControlApi,
} from "../../api/client";
import { Eye, EyeOff } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { IconButton } from "../../components/ui/IconButton";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  page: [
    "login-page min-h-[calc(100svh_-_38px)] flex flex-col max-[460.001px]:min-h-[calc(100svh_-_24px)]"
  ].join(" "),
  brand: [
    "login-brand text-[20px] tracking-[2px] font-semibold flex gap-[9px] items-center [&_svg]:w-[29px]",
    "[&_svg]:h-[29px] max-[460.001px]:text-[17px]"
  ].join(" "),
  loginStage: [
    "login-login-stage flex-1 grid place-items-center pt-10 pr-0 pb-10 pl-0",
    "max-[460.001px]:pt-8 max-[460.001px]:pr-0 max-[460.001px]:pb-8 max-[460.001px]:pl-0 [@media(max-height:_460px)]:pt-5 [@media(max-height:_460px)]:pr-0 [@media(max-height:_460px)]:pb-5 [@media(max-height:_460px)]:pl-0"
  ].join(" "),
  loginPanel: [
    "login-login-panel w-[min(410px,_100%)] pt-8 pr-8 pb-8 pl-8 bg-none bg-card",
    "border border-solid border-line rounded-scene shadow-login-login-panel backdrop-blur-[15px] [&_.login-primary]:w-full [&_.login-primary]:min-h-[46px]",
    "[&_.login-primary]:mt-5 max-[460.001px]:pt-6 max-[460.001px]:pr-6 max-[460.001px]:pb-6 max-[460.001px]:pl-6 [@media(max-height:_460px)]:pt-5.5 [@media(max-height:_460px)]:pr-5.5",
    "[@media(max-height:_460px)]:pb-5.5 [@media(max-height:_460px)]:pl-5.5"
  ].join(" "),
  loginTitle: [
    "login-login-title text-[25px] font-[450] mt-0 mr-0 mb-[5px] ml-0 max-[460.001px]:text-[23px]"
  ].join(" "),
  loginHint: [
    "login-login-hint text-ink-dim text-[12px] mt-0 mr-0 mb-[25px] ml-0 leading-[1.8]"
  ].join(" "),
  field: [
    "login-field text-[11px] text-ink-soft block mb-2"
  ].join(" "),
  passwordField: [
    "login-password-field relative [&_.login-reveal]:absolute [&_.login-reveal]:right-0.5 [&_.login-reveal]:top-px [&_.login-reveal]:border-0 [&_.login-reveal]:border-none [&_.login-reveal]:border-current",
    "[&_.login-reveal]:bg-none [&_.login-reveal]:bg-transparent [&_.login-reveal]:text-login-password-field-color [&_.login-reveal]:w-[44px] [&_.login-reveal]:min-h-[44px] [&_.login-reveal]:h-[44px] [&_.login-reveal]:pt-2.5 [&_.login-reveal]:pr-2.5",
    "[&_.login-reveal]:pb-2.5 [&_.login-reveal]:pl-2.5"
  ].join(" "),
  input: [
    "login-input w-full min-h-[46px] pt-2.5 pr-12.5 pb-2.5 pl-3 rounded-control",
    "border border-solid border-login-input-border-color bg-none bg-input text-input-ink "
  ].join(" "),
  reveal: [
    "login-reveal [&_svg]:w-[18px] [&_svg]:h-[18px]"
  ].join(" "),
  message: [
    "login-message min-h-[24px] mt-3 mr-0 mb-0 ml-0"
  ].join(" "),
  failure: [
    "login-failure text-[11px] text-login-failure-color mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  primary: [
    "login-primary"
  ].join(" "),
} as const;

export interface LoginProps {
  api: Pick<ControlApi, "login">;
  /** Runs after the cloud accepted the password; the app then loads the device list. */
  onLoggedIn: () => Promise<void> | void;
}

function describeFailure(error: unknown): string {
  if (error instanceof InvalidCredentialsError) return "密码错误，请重试。";
  if (error instanceof RateLimitedError) {
    const wait = error.retryAfterSeconds;
    return wait === null
      ? "尝试次数过多，请稍后再试。"
      : `尝试次数过多，请 ${wait} 秒后再试。`;
  }
  if (error instanceof ApiError && error.status === 403)
    return "来源不被允许，无法登录。";
  return "无法连接云端，请检查网络后重试。";
}

export function Login({ api, onLoggedIn }: LoginProps): ReactElement {
  const fieldId = useId();
  const [showPassword, setShowPassword] = useState(false);
  const [password, setPassword] = useState("");
  const [busy, setBusy] = useState(false);
  const [failure, setFailure] = useState<string | null>(null);

  async function submit(event: FormEvent<HTMLFormElement>): Promise<void> {
    event.preventDefault();
    if (busy || password === "") return;
    setBusy(true);
    setFailure(null);
    try {
      await api.login(password);
      setPassword("");
      await onLoggedIn();
    } catch (error) {
      setFailure(describeFailure(error));
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className={ui.page}>
      <header className={ui.brand}>
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
        WHEEL
      </header>
      <main className={ui.loginStage}>
        <form
          className={ui.loginPanel}
          onSubmit={(event) => void submit(event)}
        >
          <h1 className={ui.loginTitle}>登录控制台</h1>
          <p className={ui.loginHint}>输入管理员密码，连接你的车辆。</p>
          <label className={ui.field} htmlFor={fieldId}>
            管理员密码
          </label>
          <div className={ui.passwordField}>
            <input
              id={fieldId}
              className={ui.input}
              type={showPassword ? "text" : "password"}
              disabled={busy}
              autoComplete="current-password"
              value={password}
              onChange={(event) => setPassword(event.target.value)}
              aria-invalid={failure !== null}
            />
            <IconButton
              className={ui.reveal}
              label={showPassword ? "隐藏密码" : "显示密码"}
              disabled={busy}
              onClick={() => setShowPassword((value) => !value)}
            >
              {showPassword ? <EyeOff /> : <Eye />}
            </IconButton>
          </div>
          <div className={ui.message} aria-live="polite">
            {failure !== null && (
              <p className={ui.failure} role="alert">
                {failure}
              </p>
            )}
          </div>
          <Button
            variant="primary"
            className={ui.primary}
            type="submit"
            disabled={busy}
          >
            {busy ? "验证中…" : "进入控制台"}
          </Button>
        </form>
      </main>
    </div>
  );
}
