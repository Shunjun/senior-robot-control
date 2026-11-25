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
import styles from "./Login.module.css";

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
    <div className={styles.page}>
      <header className={styles.brand}>
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
      <main className={styles.loginStage}>
        <form
          className={styles.loginPanel}
          onSubmit={(event) => void submit(event)}
        >
          <h1 className={styles.loginTitle}>登录控制台</h1>
          <p className={styles.loginHint}>输入管理员密码，连接你的车辆。</p>
          <label className={styles.field} htmlFor={fieldId}>
            管理员密码
          </label>
          <div className={styles.passwordField}>
            <input
              id={fieldId}
              className={styles.input}
              type={showPassword ? "text" : "password"}
              disabled={busy}
              autoComplete="current-password"
              value={password}
              onChange={(event) => setPassword(event.target.value)}
              aria-invalid={failure !== null}
            />
            <IconButton
              className={styles.reveal}
              label={showPassword ? "隐藏密码" : "显示密码"}
              disabled={busy}
              onClick={() => setShowPassword((value) => !value)}
            >
              {showPassword ? <EyeOff /> : <Eye />}
            </IconButton>
          </div>
          <div className={styles.message} aria-live="polite">
            {failure !== null && (
              <p className={styles.failure} role="alert">
                {failure}
              </p>
            )}
          </div>
          <Button
            variant="primary"
            className={styles.primary}
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
