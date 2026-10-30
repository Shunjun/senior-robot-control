import { useId, useState, type FormEvent, type ReactElement } from "react";

import {
  ApiError,
  InvalidCredentialsError,
  RateLimitedError,
  type ControlApi,
} from "../../api/client";
import styles from "./Devices.module.css";

export interface LoginProps {
  api: Pick<ControlApi, "login">;
  /** Runs after the cloud accepted the password; the app then loads the device list. */
  onLoggedIn: () => Promise<void> | void;
}

function describeFailure(error: unknown): string {
  if (error instanceof InvalidCredentialsError) return "密码错误，请重试。";
  if (error instanceof RateLimitedError) {
    const wait = error.retryAfterSeconds;
    return wait === null ? "尝试次数过多，请稍后再试。" : `尝试次数过多，请 ${wait} 秒后再试。`;
  }
  if (error instanceof ApiError && error.status === 403) return "来源不被允许，无法登录。";
  return "无法连接云端，请检查网络后重试。";
}

export function Login({ api, onLoggedIn }: LoginProps): ReactElement {
  const fieldId = useId();
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
    <main className={styles.loginStage}>
      <form className={styles.loginPanel} onSubmit={(event) => void submit(event)}>
        <p className={styles.eyebrow}>SENIOR · CTRL / 访问控制</p>
        <h1 className={styles.loginTitle}>操作员登录</h1>
        <p className={styles.loginHint}>只有一个管理员账户，没有注册入口。</p>
        <label className={styles.field} htmlFor={fieldId}>
          管理员密码
        </label>
        <input
          id={fieldId}
          className={styles.input}
          type="password"
          autoComplete="current-password"
          value={password}
          onChange={(event) => setPassword(event.target.value)}
          aria-invalid={failure !== null}
        />
        {failure !== null && (
          <p className={styles.failure} role="alert">
            {failure}
          </p>
        )}
        <button className={styles.primary} type="submit" disabled={busy}>
          {busy ? "验证中…" : "进入控制台"}
        </button>
      </form>
    </main>
  );
}
