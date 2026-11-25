import { memo, useEffect, useRef, useState, type ReactElement } from "react";
import { validateSettingsRequest } from "../../api/vehicleState";
import type {
  DrivingSettings,
  LoggingSettings,
  SettingsData,
  SettingsGroup,
  SettingsRequest,
  VehiclePort,
} from "../../api/vehicleState";
import { useSpeedUnit, speedScale } from "../../display/speedUnits";
import { Switch } from "../../components/ui/Switch";
import { VehicleStateView } from "../devices/VehicleStateView";
import styles from "./Settings.module.css";
export interface VehicleSettingsProps {
  cameraOn?: boolean;
  onCameraChange?: ((value: boolean) => void) | undefined;
  port?: VehiclePort;
  enabled: boolean;
}
const FIELDS: ReadonlyArray<{
  key: keyof DrivingSettings;
  label: string;
  scale: number;
  unit: string;
}> = [
  {
    key: "low_linear_mps",
    label: "低速线速度上限",
    scale: 3.6,
    unit: "km/h",
  },
  {
    key: "high_linear_mps",
    label: "高速线速度上限",
    scale: 3.6,
    unit: "km/h",
  },
  {
    key: "low_angular_rps",
    label: "低速角速度请求上限",
    scale: 1,
    unit: "rad/s",
  },
  {
    key: "high_angular_rps",
    label: "高速角速度请求上限",
    scale: 1,
    unit: "rad/s",
  },
  {
    key: "degraded_linear_mps",
    label: "退化模式线速度上限",
    scale: 3.6,
    unit: "km/h",
  },
  {
    key: "reverse_linear_mps",
    label: "统一倒车速度上限",
    scale: 3.6,
    unit: "km/h",
  },
];
interface FormProps {
  data: SettingsData;
  busy: boolean;
  enabled: boolean;
  submit: (
    group: SettingsGroup,
    values: DrivingSettings | LoggingSettings,
    restore: boolean,
  ) => void;
}
function DrivingForm({ data, busy, enabled, submit }: FormProps): ReactElement {
  const unit = useSpeedUnit();
  const fields = FIELDS.map((f) =>
    f.unit === "km/h" ? { ...f, scale: speedScale(unit), unit } : f,
  );
  const [draft, setDraft] = useState(
    () =>
      Object.fromEntries(
        FIELDS.map((f) => [f.key, String(data.snapshot.driving[f.key])]),
      ) as Record<keyof DrivingSettings, string>,
  );
  const [validation, setValidation] = useState<string | null>(null);
  function save(restore: boolean): void {
    if (restore) {
      submit("driving", data.capabilities.driving_defaults, true);
      return;
    }
    const values = Object.fromEntries(
      FIELDS.map((f) => [f.key, Number(draft[f.key])]),
    ) as unknown as DrivingSettings;
    if (
      FIELDS.some(
        (f) => draft[f.key].trim() === "" || !Number.isFinite(values[f.key]),
      )
    ) {
      setValidation("请填写完整的有限数值；尚未提交");
      return;
    }
    setValidation(null);
    submit("driving", values, restore);
  }
  return (
    <fieldset disabled={!enabled || busy}>
      <legend>驾驶设置 · 生效版本 {data.snapshot.driving_version}</legend>
      <div className={styles.fields}>
        {fields.map((f) => (
          <label key={f.key}>
            {f.label}（{f.unit}）
            <input
              aria-label={f.label}
              type="number"
              step="any"
              min={data.capabilities.driving_minimum[f.key] * f.scale}
              max={data.capabilities.driving_maximum[f.key] * f.scale}
              value={
                draft[f.key] === ""
                  ? ""
                  : Number((Number(draft[f.key]) * f.scale).toPrecision(12))
              }
              onChange={(e) =>
                setDraft({
                  ...draft,
                  [f.key]:
                    e.target.value === ""
                      ? ""
                      : String(Number(e.target.value) / f.scale),
                })
              }
            />
            <small>
              已保存上限 {(data.snapshot.driving[f.key] * f.scale).toFixed(2)} ·
              默认{" "}
              {(data.capabilities.driving_defaults[f.key] * f.scale).toFixed(2)}{" "}
              · 范围{" "}
              {(data.capabilities.driving_minimum[f.key] * f.scale).toFixed(2)}–
              {(data.capabilities.driving_maximum[f.key] * f.scale).toFixed(2)}
            </small>
          </label>
        ))}
      </div>
      <p>
        手动退化模式按摇杆百分比映射已保存上限；实际满杆速度仍受当前档位、倒车和正常车体/轮能力约束，遇障碍会进一步限速。
      </p>
      <div className={styles.actions}>
        <button disabled={!data.can_apply_driving} onClick={() => save(false)}>
          保存驾驶设置
        </button>
        <button disabled={!data.can_apply_driving} onClick={() => save(true)}>
          恢复驾驶默认值
        </button>
      </div>
      {validation && <p role="alert">{validation}</p>}
      {!data.can_apply_driving && (
        <p>车端尚未允许修改驾驶参数，请先松杆、暂停任务并停稳。</p>
      )}
    </fieldset>
  );
}
function LoggingForm({ data, busy, enabled, submit }: FormProps): ReactElement {
  const [draft, setDraft] = useState(data.snapshot.logging);
  return (
    <fieldset disabled={!enabled || busy}>
      <legend>日志设置 · 生效版本 {data.snapshot.logging_version}</legend>
      <div className={styles.fields}>
        <label>
          常规日志级别
          <select
            value={draft.level}
            onChange={(e) =>
              setDraft({
                ...draft,
                level: e.target.value as LoggingSettings["level"],
              })
            }
          >
            {["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"].map((level) => (
              <option key={level}>{level}</option>
            ))}
          </select>
        </label>
        <label>
          保留时间（小时）
          <input
            type="number"
            min="1"
            max="720"
            value={draft.retention_seconds / 3600}
            onChange={(e) =>
              setDraft({
                ...draft,
                retention_seconds: Number(e.target.value) * 3600,
              })
            }
          />
          <small>允许 1 小时–30 天，默认 7 天</small>
        </label>
        <label>
          自动恢复时长（秒）
          <input
            type="number"
            min="60"
            max="3600"
            value={draft.debug_restore_seconds}
            onChange={(e) =>
              setDraft({
                ...draft,
                debug_restore_seconds: Number(e.target.value),
              })
            }
          />
          <small>允许 60–3600 秒，默认 300 秒</small>
        </label>
        <label className={styles.check}>
          <input
            type="checkbox"
            checked={draft.debug_enabled}
            onChange={(e) =>
              setDraft({ ...draft, debug_enabled: e.target.checked })
            }
          />
          临时调试日志
        </label>
      </div>
      <p>运行中可修改。自动恢复由车端负责，网页断开仍会恢复原级别。</p>
      {data.snapshot.debug_deadline_utc !== null && (
        <p>
          当前恢复截止：
          {new Date(data.snapshot.debug_deadline_utc * 1000).toLocaleString()}
        </p>
      )}
      <div className={styles.actions}>
        <button onClick={() => submit("logging", draft, false)}>
          保存日志设置
        </button>
        <button
          onClick={() =>
            submit("logging", data.capabilities.logging_defaults, true)
          }
        >
          恢复日志默认值
        </button>
      </div>
    </fieldset>
  );
}
export const VehicleSettings = memo(function VehicleSettings({
  port,
  enabled,
  cameraOn = false,
  onCameraChange,
}: VehicleSettingsProps): ReactElement {
  const [category, setCategory] = useState("driving");
  const available = enabled && port !== undefined;
  const [data, setData] = useState<SettingsData | null>(null),
    [message, setMessage] = useState<string | null>(null);
  const [busy, setBusy] = useState<SettingsGroup | null>(null),
    [unknown, setUnknown] = useState<SettingsRequest | null>(null);
  const [saved, setSaved] = useState({ driving: 0, logging: 0 });
  const scope = useRef(0);
  useEffect(() => {
    scope.current++;
    setData(null);
    setUnknown(null);
    setMessage(null);
    setBusy(null);
    return () => {
      scope.current++;
    };
  }, [port]);
  useEffect(() => {
    const token = scope.current;
    if (enabled && port)
      void port.settingsRead().then(
        (value) => {
          if (token === scope.current) acceptData(value);
        },
        (error) => {
          if (token === scope.current) setMessage(String(error));
        },
      );
  }, [port, enabled]);
  function acceptData(next: SettingsData): void {
    setData((previous) =>
      previous &&
      (next.snapshot.driving_version < previous.snapshot.driving_version ||
        next.snapshot.logging_version < previous.snapshot.logging_version)
        ? previous
        : next,
    );
  }
  async function refresh(): Promise<void> {
    const token = scope.current;
    try {
      const next = await port!.settingsRead();
      if (token === scope.current) acceptData(next);
    } catch (error) {
      if (token === scope.current) setMessage(String(error));
    }
  }
  async function send(request: SettingsRequest, query = false): Promise<void> {
    if (!port) return;
    const token = scope.current;
    setBusy(request.group);
    setMessage(query ? "正在查询原请求结果…" : "等待车端配置回执…");
    try {
      const result = query
        ? await port.settingsResult(request.request_id)
        : await port.settingsApply(request);
      if (token !== scope.current) return;
      if (result === null) {
        setUnknown(request);
        setMessage("原请求结果仍待确认，请保留请求标识");
        return;
      }
      setUnknown(null);
      const messages = {
        applied: "车端已确认配置生效，正在回读",
        version_conflict: "配置版本冲突，请核对最新生效值；草稿保留",
        stop_required: "车端拒绝：请先停稳且无执行中的运动任务",
        invalid_values: "车端拒绝了配置值，请核对范围和组合",
        request_conflict: "请求标识与内容冲突，请核查原请求",
      };
      setMessage(messages[result.reason]);
      try {
        const latest = await port.settingsRead();
        if (token !== scope.current) return;
        acceptData(latest);
        if (result.reason === "applied")
          setSaved((prev) => ({
            ...prev,
            [request.group]: prev[request.group] + 1,
          }));
      } catch (error) {
        if (token === scope.current)
          setMessage(
            `${messages[result.reason]}；当前配置回读失败：${String(error)}`,
          );
      }
    } catch (error) {
      if (token === scope.current) {
        setUnknown(request);
        setMessage(`提交结果待确认：${request.request_id} · ${String(error)}`);
      }
    } finally {
      if (token === scope.current) setBusy(null);
    }
  }
  function submit(
    group: SettingsGroup,
    values: DrivingSettings | LoggingSettings,
    restore_defaults: boolean,
  ): void {
    if (!data || unknown) return;
    const request: SettingsRequest = {
      request_id: crypto.randomUUID(),
      expected_version:
        group === "driving"
          ? data.snapshot.driving_version
          : data.snapshot.logging_version,
      group,
      values,
      restore_defaults,
    };
    try {
      validateSettingsRequest(request);
    } catch {
      setMessage("请核对完整数值、低速/高速组合和日志时长范围；尚未提交");
      return;
    }
    void send(request);
  }
  return (
    <section className={styles.card} aria-label="车辆设置">
      <header>
        <div>
          <h1>设置</h1>
          <p>管理车辆操控、预览与状态配置。</p>
        </div>
        <button
          disabled={!available || busy !== null}
          onClick={() => void refresh()}
        >
          回读生效配置
        </button>
      </header>
      <div className={styles.layout}>
        <nav className={styles.nav} aria-label="设置分类">
          {[
            ["driving", "驾驶操控"],
            ["camera", "摄像头预览"],
            ["performance", "状态与性能"],
            ["logging", "日志配置"],
          ].map(([id, label]) => (
            <button
              key={id}
              aria-pressed={category === id}
              onClick={() => setCategory(id!)}
            >
              {label}
            </button>
          ))}
        </nav>
        <div className={styles.content}>
          {!available && <p>车辆设置暂不可用，请连接支持设置的车辆。</p>}
          {message && <p role="status">{message}</p>}
          {unknown && (
            <div className={styles.actions}>
              <button
                disabled={!available || busy !== null}
                onClick={() => void send(unknown, true)}
              >
                查询原请求结果
              </button>
              <button
                disabled={!available || busy !== null}
                onClick={() => void send(unknown)}
              >
                原样重试请求
              </button>
            </div>
          )}
          {available && !data && !message && (
            <p role="status">正在读取车辆设置…</p>
          )}
          {data && (
            <div className={styles.groups}>
              <div hidden={category !== "driving"}>
                <h2>驾驶操控</h2>
                <DrivingForm
                  key={`driving-${saved.driving}`}
                  data={data}
                  busy={busy !== null || unknown !== null}
                  enabled={available}
                  submit={submit}
                />
              </div>
              <div hidden={category !== "logging"}>
                <h2>日志配置</h2>
                <LoggingForm
                  key={`logging-${saved.logging}`}
                  data={data}
                  busy={busy !== null || unknown !== null}
                  enabled={available}
                  submit={submit}
                />
              </div>
            </div>
          )}
          <section hidden={category !== "camera"} className={styles.section}>
            <h2>摄像头预览</h2>
            <p>与控制页共用摄像头开关。</p>
            <Switch
              label="显示摄像头预览"
              checked={cameraOn}
              onChange={(value) => onCameraChange?.(value)}
            />
            <p>暂未实现：画质参数写入。</p>
          </section>
          <section
            hidden={category !== "performance"}
            className={styles.section}
          >
            <h2>状态与性能</h2>
            <p>状态浮层使用车端真实样本；暂未实现采样频率写入。</p>
            <details>
              <summary>车辆诊断与固定回波校准</summary>
              <VehicleStateView
                showTaskControls={false}
                {...(port ? { port } : {})}
              />
            </details>
          </section>
        </div>
      </div>
    </section>
  );
});
