import { RefreshCw } from "lucide-react";
import { newId } from "../../ids";
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
import { Button } from "../../components/ui/Button";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  srOnly: [
    "settings-sr-only absolute w-[1px] h-[1px] overflow-hidden [clip-path:inset(50%)]"
  ].join(" "),
  fields: [
    "settings-fields mt-4.5 grid grid-cols-[1fr_1fr] gap-4 [&_label]:text-[11px] [&_label]:flex [&_label]:flex-col",
    "[&_label]:gap-[7px] [&_label]:text-field [&_input]:w-full [&_input]:pt-2.5 [&_input]:pr-2.5 [&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:bg-none",
    "[&_input]:bg-input [&_input]:border [&_input]:border-solid [&_input]:border-login-input-border-color [&_input]:rounded-input [&_input]:text-input-ink [&_input]:min-w-0",
    "[&_select]:w-full [&_select]:pt-2.5 [&_select]:pr-2.5 [&_select]:pb-2.5 [&_select]:pl-2.5 [&_select]:bg-none [&_select]:bg-input [&_select]:border",
    "[&_select]:border-solid [&_select]:border-login-input-border-color [&_select]:rounded-input [&_select]:text-input-ink [&_select]:min-w-0 [&_small]:text-ink-dim [&_small]:text-[10px]",
    "[&_.settings-check]:flex-row [&_.settings-check]:items-center max-[460.001px]:grid-cols-[1fr]"
  ].join(" "),
  configDetails: [
    "settings-config-details mt-3.5 text-muted text-[10px] [&_summary]:cursor-pointer"
  ].join(" "),
  actions: [
    "settings-actions flex gap-2 flex-wrap mt-5.5 ",
    ""
  ].join(" "),
  check: [
    "settings-check self-center [&>button]:w-full [&>button]:justify-between"
  ].join(" "),
  card: [
    "settings-card absolute inset-0 overflow-auto pt-7 pr-7 pb-7 pl-7",
    "bg-none bg-full-view min-w-0 z-[20] [&>header]:flex [&>header]:items-center [&>header]:justify-between [&>header]:gap-3",
    "[&>header]:mb-6 [&_h1]:text-[23px] [&_h1]:font-[450] [&_h1]:mt-0 [&_h1]:mr-0 [&_h1]:mb-2 [&_h1]:ml-0 [&_h2]:text-[16px]",
    "[&_h2]:font-medium [&_h2]:mt-0 [&_h2]:mr-0 [&_h2]:mb-0 [&_h2]:ml-0 [&_p]:text-[11px] [&_p]:text-ink-dim [&_p]:leading-[1.8]",
    "[&_button:not([aria-busy])]:pt-2 [&_button:not([aria-busy])]:pr-3 [&_button:not([aria-busy])]:pb-2 [&_button:not([aria-busy])]:pl-3 [&_button:not([aria-busy])]:border [&_button:not([aria-busy])]:border-solid [&_button:not([aria-busy])]:border-line [&_button:not([aria-busy])]:rounded-control",
    "[&_button:not([aria-busy])]:bg-none [&_button:not([aria-busy])]:bg-control [&_button:not([aria-busy])]:text-ink [&_button:not([aria-busy])]:cursor-pointer [&_button:not([aria-busy])]:text-[11px] [&&_.settings-nav_button]:text-left [&&_.settings-nav_button]:border-0 [&&_.settings-nav_button]:border-none",
    "[&&_.settings-nav_button]:border-current [&&_.settings-nav_button]:bg-none [&&_.settings-nav_button]:bg-transparent [&&_.settings-nav_button[aria-pressed=true]]:text-settings-card-color [&&_.settings-nav_button[aria-pressed=true]]:bg-none [&&_.settings-nav_button[aria-pressed=true]]:bg-settings-card-background-color [&_button:disabled]:opacity-[0.45] [&_button:disabled]:cursor-not-allowed",
    "[&_fieldset]:border-0 [&_fieldset]:border-none [&_fieldset]:border-current [&_fieldset]:pt-0 [&_fieldset]:pr-0 [&_fieldset]:pb-0 [&_fieldset]:pl-0 [&_fieldset]:mt-0",
    "[&_fieldset]:mr-0 [&_fieldset]:mb-0 [&_fieldset]:ml-0 [&_fieldset]:min-w-0 [&_legend]:text-[10px] [&_legend]:text-ink-dim [&_legend]:mb-4.5 max-[760.001px]:pt-4.5",
    "max-[760.001px]:pr-4.5 max-[760.001px]:pb-4.5 max-[760.001px]:pl-4.5 max-[460.001px]:pt-3.5 max-[460.001px]:pr-3.5 max-[460.001px]:pb-3.5 max-[460.001px]:pl-3.5 max-[460.001px]:[&&_.settings-nav_button]:pt-2",
    "max-[460.001px]:[&&_.settings-nav_button]:pr-2 max-[460.001px]:[&&_.settings-nav_button]:pb-2 max-[460.001px]:[&&_.settings-nav_button]:pl-2 max-[460.001px]:[&&_.settings-nav_button]:text-[10px] [&_header_p]:mt-0 [&_header_p]:mr-0 [&_header_p]:mb-0 [&_header_p]:ml-0"
  ].join(" "),
  layout: [
    "settings-layout grid grid-cols-[185px_minmax(0,_650px)] gap-5.5 max-[760.001px]:grid-cols-[1fr]"
  ].join(" "),
  nav: [
    "settings-nav flex flex-col gap-1.5 self-start max-[760.001px]:flex-row max-[760.001px]:flex-wrap max-[760.001px]:gap-0.5"
  ].join(" "),
  content: [
    "settings-content min-w-0"
  ].join(" "),
  feedback: [
    "settings-feedback pt-[9px] pr-3 pb-[9px] pl-3 border border-solid border-settings-feedback-border-color",
    "rounded-md bg-none bg-settings-card"
  ].join(" "),
  groups: [
    "settings-groups [&>div]:bg-none [&>div]:bg-settings-card [&>div]:border [&>div]:border-solid [&>div]:border-line [&>div]:rounded-panel [&>div]:pt-[19px]",
    "[&>div]:pr-[19px] [&>div]:pb-[19px] [&>div]:pl-[19px] max-[460.001px]:[&>div]:pt-3.5 max-[460.001px]:[&>div]:pr-3.5 max-[460.001px]:[&>div]:pb-3.5 max-[460.001px]:[&>div]:pl-3.5 [&>div>p]:mt-[5px]",
    "[&[hidden]]:hidden [&>div[hidden]]:hidden"
  ].join(" "),
  section: [
    "settings-section bg-none bg-settings-card border border-solid border-line rounded-panel pt-[19px]",
    "pr-[19px] pb-[19px] pl-[19px] max-[460.001px]:pt-3.5 max-[460.001px]:pr-3.5 max-[460.001px]:pb-3.5 max-[460.001px]:pl-3.5 [&>p:first-of-type]:mt-[5px]",
    "[&[hidden]]:hidden [&>button[role=switch]]:w-full [&>button[role=switch]]:justify-between [&>button[role=switch]]:border-0 [&>button[role=switch]]:border-none [&>button[role=switch]]:border-current [&>button[role=switch]]:[border-radius:0] [&>button[role=switch]]:pt-[15px]",
    "[&>button[role=switch]]:pr-0 [&>button[role=switch]]:pb-[15px] [&>button[role=switch]]:pl-0 [&>button[role=switch]]:bg-none [&>button[role=switch]]:bg-transparent [&>button[role=switch]]:border-b [&>button[role=switch]]:[border-bottom-style:solid] [&>button[role=switch]]:border-b-line-soft"
  ].join(" "),
} as const;

export interface VehicleSettingsProps {
  cameraOn?: boolean;
  onCameraChange?: ((value: boolean) => void) | undefined;
  port?: VehiclePort;
  enabled: boolean;
  onBack?: (() => void) | undefined;
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
      <legend className={ui.srOnly}>
        驾驶设置 · 配置修订号 {data.snapshot.driving_version}
      </legend>
      <div className={ui.fields}>
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
          </label>
        ))}
      </div>
      <details className={ui.configDetails}>
        <summary>生效值、范围与操控说明</summary>
        {fields.map((f) => (
          <p key={f.key}>
            {f.label} · 已保存{" "}
            {(data.snapshot.driving[f.key] * f.scale).toFixed(2)} {f.unit} ·
            默认{" "}
            {(data.capabilities.driving_defaults[f.key] * f.scale).toFixed(2)} ·
            范围{" "}
            {(data.capabilities.driving_minimum[f.key] * f.scale).toFixed(2)}–
            {(data.capabilities.driving_maximum[f.key] * f.scale).toFixed(2)}
          </p>
        ))}
        <p>
          手动退化模式按摇杆百分比映射已保存上限；实际满杆速度仍受当前档位、倒车和正常车体/轮能力约束，遇障碍会进一步限速。
        </p>
      </details>
      <div className={ui.actions}>
        <Button variant="primary" disabled={!data.can_apply_driving} onClick={() => save(false)}>
          保存驾驶设置
        </Button>
        <Button disabled={!data.can_apply_driving} onClick={() => save(true)}>
          恢复驾驶默认值
        </Button>
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
      <legend className={ui.srOnly}>
        日志设置 · 配置修订号 {data.snapshot.logging_version}
      </legend>
      <div className={ui.fields}>
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
        <div className={ui.check}>
          <Switch
            label="临时调试日志"
            checked={draft.debug_enabled}
            onChange={(debug_enabled) => setDraft({ ...draft, debug_enabled })}
          />
        </div>
      </div>
      <p>运行中可修改。自动恢复由车端负责，网页断开仍会恢复原级别。</p>
      {data.snapshot.debug_deadline_utc !== null && (
        <p>
          当前恢复截止：
          {new Date(data.snapshot.debug_deadline_utc * 1000).toLocaleString()}
        </p>
      )}
      <div className={ui.actions}>
        <Button variant="primary" onClick={() => submit("logging", draft, false)}>
          保存日志设置
        </Button>
        <Button
          onClick={() =>
            submit("logging", data.capabilities.logging_defaults, true)
          }
        >
          恢复日志默认值
        </Button>
      </div>
    </fieldset>
  );
}
export const VehicleSettings = memo(function VehicleSettings({
  port,
  enabled,
  cameraOn = false,
  onBack,
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
      request_id: newId(),
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
  const refreshControls = (
    <footer className="mt-5 flex flex-wrap items-center gap-x-3 gap-y-2 border-t border-line pt-4">
      <Button
        disabled={!available || busy !== null}
        onClick={() => void refresh()}
      >
        <RefreshCw aria-hidden="true" />
        刷新配置
      </Button>
      {data && (
        <span className="text-[10px] text-ink-dim">
          配置修订号：驾驶 {data.snapshot.driving_version} · 日志 {data.snapshot.logging_version}
        </span>
      )}
    </footer>
  );
  return (
    <section className={ui.card} aria-label="车辆设置">
      <header>
        <div>
          <h1>车辆设置</h1>
          <p>管理当前车辆的操控、预览与状态配置。</p>
        </div>
        {onBack && <Button onClick={onBack}>返回控制 ↗</Button>}
      </header>
      <div className={ui.layout}>
        <nav className={ui.nav} aria-label="设置分类">
          {[
            ["driving", "驾驶与操控"],
            ["camera", "摄像头与预览"],
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
        <div className={ui.content}>
          {!available && <p>车辆设置暂不可用，请连接支持设置的车辆。</p>}
          {message && (
            <p role="status" className={ui.feedback}>
              {message}
            </p>
          )}
          {unknown && (
            <div className={ui.actions}>
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
          <div className={ui.groups}>
            <div hidden={category !== "driving"}>
              <h2>驾驶与操控</h2>
              <p>保存后由车端确认生效；速度显示单位与控制页一致。</p>
              {data && (
                <DrivingForm
                  key={`driving-${saved.driving}`}
                  data={data}
                  busy={busy !== null || unknown !== null}
                  enabled={available}
                  submit={submit}
                />
              )}
              {refreshControls}
            </div>
            <div hidden={category !== "logging"}>
              <h2>日志配置</h2>
              {data && (
                <LoggingForm
                  key={`logging-${saved.logging}`}
                  data={data}
                  busy={busy !== null || unknown !== null}
                  enabled={available}
                  submit={submit}
                />
              )}
              {refreshControls}
            </div>
          </div>
          <section hidden={category !== "camera"} className={ui.section}>
            <h2>摄像头与预览</h2>
            <p>与控制页共用摄像头开关。</p>
            <Switch
              label="显示摄像头预览"
              checked={cameraOn}
              onChange={(value) => onCameraChange?.(value)}
            />
            <div className={ui.fields}>
              <label>
                画质
                <select disabled aria-label="画质">
                  <option>由车端提供</option>
                </select>
              </label>
            </div>
            <p>暂未实现：画质参数写入。</p>
            {refreshControls}
          </section>
          <section
            hidden={category !== "performance"}
            className={ui.section}
          >
            <h2>状态与性能</h2>
            <div className={ui.fields}>
              <label>
                状态采样间隔
                <select disabled aria-label="状态采样间隔">
                  <option>由车端提供</option>
                </select>
              </label>
            </div>
            <p>状态浮层使用车端真实样本；暂未实现采样频率写入。</p>
            <details>
              <summary>车辆诊断与固定回波校准</summary>
              <VehicleStateView
                showTaskControls={false}
                {...(port ? { port } : {})}
              />
            </details>
            {refreshControls}
          </section>
        </div>
      </div>
    </section>
  );
});
