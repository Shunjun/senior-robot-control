import { newId } from "../../ids";
import { useEffect, useRef, useState, type ReactElement } from 'react';
import type { LearningStatus, TaskAction, VehiclePort } from '../../api/vehicleState';
import { useVehicleView } from './useVehicleView';
import { useSpeedUnit, displaySpeed } from '../../display/speedUnits';

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  card: [
    "vehicle-state-view-card pt-4 pr-4 pb-4 pl-4 border border-solid border-line",
    "bg-none bg-app-empty-symbol-background-color min-w-0 [&_h3]:mt-0 [&_h3]:mr-0 [&_h3]:mb-0 [&_h3]:ml-0 [&_h4]:mt-0",
    "[&_h4]:mr-0 [&_h4]:mb-0 [&_h4]:ml-0 [&_p]:text-ink-soft [&_p]:text-xs [&_details]:mt-4 [&_details]:text-ink-dim [&_details]:text-xs",
    "[&_summary]:cursor-pointer [&_summary]:min-h-[32px]"
  ].join(" "),
  head: [
    "vehicle-state-view-head flex justify-between items-center gap-3 [&_span]:text-ink-dim [&_span]:text-xs"
  ].join(" "),
  warning: [
    "vehicle-state-view-warning pt-3 pr-3 pb-3 pl-3 border-l-[3px] [border-left-style:solid] border-l-amber",
    "bg-none bg-amber-wash"
  ].join(" "),
  speed: [
    "vehicle-state-view-speed flex items-baseline gap-3 flex-wrap mt-4 mr-0 mb-4",
    "ml-0 [&_strong]:font-normal [&_strong]:text-3xl [&_strong]:leading-[1] [&_strong]:font-data [&_span]:text-xs [&_span]:text-ink-dim"
  ].join(" "),
  source: [
    "vehicle-state-view-source text-teal!"
  ].join(" "),
  connection: [
    "vehicle-state-view-connection text-ink-dim text-xs"
  ].join(" "),
  task: [
    "vehicle-state-view-task mt-4.5 pt-4 border-t [border-top-style:solid] border-t-line"
  ].join(" "),
  actions: [
    "vehicle-state-view-actions flex flex-wrap gap-2 [&_button]:pt-2.5 [&_button]:pr-3.5 [&_button]:pb-2.5 [&_button]:pl-3.5",
    "[&_button]:min-h-[44px] [&_button]:bg-none [&_button]:bg-graphite-800 [&_button]:text-ink [&_button]:border [&_button]:border-solid [&_button]:border-line [&_button]:cursor-pointer",
    "[&_button]:rounded-control [&_button:disabled]:opacity-[0.45] [&_button:disabled]:cursor-not-allowed"
  ].join(" "),
  compactTask: [
    "vehicle-state-view-compact-task mt-2 text-xs [&_h4]:text-xs [&_h4]:font-medium [&_h4]:mt-0 [&_h4]:mr-0 [&_h4]:mb-[5px]",
    "[&_h4]:ml-0 [&_p]:text-xs [&_p]:mt-1 [&_p]:mr-0 [&_p]:mb-1 [&_p]:ml-0 [&_.vehicle-state-view-actions]:gap-[5px] [&_.vehicle-state-view-actions_button]:min-h-0",
    "[&_.vehicle-state-view-actions_button]:pt-[7px] [&_.vehicle-state-view-actions_button]:pr-2.5 [&_.vehicle-state-view-actions_button]:pb-[7px] [&_.vehicle-state-view-actions_button]:pl-2.5 [&_.vehicle-state-view-actions_button]:text-xs [&_details]:text-xs [&_details]:text-ink-dim"
  ].join(" "),
} as const;

export interface VehicleStateViewProps {
  port?: VehiclePort;
  showTaskControls?: boolean;
}
const MODES = {
  unknown: '运行状态不可用',
  idle: '静止待命',
  manual: '手动驾驶',
  task: '任务执行中',
  braking: '制动中',
  obstacle_hold: '方向障碍受限',
  feedback_stop: '反馈停车',
  execution_fault: '执行故障',
  estop: '急停已锁存'
};
const AVAILABILITY = {
  current: '实时可观测',
  missing: '尚未收到有效运行状态',
  revoked: '运行状态来源已撤销',
  ipc_unavailable: '本地状态接口不可用'
};
const TASKS = {
  starting: '启动待确认',
  running: '运行中',
  revoking: '撤销输出待确认',
  paused: '已暂停',
  succeeded: '已完成',
  canceled: '已取消',
  failed: '失败'
};
const ACTIONS: Record<TaskAction, string> = {
  pause: '暂停当前任务',
  resume: '明确继续任务',
  cancel: '取消当前任务',
  takeover: '请求手动接管'
};
export function VehicleStateView({ port, showTaskControls = true }: VehicleStateViewProps): ReactElement {
  const unit = useSpeedUnit();
  const view = useVehicleView(port), frame = view.frame;
  const [learning, setLearning] = useState<LearningStatus | null>(null);
  const [learningNote, setLearningNote] = useState('');
  const [learningPending, setLearningPending] = useState(false);
  const [learningUnknown, setLearningUnknown] = useState(false);
  const learningRequest = useRef<string | null>(null);
  const learningAttempt = useRef<string | null>(null);
  const learningSupported = !!port && view.connected && view.available && !!frame?.capabilities.includes('learning.v1');
  useEffect(() => {
    if (!port || !learningSupported) return;
    let closed = false, pending = false;
    async function refresh(): Promise<void> {
      if (!port || pending) return;
      pending = true;
      try {
        const status = await port.learningStatus();
        if (!closed) setLearning(status);
      } catch { /* Keep the last confirmed result; never restart a request. */ }
      finally { pending = false; }
    }
    void refresh();
    const timer = setInterval(() => void refresh(), 1000);
    return () => { closed = true; clearInterval(timer); };
  }, [port, learningSupported]);
  async function queryLearning(): Promise<void> {
    if (!port || learningPending) return;
    setLearningPending(true);
    try {
      const query = learningAttempt.current ? { attempt_id: learningAttempt.current }
        : learningRequest.current ? { start_request_id: learningRequest.current } : {};
      const status = await port.learningStatus(query);
      setLearning(status);
      setLearningUnknown(false);
      setLearningNote('已查询本次状态，没有重新开始校准。');
    } catch { setLearningNote('本次结果仍未确认，请恢复连接后继续查询。'); }
    finally { setLearningPending(false); }
  }
  async function startLearning(): Promise<void> {
    if (!port || !learning?.can_start || learningPending || learningUnknown) return;
    const requestId = newId();
    learningRequest.current = requestId;
    learningAttempt.current = null;
    setLearningPending(true);
    setLearningNote('等待车端接纳…');
    try {
      const receipt = await port.learningStart(requestId, learning.active_revision);
      learningAttempt.current = receipt.attempt_id;
      setLearningNote(receipt.state === 'busy' ? '当前校准仍忙，新请求没有排队。' : '本次已接纳，进度以车端录制和保存状态为准。');
      setLearning(await port.learningStatus({ attempt_id: receipt.attempt_id }));
    } catch {
      setLearningUnknown(true);
      setLearningNote('开始请求结果未确认，请查询本次结果；不会自动再次前进。');
    } finally { setLearningPending(false); }
  }
  const [now, setNow] = useState(() => performance.now());
  useEffect(() => {
    const timer = setInterval(() => setNow(performance.now()), 1000);
    return () => clearInterval(timer);
  }, []);
  if (!frame) return <section className={ui.card}>
    <h3>车端运行状态尚未接通</h3>
    <p>等待车辆报告当前状态与可用操作。</p>
    <p>连接 RTT：{view.rtt_ms === null ? '—' : `${Math.round(view.rtt_ms)} ms`}</p>
  </section>;
  const body = frame.body_reference, wheel = frame.wheel_linear_mps;
  const speedBody = body && (!body.stale || wheel === null) ? body : null;
  const speed = speedBody ? speedBody.average.vx : wheel;
  const lastMeasurement = !view.connected || !view.available || speedBody?.stale;
  const source = speedBody
    ? `${speedBody.source} · ${(speedBody.age_seconds + Math.max(0, now - (view.received_at_ms ?? now)) / 1000).toFixed(2)} 秒前`
    : wheel !== null ? '轮速里程计' : '实际速度测量不可用';
  return <section className={ui.card} aria-label="车辆运行状态">
    <header className={ui.head}>
      <h3>{MODES[frame.mode]}</h3>
      <span>{view.connected && view.available ? (frame.mode === 'unknown' ? '上报在线 · 状态未知' : '车端确认状态') : '最后已知 · 非实时'}</span>
    </header>
    {!view.connected && <p className={ui.warning}>连接中断，任务可能继续；重连后查询原任务，不重新下发。</p>}
    {view.connected && !view.available && <p className={ui.warning}>运行状态通道未更新，显示最后确认状态；连接 RTT 不代表车端已允许操作。</p>}
    <p>{frame.observation && frame.mode === 'unknown' ? AVAILABILITY[frame.observation.status] : frame.reason}</p>
    {frame.observation && <p>最后有效观测：{frame.observation.age_seconds === null ? '尚无' : `${(frame.observation.age_seconds + Math.max(0, now - (view.received_at_ms ?? now)) / 1000).toFixed(2)} 秒前`} · 急停状态{frame.confirmed_estop_status === 'unknown' ? '未知' : '已报告'}</p>}
    <div className={ui.speed}>
      <strong>{speed === null ? '—' : displaySpeed(speed, unit).toFixed(2)}</strong>
      <span>{unit} · {speedBody ? '车体纵向区间平均' : '纵向测量速度'}</span>
    </div>
    <p className={ui.source}>{source}{speed !== null && lastMeasurement ? ' · 已过期 / 最后测量' : ''}</p>
    <p className={ui.connection}>连接 RTT：{view.rtt_ms === null ? '—' : `${Math.round(view.rtt_ms)} ms`} · 仅作连接诊断</p>
    <section className={ui.task} aria-label="固定回波校准">
      <header className={ui.head}><h4>固定回波校准</h4><span>{learningSupported ? '车端状态' : '当前未接通校准能力'}</span></header>
      <p>停稳且无任务时，将以 {displaySpeed(0.05, unit).toFixed(2)} {unit} 前进 3 秒并停车；可随时手动接管。</p>
      <p className={ui.source}>{learning?.active_model
        ? `当前生效：${learning.active_model.point_count} 个真实参考点`
        : learning ? '当前尚无生效模型' : '等待车端报告生效模型'}</p>
      {learning && <p role="status">{learning.durable ? '新结果已保存并生效'
        : learning.phase === 'failed' || learning.phase === 'unconfirmed' ? '本次未完成'
        : learning.compute_complete ? '正在保存新结果'
        : learning.capture_complete ? '已录齐 · 后台计算中'
        : learning.motion_complete ? '前进已结束 · 停车验证中'
        : learning.phase === 'moving' ? '前进并录制'
        : learning.phase === 'sampling' ? '正在录制准备帧' : '等待校准'}</p>}
      {learning && !learning.durable && learning.active_model && <p>新结果成功保存前，沿用上次生效结果。</p>}
      {learning?.capture_complete && !learning.durable && <p>数据已录齐，手动接管后仍继续计算和保存；控制权不等待后台。</p>}
      {learning && !learning.capture_complete && !learning.durable && <p>录齐前接管会结束本次采集，不保存不完整结果，也不会自行继续前进。</p>}
      <div className={ui.actions}>
        <button type="button" disabled={!learningSupported || !learning?.can_start || learningPending || learningUnknown} onClick={() => void startLearning()}>重新校准</button>
        <button type="button" disabled={!learningSupported || learningPending} onClick={() => void queryLearning()}>{learningUnknown ? '查询本次结果' : '刷新校准状态'}</button>
      </div>
      {(learningNote || learningUnknown) && <p role="status">{learningNote}</p>}
      {learning && !learning.can_start && <p>{learning.cannot_start_reason === 'learning_busy' ? '当前校准仍忙，新请求不排队。' : '请先停稳并结束当前任务，再显式开始校准。'}</p>}
    </section>
    {showTaskControls && <TaskExecutionControls port={port} />}
    <details>
      <summary>测量与安全诊断</summary>
      <p>轮速参考：{frame.wheel_linear_mps === null ? '—' : `${displaySpeed(frame.wheel_linear_mps, unit).toFixed(2)} ${unit}`}（轮速里程计）</p>
      <p>急停版本 {frame.estop_revision} · 故障版本 {frame.fault_revision}</p>
    </details>
  </section>;
}

/** Existing execution IDs, versions and capability guards stay in VehiclePort.taskCommand. */
interface TaskExecutionControlsProps {
  port: VehiclePort | undefined;
  compact?: boolean;
  definitionId?: string;
  unmatchedIds?: readonly string[];
}
export function TaskExecutionControls({
  port,
  compact = false,
  definitionId,
  unmatchedIds,
}: TaskExecutionControlsProps): ReactElement | null {
  const view = useVehicleView(port),
    frame = view.frame,
    task = frame?.task;
  const [note, setNote] = useState<string | null>(null),
    [busy, setBusy] = useState(false);
  async function command(action: TaskAction): Promise<void> {
    if (!port || !task) return;
    const current = port.getSnapshot().frame?.task;
    if (current?.task_id !== task.task_id || current.version !== task.version) {
      setNote("任务状态已变化，请核对最新状态");
      return;
    }
    setBusy(true);
    setNote("等待车端回执…");
    try {
      await port.taskCommand(action);
      setNote("车端已应答，执行状态以车端后续更新为准");
    } catch (error) {
      setNote(`操作未完成：${String(error)}`);
    } finally {
      setBusy(false);
    }
  }
  if (!frame || !task) return null;
  const definition = task.definition_task_id ?? task.task_id;
  if (definitionId !== undefined && definition !== definitionId) return null;
  if (unmatchedIds?.includes(definition)) return null;
  return (
    <section
      className={compact ? ui.compactTask : ui.task}
      aria-label="当前执行实例操作"
    >
      <h4>
        当前任务{" "}
        {frame.capabilities.includes("task.names.v1")
          ? (task.name ?? task.task_id)
          : task.task_id}{" "}
        · {TASKS[task.phase]}
      </h4>
      <details>
        <summary>实例详情</summary>
        <p className={ui.source}>
          内部编号 {task.definition_task_id ?? task.task_id} · 执行实例{" "}
          {task.task_id}
        </p>
      </details>
      <p>
        已完成动作 {task.completed} / {task.total}
        {task.stopped_for ? ` · ${task.stopped_for}` : ""}
      </p>
      <div className={ui.actions}>
        {task.allowed_actions.map((action) => (
          <button
            key={action}
            disabled={
              !view.connected ||
              !view.available ||
              busy ||
              !frame.capabilities.includes("task.execution.v1")
            }
            onClick={() => void command(action)}
          >
            {ACTIONS[action]}
          </button>
        ))}
      </div>
      {note && <p role="status">{note}</p>}
    </section>
  );
}
