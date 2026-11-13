import { useEffect, useState, type ReactElement } from 'react';
import type { TaskAction, VehiclePort } from '../../api/vehicleState';
import { useVehicleView } from './useVehicleView';
import styles from './VehicleStateView.module.css';
export interface VehicleStateViewProps {
  port?: VehiclePort;
}
const MODES = {
  idle: '静止待命',
  manual: '手动驾驶',
  task: '任务执行中',
  braking: '制动中',
  obstacle_hold: '方向障碍受限',
  feedback_stop: '反馈停车',
  execution_fault: '执行故障',
  estop: '急停已锁存'
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
export function VehicleStateView({ port }: VehicleStateViewProps): ReactElement {
  const view = useVehicleView(port), frame = view.frame;
  const [now, setNow] = useState(() => performance.now());
  useEffect(() => {
    const timer = setInterval(() => setNow(performance.now()), 1000);
    return () => clearInterval(timer);
  }, []);
  const [note, setNote] = useState<string | null>(null), [busy, setBusy] = useState(false);
  if (!frame) return <section className={styles.card}>
    <h3>车端运行状态尚未接通</h3>
    <p>等待车辆报告当前状态与可用操作。</p>
    <p>连接 RTT：{view.rtt_ms === null ? '—' : `${Math.round(view.rtt_ms)} ms`}</p>
  </section>;
  const body = frame.body_reference, task = frame.task;
  async function command(action: TaskAction): Promise<void> {
    if (!port || !task) return;
    const current = port.getSnapshot().frame?.task;
    if (current?.task_id !== task.task_id || current.version !== task.version) {
      setNote('任务状态已变化，请核对最新状态');
      return;
    }
    setBusy(true);
    setNote('等待车端回执…');
    try {
      await port.taskCommand(action);
      setNote('车端已应答，执行状态以车端后续更新为准');
    }
    catch (error) {
      setNote(`操作未完成：${String(error)}`);
    }
    finally {
      setBusy(false);
    }
  }
  return <section className={styles.card} aria-label="车辆运行状态">
    <header className={styles.head}>
      <h3>{MODES[frame.mode]}</h3>
      <span>{view.connected && view.available ? '车端确认状态' : '最后已知 · 非实时'}</span>
    </header>
    {!view.connected && <p className={styles.warning}>连接中断，任务可能继续；重连后查询原任务，不重新下发。</p>}
    {view.connected && !view.available && <p className={styles.warning}>运行状态通道未更新，显示最后确认状态；连接 RTT 不代表车端已允许操作。</p>}
    <p>{frame.reason}</p>
    <div className={styles.speed}>
      <strong>{body ? (body.average.vx * 3.6).toFixed(2) : '—'}</strong>
      <span>km/h · 车体纵向区间平均</span>
    </div>
    <p className={styles.source}>{body ? `${body.source} · ${(body.age_seconds + Math.max(0, now - (view.received_at_ms ?? now)) / 1000).toFixed(2)} 秒前${body.stale || !view.connected || !view.available ? ' · 已过期 / 最后测量' : ''}` : '独立车体速度不可用'}</p>
    <p className={styles.connection}>连接 RTT：{view.rtt_ms === null ? '—' : `${Math.round(view.rtt_ms)} ms`} · 仅作连接诊断</p>
    {task && <div className={styles.task}>
      <h4>当前任务 {task.task_id} · {TASKS[task.phase]}</h4>
      <p>已完成动作 {task.completed} / {task.total}{task.stopped_for ? ` · ${task.stopped_for}` : ''}</p>
      <div className={styles.actions}>{task.allowed_actions.map(action => <button key={action} disabled={!view.connected || !view.available || busy || !frame.capabilities.includes('task.execution.v1')} onClick={() => void command(action)}>{ACTIONS[action]}</button>)}</div>
    </div>}
    {note && <p role="status">{note}</p>}
    <details>
      <summary>测量与安全诊断</summary>
      <p>轮速参考：{frame.wheel_linear_mps === null ? '—' : `${frame.wheel_linear_mps.toFixed(2)} m/s`}（不替代主速度）</p>
      <p>急停版本 {frame.estop_revision} · 故障版本 {frame.fault_revision}</p>
    </details>
  </section>;
}
