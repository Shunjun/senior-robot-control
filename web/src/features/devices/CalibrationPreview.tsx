import { useEffect, useState, type ReactElement } from 'react';
import type { VehicleFrame } from '../../api/vehicleState';
import styles from './VehicleStateView.module.css';

// Temporary local preview. No port, network transport or vehicle command is used here.
export const CALIBRATION_PREVIEW_FRAME: VehicleFrame = {
  schema: 1,
  sequence: 1,
  mode: 'idle',
  reason: '本地模拟仪表 · 未连接车辆',
  estop_revision: 0,
  fault_revision: 0,
  confirmed_estop_status: 'clear',
  body_reference: null,
  wheel_linear_mps: 0,
  task: null,
  allowed_actions: [],
  capabilities: ['runtime.v1'],
  manual_authority: null
};

type Phase = 'ready' | 'moving' | 'validating' | 'computing' | 'saving' | 'saved' | 'interrupted' | 'timeout';
interface PreviewModel {
  version: number;
  points: number;
}
interface CalibrationPreviewProps {
  onFrameChange: (frame: VehicleFrame) => void;
}
const TITLES: Record<Phase, string> = {
  ready: '停稳待校准',
  moving: '前进并录制',
  validating: '已停车 · 验证录制中',
  computing: '已录齐 · 后台计算中',
  saving: '正在保存新结果',
  saved: '新结果已生效',
  interrupted: '采集未完成',
  timeout: '开始请求结果未确认'
};

export function CalibrationPreview({ onFrameChange }: CalibrationPreviewProps): ReactElement {
  const [phase, setPhase] = useState<Phase>('ready');
  const [seconds, setSeconds] = useState(3);
  const [model, setModel] = useState<PreviewModel | null>({ version: 1, points: 72 });
  const [manual, setManual] = useState(false);
  const [note, setNote] = useState('');
  const busy = phase === 'moving' || phase === 'validating' || phase === 'computing' || phase === 'saving';

  useEffect(() => {
    if (phase !== 'moving') return;
    const until = performance.now() + 3000;
    const timer = setInterval(() => {
      const remaining = Math.max(0, Math.ceil((until - performance.now()) / 1000));
      setSeconds(remaining);
      if (remaining === 0) {
        clearInterval(timer);
        setPhase('validating');
        onFrameChange(CALIBRATION_PREVIEW_FRAME);
      }
    }, 100);
    return () => clearInterval(timer);
  }, [phase, onFrameChange]);

  function start(): void {
    if (busy || phase === 'timeout') return;
    setSeconds(3);
    setManual(false);
    setNote('本地模拟已接纳本次校准。');
    setPhase('moving');
    onFrameChange({ ...CALIBRATION_PREVIEW_FRAME, mode: 'task', wheel_linear_mps: 0.05 });
  }
  function takeOver(): void {
    setManual(true);
    onFrameChange({ ...CALIBRATION_PREVIEW_FRAME, mode: 'manual' });
    if (phase === 'moving' || phase === 'validating') {
      setPhase('interrupted');
      setNote('手动已接管。数据尚未录齐，本次不保存新结果，也不会自行继续前进。');
    } else {
      setNote('手动已接管，当前模拟停稳。已录齐的数据继续计算和保存，不等待控制权。');
    }
  }
  function save(): void {
    if (phase !== 'saving') return;
    setModel(previous => ({ version: (previous?.version ?? 0) + 1, points: 96 }));
    setPhase('saved');
    setNote('模拟持久化成功，生效模型一次替换；没有再次发起运动。');
  }
  function timeout(): void {
    setPhase('timeout');
    setNote('未收到开始回执，不能确定本次进度。请查询本次结果，不重新下发开始。');
    onFrameChange({ ...CALIBRATION_PREVIEW_FRAME, mode: 'unknown', wheel_linear_mps: null });
  }
  function query(): void {
    setPhase('computing');
    setNote('查询到本次已经录齐并停稳，后台仍在计算。没有发送新的开始请求。');
    onFrameChange(CALIBRATION_PREVIEW_FRAME);
  }
  function reset(withModel = true): void {
    setPhase('ready');
    setManual(false);
    setSeconds(3);
    setNote('');
    setModel(withModel ? { version: 1, points: 72 } : null);
    onFrameChange(CALIBRATION_PREVIEW_FRAME);
  }

  return <section className={styles.task} aria-label="固定回波校准预览">
    <header className={styles.head}>
      <h4>固定回波校准</h4>
      <span>模拟交互</span>
    </header>
    <p className={styles.warning} role="note">仅本地预览 · 所有仪表、校准及接管均为模拟，不发送任何车辆请求。</p>
    <p>停稳且无任务时，将以 0.05 m/s 前进 3 秒并停车；可随时手动接管。</p>
    <p className={styles.source}>
      {model ? `当前生效：模拟结果 ${model.version} · ${model.points} 个参考点` : '当前尚无生效模型（模拟）'}
    </p>
    <p role="status" aria-live="polite">
      <strong>{TITLES[phase]}</strong>{phase === 'moving' ? ` · 剩余 ${seconds} 秒` : ''}
      {manual && ' · 手动控制'}
    </p>
    {phase === 'validating' && <p>前进已结束，控制权不等待计算；停车验证尚未录齐。</p>}
    {(phase === 'computing' || phase === 'saving') && <p>运动与停车验证数据已录齐，后台继续处理；手动驾驶不受等待影响。</p>}
    {(busy || phase === 'timeout' || phase === 'interrupted') && <p>{model ? '新结果成功保存前，沿用上次生效结果。' : '本次成功保存前，不显示已获得新模型。'}</p>}
    <div className={styles.actions}>
      <button type="button" disabled={busy || phase === 'timeout'} onClick={start}>重新校准</button>
      {phase === 'timeout' && <button type="button" onClick={query}>查询本次结果</button>}
    </div>
    {note && <p role="status">{note}</p>}
    <details open>
      <summary>预览场景操作 · 仅模拟</summary>
      <p>等待三秒自动停车，再模拟录齐、计算和保存。接管按钮可分别体验录齐前后的区别。</p>
      <div className={styles.actions}>
        <button type="button" disabled={!busy || manual} onClick={takeOver}>模拟手动接管</button>
        <button type="button" disabled={phase !== 'validating'} onClick={() => {
          setPhase('computing');
          setNote('模拟已录齐停车验证；现在接管也会继续保存。');
        }}>模拟录齐停车验证</button>
        <button type="button" disabled={phase !== 'computing'} onClick={() => setPhase('saving')}>模拟计算完成</button>
        <button type="button" disabled={phase !== 'saving'} onClick={save}>模拟保存成功</button>
        <button type="button" disabled={!busy} onClick={() => setNote('当前校准仍忙。重复请求仍指向本次；新的请求被拒绝，没有排队。')}>模拟重复点击</button>
        <button type="button" disabled={busy || phase === 'timeout'} onClick={timeout}>模拟开始回执超时</button>
        <button type="button" onClick={() => reset()}>重置演示</button>
        <button type="button" onClick={() => reset(false)}>暂无旧模型场景</button>
      </div>
    </details>
  </section>;
}
