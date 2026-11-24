import { useEffect, useMemo, useRef, useState, type ReactElement } from 'react';
import type { RobotLink } from '../../api/client';
import type { RobotSnapshot } from '../../api/protocol';
import { useMeasuredSpeed, useVehicleView } from '../devices/useVehicleView';
import type { ReleaseReason } from './Joystick';
import { DriveJoystick } from './DriveJoystick';
import { ManualController, type ManualPhase } from './ManualController';
import { displaySpeed, useSpeedUnit } from '../../display/speedUnits';
import styles from './Drive.module.css';
export interface DriveViewProps {
  robotId: string;
  link: RobotLink;
  snapshot: RobotSnapshot | null;
  active?: boolean;
  compact?: boolean;
}
export function DriveView({ robotId, link, active = true, compact = false }: DriveViewProps): ReactElement {
  const unit = useSpeedUnit();
  const view = useVehicleView(link.vehicle);
  const [phase, setPhase] = useState<ManualPhase>('none');
  const [error, setError] = useState<string | null>(null);
  const [resetToken, reset] = useState(0);
  const [gear, setGear] = useState<'low' | 'high'>('low');
  const values = useRef({ linear: 0, turn: 0 });
  const controller = useMemo(() => link.vehicle ? new ManualController(link.vehicle, (next, issue) => {
    setPhase(next);
    if (issue) setError(issue);
  }) : null, [link, robotId]);
  const measured = useMeasuredSpeed(view);
  const enabled = active && view.manual_connected;
  useEffect(() => {
    if (!enabled) {
      controller?.release();
      values.current = { linear: 0, turn: 0 };
      reset(n => n + 1);
    }
  }, [controller, enabled]);
  useEffect(() => {
    if (view.frame?.manual_authority?.state === 'closed') {
      controller?.observeClosed(view.frame.manual_authority.authority_id);
    }
  }, [controller, view.frame?.manual_authority]);
  useEffect(() => {
    controller?.attach();
    const stop = () => {
      controller?.release();
      values.current = { linear: 0, turn: 0 };
      reset(n => n + 1);
    };
    const hidden = () => {
      if (document.visibilityState === 'hidden') stop();
    };
    window.addEventListener('blur', stop);
    window.addEventListener('pagehide', stop);
    document.addEventListener('visibilitychange', hidden);
    return () => {
      window.removeEventListener('blur', stop);
      window.removeEventListener('pagehide', stop);
      document.removeEventListener('visibilitychange', hidden);
      controller?.dispose();
    };
  }, [controller]);
  useEffect(() => {
    if (phase === 'none') {
      values.current = { linear: 0, turn: 0 };
      reset(n => n + 1);
    }
  }, [phase]);
  function change(linear: number, turn: number): void {
    values.current = {linear,turn};controller?.update(linear,turn,gear);
  }
  function press(linear: number, turn: number): boolean {
    if (controller?.phase === 'prepared') { if(!controller.begin(linear,turn,gear)) return false; }
    else if(controller?.phase === 'active') controller.update(linear,turn,gear);
    else return false;
    values.current={linear,turn};return true;
  }
  function stopGesture(reason: ReleaseReason): void {
    if (reason === 'blur' || reason === 'hidden') {
      release();
      return;
    }
    controller?.stopGesture();
    values.current = { linear: 0, turn: 0 };
    reset(n => n + 1);
  }
  function release(): void {
    controller?.release();
    values.current = { linear: 0, turn: 0 };
    reset(n => n + 1);
  }
  return <section className={styles.drive} data-compact={compact} aria-label="手动驾驶">
    <header className={styles.driveHead}>
      <h3>手动操纵</h3>
      <span className={styles.leaseTag}>{({
        none: '未获取控制',
        preparing: '等待车端准备',
        prepared: '已准备 · 等待新按压',
        active: '手动会话进行中',
        stopping: '正在停车 · 保留控制权'
      })[phase]}</span>
    </header>
    <p className={styles.measurement}>{measured === null ? "不可用" : displaySpeed(measured, unit).toFixed(2)} <small>{unit} · 真实测量</small></p>
    <div className={styles.leaseGroup}>
      <button className={styles.primary} disabled={!enabled || phase !== 'none'} onClick={() => {
        setError(null);
        void controller?.prepare();
      }}>获取控制权</button>
      <button className={styles.ghost} disabled={phase === 'none'} onClick={release}>释放控制权</button>
    </div>
    <div className={styles.leaseGroup} role="group" aria-label="档位">{(['low', 'high'] as const).map(value => <button key={value} className={gear === value ? styles.primary : styles.ghost} aria-pressed={gear === value} onClick={() => {
      setGear(value);
      controller?.update(values.current.linear, values.current.turn, value);
    }}>{value === 'low' ? '低速' : '高速'}</button>)}</div>
    <DriveJoystick disabled={!enabled || !['prepared','active'].includes(phase)} resetToken={resetToken} onPress={press} onChange={change} onRelease={stopGesture} />
    <p className={styles.quiet}>两档只表达方向与力度。松杆停车并保留控制权，再次按压即可继续驾驶。</p>
    {error && <p role="alert" className={styles.failure}>{error}</p>}
  </section>;
}
