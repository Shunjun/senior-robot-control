import { useEffect, useMemo, useRef, useState, type ReactElement } from 'react';
import type { RobotLink } from '../../api/client';
import type { RobotSnapshot } from '../../api/protocol';
import { useVehicleView } from '../devices/useVehicleView';
import { Joystick } from './Joystick';
import { ManualController, type ManualPhase } from './ManualController';
import styles from './Drive.module.css';
export interface DriveViewProps {
  robotId: string;
  link: RobotLink;
  snapshot: RobotSnapshot | null;
  active?: boolean;
  compact?: boolean;
}
export function DriveView({ robotId, link, snapshot, active = true, compact = false }: DriveViewProps): ReactElement {
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
  const supported = view.frame?.capabilities.includes('motion.intent.v1') === true;
  const enabled = active && view.available && view.connected && snapshot?.stale !== true && supported;
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
  function change(axis: 'linear' | 'turn', value: number): void {
    values.current[axis] = value;
    const { linear, turn } = values.current;
    if (controller?.phase === 'prepared') controller.begin(linear, turn, gear);
    else controller?.update(linear, turn, gear);
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
        active: '手动会话进行中'
      })[phase]}</span>
    </header>
    {!supported && <p className={styles.lock}>方向力度控制尚未接通，请等待车辆连接就绪。</p>}
    <div className={styles.leaseGroup}>
      <button className={styles.primary} disabled={!enabled || !view.frame?.allowed_actions.includes('acquire') || phase !== 'none'} onClick={() => {
        setError(null);
        void controller?.prepare();
      }}>获取控制权</button>
      <button className={styles.ghost} disabled={phase === 'none'} onClick={release}>释放控制权</button>
    </div>
    <div className={styles.leaseGroup} role="group" aria-label="档位">{(['low', 'high'] as const).map(value => <button key={value} className={gear === value ? styles.primary : styles.ghost} aria-pressed={gear === value} onClick={() => {
      setGear(value);
      controller?.update(values.current.linear, values.current.turn, value);
    }}>{value === 'low' ? '低档' : '高档'}</button>)}</div>
    <div className={styles.pads}>
      <Joystick label="前进后退摇杆" axis="vertical" hint="按住前进 / 后退，松开归零" disabled={!enabled || !['prepared', 'active'].includes(phase)} resetToken={resetToken} onChange={value => change('linear', value)} onRelease={release} />
      <Joystick label="左转右转摇杆" axis="horizontal" hint="按住转向，松开归零" disabled={!enabled || !['prepared', 'active'].includes(phase)} resetToken={resetToken} onChange={value => change('turn', value)} onRelease={release} />
    </div>
    <p className={styles.quiet}>两档只表达方向与力度。松杆关闭本次会话，再次驾驶需重新获取控制。</p>
    {error && <p role="alert" className={styles.failure}>{error}</p>}
  </section>;
}
