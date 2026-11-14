import { useEffect, useRef, useState, type ReactElement } from 'react';
import { CommandError, describeCommandError, type RobotLink } from '../../api/client';
import { useVehicleView } from '../devices/useVehicleView';
import styles from './Drive.module.css';
export interface EmergencyStopProps {
  link: RobotLink;
  compact?: boolean;
}
export function EmergencyStop({ link, compact = false }: EmergencyStopProps): ReactElement {
  const { frame, connected, available } = useVehicleView(link.vehicle);
  const [pending, setPending] = useState(false), [note, setNote] = useState<string | null>(null);
  const generation = useRef(0);
  useEffect(() => () => {
    generation.current++;
  }, [link]);
  async function press(): Promise<void> {
    const mine = ++generation.current;
    setPending(true);
    setNote(null);
    try {
      const request = link.request('safety.estop', {});
      link.vehicle?.fenceManual();
      await request;
      if (mine === generation.current) setNote('车端已应答，等待车端状态确认');
    }
    catch (error) {
      if (mine === generation.current) setNote(`急停未确认：${error instanceof CommandError ? describeCommandError(error.code) : '无法发送'}。请立即使用车上实体急停`);
    }
    finally {
      if (mine === generation.current) setPending(false);
    }
  }
  const confirmed = frame?.confirmed_estop_status;
  const resetAllowed = frame?.allowed_actions.includes('estop_reset') === true;
  async function recover(kind: 'estop_reset' | 'fault_recover'): Promise<void> {
    try {
      await link.vehicle?.recover(kind);
      setNote('恢复请求已应答，等待车端状态确认');
    } catch (error) {
      setNote(String(error));
    }
  }
  return <div className={styles.estopDock} data-compact={compact}>
    {confirmed === 'hardware_latched' && <p className={styles.estopFailure}>实体急停已锁存，请现场释放；网页无法解除</p>}
    {confirmed === 'software_latched' && <p className={styles.estopConfirmed}>软件急停已锁存</p>}
    {pending && <p role="status">急停等待确认</p>}
    {note && <p role={note.startsWith('急停未确认') ? 'alert' : 'status'}>{note}</p>}
    <button className={styles.estop} aria-label="急停" onClick={() => void press()}>
      <span className={styles.estopCap}>急停</span>
      <span className={styles.estopTag}>E-STOP</span>
    </button>
    {confirmed !== 'hardware_latched' && (confirmed === 'software_latched' || resetAllowed) && <>
      <button className={styles.ghost} disabled={!connected || !available || !resetAllowed} onClick={() => void recover('estop_reset')}>复位软件急停</button>
      {confirmed === 'software_latched' && !resetAllowed && <p className={styles.quiet}>车端尚未许可复位，等待运动证据有效并确认停稳。</p>}
    </>}
    {frame?.allowed_actions.includes('fault_recover') && <button className={styles.ghost} disabled={!connected || !available} onClick={() => void recover('fault_recover')}>恢复执行故障</button>}
  </div>;
}
