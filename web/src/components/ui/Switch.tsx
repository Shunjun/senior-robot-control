import type { ReactElement } from 'react';
import styles from './Ui.module.css';
export interface SwitchProps { label: string; checked: boolean; onChange: (checked: boolean) => void; disabled?: boolean; busy?: boolean }
export function Switch({ label, checked, onChange, disabled = false, busy = false }: SwitchProps): ReactElement {
  return <button type="button" role="switch" aria-label={label} aria-checked={checked} aria-busy={busy} disabled={disabled || busy} className={styles.switchLabel} onClick={() => onChange(!checked)}>
    <span>{label}</span><span className={styles.switchTrack} data-checked={checked}><span /></span>
  </button>;
}
