import type { ReactElement, ReactNode } from 'react';
import styles from './Ui.module.css';
export interface SegmentedControlProps<T extends string> { label: string; value: T; options: readonly { value: T; label: ReactNode }[]; onChange: (value: T) => void; disabled?: boolean }
export function SegmentedControl<T extends string>({ label, value, options, onChange, disabled }: SegmentedControlProps<T>): ReactElement {
  return <div className={styles.segmented} role="group" aria-label={label}>{options.map(option => <button type="button" key={option.value} disabled={disabled} aria-pressed={option.value === value} onClick={() => onChange(option.value)}>{option.label}</button>)}</div>;
}
