import type { ButtonHTMLAttributes, ReactElement } from 'react';
import styles from './Ui.module.css';
export interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'primary' | 'outline' | 'quiet';
  busy?: boolean;
}
export function Button({ variant = 'quiet', busy = false, disabled, className = '', children, ...props }: ButtonProps): ReactElement {
  return <button type="button" {...props} disabled={disabled || busy} aria-busy={busy} className={`${styles.button} ${styles[variant]} ${className}`}>{children}</button>;
}
