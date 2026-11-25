import type { HTMLAttributes, ReactElement } from 'react';
import styles from './Ui.module.css';
export function FloatingCard({ className = '', ...props }: HTMLAttributes<HTMLElement>): ReactElement {
  return <section {...props} className={`${styles.card} ${className}`} />;
}
