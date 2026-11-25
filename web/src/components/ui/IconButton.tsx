import type { ReactElement } from 'react';
import { Button, type ButtonProps } from './Button';
export interface IconButtonProps extends ButtonProps { label: string }
export function IconButton({ label, children, ...props }: IconButtonProps): ReactElement {
  return <Button {...props} aria-label={label} title={label}>{children}</Button>;
}
