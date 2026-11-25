import type { ReactElement } from 'react';

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  switchLabel: [
    "ui-switch-label border border-solid border-line rounded-control text-ink cursor-pointer",
    "inline-flex items-center justify-center gap-2.5 whitespace-nowrap text-[11px] bg-none bg-switch-surface",
    "pt-[9px] pr-3 pb-[9px] pl-3 [&:disabled]:opacity-[0.36] [&:disabled]:cursor-not-allowed [&[data-compact=true]]:pt-0 [&[data-compact=true]]:pr-0",
    "[&[data-compact=true]]:pb-0 [&[data-compact=true]]:pl-0 [&[data-compact=true]]:border-0 [&[data-compact=true]]:border-none [&[data-compact=true]]:border-current [&[data-compact=true]]:bg-none [&[data-compact=true]]:bg-transparent [&[data-compact=true]>span:first-child]:absolute",
    "[&[data-compact=true]>span:first-child]:w-[1px] [&[data-compact=true]>span:first-child]:h-[1px] [&[data-compact=true]>span:first-child]:overflow-hidden [&[data-compact=true]>span:first-child]:[clip-path:inset(50%)] max-[760.001px]:pt-[7px] max-[760.001px]:pr-[9px] max-[760.001px]:pb-[7px] max-[760.001px]:pl-[9px]",
    "max-[760.001px]:text-[10px]"
  ].join(" "),
  switchTrack: [
    "ui-switch-track relative w-[32px] h-[18px] rounded-full bg-none bg-switch-off flex-none",
    "[&>span]:absolute [&>span]:top-[50%] [&>span]:left-[3px] [&>span]:-translate-y-1/2 [&>span]:w-[12px] [&>span]:h-[12px] [&>span]:rounded-full [&>span]:bg-none",
    "[&>span]:bg-white [&>span]:shadow-ui-switch-track [&[data-checked=true]]:bg-none [&[data-checked=true]]:bg-amber [&[data-checked=true]>span]:left-[17px]"
  ].join(" "),
} as const;

export interface SwitchProps {
  label: string;
  checked: boolean;
  onChange: (checked: boolean) => void;
  disabled?: boolean;
  busy?: boolean;
  compact?: boolean;
}
export function Switch({
  label,
  checked,
  onChange,
  disabled = false,
  busy = false,
  compact = false,
}: SwitchProps): ReactElement {
  return (
    <button
      type="button"
      data-compact={compact}
      role="switch"
      aria-label={label}
      aria-checked={checked}
      aria-busy={busy}
      disabled={disabled || busy}
      className={ui.switchLabel}
      onClick={() => onChange(!checked)}
    >
      <span>{label}</span>
      <span className={ui.switchTrack} data-checked={checked}>
        <span />
      </span>
    </button>
  );
}
