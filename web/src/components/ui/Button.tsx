import type { ButtonHTMLAttributes, ReactElement } from "react";

const variants = {
  primary: "bg-amber border-amber text-white enabled:hover:bg-amber-deep",
  outline: "bg-transparent border-release-line text-release-ink enabled:hover:bg-amber-wash",
  quiet: "bg-control border-line text-ink enabled:hover:bg-control-hover",
} as const;

export interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: keyof typeof variants;
  busy?: boolean;
}
export function Button({ variant = "quiet", busy = false, disabled, className = "", children, ...props }: ButtonProps): ReactElement {
  return <button type="button" {...props} disabled={disabled || busy} aria-busy={busy}
    className={`inline-flex items-center justify-center gap-2 rounded-control border px-[13px] py-[9px] cursor-pointer disabled:opacity-[.36] disabled:cursor-not-allowed [&_svg]:size-4 [&_svg]:shrink-0 ${variants[variant]} ${className}`}>
    {children}
  </button>;
}
