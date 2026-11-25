import type { ReactElement, ReactNode } from "react";

export interface SegmentedControlProps<T extends string> {
  label: string;
  value: T;
  options: readonly { value: T; label: ReactNode }[];
  onChange: (value: T) => void;
  disabled?: boolean;
  variant?: "neutral" | "accent";
}
export function SegmentedControl<T extends string>({
  label,
  value,
  options,
  onChange,
  disabled,
  variant = "neutral",
}: SegmentedControlProps<T>): ReactElement {
  return (
    <div
      className="flex gap-1 rounded-md bg-segmented p-[3px]"
      role="group"
      aria-label={label}
    >
      {options.map((option) => (
        <button
          type="button"
          key={option.value}
          className={`inline-flex items-center justify-center gap-2 rounded-control border-0 px-[11px] py-1.5 text-[11px] whitespace-nowrap cursor-pointer disabled:opacity-[.36] disabled:cursor-not-allowed ${option.value === value ? variant === "accent" ? "bg-amber text-white" : "bg-selected text-selected-ink" : "bg-transparent text-ink"}`}
          disabled={disabled}
          aria-pressed={option.value === value}
          onClick={() => onChange(option.value)}
        >
          {option.label}
        </button>
      ))}
    </div>
  );
}
