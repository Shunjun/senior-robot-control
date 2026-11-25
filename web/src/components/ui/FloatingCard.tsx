import type { HTMLAttributes, ReactElement } from 'react';

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  card: [
    "ui-card bg-none bg-card border border-solid border-line rounded-card pt-[19px]",
    "pr-[19px] pb-[19px] pl-[19px] shadow-ui-card backdrop-blur-[15px]"
  ].join(" "),
} as const;

export function FloatingCard({ className = '', ...props }: HTMLAttributes<HTMLElement>): ReactElement {
  return <section {...props} className={`${ui.card} ${className}`} />;
}
