import { useEffect, useLayoutEffect, useId, useRef, useState, type KeyboardEvent, type PointerEvent, type ReactElement } from "react";


// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  stickBox: "grid gap-1.5",
  stick: "relative touch-none select-none rounded-xl border border-line bg-stick",
  stickVertical: "w-[110px] h-[190px]",
  stickHorizontal: "w-[190px] h-[110px]",
  track: "absolute inset-3 rounded-md border border-line",
  thumb: "absolute size-12 left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 rounded-full bg-amber",
  stickHint: "text-xs text-ink-dim",
} as const;

/** Why a stick let go. The drive view passes it on as the reason for the stop. */
export type ReleaseReason = "release" | "cancel" | "blur" | "hidden";

export interface JoystickProps {
  /** Accessible name, e.g. 前进后退摇杆. */
  label: string;
  axis: "vertical" | "horizontal";
  /** Visible instruction for the keyboard and pointer. Linked with aria-describedby. */
  hint: string;
  disabled: boolean;
  resetToken: number;
  /** A real press edge; false leaves the stick unheld. */
  onPress: (value: number) => boolean;
  /** Held at `value` in [-1, 1]. Positive is forward (vertical) or a left turn (horizontal). */
  onChange: (value: number) => void;
  onRelease: (reason: ReleaseReason) => void;
}

const KEYS: Record<JoystickProps["axis"], Record<string, number>> = {
  vertical: { ArrowUp: 1, ArrowDown: -1 },
  horizontal: { ArrowLeft: 1, ArrowRight: -1 },
};

function clamp(value: number): number {
  return Math.max(-1, Math.min(1, value));
}

/**
 * One thumb, one axis. Springs back to zero: the value exists only while a pointer or an arrow
 * key holds it. Focus loss, a hidden page and a cancelled pointer all count as letting go.
 */
export function Joystick({
  label,
  axis,
  hint,
  disabled,
  resetToken,
  onPress,
  onChange,
  onRelease,
}: JoystickProps): ReactElement {
  const hintId = useId();
  const [value, setValue] = useState(0);
  const pointer = useRef<number | null>(null);
  const key = useRef<string | null>(null);
  const callbacks = useRef({ onPress, onChange, onRelease });
  callbacks.current = { onPress, onChange, onRelease };

  function held(): boolean {
    return pointer.current !== null || key.current !== null;
  }

  function letGo(reason: ReleaseReason): void {
    if (!held()) return;
    pointer.current = null;
    key.current = null;
    setValue(0);
    callbacks.current.onRelease(reason);
  }

  const letGoRef = useRef(letGo);
  letGoRef.current = letGo;

  useEffect(() => {
    const onBlur = () => letGoRef.current("blur");
    const onVisibility = () => {
      if (document.visibilityState === "hidden") letGoRef.current("hidden");
    };
    window.addEventListener("blur", onBlur);
    document.addEventListener("visibilitychange", onVisibility);
    window.addEventListener("pagehide", onBlur);
    return () => {
      window.removeEventListener("blur", onBlur);
      document.removeEventListener("visibilitychange", onVisibility);
      window.removeEventListener("pagehide", onBlur);
    };
  }, []);

  useLayoutEffect(() => {
    pointer.current = null;
    key.current = null;
    setValue(0);
  }, [resetToken]);

  // A stick that becomes disabled while held has already been stopped by the drive view.
  useEffect(() => {
    if (disabled && held()) {
      pointer.current = null;
      key.current = null;
      setValue(0);
    }
  }, [disabled]);

  function fromPointer(event: PointerEvent<HTMLDivElement>): number {
    const rect = event.currentTarget.getBoundingClientRect();
    if (axis === "vertical") {
      const half = rect.height / 2;
      return half > 0 ? clamp(-(event.clientY - (rect.top + half)) / half) : 0;
    }
    const half = rect.width / 2;
    return half > 0 ? clamp(-(event.clientX - (rect.left + half)) / half) : 0;
  }

  function hold(raw: number): void {
    // Steps of 5%: a finger sweeping the stick makes tens of commands, not hundreds.
    const next = Math.round(raw * 20) / 20 || 0;
    setValue(next);
    callbacks.current.onChange(next);
  }

  function onPointerDown(event: PointerEvent<HTMLDivElement>): void {
    if (disabled || pointer.current !== null) return;
    const next = Math.round(fromPointer(event) * 20) / 20 || 0;
    if (!callbacks.current.onPress(next)) return;
    pointer.current = event.pointerId;
    setValue(next);
    try {
      event.currentTarget.setPointerCapture?.(event.pointerId);
    } catch {
      // The pointer may already be gone; the stick still follows the events it gets.
    }
  }

  function onPointerMove(event: PointerEvent<HTMLDivElement>): void {
    if (pointer.current !== event.pointerId) return;
    hold(fromPointer(event));
  }

  function onPointerUp(event: PointerEvent<HTMLDivElement>): void {
    if (pointer.current !== event.pointerId) return;
    letGo("release");
  }

  function onPointerCancel(event: PointerEvent<HTMLDivElement>): void {
    if (pointer.current !== event.pointerId) return;
    letGo("cancel");
  }

  function onKeyDown(event: KeyboardEvent<HTMLDivElement>): void {
    const next = KEYS[axis][event.key];
    if (next === undefined || disabled) return;
    event.preventDefault();
    // Auto-repeat is not a new press; the drive view resends on its own clock.
    if (event.repeat || key.current === event.key) return;
    if (!callbacks.current.onPress(next)) return;
    key.current = event.key;
    setValue(next);
  }

  function onKeyUp(event: KeyboardEvent<HTMLDivElement>): void {
    if (key.current !== event.key) return;
    letGo("release");
  }

  const offset = `${50 - value * 50}%`;
  return (
    <div className={ui.stickBox}>
      <div
        role="group"
        aria-label={label}
        aria-describedby={hintId}
        aria-disabled={disabled}
        tabIndex={0}
        className={`${ui.stick} ${axis === "vertical" ? ui.stickVertical : ui.stickHorizontal}`}
        data-value={value.toFixed(2)}
        data-held={value !== 0}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerCancel}
        onLostPointerCapture={(event) => {
          if (pointer.current === event.pointerId) letGo("cancel");
        }}
        onKeyDown={onKeyDown}
        onKeyUp={onKeyUp}
        onBlur={() => letGo("blur")}
      >
        <span aria-hidden="true" className={ui.track} />
        <span
          aria-hidden="true"
          className={ui.thumb}
          style={axis === "vertical" ? { top: offset } : { left: offset }}
        />
      </div>
      <p id={hintId} className={ui.stickHint}>
        {hint}
      </p>
    </div>
  );
}
