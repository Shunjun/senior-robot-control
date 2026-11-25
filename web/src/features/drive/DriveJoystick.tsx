import {
  useLayoutEffect,
  useRef,
  useState,
  type ReactElement,
  type PointerEvent,
  type KeyboardEvent,
} from "react";
import { Grip } from "lucide-react";
import type { ReleaseReason } from "./Joystick";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  singleStickBox: [
    "drive-single-stick-box [&>.drive-quiet]:absolute [&>.drive-quiet]:w-[1px] [&>.drive-quiet]:h-[1px] [&>.drive-quiet]:overflow-hidden [&>.drive-quiet]:[clip-path:inset(50%)] text-center max-[460.001px]:[grid-column:2]",
    "max-[460.001px]:[grid-row:1/6] max-[460.001px]:self-center"
  ].join(" "),
  singleStick: [
    "drive-single-stick w-[184px] h-[184px] mt-[17px] mr-auto mb-3 ml-auto rounded-full",
    "relative grid place-items-center touch-none select-none cursor-grab border border-solid",
    "border-drive-single-stick-border-color [background:radial-gradient(circle,_#ff874a1c_0_36%,_#ff753209_37%_60%,_transparent_61%),_var(--color-stick)] before:content-[''] [&::before]:absolute [&::before]:bg-none [&::before]:bg-drive-single-stick-background-color [&::before]:pointer-events-none [&::before]:h-[1px]",
    "[&::before]:left-[15px] [&::before]:right-[15px] [&::before]:top-[50%] [&::after]:[content:\"\"] [&::after]:absolute [&::after]:bg-none [&::after]:bg-drive-single-stick-background-color [&::after]:pointer-events-none",
    "[&::after]:w-[1px] [&::after]:top-[15px] [&::after]:bottom-[15px] [&::after]:left-[50%] [&[aria-disabled=true]]:opacity-[0.45] [&[aria-disabled=true]]:cursor-not-allowed min-[1500px]:w-[200px] min-[1500px]:h-[200px]",
    "max-[760.001px]:w-[151px] max-[760.001px]:h-[151px] max-[760.001px]:mt-3 max-[760.001px]:mr-auto max-[760.001px]:mb-3 max-[760.001px]:ml-auto max-[460.001px]:w-[142px] max-[460.001px]:h-[142px]",
    "max-[460.001px]:mt-0 max-[460.001px]:mr-0 max-[460.001px]:mb-0 max-[460.001px]:ml-0"
  ].join(" "),
  axisUp: [
    "drive-axis-up absolute font-normal text-xs leading-[normal] font-mono text-drive-axis-up-color pointer-events-none",
    "top-2"
  ].join(" "),
  axisDown: [
    "drive-axis-down absolute font-normal text-xs leading-[normal] font-mono text-drive-axis-up-color pointer-events-none",
    "bottom-2"
  ].join(" "),
  axisLeft: [
    "drive-axis-left absolute font-normal text-xs leading-[normal] font-mono text-drive-axis-up-color pointer-events-none",
    "left-2.5"
  ].join(" "),
  axisRight: [
    "drive-axis-right absolute font-normal text-xs leading-[normal] font-mono text-drive-axis-up-color pointer-events-none",
    "right-2.5"
  ].join(" "),
  singleKnob: [
    "drive-single-knob w-[53px] h-[53px] rounded-full bg-none bg-amber text-white grid",
    "place-items-center shadow-drive-single-knob pointer-events-none"
  ].join(" "),
  quiet: [
    "drive-quiet text-muted mt-2.5 mr-0 mb-0 ml-0 leading-[1.8] text-xs"
  ].join(" "),
} as const;

export interface DriveJoystickProps {
  disabled: boolean;
  resetToken: number;
  onPress: (linear: number, turn: number) => boolean;
  onChange: (linear: number, turn: number) => void;
  onRelease: (reason: ReleaseReason) => void;
}

function geometry(pad: HTMLDivElement, knob: HTMLSpanElement) {
  const box = pad.getBoundingClientRect();
  const handle = knob.getBoundingClientRect();
  const style = getComputedStyle(pad);
  // Keep the whole handle inside the inner edge of the circular border.
  const border = Math.max(...[
    style.borderTopWidth, style.borderRightWidth,
    style.borderBottomWidth, style.borderLeftWidth,
  ].map((width) => Number.parseFloat(width) || 0));
  const travel = Math.max(0,
    Math.min(box.width, box.height) / 2 - border - Math.max(handle.width, handle.height) / 2,
  );
  return { box, travel };
}
/** 10% entry / 18% exit axis band; 8% center dead-zone, 5% output steps. */
export function DriveJoystick({
  disabled,
  resetToken,
  onPress,
  onChange,
  onRelease,
}: DriveJoystickProps): ReactElement {
  const [value, setValue] = useState({ linear: 0, turn: 0 });
  const [travel, setTravel] = useState(0);
  const pad = useRef<HTMLDivElement>(null);
  const knob = useRef<HTMLSpanElement>(null);
  const pointer = useRef<number | null>(null);
  const keys = useRef(new Set<string>());
  const snaps = useRef({ linear: false, turn: false });
  useLayoutEffect(() => {
    const disc = pad.current, handle = knob.current;
    if (!disc || !handle) return;
    const measure = () => setTravel(geometry(disc, handle).travel);
    measure();
    if (typeof ResizeObserver === "undefined") return;
    const observer = new ResizeObserver(measure);
    observer.observe(disc);
    observer.observe(handle);
    return () => observer.disconnect();
  }, []);
  useLayoutEffect(() => {
    pointer.current = null;
    keys.current.clear();
    snaps.current = { linear: false, turn: false };
    setValue({ linear: 0, turn: 0 });
  }, [resetToken, disabled]);
  function finish(reason: ReleaseReason): void {
    if (pointer.current === null && keys.current.size === 0) return;
    pointer.current = null;
    keys.current.clear();
    snaps.current = { linear: false, turn: false };
    setValue({ linear: 0, turn: 0 });
    onRelease(reason);
  }
  function position(event: PointerEvent<HTMLDivElement>) {
    const { box, travel: radius } = geometry(event.currentTarget, knob.current!);
    setTravel(radius);
    let linear = radius
      ? -(event.clientY - box.top - box.height / 2) / radius
      : 0;
    let turn = radius
      ? -(event.clientX - box.left - box.width / 2) / radius
      : 0;
    const norm = Math.max(1, Math.hypot(linear, turn));
    linear /= norm;
    turn /= norm;
    if (Math.hypot(linear, turn) < 0.08) return { linear: 0, turn: 0 };
    const next = { linear, turn };
    for (const axis of ["linear", "turn"] as const) {
      snaps.current[axis] =
        Math.abs(next[axis]) < (snaps.current[axis] ? 0.18 : 0.1);
      next[axis] = snaps.current[axis]
        ? 0
        : Math.round(next[axis] * 20) / 20 || 0;
    }
    return next;
  }
  const keyAxis: Record<string, [number, number]> = {
    ArrowUp: [1, 0],
    w: [1, 0],
    ArrowDown: [-1, 0],
    s: [-1, 0],
    ArrowLeft: [0, 1],
    a: [0, 1],
    ArrowRight: [0, -1],
    d: [0, -1],
  };
  function keyboardValue() {
    let linear = 0,
      turn = 0;
    keys.current.forEach((key) => {
      const delta = keyAxis[key];
      if (delta) {
        linear += delta[0];
        turn += delta[1];
      }
    });
    const norm = Math.max(1, Math.hypot(linear, turn));
    return { linear: linear / norm, turn: turn / norm };
  }
  function handlePointerDown(event: PointerEvent<HTMLDivElement>): void {
    if (disabled || pointer.current !== null || keys.current.size) return;
    const next = position(event);
    if (!onPress(next.linear, next.turn)) return;
    pointer.current = event.pointerId;
    setValue(next);
    try {
      event.currentTarget.setPointerCapture?.(event.pointerId);
    } catch {
      /* pointer already retired */
    }
  }

  function handlePointerMove(event: PointerEvent<HTMLDivElement>): void {
    if (pointer.current !== event.pointerId) return;
    const next = position(event);
    setValue(next);
    onChange(next.linear, next.turn);
  }

  function handleKeyDown(event: KeyboardEvent<HTMLDivElement>): void {
    if (
      disabled ||
      !keyAxis[event.key] ||
      event.repeat ||
      pointer.current !== null ||
      keys.current.has(event.key)
    )
      return;
    event.preventDefault();
    const held = keys.current.size > 0;
    keys.current.add(event.key);
    const next = keyboardValue();
    if (!held && !onPress(next.linear, next.turn)) {
      keys.current.clear();
      return;
    }
    if (held) onChange(next.linear, next.turn);
    setValue(next);
  }

  function handleKeyUp(event: KeyboardEvent<HTMLDivElement>): void {
    if (!keys.current.has(event.key)) return;
    event.preventDefault();
    if (keys.current.size === 1) {
      finish("release");
      return;
    }
    keys.current.delete(event.key);
    const next = keyboardValue();
    setValue(next);
    onChange(next.linear, next.turn);
  }

  // Per-axis rounding can slightly exceed the unit circle; clamp only drawing.
  const visualNorm = Math.max(1, Math.hypot(value.linear, value.turn));
  return (
    <div className={ui.singleStickBox}>
      <div
        ref={pad}
        role="group"
        aria-label="驾驶摇杆"
        aria-describedby="drive-stick-hint"
        aria-disabled={disabled}
        tabIndex={0}
        className={ui.singleStick}
        data-linear={value.linear}
        data-turn={value.turn}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={(event) => {
          if (pointer.current === event.pointerId) finish("release");
        }}
        onPointerCancel={() => finish("cancel")}
        onLostPointerCapture={() => finish("cancel")}
        onBlur={() => finish("blur")}
        onKeyDown={handleKeyDown}
        onKeyUp={handleKeyUp}
      >
        <span className={ui.axisUp}>↑</span>
        <span className={ui.axisDown}>↓</span>
        <span className={ui.axisLeft}>←</span>
        <span className={ui.axisRight}>→</span>
        <span
          ref={knob}
          className={ui.singleKnob}
          style={{
            transform: `translate(${-value.turn * travel / visualNorm}px,${-value.linear * travel / visualNorm}px)`,
          }}
        >
          <Grip size={24} />
        </span>
      </div>
      <p id="drive-stick-hint" className={ui.quiet}>
        按住拖动或方向键 / WASD；松开停车，靠近中轴吸附。
      </p>
    </div>
  );
}
