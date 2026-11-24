import { useLayoutEffect, useRef, useState, type ReactElement, type PointerEvent } from 'react';
import { Move } from 'lucide-react';
import type { ReleaseReason } from './Joystick';
import styles from './Drive.module.css';
export interface DriveJoystickProps {
  disabled: boolean; resetToken: number;
  onPress: (linear: number, turn: number) => boolean;
  onChange: (linear: number, turn: number) => void;
  onRelease: (reason: ReleaseReason) => void;
}
/** 10% entry / 18% exit axis band; 8% center dead-zone, 5% output steps. */
export function DriveJoystick({ disabled, resetToken, onPress, onChange, onRelease }: DriveJoystickProps): ReactElement {
  const [value, setValue] = useState({linear:0,turn:0});
  const pointer = useRef<number | null>(null);
  const keys = useRef(new Set<string>());
  const snaps = useRef({linear:false,turn:false});
  useLayoutEffect(() => { pointer.current=null;keys.current.clear();snaps.current={linear:false,turn:false};setValue({linear:0,turn:0}); }, [resetToken, disabled]);
  function finish(reason: ReleaseReason): void {
    if (pointer.current === null && keys.current.size === 0) return;
    pointer.current=null;keys.current.clear();snaps.current={linear:false,turn:false};setValue({linear:0,turn:0});onRelease(reason);
  }
  function position(event: PointerEvent<HTMLDivElement>) {
    const box = event.currentTarget.getBoundingClientRect();
    let linear=box.height ? -(event.clientY-box.top-box.height/2)/(box.height/2) : 0;
    let turn=box.width ? -(event.clientX-box.left-box.width/2)/(box.width/2) : 0;
    const norm=Math.max(1,Math.hypot(linear,turn));linear/=norm;turn/=norm;
    if (Math.hypot(linear,turn) < .08) return {linear:0,turn:0};
    const next={linear,turn};
    for (const axis of ['linear','turn'] as const) {
      snaps.current[axis]=Math.abs(next[axis]) < (snaps.current[axis] ? .18 : .10);
      next[axis]=snaps.current[axis] ? 0 : Math.round(next[axis]*20)/20 || 0;
    }
    return next;
  }
  const keyAxis: Record<string,[number,number]>={ArrowUp:[1,0],w:[1,0],ArrowDown:[-1,0],s:[-1,0],ArrowLeft:[0,1],a:[0,1],ArrowRight:[0,-1],d:[0,-1]};
  function keyboardValue() {
    let linear=0,turn=0;
    keys.current.forEach(key => { const delta=keyAxis[key];if(delta){linear+=delta[0];turn+=delta[1];} });
    const norm=Math.max(1,Math.hypot(linear,turn));return {linear:linear/norm,turn:turn/norm};
  }
  return <div className={styles.singleStickBox}><div role="group" aria-label="驾驶摇杆" aria-describedby="drive-stick-hint" aria-disabled={disabled} tabIndex={0}
    className={styles.singleStick} data-linear={value.linear} data-turn={value.turn}
    onPointerDown={event => {
      if(disabled || pointer.current!==null || keys.current.size) return;
      const next=position(event);if(!onPress(next.linear,next.turn)) return;
      pointer.current=event.pointerId;setValue(next);
      try { event.currentTarget.setPointerCapture?.(event.pointerId); } catch { /* pointer already retired */ }
    }} onPointerMove={event => { if(pointer.current!==event.pointerId) return; const next=position(event);setValue(next);onChange(next.linear,next.turn); }}
    onPointerUp={event => { if(pointer.current===event.pointerId) finish('release'); }} onPointerCancel={() => finish('cancel')} onLostPointerCapture={() => finish('cancel')}
    onBlur={() => finish('blur')}
    onKeyDown={event => {
      if(disabled || !keyAxis[event.key] || event.repeat || pointer.current!==null || keys.current.has(event.key)) return;
      event.preventDefault();const held=keys.current.size>0;keys.current.add(event.key);const next=keyboardValue();
      if(!held && !onPress(next.linear,next.turn)){keys.current.clear();return;}
      if(held) onChange(next.linear,next.turn);setValue(next);
    }} onKeyUp={event => { if(!keys.current.has(event.key)) return;event.preventDefault();if(keys.current.size===1){finish('release');return;}keys.current.delete(event.key);const next=keyboardValue();setValue(next);onChange(next.linear,next.turn); }}>
      <span className={styles.singleKnob} style={{ transform:`translate(${-value.turn*65}px,${-value.linear*65}px)` }}><Move size={24} /></span>
    </div><p className={styles.intent}>操控意图 · 前后 {Math.round(value.linear*100)}% · 转向 {Math.round(value.turn*100)}%</p><p id="drive-stick-hint" className={styles.quiet}>按住拖动或方向键 / WASD；松开停车，靠近中轴吸附。</p></div>;
}
