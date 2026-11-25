import { act, cleanup, fireEvent, render, renderHook, screen } from '@testing-library/react';
import { afterEach, expect, it, vi } from 'vitest';
import { runtime, testPort } from '../../api/vehicleState.testUtils';
import { useMeasuredSpeed, useVehicleView } from './useVehicleView';
import { DriveJoystick } from '../drive/DriveJoystick';
afterEach(() => { cleanup();vi.unstubAllGlobals();vi.useRealTimers(); });
it('normalizes both axes, snaps with hysteresis and returns to zero in one press', () => {
  vi.stubGlobal("PointerEvent", MouseEvent);
  const press = vi.fn(() => true), change = vi.fn(), release = vi.fn();
  render(<DriveJoystick disabled={false} resetToken={0} onPress={press} onChange={change} onRelease={release} />);
  const pad = screen.getByRole('group', { name: '驾驶摇杆' });
  vi.spyOn(pad, 'getBoundingClientRect').mockReturnValue({ left:0,top:0,width:200,height:200,right:200,bottom:200,x:0,y:0,toJSON(){} });
  const knob = pad.querySelector('.drive-single-knob')!;
  vi.spyOn(knob, 'getBoundingClientRect').mockReturnValue({ width:53,height:53 } as DOMRect);
  pad.style.border = '1px solid';
  // The 200px disc has 72.5px of legal center travel, not the old 100px radius.
  const move = (x: number, y: number) => ({pointerId:1, clientX:100+x*72.5, clientY:100+y*72.5});
  fireEvent.pointerDown(pad, move(-.6,-.8));
  expect(press).toHaveBeenCalledWith(.8,.6);
  fireEvent.pointerMove(pad,move(-0.07,-0.8));
  expect(change).toHaveBeenLastCalledWith(.8,0);
  fireEvent.pointerMove(pad,move(-0.13,-0.8));
  expect(change).toHaveBeenLastCalledWith(.8,0);
  fireEvent.pointerMove(pad,move(-0.22,-0.8));
  expect(change).toHaveBeenLastCalledWith(.8,.2);
  fireEvent.pointerMove(pad,move(-0.8,-0.03));
  expect(change).toHaveBeenLastCalledWith(0,.8);
  fireEvent.pointerMove(pad,move(0,0));
  expect(change).toHaveBeenLastCalledWith(0,0);
  fireEvent.pointerUp(pad,{pointerId:1});
  expect(release).toHaveBeenCalledWith('release');
});

it('expires an unchanged runtime speed sample and accepts only a newer sequence as fresh', () => {
  vi.useFakeTimers();
  const port=testPort();
  port.update(runtime({sequence:2,wheel_linear_mps:.5}),true);
  const reading=renderHook(()=>useMeasuredSpeed(useVehicleView(port)));
  expect(reading.result.current).toBe(.5);
  act(()=>vi.advanceTimersByTime(5001));
  expect(reading.result.current).toBeNull();
  act(()=>port.update(runtime({sequence:2,wheel_linear_mps:.8}),true));
  expect(reading.result.current).toBeNull();
  act(()=>port.update(runtime({sequence:3,wheel_linear_mps:.5}),true));
  expect(reading.result.current).toBe(.5);
});
