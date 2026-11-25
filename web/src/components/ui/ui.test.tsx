import { cleanup, fireEvent, render, screen } from '@testing-library/react';
import { afterEach, expect, it, vi } from 'vitest';
import { Switch } from './Switch';
import { SegmentedControl } from './SegmentedControl';
afterEach(cleanup);
it('keeps switches controlled and exposes their state and disabled semantics', () => {
  const change = vi.fn();
  const page = render(<Switch label="摄像头" checked={false} onChange={change} />);
  fireEvent.click(screen.getByRole('switch', { name: '摄像头' }));
  expect(change).toHaveBeenCalledWith(true);
  expect(screen.getByRole('switch').getAttribute('aria-checked')).toBe('false');
  page.rerender(<Switch label="摄像头" checked disabled onChange={change} />);
  fireEvent.click(screen.getByRole('switch'));
  expect(change).toHaveBeenCalledTimes(1);
});
it('reports segment selection without storing another value', () => {
  const change = vi.fn();
  render(<SegmentedControl label="视图" value="control" options={[{ value: 'control', label: '控制' }, { value: 'logs', label: '日志' }]} onChange={change} />);
  fireEvent.click(screen.getByRole('button', { name: '日志' }));
  expect(change).toHaveBeenCalledWith('logs');
  expect(screen.getByRole('button', { name: '控制' }).getAttribute('aria-pressed')).toBe('true');
});
