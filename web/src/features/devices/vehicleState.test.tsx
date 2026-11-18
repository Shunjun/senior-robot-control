import { act, cleanup, fireEvent, render, screen } from '@testing-library/react';
import { afterEach, expect, it } from 'vitest';
import { runtime, testPort } from '../../api/vehicleState.testUtils';
import { VehicleStateView } from './VehicleStateView';
afterEach(cleanup);
it('keeps disconnected wheel speed and running task as last measurements', () => {
  const port = testPort();
  port.update(runtime({
    sequence: 2, mode: 'task', task: {
      task_id: 'task-1',
      version: 3,
      phase: 'running',
      completed: 2,
      total: 5,
      stopped_for: null,
      allowed_actions: ['pause']
    }
  }), true);
  port.setConnection(false);
  render(<VehicleStateView port={port} />);
  expect(screen.getByText('1.44', { selector: 'strong' })).toBeTruthy();
  expect(screen.getByText('轮速里程计 · 已过期 / 最后测量')).toBeTruthy();
  expect(screen.getByText('最后已知 · 非实时')).toBeTruthy();
  expect(screen.getByText(/任务可能继续/)).toBeTruthy();
  expect(screen.getByText(/task-1 · 运行中/)).toBeTruthy();
  expect((screen.getByRole('button', { name: '暂停当前任务' }) as HTMLButtonElement).disabled).toBe(true);
});
it('requests explicit resume without projecting local task completion or mode changes', async () => {
  const requests: unknown[] = [];
  const port = testPort(async (_type, payload) => {
    requests.push(payload);
    const request = payload as {
      request_id: string;
      task_id: string;
    };
    return {
      request_id: request.request_id,
      task_id: request.task_id,
      accepted: true,
      reason: 'accepted',
      version: 5
    };
  });
  port.update(runtime({
    sequence: 2, task: {
      task_id: 'paused',
      version: 4,
      phase: 'paused',
      completed: 2,
      total: 5,
      stopped_for: 'takeover',
      allowed_actions: ['resume']
    }
  }), true);
  render(<VehicleStateView port={port} />);
  await act(async () => fireEvent.click(screen.getByRole('button', { name: '明确继续任务' })));
  expect(requests[0]).toMatchObject({ action: 'resume', task_id: 'paused', expected_version: 4 });
  expect(screen.getByText(/paused · 已暂停/)).toBeTruthy();
  expect(screen.queryByRole('button', { name: /replan/ })).toBeNull();
});
it('shows missing runtime as an online report with unknown safety and speed', () => {
  const port = testPort();
  port.update(runtime({
    sequence: 2, mode: 'unknown', confirmed_estop_status: 'unknown',
    wheel_linear_mps: null, allowed_actions: [],
    observation: { status: 'missing', last_sampled_steady: null, age_seconds: null }
  }), true);
  render(<VehicleStateView port={port} />);
  expect(screen.getByText('运行状态不可用')).toBeTruthy();
  expect(screen.getByText('上报在线 · 状态未知')).toBeTruthy();
  expect(screen.getByText('尚未收到有效运行状态')).toBeTruthy();
  expect(screen.getByText(/急停状态未知/)).toBeTruthy();
  expect(screen.getByText('—', { selector: 'strong' })).toBeTruthy();
});

it('shows measured wheel speed with its source when the body estimate is absent', () => {
  const port = testPort();
  port.update(runtime({ sequence: 2, body_reference: null, wheel_linear_mps: -0.5 }), true);
  render(<VehicleStateView port={port} />);
  expect(screen.getByText('-1.80', { selector: 'strong' })).toBeTruthy();
  expect(screen.getByText('轮速里程计')).toBeTruthy();
  expect(screen.queryByText(/已过期 \/ 最后测量/)).toBeNull();
});
it('shows a measured zero wheel speed as zero instead of missing', () => {
  const port = testPort();
  port.update(runtime({ sequence: 2, body_reference: null, wheel_linear_mps: 0 }), true);
  render(<VehicleStateView port={port} />);
  expect(screen.getByText('0.00', { selector: 'strong' })).toBeTruthy();
  expect(screen.getByText('轮速里程计')).toBeTruthy();
});
