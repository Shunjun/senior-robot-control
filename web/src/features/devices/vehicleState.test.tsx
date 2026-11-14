import { act, cleanup, fireEvent, render, screen } from '@testing-library/react';
import { afterEach, expect, it } from 'vitest';
import { runtime, testPort } from '../../api/vehicleState.testUtils';
import { VehicleStateView } from './VehicleStateView';
afterEach(cleanup);
it('keeps disconnected running task as last known and never replaces missing body with wheels', () => {
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
  expect(screen.getByText('—', { selector: 'strong' })).toBeTruthy();
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
