import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { afterEach, expect, it } from 'vitest';
import { runtime, testPort } from '../../api/vehicleState.testUtils';
import { VehicleTransport, type LearningStatus } from '../../api/vehicleState';
import { VehicleStateView, TaskExecutionControls } from './VehicleStateView';
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
  expect(screen.getByText('0.40', { selector: 'strong' })).toBeTruthy();
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
  expect(screen.getByText('-0.50', { selector: 'strong' })).toBeTruthy();
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


it('queries the original uncertain calibration and retains the old model through manual takeover', async () => {
  let next: LearningStatus = {
    run_id: 'b'.repeat(32), active_revision: 1,
    active_model: { attempt_id: 'c'.repeat(32), model_id: 'd'.repeat(32), point_count: 72 },
    attempt_id: 'c'.repeat(32), phase: 'completed', reason: 'classified_body_returns',
    motion_complete: true, capture_complete: true, compute_complete: true, durable: true,
    can_start: true, cannot_start_reason: null
  };
  let starts = 0, original = '';
  const queries: unknown[] = [];
  const port = new VehicleTransport(async (type, payload, requestId) => {
    if (type === 'learning.start') {
      starts++;
      original = requestId!;
      throw new Error('timeout');
    }
    queries.push(payload);
    return next;
  });
  port.update(runtime({ capabilities: ['runtime.v1', 'learning.v1'] }), true);
  render(<VehicleStateView port={port} />);
  const button = screen.getByRole('button', { name: '重新校准' });
  await waitFor(() => expect((button as HTMLButtonElement).disabled).toBe(false));
  await act(async () => fireEvent.click(button));
  expect((button as HTMLButtonElement).disabled).toBe(true);
  next = { ...next, phase: 'validating', durable: false, compute_complete: false,
    attempt_id: 'a'.repeat(32), can_start: false, cannot_start_reason: 'learning_busy' };
  await act(async () => fireEvent.click(screen.getByRole('button', { name: '查询本次结果' })));
  expect(queries.at(-1)).toEqual({ start_request_id: original });
  expect(screen.getByText('已录齐 · 后台计算中')).toBeTruthy();
  expect(screen.getByText('当前生效：72 个真实参考点')).toBeTruthy();
  act(() => port.update(runtime({ sequence: 2, mode: 'manual', capabilities: ['runtime.v1', 'learning.v1'] }), true));
  expect(screen.getByRole('heading', { name: '手动驾驶' })).toBeTruthy();
  next = { ...next, phase: 'completed', compute_complete: true, durable: true,
    active_revision: 2, active_model: { attempt_id: 'a'.repeat(32), model_id: 'e'.repeat(32), point_count: 96 },
    can_start: true, cannot_start_reason: null };
  await act(async () => fireEvent.click(screen.getByRole('button', { name: '刷新校准状态' })));
  expect(screen.getByText('当前生效：96 个真实参考点')).toBeTruthy();
  expect(screen.getByRole('heading', { name: '手动驾驶' })).toBeTruthy();
  expect(starts).toBe(1);
});

it('runs one real execution through pause, resume and cancel with the current version', async () => {
  const requests:Array<{action:string;task_id:string;expected_version:number}>=[];
  const port=testPort(async (_type,payload)=>{const q=payload as typeof requests[number] & {request_id:string};requests.push(q);return {request_id:q.request_id,task_id:q.task_id,accepted:true,reason:'accepted',version:q.expected_version+1};});
  const report=(sequence:number,version:number,phase:'running'|'paused',action:'pause'|'resume'|'cancel')=>port.update(runtime({sequence,task:{task_id:'execution-id',version,phase,completed:1,total:3,stopped_for:null,allowed_actions:[action]}}),true);
  report(2,3,'running','pause');
  render(<TaskExecutionControls port={port} />);
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'暂停当前任务'})));
  expect(screen.getByText(/execution-id · 运行中/)).toBeTruthy();
  act(()=>report(3,4,'paused','resume'));
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'明确继续任务'})));
  act(()=>report(4,5,'running','cancel'));
  await act(async()=>fireEvent.click(screen.getByRole('button',{name:'取消当前任务'})));
  expect(requests.map(q=>[q.action,q.task_id,q.expected_version])).toEqual([['pause','execution-id',3],['resume','execution-id',4],['cancel','execution-id',5]]);
});
