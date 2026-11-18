import { describe, expect, it } from 'vitest';
import { decodeVehicleFrame, VehicleTransport } from './vehicleState';

export const frame = {
  manual_authority: null,
  schema: 1,
  sequence: 1,
  mode: 'idle',
  reason: 'standby',
  estop_revision: 7,
  fault_revision: 9,
  confirmed_estop_status: 'clear',
  body_reference: null,
  wheel_linear_mps: 0.4,
  task: null,
  allowed_actions: ['acquire'],
  capabilities: ['motion.intent.v1', 'settings.v1', 'task.execution.v1'],
};

describe('vehicle transport boundary', () => {
  it('strictly decodes independent body state and never substitutes wheel speed', () => {
    expect(decodeVehicleFrame(frame).body_reference).toBeNull();
    expect(() => decodeVehicleFrame({ ...frame, mode: 'made-up' })).toThrow();
    expect(() => decodeVehicleFrame({ ...frame, sequence: NaN })).toThrow();
    expect(() => decodeVehicleFrame({ ...frame, allowed_actions: ['replan_task'] })).toThrow();
  });
  it('acquires through the independent control connection without a report', async () => {
    const sent: string[] = [];
    const port = new VehicleTransport(async type => {
      sent.push(type);
      return { authority_id: 'a', state: 'prepared' };
    });
    expect(await port.acquire()).toEqual({ authority_id: 'a', state: 'prepared' });
    expect(sent).toEqual(['control.acquire']);
  });
  it('preserves confirmed state on disconnect and rejects old safety revisions', () => {
    const port = new VehicleTransport(async () => ({}));
    port.update(frame, true);
    port.update({ ...frame, sequence: 2, estop_revision: 6 }, true);
    expect(port.getSnapshot().frame?.sequence).toBe(1);
    port.setConnection(false);
    expect(port.getSnapshot().frame?.mode).toBe('idle');
    expect(port.getSnapshot().connected).toBe(false);
  });
});

it('keeps unavailable reports live across reconnect without restoring old permissions', async () => {
  const port = new VehicleTransport(async () => ({ authority_id: 'a', state: 'prepared' }));
  port.update(frame, true);
  const unavailable = {
    ...frame, sequence: 2, mode: 'unknown', confirmed_estop_status: 'unknown',
    wheel_linear_mps: null, allowed_actions: [],
    observation: { status: 'missing', last_sampled_steady: 10, age_seconds: 1 }
  };
  port.update(unavailable, true);
  expect(port.getSnapshot().available).toBe(true);
  expect((await port.acquire()).authority_id).toBe('a');
  port.setConnection(false);
  port.setConnection(true);
  port.update(frame, true);
  expect(port.getSnapshot().available).toBe(false);
  port.update({ ...unavailable, sequence: 3 }, true);
  expect(port.getSnapshot().frame?.mode).toBe('unknown');
  expect((await port.acquire()).authority_id).toBe('a');
  port.update({ ...frame, sequence: 4 }, true);
  expect(port.getSnapshot().frame?.allowed_actions).toEqual(['acquire']);
  expect(() => decodeVehicleFrame({ ...unavailable, allowed_actions: ['acquire'] })).toThrow();
});

it('sends a scoped gesture stop separately from releasing authority', async () => {
  const sent: [string, unknown][] = [];
  const port = new VehicleTransport(async (type, payload) => { sent.push([type, payload]); });
  await port.stop('a', 3);
  await port.release('a');
  expect(sent).toEqual([
    ['drive.stop', { authority_id: 'a', gesture: 3 }],
    ['control.release', { authority_id: 'a' }]
  ]);
  await expect(port.stop('a', 0)).rejects.toThrow();
  await expect(port.stop('', 1)).rejects.toThrow();
  expect(sent).toHaveLength(2);
});

it('keeps calibration start identity and status query on distinct explicit requests', async () => {
  const sent: unknown[] = [];
  const status = {
    run_id: 'b'.repeat(32), active_revision: 1, active_model: null, attempt_id: 'a'.repeat(32),
    phase: 'validating', reason: 'post_motion_validation', motion_complete: true,
    capture_complete: true, compute_complete: false, durable: false, can_start: false,
    cannot_start_reason: 'learning_busy'
  };
  const port = new VehicleTransport(async (type, payload, requestId) => {
    sent.push({ type, payload, requestId });
    return type === 'learning.start'
      ? { start_request_id: requestId, attempt_id: 'a'.repeat(32), state: 'busy' } : status;
  });
  port.update({ ...frame, capabilities: ['runtime.v1', 'learning.v1'],
    manual_authority: { authority_id: 'prepared', state: 'prepared' } }, true);
  expect((await port.learningStart('original-start', 1)).state).toBe('busy');
  expect(await port.learningStatus({ start_request_id: 'original-start' })).toEqual(status);
  expect(sent).toEqual([
    { type: 'learning.start', requestId: 'original-start', payload: { expected_active_revision: 1, expected_authority_id: 'prepared' } },
    { type: 'learning.status', requestId: undefined, payload: { start_request_id: 'original-start' } }
  ]);
});
