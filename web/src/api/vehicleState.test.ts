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
  it('rejects unsupported capability without sending a request', async () => {
    const sent: string[] = [];
    const port = new VehicleTransport(async type => {
      sent.push(type);
      return {};
    });
    await expect(port.acquire()).rejects.toThrow('未接通');
    expect(sent).toEqual([]);
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
  const port = new VehicleTransport(async () => ({}));
  port.update(frame, true);
  const unavailable = {
    ...frame, sequence: 2, mode: 'unknown', confirmed_estop_status: 'unknown',
    wheel_linear_mps: null, allowed_actions: [],
    observation: { status: 'expired', last_sampled_steady: 10, age_seconds: 1 }
  };
  port.update(unavailable, true);
  expect(port.getSnapshot().available).toBe(true);
  await expect(port.acquire()).rejects.toThrow();
  port.setConnection(false);
  port.setConnection(true);
  port.update(frame, true);
  expect(port.getSnapshot().available).toBe(false);
  port.update({ ...unavailable, sequence: 3 }, true);
  expect(port.getSnapshot().frame?.mode).toBe('unknown');
  await expect(port.acquire()).rejects.toThrow();
  port.update({ ...frame, sequence: 4 }, true);
  expect(port.getSnapshot().frame?.allowed_actions).toEqual(['acquire']);
  expect(() => decodeVehicleFrame({ ...unavailable, allowed_actions: ['acquire'] })).toThrow();
});
