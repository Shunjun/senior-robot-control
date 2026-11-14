import { VehicleTransport, type VehicleFrame } from './vehicleState';
export function runtime(overrides: Partial<VehicleFrame> = {}): VehicleFrame {
  return {
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
    capabilities: ['runtime.v1', 'motion.intent.v1', 'settings.v1', 'task.execution.v1'],
    ...overrides
  };
}
export function testPort(send: (type: string, payload: unknown) => Promise<unknown> = async () => ({ authority_id: 'authority', state: 'prepared' })) {
  const port = new VehicleTransport(send);
  port.update(runtime(), true);
  return port;
}
export const settingsFixture = {
  snapshot: {
    driving: {
      low_linear_mps: 5 / 9,
      high_linear_mps: 25 / 18,
      low_angular_rps: 1,
      high_angular_rps: 1,
      reverse_linear_mps: 0.3
    },
    logging: {
      level: 'INFO' as const,
      retention_seconds: 604800,
      debug_enabled: false,
      debug_restore_seconds: 300
    },
    driving_version: 2,
    logging_version: 7,
    debug_original_level: null,
    debug_deadline_utc: null
  },
  capabilities: {
    driving_defaults: {
      low_linear_mps: 5 / 9,
      high_linear_mps: 25 / 18,
      low_angular_rps: 1,
      high_angular_rps: 1,
      reverse_linear_mps: 0.3
    },
    driving_minimum: {
      low_linear_mps: 0,
      high_linear_mps: 0,
      low_angular_rps: 0,
      high_angular_rps: 0,
      reverse_linear_mps: 0
    },
    driving_maximum: {
      low_linear_mps: 2,
      high_linear_mps: 2,
      low_angular_rps: 2,
      high_angular_rps: 2,
      reverse_linear_mps: 1
    },
    logging_defaults: {
      level: 'INFO' as const,
      retention_seconds: 604800,
      debug_enabled: false,
      debug_restore_seconds: 300
    },
    retention_min_seconds: 3600,
    retention_max_seconds: 2592000,
    debug_restore_min_seconds: 60,
    debug_restore_max_seconds: 3600
  }, can_apply_driving: true,
};
