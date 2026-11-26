import { renderHook, act, cleanup } from "@testing-library/react";
import { useCurrentScan } from "../features/map/useCurrentScan";
import { RobotClient, type SocketLike } from "./client";
import type { RobotSnapshot } from "./protocol";
import { parseInbound } from "./protocol";
import { afterEach, describe, expect, it, vi } from 'vitest';
import { decodeVehicleFrame, isTaskName, normalizeTaskName, VehicleTransport } from './vehicleState';

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

it('accepts named execution metadata and rejects FEFF-only or NEL-only names', () => {
  const task={task_id:'exec-88',definition_task_id:'route-01',name:'东区巡检',version:3,phase:'running',completed:1,total:3,stopped_for:null,allowed_actions:['pause']};
  const named={...frame,capabilities:[...frame.capabilities,'task.names.v1'],task};
  expect(decodeVehicleFrame(named).task).toMatchObject(task);
  for (const patch of [
    { name: '\u0085' },
    { name: '\uFEFF' },
  ]) {
    expect(() => decodeVehicleFrame({ ...named, task: { ...task, ...patch } })).toThrow();
  }
});

it('preserves Chinese task summary names and rejects FEFF-only or NEL-only names', () => {
  const envelope={v:1,robot_id:'robot',session_id:'cloud',connection_epoch:1,request_id:'snapshot',seq:1,expires_at:2000,type:'snapshot',payload:{connected:true,localized:true,map_ref:null,home:null,home_revision:0,task_revision:1,runnable_task_ids:[],execution_status:'idle',collected_at:1000,stale:false,last_synced_at:1000,tasks:[{task_id:'route-01',name:'东区巡检',revision:1,enabled:true}]}};
  expect(parseInbound(JSON.stringify(envelope))).toMatchObject({payload:{tasks:[{task_id:'route-01',name:'东区巡检'}]}});
  for (const name of ['\u0085', '\uFEFF']) {
    expect(parseInbound(JSON.stringify({
      ...envelope,
      payload: { ...envelope.payload, tasks: [{ ...envelope.payload.tasks[0], name }] },
    }))).toBeNull();
  }
});

it('uses the agreed boundary whitespace for name validation without filtering internal characters', () => {
  expect(normalizeTaskName(' \u0085东区 \u0085\uFEFF 巡检\uFEFF ')).toBe('东区 \u0085\uFEFF 巡检');
  expect(isTaskName('\u0085')).toBe(false);
  expect(isTaskName('\uFEFF')).toBe(false);
  expect(isTaskName(' \u0085' + '😀'.repeat(128) + '\uFEFF ')).toBe(true);
});

it('decodes the approved scan identity and rejects unsafe or inconsistent display metadata', () => {
  const layer = {
    map_ref: { map_id: 'map', reference_revision: 1 }, map_generation: 'R1',
    backend_boot: 'boot', source_id: 'a'.repeat(64), sequence: 1,
    sampled_at: 10, expires_at: 10.5, remaining_s: .2,
    hit_count: 2048, preview_count: 1, display_mode: 'preview', valid: true,
  };
  const parse = (scan_layer: unknown) => parseInbound(JSON.stringify({
    v: 1, robot_id: 'robot', session_id: 'cloud', connection_epoch: 1,
    request_id: 'scan', seq: 0, expires_at: 2000, type: 'snapshot', payload: {
      connected: true, localized: true, map_ref: layer.map_ref, home: null,
      task_revision: 0, runnable_task_ids: [], execution_status: 'idle', collected_at: 10,
      stale: false, last_synced_at: 10, scan_points: [{ x: 1, y: 2 }], scan_layer,
    },
  }));
  expect(parse(layer)).toMatchObject({ payload: { scan_layer: layer } });
  for (const patch of [
    { sequence: 2 ** 53 }, { remaining_s: .51 }, { expires_at: 11 },
    { source_id: 'guess' }, { preview_count: 2 }, { valid: false }, { frame: 'map' },
  ]) expect(parse({ ...layer, ...patch })).toBeNull();
});

const scanClients: RobotClient[] = [];
afterEach(() => { for (const client of scanClients.splice(0)) client.disconnect(); });

function scanProjection() {
  const socket: SocketLike = { readyState: 1, bufferedAmount: 0, onopen: null,
    onclose: null, onmessage: null, onerror: null, send() {}, close() {} };
  const client = new RobotClient({ socketFactory: () => socket });
  scanClients.push(client);
  void client.connect('robot'); socket.onopen?.();
  socket.onmessage?.({ data: JSON.stringify({ v: 1, type: 'control.connection',
    robot_id: 'robot', connection_epoch: 1 }) });
  let sequence = 0;
  return (value: RobotSnapshot): RobotSnapshot => {
    let projected: RobotSnapshot | undefined;
    const stop = client.subscribe(next => { projected = next; });
    const { robot_id, display_order: _order, ...payload } = value;
    socket.onmessage?.({ data: JSON.stringify({ v: 1, type: 'snapshot', robot_id,
      session_id: 'cloud-cache', connection_epoch: 1, request_id: 'snapshot',
      seq: ++sequence, expires_at: 1000, payload }) });
    stop();
    if (projected === undefined) throw new Error('snapshot was not accepted by original client');
    return projected;
  };
}

const scanSnapshot = () => ({
  robot_id: 'robot', connected: true, localized: true, stale: false,
  map_ref: { map_id: 'map', reference_revision: 1 }, map_generation: 'R1',
  home: null, home_revision: 0, task_revision: 0, runnable_task_ids: [],
  execution_status: 'idle', collected_at: 10, last_synced_at: 10,
  scan_points: [{ x: 1, y: 2 }],
  scan_layer: {
    map_ref: { map_id: 'map', reference_revision: 1 }, map_generation: 'R1',
    backend_boot: 'boot', source_id: 'a'.repeat(64), sequence: 1,
    sampled_at: 10, expires_at: 10.5, remaining_s: .2,
    hit_count: 1, preview_count: 1, display_mode: 'preview' as const, valid: true,
  },
});

it('expires a repeated scan without renewing its local deadline', () => {
  vi.useFakeTimers();
  const project = scanProjection();
  const scan = project(scanSnapshot());
  try {
    const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
      initialProps: { value: scan },
    });
    expect(result.current).toEqual(scan.scan_points);
    act(() => vi.advanceTimersByTime(100));
    rerender({ value: project({ ...scan, scan_layer: { ...scan.scan_layer!, remaining_s: .5 } }) });
    act(() => vi.advanceTimersByTime(101));
    expect(result.current).toEqual([]);
  } finally { cleanup(); vi.useRealTimers(); }
});

it('cancels an older Q timer without clearing the newer Q', () => {
  vi.useFakeTimers();
  const project = scanProjection();
  const scan = project(scanSnapshot());
  try {
    const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
      initialProps: { value: scan },
    });
    act(() => vi.advanceTimersByTime(100));
    const next = { ...scan, scan_layer: { ...scan.scan_layer!, sequence: 2, remaining_s: .5 } };
    rerender({ value: project(next) });
    act(() => vi.advanceTimersByTime(101));
    expect(result.current).toEqual(scan.scan_points);
    act(() => vi.advanceTimersByTime(400));
    expect(result.current).toEqual([]);
  } finally { cleanup(); vi.useRealTimers(); }
});

it('keeps a known source retired', () => {
  const project = scanProjection();
  const scan = project(scanSnapshot());
  const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
    initialProps: { value: scan },
  });
  const late = project({ ...scan, scan_layer: { ...scan.scan_layer!, sequence: 10 } });
  const next = project({ ...scan, scan_layer: { ...scan.scan_layer!, source_id: 'b'.repeat(64) } });
  rerender({ value: next });
  expect(result.current).toEqual(scan.scan_points);
  rerender({ value: late });
  expect(result.current).toEqual([]);
  cleanup();
});

it('retires an old reference before the new reference has Q', () => {
  const project = scanProjection();
  const scan = project(scanSnapshot());
  const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
    initialProps: { value: scan as RobotSnapshot },
  });
  const next = { ...scan, map_generation: 'R2', scan_layer: { ...scan.scan_layer!, map_generation: 'R2' } };
  const { scan_layer: _pendingLayer, ...pending } = next;
  rerender({ value: project(pending) });
  rerender({ value: scan });
  expect(result.current).toEqual([]);
  const admitted = project(next);
  rerender({ value: admitted });
  expect(result.current).toEqual(scan.scan_points);
  rerender({ value: scan });
  expect(result.current).toEqual([]);
  rerender({ value: admitted });
  expect(result.current).toEqual(scan.scan_points);
  cleanup();
});

it('revokes a Q on invalid and never restores that same identity', () => {
  const project = scanProjection();
  const scan = project(scanSnapshot());
  const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
    initialProps: { value: scan },
  });
  rerender({ value: project({ ...scan, scan_points: [], scan_layer: {
    ...scan.scan_layer!, valid: false, preview_count: 0, remaining_s: 0,
  } }) });
  expect(result.current).toEqual([]);
  rerender({ value: scan });
  expect(result.current).toEqual([]);
  cleanup();
});

it('rejects a lower scan sequence without changing the newer lease', () => {
  const project = scanProjection();
  const scan = project(scanSnapshot());
  const next = project({ ...scan, scan_layer: { ...scan.scan_layer!, sequence: 2 } });
  const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
    initialProps: { value: next },
  });
  rerender({ value: project(scan) });
  expect(result.current).toEqual([]);
  rerender({ value: project(next) });
  expect(result.current).toEqual(scan.scan_points);
  cleanup();
});

it('keeps a known backend boot retired', () => {
  const project = scanProjection();
  const scan = project(scanSnapshot());
  const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
    initialProps: { value: scan },
  });
  rerender({ value: project({ ...scan, scan_layer: { ...scan.scan_layer!, backend_boot: 'new' } }) });
  expect(result.current).toEqual(scan.scan_points);
  rerender({ value: scan });
  expect(result.current).toEqual([]);
  cleanup();
});

it('never treats legacy scan points as qualified Q', () => {
  const { scan_layer: _layer, ...legacy } = scanSnapshot();
  const { result } = renderHook(() => useCurrentScan(legacy));
  expect(result.current).toEqual([]);
  cleanup();
});

it('rejects unsafe publication ordinals without changing other envelope integer domains', () => {
  const value = { v: 1, robot_id: 'robot', session_id: 'cloud', connection_epoch: 1,
    request_id: 'snapshot', seq: 2 ** 53, expires_at: 1000, type: 'snapshot',
    payload: scanSnapshot() };
  // Use only wire payload fields, not the client-owned robot id.
  const { robot_id: _robot, ...payload } = value.payload;
  expect(parseInbound(JSON.stringify({ ...value, payload }))).toBeNull();
});

it('ignores an older reference withdrawal without shortening the current Q deadline', () => {
  vi.useFakeTimers();
  const project = scanProjection();
  const raw = scanSnapshot();
  raw.scan_layer.remaining_s = .4;
  const first = project(raw);
  const { scan_layer: _layer, ...noLayer } = raw;
  const oldWithdrawal = project({ ...noLayer, scan_points: [] });
  const next = project({ ...raw, map_generation: 'R2', scan_layer: {
    ...raw.scan_layer, map_generation: 'R2',
  } });
  try {
    const { result, rerender } = renderHook(({ value }) => useCurrentScan(value), {
      initialProps: { value: first },
    });
    rerender({ value: next });
    act(() => vi.advanceTimersByTime(50));
    rerender({ value: oldWithdrawal });
    expect(result.current).toEqual([]);
    rerender({ value: next });
    expect(result.current).toEqual(next.scan_points);
    act(() => vi.advanceTimersByTime(349));
    expect(result.current).toEqual(next.scan_points);
    act(() => vi.advanceTimersByTime(1));
    expect(result.current).toEqual([]);
  } finally { cleanup(); vi.useRealTimers(); }
});
