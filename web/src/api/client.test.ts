import { afterEach, expect, it, vi } from 'vitest';
import { RobotClient, type SocketLike } from './client';
import { runtime } from './vehicleState.testUtils';
import { parseInbound, type ResponsePayload } from './protocol';
import { ManualController } from '../features/drive/ManualController';
class Socket implements SocketLike {
  readyState = 0;
  bufferedAmount = 0;
  sent: Record<string, unknown>[] = [];
  onopen: (() => void) | null = null;
  onmessage: ((event: {
    data: unknown;
  }) => void) | null = null;
  onclose: ((event: {
    code: number;
  }) => void) | null = null;
  onerror: (() => void) | null = null;
  send(data: string): void {
    this.sent.push(JSON.parse(data) as Record<string, unknown>);
  }
  close(): void {
    this.readyState = 3;
  }
  receive(value: unknown): void {
    this.onmessage?.({ data: JSON.stringify(value) });
  }
}
async function connected() {
  const socket = new Socket(), client = new RobotClient({ socketFactory: () => socket, requestTimeoutMs: 120000, staleAfterMs: 120000 });
  const opening = client.connect('robot');
  socket.readyState = 1;
  socket.onopen?.();
  await opening;
  socket.receive(snapshot());
  return { client, socket };
}
function snapshot(epoch = 3) {
  return {
    v: 1,
    robot_id: 'robot',
    session_id: 'cloud',
    connection_epoch: epoch,
    request_id: 'snapshot',
    seq: 1,
    expires_at: Date.now() / 1000 + 10,
    type: 'snapshot',
    payload: {
      connected: true,
      localized: true,
      map_ref: null,
      home: null,
      task_revision: 0,
      runnable_task_ids: [],
      execution_status: 'idle',
      collected_at: 1000,
      stale: false,
      last_synced_at: 1000,
      home_revision: 0,
      vehicle_ui: runtime()
    }
  };
}
function respond(socket: Socket, sent: Record<string, unknown>, data: unknown) {
  socket.receive({ ...sent, type: 'response', payload: { request_type: sent.type, ok: true, data } });
}
afterEach(() => vi.useRealTimers());
it('uses the existing authenticated envelope, keeps RTT diagnostic, and never sends timing challenges', async () => {
  vi.useFakeTimers();
  const { client, socket } = await connected();
  try {
    const stop = client.request('safety.estop', {});
    const sent = socket.sent.at(-1)!;
    await vi.advanceTimersByTimeAsync(1200);
    respond(socket, sent, undefined);
    await stop;
    expect(client.vehicle.getSnapshot().rtt_ms).not.toBeNull();
    const acquire = client.vehicle.acquire();
    const request = socket.sent.at(-1)!;
    expect(request).toMatchObject({ type: 'control.acquire', robot_id: 'robot', connection_epoch: 3 });
    respond(socket, request, { authority_id: 'prepared', state: 'prepared' });
    expect(await acquire).toEqual({ authority_id: 'prepared', state: 'prepared' });
    expect(socket.sent.map(item => item.type)).toEqual(['safety.estop', 'control.acquire']);
    const intent = client.vehicle.intent({
      authority_id: 'prepared',
      gesture: 1,
      sequence: 1,
      linear: 1,
      turn: -0.5,
      gear: 'high'
    });
    const frame = socket.sent.at(-1)!;
    expect(frame.payload).toEqual({
      authority_id: 'prepared',
      gesture: 1,
      sequence: 1,
      linear: 1,
      turn: -0.5,
      gear: 'high'
    });
    respond(socket, frame, {});
    await intent;
  } finally {
    client.disconnect();
  }
});
it('ignores mismatched session replies and accepts the matching prepared receipt', async () => {
  const { client, socket } = await connected();
  try {
    let received = false;
    const pending = client.vehicle.acquire().then(value => {
      received = true;
      return value;
    });
    const request = socket.sent.at(-1)!;
    respond(socket, { ...request, session_id: 'other' }, { authority_id: 'wrong', state: 'prepared' });
    await Promise.resolve();
    expect(received).toBe(false);
    respond(socket, request, { authority_id: 'correct', state: 'prepared' });
    expect((await pending).authority_id).toBe('correct');
  } finally {
    client.disconnect();
  }
});

it('does not send a new route when the vehicle has not advertised its capability', async () => {
  const { client, socket } = await connected();
  try {
    client.vehicle.update(runtime({ sequence: 2, capabilities: ['runtime.v1'] }), true);
    await expect(client.request('settings.get', {})).rejects.toMatchObject({ code: 'unavailable' });
    expect(socket.sent).toEqual([]);
  } finally {
    client.disconnect();
  }
});

it('does not append a motion sample behind a buffered socket', async () => {
  const { client, socket } = await connected();
  try {
    socket.bufferedAmount = 100;
    await expect(client.vehicle.intent({
      authority_id: 'a',
      gesture: 1,
      sequence: 1,
      linear: 1,
      turn: 0,
      gear: 'low'
    })).rejects.toMatchObject({ name: 'IntentBackpressureError' });
    expect(socket.sent).toEqual([]);
  } finally {
    client.disconnect();
  }
});


it('keeps held input during backpressure, sends only current effort after draining, and drops released input', async () => {
  vi.useFakeTimers();
  const { client, socket } = await connected();
  const controller = new ManualController(client.vehicle, () => {});
  try {
    const preparing = controller.prepare();
    respond(socket, socket.sent.at(-1)!, { authority_id: 'a', state: 'prepared' });
    await preparing;
    socket.bufferedAmount = 100;
    controller.begin(0.1, 0, 'low');
    await vi.advanceTimersByTimeAsync(0);
    expect(controller.phase).toBe('active');
    controller.update(0.4, 0, 'low');
    controller.update(0.9, -0.2, 'high');
    await vi.advanceTimersByTimeAsync(400);
    expect(socket.sent.map(item => item.type)).toEqual(['control.acquire']);
    socket.bufferedAmount = 0;
    await vi.advanceTimersByTimeAsync(100);
    expect(socket.sent.filter(item => item.type === 'drive.intent')).toHaveLength(1);
    const latest = socket.sent.at(-1)!;
    expect(latest.payload).toMatchObject({ authority_id: 'a', linear: 0.9, turn: -0.2, gear: 'high' });
    respond(socket, latest, {});
    await vi.advanceTimersByTimeAsync(0);
    socket.bufferedAmount = 100;
    controller.update(-1, 0, 'low');
    await vi.advanceTimersByTimeAsync(0);
    controller.release();
    respond(socket, socket.sent.at(-1)!, {});
    socket.bufferedAmount = 0;
    await vi.advanceTimersByTimeAsync(500);
    expect(controller.phase).toBe('none');
    expect(socket.sent.filter(item => item.type === 'drive.intent')).toHaveLength(1);
  } finally {
    controller.dispose();
    client.disconnect();
  }
});

it('retires manual input when agent epoch changes on the same browser socket without canceling tasks', async () => {
  vi.useFakeTimers();
  const { client, socket } = await connected();
  const controller = new ManualController(client.vehicle, () => {});
  try {
    const preparing = controller.prepare();
    respond(socket, socket.sent.at(-1)!, { authority_id: 'old', state: 'prepared' });
    await preparing;
    controller.begin(1, 0, 'low');
    respond(socket, socket.sent.at(-1)!, {});
    await vi.advanceTimersByTimeAsync(0);
    const next = snapshot(4);
    next.payload.vehicle_ui = runtime({ sequence: 2, mode: 'task', task: {
      task_id: 'nav', version: 1, phase: 'running', completed: 0, total: 1,
      stopped_for: null, allowed_actions: ['pause']
    } });
    socket.receive(next);
    expect(controller.phase).toBe('none');
    const boundary = socket.sent.length;
    controller.update(0.7, 0, 'high');
    await vi.advanceTimersByTimeAsync(500);
    expect(socket.sent.slice(boundary)).toEqual([]);
    expect(socket.sent.some(item => item.type === 'task.execution.command' || item.type === 'nav.cancel')).toBe(false);
    expect(client.vehicle.getSnapshot().frame?.task?.phase).toBe('running');
    expect(socket.sent.filter(item => item.type === 'drive.intent').every(item => item.connection_epoch === 3)).toBe(true);
  } finally {
    controller.dispose();
    client.disconnect();
  }
});

it('still closes held input on a real car rejection', async () => {
  vi.useFakeTimers();
  const { client, socket } = await connected();
  const controller = new ManualController(client.vehicle, () => {});
  try {
    const preparing = controller.prepare();
    respond(socket, socket.sent.at(-1)!, { authority_id: 'a', state: 'prepared' });
    await preparing;
    controller.begin(1, 0, 'low');
    const intent = socket.sent.at(-1)!;
    socket.receive({ ...intent, type: 'response', payload: {
      request_type: 'drive.intent', ok: false, error: 'safety_rejected'
    } });
    await vi.advanceTimersByTimeAsync(0);
    expect(controller.phase).toBe('none');
    expect(socket.sent.at(-1)?.type).toBe('control.release');
    respond(socket, socket.sent.at(-1)!, {});
    await vi.advanceTimersByTimeAsync(500);
    expect(socket.sent.filter(item => item.type === 'drive.intent')).toHaveLength(1);
  } finally {
    controller.dispose();
    client.disconnect();
  }
});


const taskState = {
  execution_id: 'execution-1', phase: 'running', epoch: 4,
  completed: ['action-1'], remaining: ['action-2']
};
it.each(['navigation.goal', 'navigation.return_home', 'operation.query'])(
  'accepts serialized %s lifecycle details without claiming completion', async type => {
    const { client, socket } = await connected();
    try {
      const pending = client.request<ResponsePayload>(type, {});
      void pending.catch(() => {});
      const sent = socket.sent.at(-1)!;
      const raw = JSON.stringify({ ...sent, type: 'response', payload: {
        request_type: type, ok: true, status: 'accepted', data: taskState
      } });
      expect(parseInbound(raw)?.payload).toMatchObject({ data: taskState });
      socket.onmessage?.({ data: raw });
      const receipt = await pending;
      expect(receipt.status).toBe('accepted');
      expect(receipt.data).toEqual(taskState);
      expect(receipt.data?.phase).toBe('running');
      expect(client.vehicle.getSnapshot().frame?.task).toBeNull();
    } finally {
      client.disconnect();
    }
  }
);
it('preserves legacy non-task operation query replies without data', async () => {
  const { client, socket } = await connected();
  try {
    const pending = client.request<ResponsePayload>('operation.query', { operation_id: 'legacy' });
    respond(socket, socket.sent.at(-1)!, undefined);
    expect((await pending).data).toBeUndefined();
  } finally {
    client.disconnect();
  }
});
it('strictly restricts lifecycle data shape and request types', () => {
  const envelope = { ...snapshot(), type: 'response' };
  const invalid = [
    { ...taskState, extra: true }, { ...taskState, execution_id: '' },
    { ...taskState, phase: 'accepted' }, { ...taskState, phase: ['running'] },
    { ...taskState, epoch: 1.5 },
    { ...taskState, epoch: Number.MAX_SAFE_INTEGER + 1 },
    { ...taskState, completed: 1 }, { ...taskState, remaining: ['action-1'] }
  ];
  for (const data of invalid) {
    expect(parseInbound(JSON.stringify({ ...envelope, payload: {
      request_type: 'navigation.goal', ok: true, data
    } }))).toBeNull();
  }
  expect(parseInbound(JSON.stringify({ ...envelope, payload: {
    request_type: 'safety.estop', ok: true, data: taskState
  } }))).toBeNull();
  expect(parseInbound(JSON.stringify({ ...envelope, payload: {
    request_type: 'navigation.return_home', ok: false, error: 'conflict', data: taskState
  } }))).toBeNull();
});
