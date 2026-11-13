import { expect, it, vi } from 'vitest';
import { BrowserVehicleLogs } from './logs';
import type { SocketLike } from './client';
import type { LogQuery } from './vehicleState';
class Socket implements SocketLike {
  readyState = 0;
  bufferedAmount = 0;
  sent: unknown[] = [];
  onopen: (() => void) | null = null;
  onmessage: ((event: { data: unknown }) => void) | null = null;
  onclose: ((event: { code: number }) => void) | null = null;
  onerror: (() => void) | null = null;
  send(raw: string): void { this.sent.push(JSON.parse(raw)); }
  close(): void { this.readyState = 3; }
  receive(data: unknown): void { this.onmessage?.({ data: JSON.stringify({ ok: true, data }) }); }
}
const query: LogQuery = { after: 0, limit: 200, levels: [], module: null, request_id: null, task_id: null, since_utc: null, until_utc: null };
const page = { entries: [], gaps: [], next_cursor: 0, high_water: 0, has_more: false, scanned: 0 };
it('uses independent sockets, validates query and segmented download replies', async () => {
  const sockets: Socket[] = [], urls: string[] = [];
  const logs = new BrowserVehicleLogs(() => 'robot', url => { urls.push(url); const socket = new Socket(); sockets.push(socket); return socket; });
  const pending = logs.query(query);
  const socket = sockets[0]!;
  socket.onopen?.();
  expect(urls[0]).toContain('/ws/logs?robot_id=robot');
  expect(socket.sent).toEqual([{ action: 'query', query }]);
  socket.receive(page);
  await expect(pending).resolves.toEqual(page);
  expect(socket.readyState).toBe(3);
  const download = logs.download(query);
  sockets[1]!.onopen?.();
  sockets[1]!.receive({ data: '{}\n', gaps: [], next_cursor: 4, has_more: true });
  await expect(download).resolves.toMatchObject({ next_cursor: 4, has_more: true });
});
it('pause closes follow and resume starts from the supplied cursor', () => {
  const sockets: Socket[] = [];
  const logs = new BrowserVehicleLogs(() => 'robot', () => { const socket = new Socket(); sockets.push(socket); return socket; });
  const receive = vi.fn(), error = vi.fn();
  const pause = logs.follow(query, receive, error);
  sockets[0]!.onopen?.();
  sockets[0]!.receive(page);
  pause();
  expect(error).not.toHaveBeenCalled();
  logs.follow({ ...query, after: 7 }, receive, error);
  sockets[1]!.onopen?.();
  expect(sockets[1]!.sent).toEqual([{ action: 'follow', query: { ...query, after: 7 } }]);
  logs.close();
  expect(error).toHaveBeenCalledTimes(1);
});
it('rejects pending reads when the owning vehicle connection is closed', async () => {
  const logs = new BrowserVehicleLogs(() => 'robot', () => new Socket());
  const pending = logs.query(query);
  logs.close();
  await expect(pending).rejects.toThrow('日志连接已关闭');
});
