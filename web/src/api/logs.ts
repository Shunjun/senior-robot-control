import type { SocketLike } from './client';
import { decodeLogDownload, decodeLogPage, record, boolean, type LogDownload, type LogPage, type LogQuery, type VehicleLogPort } from './vehicleState';

/** Each operation owns an authenticated log socket; control never carries log bytes. */
export class BrowserVehicleLogs implements VehicleLogPort {
  private readonly active = new Set<() => void>();
  constructor(
    private readonly robot: () => string | null,
    private readonly socketFactory: (url: string) => SocketLike,
  ) {}
  close(): void {
    for (const close of [...this.active]) close();
  }
  private open(action: string, query: LogQuery, receive: (data: unknown) => void, error: (error: Error) => void): () => void {
    const robot = this.robot();
    if (!robot) {
      error(new Error('车辆日志连接不可用'));
      return () => {};
    }
    const url = `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws/logs?robot_id=${encodeURIComponent(robot)}`;
    const socket = this.socketFactory(url);
    let closed = false;
    let timer: ReturnType<typeof setTimeout> | null = setTimeout(() => fail(new Error('日志连接超时')), 10000);
    const clear = (): void => { if (timer !== null) clearTimeout(timer); timer = null; };
    const close = (): void => {
      if (closed) return;
      closed = true;
      clear();
      this.active.delete(cancel);
      socket.onopen = socket.onmessage = socket.onclose = socket.onerror = null;
      socket.close(1000);
    };
    const fail = (reason: Error): void => { if (!closed) { close(); error(reason); } };
    const cancel = (): void => fail(new Error('日志连接已关闭'));
    this.active.add(cancel);
    socket.onopen = () => {
      socket.send(JSON.stringify({ action, query }));
      // An idle follow is healthy without new events; ASGI monitors authorization.
      if (action === 'follow') clear();
    };
    socket.onmessage = event => {
      try {
        if (typeof event.data !== 'string' || new TextEncoder().encode(event.data).length > 5 * 1024 * 1024) throw new Error('日志响应超出上限');
        const frame = record(JSON.parse(event.data), ['ok', 'data']);
        if (!boolean(frame.ok)) throw new Error('车端日志读取失败');
        receive(frame.data);
        if (action !== 'follow') close();
      } catch (reason) { fail(reason instanceof Error ? reason : new Error(String(reason))); }
    };
    socket.onclose = () => fail(new Error('日志连接已断开'));
    socket.onerror = () => fail(new Error('日志连接失败'));
    return close;
  }
  query(query: LogQuery): Promise<LogPage> {
    return new Promise((resolve, reject) => this.open('query', query, value => resolve(decodeLogPage(value)), reject));
  }
  download(query: LogQuery): Promise<LogDownload> {
    return new Promise((resolve, reject) => this.open('download', query, value => resolve(decodeLogDownload(value)), reject));
  }
  follow(query: LogQuery, onPage: (page: LogPage) => void, onError: (error: Error) => void): () => void {
    return this.open('follow', query, value => onPage(decodeLogPage(value)), onError);
  }
}
