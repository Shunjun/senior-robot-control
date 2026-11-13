import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { afterEach, expect, it, vi } from 'vitest';
import type { LogPage, VehicleLogPort } from '../../api/vehicleState';
import { VehicleLogs } from './VehicleLogs';
afterEach(() => {
  cleanup();
  vi.unstubAllGlobals();
});
function page(first: number, last: number, more = false): LogPage {
  return {
    entries: Array.from({ length: last - first + 1 }, (_, i) => ({
      sequence: first + i,
      kind: 'event' as const,
      level: 'INFO' as const,
      module: 'motion',
      summary: `记录 ${first + i}`,
      detail: '<img src=x onerror=alert(1)>',
      fields_json: '{}',
      source_utc: 1000,
      received_utc: 1001,
      request_id: null,
      task_id: null,
      loss_reason: null,
      loss_count: 0
    })),
    gaps: [],
    next_cursor: last,
    high_water: more ? last + 1 : last,
    has_more: more,
    scanned: last - first + 1
  };
}
it('keeps cursor/gap with pagination and escapes detail text', async () => {
  const queries: unknown[] = [];
  const port: VehicleLogPort = {
    query: async query => {
      queries.push(query);
      return query.after === 0 ? {
        ...page(1, 2, true), gaps: [{
          reason: 'evicted',
          count: 4,
          after: 0,
          through: 0,
          durable: true
        }]
      } : page(3, 3);
    }, follow: () => () => {
    }, download: async () => ({
      data: '',
      next_cursor: 0,
      has_more: false,
      gaps: []
    })
  };
  render(<VehicleLogs port={port} enabled />);
  await screen.findByText('记录 1');
  expect(screen.getByText(/日志缺口：evicted/)).toBeTruthy();
  expect(document.querySelector('img')).toBeNull();
  fireEvent.click(screen.getByRole('button', { name: '下一页历史' }));
  await screen.findByText('记录 3');
  expect(queries[1]).toMatchObject({ after: 2, limit: 200 });
  expect(screen.getAllByText(/接收/).length).toBeGreaterThan(1);
});
it('bounds the browser window and pauses only its independent subscription', async () => {
  let receive!: (page: LogPage) => void;
  const close = vi.fn();
  const port: VehicleLogPort = {
    query: async () => page(1, 1), follow: (_query, callback) => {
      receive = callback;
      return close;
    }, download: async () => ({
      data: '',
      next_cursor: 0,
      has_more: false,
      gaps: []
    })
  };
  render(<VehicleLogs port={port} enabled />);
  await screen.findByText('记录 1');
  fireEvent.click(screen.getByRole('button', { name: '继续跟随' }));
  for (let i = 0;i < 6;i++) await act(async () => receive(page(2 + 200 * i, 201 + 200 * i)));
  expect(screen.getByText(/当前 1000 条/)).toBeTruthy();
  expect(screen.getByText(/浏览器窗口已裁剪/)).toBeTruthy();
  expect(screen.queryByText('记录 1')).toBeNull();
  fireEvent.click(screen.getByRole('button', { name: '暂停跟随' }));
  expect(close).toHaveBeenCalledOnce();
});
it('reports bounded segmented download and keeps its continuation cursor', async () => {
  const download = vi.fn(async () => ({
    data: '{}\n',
    next_cursor: 3,
    has_more: true,
    gaps: []
  }));
  const create = vi.fn(() => 'blob:test');
  Object.defineProperty(URL, 'createObjectURL', { configurable: true, value: create });
  Object.defineProperty(URL, 'revokeObjectURL', { configurable: true, value: vi.fn() });
  const click = vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(() => {
  });
  const port: VehicleLogPort = {
    query: async () => page(1, 1), follow: () => () => {
    }, download
  };
  render(<VehicleLogs port={port} enabled />);
  await screen.findByText('记录 1');
  fireEvent.click(screen.getByRole('button', { name: '下载当前条件' }));
  await screen.findByRole('button', { name: '下载下一段' });
  fireEvent.click(screen.getByRole('button', { name: '下载下一段' }));
  await waitFor(() => expect(download.mock.calls).toHaveLength(2));
  expect(download.mock.calls[1]).toEqual([expect.objectContaining({ after: 3 })]);
  expect(create).toHaveBeenCalled();
  click.mockRestore();
});
