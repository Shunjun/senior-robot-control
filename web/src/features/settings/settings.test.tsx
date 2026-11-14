import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { afterEach, expect, it } from 'vitest';
import { testPort, settingsFixture } from '../../api/vehicleState.testUtils';
import type { SettingsRequest } from '../../api/vehicleState';
import { VehicleSettings } from './VehicleSettings';
afterEach(cleanup);
it('converts only displayed km/h to SI, submits one group and reads effective values', async () => {
  let current = structuredClone(settingsFixture);
  const submitted: SettingsRequest[] = [];
  const port = testPort(async (type, payload) => {
    if (type === 'settings.get') return current;
    const request = payload as SettingsRequest;
    submitted.push(request);
    current = { ...current, snapshot: { ...current.snapshot, driving: request.values as typeof current.snapshot.driving, driving_version: 3 } };
    return {
      request_id: request.request_id,
      group: request.group,
      reason: 'applied',
      snapshot: current.snapshot,
      capabilities: current.capabilities
    };
  });
  render(<VehicleSettings port={port} enabled />);
  const field = await screen.findByLabelText('低档线速度上限');
  expect((field as HTMLInputElement).value).toBe('2');
  fireEvent.change(field, { target: { value: '4' } });
  fireEvent.click(screen.getByRole('button', { name: '保存驾驶设置' }));
  await screen.findByText(/车端已确认配置生效/);
  expect(submitted[0]).toMatchObject({ group: 'driving', expected_version: 2, values: { low_linear_mps: 4 / 3.6, high_angular_rps: 1 } });
  await screen.findByText('驾驶设置 · 生效版本 3');
});
it('keeps unknown request identity through disconnect and queries instead of creating another apply', async () => {
  let request: SettingsRequest | null = null;
  const queries: unknown[] = [];
  const port = testPort(async (type, payload) => {
    if (type === 'settings.get') return settingsFixture;
    if (type === 'settings.apply') {
      request = payload as SettingsRequest;
      throw new Error('timeout');
    }
    queries.push(payload);
    return {
      request_id: request!.request_id,
      group: request!.group,
      reason: 'applied',
      snapshot: settingsFixture.snapshot,
      capabilities: settingsFixture.capabilities
    };
  });
  const mounted = render(<VehicleSettings port={port} enabled />);
  await screen.findByLabelText('低档线速度上限');
  fireEvent.click(screen.getByRole('button', { name: '保存驾驶设置' }));
  await screen.findByText(/提交结果待确认/);
  mounted.rerender(<VehicleSettings port={port} enabled={false} />);
  mounted.rerender(<VehicleSettings port={port} enabled />);
  await act(async () => {
  });
  fireEvent.click(screen.getByRole('button', { name: '查询原请求结果' }));
  await waitFor(() => expect(queries).toEqual([{ request_id: request!.request_id }]));
});
it('allows logging while driving changes are refused by the car', async () => {
  const submitted: SettingsRequest[] = [];
  const data = { ...settingsFixture, can_apply_driving: false };
  const port = testPort(async (type, payload) => {
    if (type === 'settings.get') return data;
    const request = payload as SettingsRequest;
    submitted.push(request);
    return {
      request_id: request.request_id,
      group: request.group,
      reason: 'applied',
      snapshot: data.snapshot,
      capabilities: data.capabilities
    };
  });
  render(<VehicleSettings port={port} enabled />);
  await screen.findByLabelText('低档线速度上限');
  expect((screen.getByRole('button', { name: '保存驾驶设置' }) as HTMLButtonElement).disabled).toBe(true);
  fireEvent.click(screen.getByRole('button', { name: '保存日志设置' }));
  await waitFor(() => expect(submitted[0]).toMatchObject({ group: 'logging', expected_version: 7 }));
  expect(screen.queryByLabelText(/磁盘/)).toBeNull();
});

it('can restore defaults even while a draft field is empty', async () => {
  const submitted: SettingsRequest[] = [];
  const port = testPort(async (type, payload) => {
    if (type === 'settings.get') return settingsFixture;
    const request = payload as SettingsRequest;
    submitted.push(request);
    return {
      request_id: request.request_id,
      group: request.group,
      reason: 'applied',
      snapshot: settingsFixture.snapshot,
      capabilities: settingsFixture.capabilities
    };
  });
  render(<VehicleSettings port={port} enabled />);
  const field = await screen.findByLabelText('低档线速度上限');
  fireEvent.change(field, { target: { value: '' } });
  fireEvent.click(screen.getByRole('button', { name: '恢复驾驶默认值' }));
  await waitFor(() => expect(submitted[0]).toMatchObject({ restore_defaults: true, values: settingsFixture.capabilities.driving_defaults }));
});
