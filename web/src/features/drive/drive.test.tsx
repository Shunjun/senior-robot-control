import { StrictMode } from 'react';
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { testPort } from '../../api/vehicleState.testUtils';
import { ManualController } from './ManualController';
import { DriveView } from './DriveView';
import { EmergencyStop } from './EmergencyStop';
import type { RobotLink } from '../../api/client';

afterEach(() => {
  cleanup();
  vi.useRealTimers();
});
function link(port = testPort()): RobotLink {
  return {
    vehicle: port,
    connect: async () => {
    },
    disconnect() {
    },
    request: vi.fn(async () => ({ ok: true })) as RobotLink['request'],
    subscribe: () => () => {
    },
    subscribeStatus: () => () => {
    }
  };
}
describe('normalized manual ownership', () => {
  it('prepared authority stays still until a fresh press and sends normalized gear intent', async () => {
    vi.useFakeTimers();
    const sent: [string, unknown][] = [];
    const port = testPort(async (type, value) => {
      sent.push([type, value]);
      return { authority_id: 'a', state: 'prepared' };
    });
    const controller = new ManualController(port, () => {
    });
    await controller.prepare();
    await vi.advanceTimersByTimeAsync(10000);
    expect(sent.map(x => x[0])).toEqual(['control.acquire']);
    controller.begin(1, 0.5, 'high');
    await Promise.resolve();
    expect(sent[1]).toEqual(['drive.intent', {
      authority_id: 'a',
      gesture: 1,
      sequence: 1,
      linear: 1,
      turn: 0.5,
      gear: 'high'
    }]);
    controller.release();
    expect(sent.map(x => x[0])).not.toContain('drive.velocity');
  });
  it('late acquire cannot revive a released press', async () => {
    let finish!: (value: unknown) => void;
    const sent: string[] = [];
    const port = testPort(async type => {
      sent.push(type);
      return type === 'control.acquire' ? new Promise(r => {
        finish = r;
      }) : null;
    });
    const controller = new ManualController(port, () => {
    });
    const preparing = controller.prepare();
    controller.begin(1, 0, 'low');
    controller.release();
    finish({ authority_id: 'late', state: 'prepared' });
    await preparing;
    controller.update(1, 0, 'low');
    expect(sent).not.toContain('drive.intent');
    expect(controller.phase).toBe('none');
  });
  it('holds one unconfirmed and one latest sample and fences both on release', async () => {
    const resolve: (() => void)[] = [];
    const intents: unknown[] = [];
    const port = testPort(async (type, value) => {
      if (type === 'drive.intent') {
        intents.push(value);
        return new Promise<void>(r => resolve.push(r));
      }
      return { authority_id: 'a', state: 'prepared' };
    });
    const controller = new ManualController(port, () => {
    });
    await controller.prepare();
    controller.begin(0.1, 0, 'low');
    controller.update(0.2, 0, 'low');
    controller.update(0.9, 0, 'low');
    expect(intents).toHaveLength(1);
    resolve[0]!();
    await new Promise(r => setTimeout(r, 0));
    expect(intents).toHaveLength(2);
    expect(intents[1]).toMatchObject({ linear: 0.9 });
    controller.release();
    resolve[1]!();
    await Promise.resolve();
    expect(intents).toHaveLength(2);
  });
});
it('shows unavailable instead of enabling old velocity controls', () => {
  const old = link();
  delete (old as {
    vehicle?: unknown;
  }).vehicle;
  render(<DriveView robotId="robot" link={old} snapshot={null} />);
  expect(screen.getByText(/方向力度控制尚未接通/)).toBeTruthy();
  expect((screen.getByRole('button', { name: '获取控制权' }) as HTMLButtonElement).disabled).toBe(true);
});
it('a request acknowledgement does not claim confirmed emergency stop', async () => {
  const l = link();
  render(<EmergencyStop link={l} />);
  await act(async () => fireEvent.click(screen.getByRole('button', { name: '急停' })));
  expect(screen.getByText(/等待车端状态确认/)).toBeTruthy();
  expect(screen.queryByText('软件急停已锁存')).toBeNull();
});

it('actual controls send full high-gear effort, then close the authority on key release', async () => {
  const sent: [string, unknown][] = [];
  const port = testPort(async (type, value) => {
    sent.push([type, value]);
    return { authority_id: 'a', state: 'prepared' };
  });
  render(<DriveView robotId="robot" link={link(port)} snapshot={null} />);
  await act(async () => fireEvent.click(screen.getByRole('button', { name: '获取控制权' })));
  fireEvent.click(screen.getByRole('button', { name: '高档' }));
  const stick = screen.getByRole('group', { name: '前进后退摇杆' });
  await act(async () => fireEvent.keyDown(stick, { key: 'ArrowUp' }));
  expect(sent.find(item => item[0] === 'drive.intent')?.[1]).toMatchObject({ linear: 1, turn: 0, gear: 'high' });
  await act(async () => fireEvent.keyUp(stick, { key: 'ArrowUp' }));
  expect(sent.at(-1)).toEqual(['control.release', { authority_id: 'a' }]);
  expect(screen.getByText('未获取控制')).toBeTruthy();
});
it('sends emergency stop before fencing a held input, and keeps hardware reset absent', async () => {
  const sent: string[] = [];
  const port = testPort(async type => {
    sent.push(type);
    return { authority_id: 'a', state: 'prepared' };
  });
  const l = link(port);
  l.request = (async () => {
    sent.push('safety.estop');
    return { ok: true };
  }) as RobotLink['request'];
  render(<StrictMode>
    <DriveView robotId="robot" link={l} snapshot={null} />
    <EmergencyStop link={l} />
  </StrictMode>);
  await act(async () => fireEvent.click(screen.getByRole('button', { name: '获取控制权' })));
  await act(async () => fireEvent.keyDown(screen.getByRole('group', { name: '前进后退摇杆' }), { key: 'ArrowUp' }));
  await act(async () => fireEvent.click(screen.getByRole('button', { name: '急停' })));
  expect(sent.slice(-2)).toEqual(['safety.estop', 'control.release']);
  const old = port.getSnapshot().frame!;
  act(() => port.update({
    ...old,
    sequence: 2,
    mode: 'estop',
    confirmed_estop_status: 'hardware_latched',
    estop_revision: 8,
    allowed_actions: []
  }, true));
  expect(screen.queryByRole('button', { name: '复位软件急停' })).toBeNull();
});
