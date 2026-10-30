import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

import type { RobotLink } from "../../api/client";
import type { LinkStatus, RobotSnapshot } from "../../api/protocol";
import { ScheduleEditor, type TaskDraft } from "./ScheduleEditor";
import { TaskList } from "./TaskList";

afterEach(cleanup);

const snapshot: RobotSnapshot = {
  robot_id: "robot-alpha",
  connected: true,
  localized: true,
  map_ref: { map_id: "map-1", reference_revision: 2 },
  home: null,
  task_revision: 7,
  runnable_task_ids: ["daily"],
  execution_status: "idle",
  collected_at: 1,
  stale: false,
  last_synced_at: 1,
  home_revision: 0,
};

function link(): RobotLink {
  return {
    connect: async () => undefined,
    disconnect: () => undefined,
    request: vi.fn(async () => ({
      request_type: "task.create",
      ok: true,
      status: "succeeded",
    })) as unknown as RobotLink["request"],
    subscribe: () => () => undefined,
    subscribeStatus: (_listener: (status: LinkStatus) => void) => () => undefined,
  };
}

describe("task schedules", () => {
  it("test_daily_interval_weekly_cron_roundtrip", () => {
    const saved: TaskDraft[] = [];
    const { rerender } = render(<ScheduleEditor value={null} onSave={(draft) => saved.push(draft)} />);
    fireEvent.change(screen.getByLabelText("任务编号"), { target: { value: "daily" } });
    fireEvent.change(screen.getByLabelText("间隔秒数"), { target: { value: "86400" } });
    fireEvent.click(screen.getByRole("button", { name: "保存草稿" }));
    expect(saved[0]?.schedule).toEqual({ kind: "interval", every_seconds: 86400 });
    expect(saved[0]?.timezone).toBe("Asia/Shanghai");

    rerender(<ScheduleEditor value={saved[0]!} onSave={(draft) => saved.push(draft)} />);
    fireEvent.change(screen.getByLabelText("计划类型"), { target: { value: "cron" } });
    fireEvent.change(screen.getByLabelText("Cron 表达式"), { target: { value: "0 9 * * 1" } });
    fireEvent.click(screen.getByRole("button", { name: "保存草稿" }));
    expect(saved[1]?.schedule).toEqual({ kind: "cron", expression: "0 9 * * 1" });
    expect(saved[1]?.timezone).toBe("Asia/Shanghai");
  });

  it("submits a waypoint and edits each task with its own revision", async () => {
    const robotLink = link();
    const listed = {
      ...snapshot,
      tasks: [
        { task_id: "daily", revision: 4, enabled: true },
        { task_id: "night", revision: 9, enabled: false },
      ],
    };
    render(<TaskList snapshot={listed} link={robotLink} />);
    fireEvent.change(screen.getByLabelText("任务编号"), { target: { value: "evening" } });
    fireEvent.change(screen.getByLabelText("航点编号"), { target: { value: "gate" } });
    fireEvent.click(screen.getByRole("button", { name: "保存草稿" }));
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await waitFor(() => expect(screen.getByRole("status").textContent).toContain("已确认保存"));
    const created = vi.mocked(robotLink.request).mock.calls[0]?.[1] as {
      body: { waypoint_ids: string[] };
    };
    expect(created.body.waypoint_ids).toEqual(["gate"]);

    fireEvent.click(screen.getByRole("button", { name: "暂停" }));
    await waitFor(() => expect(vi.mocked(robotLink.request).mock.calls).toHaveLength(2));
    fireEvent.click(screen.getByRole("button", { name: "启用" }));
    await waitFor(() => expect(vi.mocked(robotLink.request).mock.calls).toHaveLength(3));
    fireEvent.click(screen.getAllByRole("button", { name: "删除" })[1]!);
    await waitFor(() => expect(vi.mocked(robotLink.request).mock.calls).toHaveLength(4));
    const calls = vi.mocked(robotLink.request).mock.calls;
    expect(calls[1]?.[0]).toBe("task.enable");
    expect(calls[1]?.[1]).toMatchObject({
      expected_revision: 4,
      body: { task_id: "daily", enabled: false },
    });
    expect(calls[2]?.[1]).toMatchObject({
      expected_revision: 9,
      body: { task_id: "night", enabled: true },
    });
    expect(calls[3]?.[0]).toBe("task.delete");
    expect(calls[3]?.[1]).toMatchObject({
      expected_revision: 9,
      body: { task_id: "night" },
    });
  });

  it("test_offline_edits_disabled", async () => {
    const stale = { ...snapshot, connected: false, stale: true };
    const robotLink = link();
    render(<TaskList snapshot={stale} link={robotLink} />);
    expect((screen.getByRole("button", { name: "提交任务" }) as HTMLButtonElement).disabled).toBe(true);
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await waitFor(() => expect(robotLink.request).not.toHaveBeenCalled());
  });
});
