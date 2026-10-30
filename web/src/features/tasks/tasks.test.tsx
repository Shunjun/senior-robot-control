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

  it("test_offline_edits_disabled", async () => {
    const stale = { ...snapshot, connected: false, stale: true };
    const robotLink = link();
    render(<TaskList snapshot={stale} link={robotLink} />);
    expect((screen.getByRole("button", { name: "提交任务" }) as HTMLButtonElement).disabled).toBe(true);
    fireEvent.click(screen.getByRole("button", { name: "提交任务" }));
    await waitFor(() => expect(robotLink.request).not.toHaveBeenCalled());
  });
});
