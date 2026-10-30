import { useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import type { ResponsePayload, RobotSnapshot } from "../../api/protocol";
import { ScheduleEditor, type TaskDraft } from "./ScheduleEditor";
import styles from "./Tasks.module.css";

export function TaskList({ snapshot, link }: { snapshot: RobotSnapshot; link: RobotLink }): ReactElement {
  const [draft, setDraft] = useState<TaskDraft | null>(null);
  const [pending, setPending] = useState(false);
  const [message, setMessage] = useState<string | null>(null);
  const live = snapshot.connected && !snapshot.stale;

  async function submit(): Promise<void> {
    if (!live || draft === null || snapshot.map_ref === null) return;
    setPending(true);
    setMessage("等待车端确认保存…");
    const createdAt = new Date().toISOString();
    try {
      const response = await link.request<ResponsePayload>("task.create", {
        operation_id: crypto.randomUUID(),
        body: {
          task_id: draft.task_id,
          created_at: createdAt,
          timezone: draft.timezone,
          schedule: draft.schedule,
          map_id: snapshot.map_ref.map_id,
          reference_revision: snapshot.map_ref.reference_revision,
          waypoint_ids: [],
          enabled: true,
        },
      });
      setMessage(response.ok ? "车端已确认保存任务。" : "保存未确认。");
    } catch (error) {
      const detail = error instanceof CommandError ? describeCommandError(error.code) : "无法发送";
      setMessage(`保存未确认：${detail}`);
    } finally {
      setPending(false);
    }
  }

  async function changeTask(taskId: string, type: "task.enable" | "task.delete"): Promise<void> {
    if (!live) return;
    setPending(true);
    setMessage("等待车端确认变更…");
    try {
      await link.request<ResponsePayload>(type, {
        operation_id: crypto.randomUUID(),
        expected_revision: snapshot.task_revision,
        body: type === "task.enable" ? { task_id: taskId, enabled: false } : { task_id: taskId },
      });
      setMessage("车端已确认任务变更。");
    } catch (error) {
      const detail = error instanceof CommandError ? describeCommandError(error.code) : "无法发送";
      setMessage(`任务变更未确认：${detail}`);
    } finally {
      setPending(false);
    }
  }

  return (
    <section className={styles.panel} aria-label="任务管理">
      <header>
        <p className={styles.kicker}>TASKS / DEVICE AUTHORITY</p>
        <h3>任务与计划</h3>
      </header>
      <dl className={styles.facts}>
        <div><dt>权威来源</dt><dd>车端快照</dd></div>
        <div><dt>任务版本</dt><dd>r{snapshot.task_revision}</dd></div>
        <div><dt>执行状态</dt><dd>车端：{snapshot.execution_status}</dd></div>
        <div><dt>下次触发</dt><dd>保存后由车端计划器计算</dd></div>
      </dl>
      <div>
        <p>可执行任务：{snapshot.runnable_task_ids.length > 0 ? "" : "无"}</p>
        {snapshot.runnable_task_ids.map((taskId) => (
          <div className={styles.taskRow} key={taskId}>
            <code>{taskId}</code>
            <button type="button" disabled={!live || pending} onClick={() => void changeTask(taskId, "task.enable")}>暂停</button>
            <button type="button" disabled={!live || pending} onClick={() => void changeTask(taskId, "task.delete")}>删除</button>
          </div>
        ))}
      </div>
      {!live && <p className={styles.warning}>快照已过期或车辆断开，任务编辑已锁定。</p>}
      <ScheduleEditor value={draft} onSave={setDraft} />
      <button
        type="button"
        className={styles.primary}
        disabled={!live || draft === null || snapshot.map_ref === null || pending}
        onClick={() => void submit()}
      >
        提交任务
      </button>
      <p className={styles.note}>暂停或跳过原因以车端执行状态为准；当前：{snapshot.execution_status}</p>
      {message !== null && <p role="status">{message}</p>}
    </section>
  );
}
