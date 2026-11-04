import { useEffect, useRef, useState, type ReactElement } from "react";

import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import type { ResponsePayload, ResponseStatus, RobotSnapshot } from "../../api/protocol";
import { ImportDialog } from "./ImportDialog";
import styles from "./Mapping.module.css";

const NAME = /^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$/;
const FINISHED = new Set<ResponseStatus>(["succeeded", "failed", "needs_review"]);

export function MapManager({ robotId, snapshot, link }: {
  robotId: string;
  snapshot: RobotSnapshot;
  link: RobotLink;
}): ReactElement {
  const [name, setName] = useState("");
  const [confirmation, setConfirmation] = useState("");
  const [operation, setOperation] = useState<{ id: string; status: ResponseStatus | null } | null>(null);
  const operationRef = useRef(operation);
  operationRef.current = operation;
  const live = snapshot.connected && !snapshot.stale;
  const validName = NAME.test(name);

  async function query(id: string): Promise<void> {
    try {
      const response = await link.request<ResponsePayload>("operation.query", {
        operation_id: id,
        body: {},
      });
      setOperation({ id, status: response.status ?? null });
    } catch {
      setOperation({ id, status: "needs_review" });
    }
  }

  useEffect(
    () =>
      link.subscribeStatus((status) => {
        const current = operationRef.current;
        if (
          status.state === "open" &&
          status.robotId === robotId &&
          current !== null &&
          (current.status === null || !FINISHED.has(current.status))
        ) {
          void query(current.id);
        }
      }),
    [link, robotId],
  );

  useEffect(() => {
    if (operation === null || operation.status === null || FINISHED.has(operation.status) || !live) return;
    const timer = setTimeout(() => void query(operation.id), 1_000);
    return () => clearTimeout(timer);
  }, [operation, live]);

  async function mutate(type: "map.export" | "map.import" | "map.clear"): Promise<void> {
    if (!live || snapshot.map_ref === null && type !== "map.import") return;
    const operationId = crypto.randomUUID();
    const body =
      type === "map.import"
        ? { name }
        : type === "map.export"
          ? { ...snapshot.map_ref!, name }
          : { ...snapshot.map_ref! };
    setOperation({ id: operationId, status: "accepted" });
    try {
      const response = await link.request<ResponsePayload>(type, {
        operation_id: operationId,
        body,
      });
      const id = response.operation_id ?? operationId;
      setOperation({ id, status: response.status ?? null });
    } catch (error) {
      const message = error instanceof CommandError ? describeCommandError(error.code) : "操作未确认";
      setOperation({ id: `${operationId} · ${message}`, status: "needs_review" });
    }
  }

  return (
    <section className={styles.panel} aria-label="地图生命周期管理">
      <header><p className={styles.kicker}>MAP LIFECYCLE</p><h3>地图包与清理</h3></header>
      <p>当前地图：{snapshot.map_ref === null ? "无" : `${snapshot.map_ref.map_id} · r${snapshot.map_ref.reference_revision}`}</p>
      <label>
        地图包名称
        <input aria-label="地图包名称" value={name} onChange={(event) => setName(event.target.value)} />
      </label>
      <p className={styles.note}>此控制通道只发送地图包名称，不携带文件字节。传输由车端受控目录完成。</p>
      <div className={styles.actions}>
        <button type="button" disabled={!live || !validName || snapshot.map_ref === null} onClick={() => void mutate("map.export")}>
          导出地图包
        </button>
        <ImportDialog disabled={!live || !snapshot.localized || !validName} onImport={() => void mutate("map.import")} />
      </div>
      {!snapshot.localized && <p className={styles.warning}>完成定位后才可导入地图包。</p>}
      <label>
        输入机器人编号确认
        <input
          aria-label="输入机器人编号确认"
          value={confirmation}
          onChange={(event) => setConfirmation(event.target.value)}
        />
      </label>
      <button
        type="button"
        className={styles.danger}
        disabled={!live || snapshot.map_ref === null || confirmation !== robotId}
        onClick={() => void mutate("map.clear")}
      >
        清除地图
      </button>
      {!live && <p className={styles.warning}>快照已过期或车辆断开，地图变更已锁定。</p>}
      {snapshot.home?.status === "pending_confirmation" && (
        <p className={styles.warning}>跨车 Home 正在等待确认，当前不会采用，也不会发送替换指令。</p>
      )}
      {operation !== null && <p role="status">操作 {operation.id}：{operation.status ?? "等待车端确认"}</p>}
    </section>
  );
}
