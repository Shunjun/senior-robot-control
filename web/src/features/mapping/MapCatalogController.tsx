import { useEffect, useLayoutEffect, useRef, useState, type ReactElement } from "react";
import { CommandError, describeCommandError, type RobotLink } from "../../api/client";
import { readMappingCatalog, sendMapCommand, type MappingCatalog, type MapCommand } from "../../api/mapping";
import type { ResponsePayload, ResponseStatus } from "../../api/protocol";
import { MapCatalog } from "./MapCatalog";
import styles from "./Mapping.module.css";

interface MapCatalogControllerProps { robotId: string; link: RobotLink; live: boolean; }
interface CatalogResult { robotId: string; link: RobotLink; data: MappingCatalog; revision: number; }
interface PendingOperation { id: string; status: ResponseStatus; }
const FINISHED = new Set<ResponseStatus>(["succeeded", "failed", "needs_review"]);
const dateFormat = new Intl.DateTimeFormat(undefined, { dateStyle: "short", timeStyle: "medium" });
const explain = (error: unknown) => error instanceof CommandError ? describeCommandError(error.code) : "操作结果未确认";

export function MapCatalogController({ robotId, link, live }: MapCatalogControllerProps): ReactElement {
  const [result, setResult] = useState<CatalogResult | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [readError, setReadError] = useState<string | null>(null);
  const [operation, setOperation] = useState<PendingOperation | null>(null);
  const [refresh, setRefresh] = useState(0);
  const scope = useRef(0);
  const readRevision = useRef(0);
  const inFlight = useRef<symbol | null>(null);
  useLayoutEffect(() => {
    scope.current += 1;
    readRevision.current += 1;
    inFlight.current = null;
    setOperation(null); setError(null); setReadError(null); setResult(null);
    return () => { scope.current += 1; };
  }, [robotId, link, live]);
  useEffect(() => {
    if (!live) return;
    let disposed = false;
    const revision = readRevision.current;
    let timer: ReturnType<typeof setTimeout> | undefined;
    async function load(): Promise<void> {
      try {
        const data = await readMappingCatalog(link, robotId);
        if (!disposed && revision === readRevision.current) { setResult({ robotId, link, data, revision }); setReadError(null); }
      } catch (reason) {
        if (!disposed && revision === readRevision.current) setReadError(explain(reason));
      } finally {
        if (!disposed && revision === readRevision.current) timer = setTimeout(() => void load(), 2000);
      }
    }
    void load();
    return () => { disposed = true; clearTimeout(timer); };
  }, [link, robotId, live, refresh]);
  const data = live && result?.robotId === robotId && result.link === link ? result.data : null;
  const fresh = result?.revision === readRevision.current && readError === null;
  function reload(): void { readRevision.current += 1; setRefresh(value => value + 1); }
  const pending = operation !== null && !FINISHED.has(operation.status);
  const busy = !live || !fresh || data === null || !data.runtime_available || pending || data.phase === "matching";

  async function mutate(kind: MapCommand, fields: Record<string, unknown>): Promise<boolean> {
    if (busy || data === null || inFlight.current !== null) return false;
    const token = Symbol(), owner = scope.current, id = crypto.randomUUID();
    inFlight.current = token;
    setOperation({ id, status: "accepted" }); setError(null);
    let status: ResponseStatus = "running";
    try {
      try {
        const reply = await sendMapCommand(link, kind, id, { ...fields, expected_generation: data.selection.generation });
        status = reply.status!;
      } catch (reason) {
        if (reason instanceof CommandError && reason.code !== "timeout" && reason.code !== "operation_unknown") throw reason;
        // An unconfirmed acknowledgement never triggers a second mutation.
      }
      while (owner === scope.current && !FINISHED.has(status)) {
        setOperation({ id, status });
        await new Promise(resolve => setTimeout(resolve, 1000));
        if (owner !== scope.current) return false;
        const reply = await link.request<ResponsePayload>("operation.query", { operation_id: id, body: {} });
        if (!reply.ok || reply.operation_id !== id || reply.status == null) throw new CommandError("operation_unknown", "操作未确认");
        status = reply.status;
      }
      if (owner !== scope.current) return false;
      setOperation({ id, status });
      if (status !== "succeeded") { setError(status === "needs_review" ? "操作结果需要核查" : "车端未完成地图操作"); return false; }
      reload();
      return true;
    } catch (reason) {
      if (owner === scope.current) { setOperation({ id, status: "needs_review" }); setError(explain(reason)); }
      return false;
    } finally {
      if (inFlight.current === token) inFlight.current = null;
    }
  }
  function target(kind: "map.activate" | "map.delete", id: string): void {
    const entry = data?.entries.find(row => row.map_ref.map_id === id);
    if (entry !== undefined) void mutate(kind, { ...entry.map_ref });
  }
  return <>
    {data !== null ? <MapCatalog entries={data.entries.map(row => ({ map_id: row.map_ref.map_id, name: row.name,
      saved_at: row.saved_at === null ? null : dateFormat.format(new Date(row.saved_at * 1000)) }))}
      activeId={data.selection.map_ref?.map_id ?? null} status={readError === null ? data.phase : "unavailable"} busy={busy}
      onCreate={name => mutate("map.create", { name })} onActivate={id => target("map.activate", id)} onDelete={id => target("map.delete", id)} />
      : <section className={styles.panel} aria-label="活动地图管理"><h3>地图管理</h3><p>{live ? "地图目录暂不可用" : "车辆未实时连接"}</p></section>}
    <div className={styles.actions}><button disabled={!live} onClick={reload}>刷新地图目录</button></div>
    {readError !== null && <p className={styles.warning} role="alert">目录更新失败：{readError}</p>}
    {error !== null && <p className={styles.warning} role="alert">{error}</p>}
    {operation !== null && <p role="status">操作 {operation.id}：{operation.status}</p>}
  </>;
}
