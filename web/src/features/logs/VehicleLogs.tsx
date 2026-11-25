import { memo, useEffect, useRef, useState, type ReactElement } from "react";
import {
  decodeLogDownload,
  decodeLogPage,
  type Level,
  type LogEntry,
  type LogGap,
  type LogPage,
  type LogQuery,
  type VehicleLogPort,
} from "../../api/vehicleState";
import { Button } from "../../components/ui/Button";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  card: [
    "logs-card absolute inset-0 overflow-auto pt-7 pr-7 pb-7 pl-7",
    "bg-none bg-full-view min-w-0 z-[20] [&>header]:flex [&>header]:justify-between [&>header]:items-center [&>header]:gap-2.5",
    "[&>header]:mb-6 [&_footer]:flex [&_footer]:justify-between [&_footer]:items-center [&_footer]:gap-2.5 [&_footer]:mt-3.5 [&_footer]:text-muted [&_footer]:text-[10px]",
    "[&_h1]:text-[23px] [&_h1]:font-[450] [&_h1]:mt-0 [&_h1]:mr-0 [&_h1]:mb-2 [&_h1]:ml-0 [&_header_p]:mt-0 [&_header_p]:mr-0",
    "[&_header_p]:mb-0 [&_header_p]:ml-0 [&_header_p]:text-[11px] [&_header_p]:text-muted [&_header_p]:leading-[1.8] [&_footer_button]:pt-[7px] [&_footer_button]:pr-2.5 [&_footer_button]:pb-[7px]",
    "[&_footer_button]:pl-2.5 [&_footer_button]:text-inherit [&_footer_button]:bg-none [&_footer_button]:bg-control [&_footer_button]:border [&_footer_button]:border-solid [&_footer_button]:border-devices-item-border-color [&_footer_button]:rounded-control",
    "[&_footer_button]:cursor-pointer [&_footer_button:disabled]:opacity-[0.36] [&_footer_button:disabled]:cursor-not-allowed max-[760.001px]:pt-4.5 max-[760.001px]:pr-4.5 max-[760.001px]:pb-4.5 max-[760.001px]:pl-4.5 max-[460.001px]:pt-3.5",
    "max-[460.001px]:pr-3.5 max-[460.001px]:pb-3.5 max-[460.001px]:pl-3.5"
  ].join(" "),
  table: [
    "logs-table bg-none bg-log-surface border border-solid border-line rounded-panel pt-[19px]",
    "pr-5 pb-[19px] pl-5 max-[460.001px]:pt-3 max-[460.001px]:pr-3 max-[460.001px]:pb-3 max-[460.001px]:pl-3"
  ].join(" "),
  tools: [
    "logs-tools flex gap-[9px] mb-4 [&_input]:flex-1 [&_input]:min-w-0 [&_input]:pt-2.5 [&_input]:pr-2.5",
    "[&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:text-input-ink [&_input]:bg-none [&_input]:bg-input [&_input]:border [&_input]:border-solid",
    "[&_input]:border-login-input-border-color [&_input]:rounded-input [&_select]:w-[130px] [&_select]:min-w-0 [&_select]:pt-2.5 [&_select]:pr-2.5 [&_select]:pb-2.5 [&_select]:pl-2.5",
    "[&_select]:text-input-ink [&_select]:bg-none [&_select]:bg-input [&_select]:border [&_select]:border-solid [&_select]:border-login-input-border-color [&_select]:rounded-input",
    "[&_button]:whitespace-nowrap max-[460.001px]:flex-wrap max-[460.001px]:[&_input]:[flex-basis:100%] max-[460.001px]:[&_select]:w-[100px]"
  ].join(" "),
  caption: [
    "logs-caption flex justify-between gap-[15px] text-log-caption text-[10px] mt-[19px] mr-0",
    "mb-[15px] ml-0 pb-[13px] border-b [border-bottom-style:solid] border-b-logs-caption-border-bottom-color [&_i]:[display:inline-block] [&_i]:w-[5px]",
    "[&_i]:h-[5px] [&_i]:mr-1.5 [&_i]:rounded-full [&_i]:bg-none [&_i]:bg-online max-[460.001px]:text-[9px] max-[460.001px]:gap-2"
  ].join(" "),
  collection: [
    "logs-collection text-[9px] text-muted mb-3 [&_summary]:cursor-pointer"
  ].join(" "),
  collectionTools: [
    "logs-collection-tools [&_input]:min-w-0 [&_input]:pt-2.5 [&_input]:pr-2.5 [&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:text-input-ink",
    "[&_input]:bg-none [&_input]:bg-input [&_input]:border [&_input]:border-solid [&_input]:border-login-input-border-color [&_input]:rounded-input flex items-end",
    "flex-wrap gap-[9px] mt-3 [&_label]:grid [&_label]:gap-[7px]"
  ].join(" "),
  gap: [
    "logs-gap border-l-[3px] [border-left-style:solid] border-l-amber bg-none bg-amber-wash text-amber pt-2.5",
    "pr-2.5 pb-2.5 pl-2.5 text-[11px]"
  ].join(" "),
  stream: [
    "logs-stream font-normal text-[12px] leading-[1.95] font-log [overflow-wrap:anywhere]"
  ].join(" "),
  entry: [
    "logs-entry flex flex-wrap items-baseline [column-gap:11px] pt-[7px] pr-2.5 pb-[7px]",
    "pl-2.5 rounded-xs mt-px mr-0 mb-px ml-0 [&:hover]:bg-none [&:hover]:bg-app-empty-symbol-background-color",
    "[&_time]:text-[11px] [&_time]:text-log-time [&_time]:tabular-nums [&_time]:whitespace-nowrap [&_details]:[flex-basis:100%] [&_details]:text-muted [&_details]:text-[9px] [&_summary]:cursor-pointer",
    "[&_details_small]:block [&_details_small]:text-[9px] [&_pre]:whitespace-pre-wrap [&_pre]:[overflow-wrap:anywhere] [&_pre]:font-normal [&_pre]:text-[11px] [&_pre]:leading-[1.8] [&_pre]:font-data",
    "[&[data-level=INFO]_.logs-level]:text-log-info [&[data-level=WARNING]]:bg-none [&[data-level=WARNING]]:bg-log-warning-surface [&[data-level=WARNING]_.logs-level]:text-log-warning [&[data-level=WARNING]_.logs-message]:text-log-warning-message [&[data-level=ERROR]_.logs-level]:text-log-error [&[data-level=CRITICAL]_.logs-level]:text-log-error [&[data-level=ERROR]_.logs-message]:font-semibold",
    "[&[data-level=ERROR]_.logs-message]:text-log-error-message [&[data-level=CRITICAL]_.logs-message]:font-semibold [&[data-level=CRITICAL]_.logs-message]:text-log-error-message [&[data-module=connection]_.logs-module]:text-log-connection [&[data-module=mapping]_.logs-module]:text-log-mapping [&[data-module=scheduler]_.logs-module]:text-log-scheduler [&[data-module=safety]_.logs-module]:text-log-safety max-[460.001px]:pt-[9px]",
    "max-[460.001px]:pr-[3px] max-[460.001px]:pb-[9px] max-[460.001px]:pl-[3px] max-[460.001px]:[column-gap:8px] max-[460.001px]:[&_time]:text-[10px]"
  ].join(" "),
  level: [
    "logs-level text-[10px] tracking-[0.3px] font-semibold text-log-level"
  ].join(" "),
  module: [
    "logs-module text-[11px] text-log-module max-[460.001px]:text-[10px]"
  ].join(" "),
  message: [
    "logs-message flex-1 min-w-[180px] whitespace-pre-wrap text-log-message max-[460.001px]:[flex-basis:100%] max-[460.001px]:text-[11px] max-[460.001px]:leading-[1.9]",
    "max-[460.001px]:mt-0.5"
  ].join(" "),
} as const;

export interface VehicleLogsProps {
  port: VehicleLogPort | null;
  enabled: boolean;
  onBack?: (() => void) | undefined;
}
interface Window {
  entries: readonly LogEntry[];
  gaps: readonly LogGap[];
  cursor: number;
  high: number;
  more: boolean;
  trimmed: boolean;
}
const EMPTY: Window = {
  entries: [],
  gaps: [],
  cursor: 0,
  high: 0,
  more: false,
  trimmed: false,
};
const queryFor = (after: number, level: string, module: string): LogQuery => ({
  after,
  limit: 200,
  levels: level ? [level as Level] : [],
  module: module || null,
  request_id: null,
  task_id: null,
  since_utc: null,
  until_utc: null,
});
function merge(previous: Window, page: LogPage): Window {
  const byId = new Map(
    previous.entries.map((entry) => [entry.sequence, entry]),
  );
  page.entries.forEach((entry) => byId.set(entry.sequence, entry));
  const entries = [...byId.values()].sort((a, b) => a.sequence - b.sequence);
  const gapMap = new Map(
    [...previous.gaps, ...page.gaps].map((gap) => [
      `${gap.reason}:${gap.after}:${gap.through}:${gap.durable}`,
      gap,
    ]),
  );
  const gaps = [...gapMap.values()];
  const high = Math.max(previous.high, page.high_water),
    cursor = Math.max(previous.cursor, page.next_cursor);
  return {
    entries: entries.slice(-1000),
    gaps: gaps.slice(-32),
    cursor,
    high,
    more: cursor < high,
    trimmed: previous.trimmed || entries.length > 1000 || gaps.length > 32,
  };
}
export const VehicleLogs = memo(function VehicleLogs({
  port,
  enabled,
  onBack,
}: VehicleLogsProps): ReactElement {
  const [window, setWindow] = useState(EMPTY),
    [error, setError] = useState<string | null>(null),
    [busy, setBusy] = useState(false);
  const [draft, setDraft] = useState({ level: "", module: "" }),
    [filters, setFilters] = useState({ level: "", module: "" });
  const [search, setSearch] = useState("");
  const [following, setFollowing] = useState(false),
    [downloadMore, setDownloadMore] = useState(false);
  const cursor = useRef(0),
    scope = useRef(0),
    downloadCursor = useRef(0);
  const available = enabled && port !== null;
  function accept(raw: LogPage): void {
    const page = decodeLogPage(raw);
    cursor.current = Math.max(cursor.current, page.next_cursor);
    setWindow((prev) => merge(prev, page));
  }
  useEffect(() => {
    scope.current++;
    cursor.current = downloadCursor.current = 0;
    setWindow(EMPTY);
    setError(null);
    setDownloadMore(false);
    return () => {
      scope.current++;
    };
  }, [port, filters]);
  useEffect(() => {
    const token = scope.current;
    if (available)
      void port!
        .query(queryFor(cursor.current, filters.level, filters.module))
        .then(
          (page) => {
            if (token === scope.current) {
              try {
                accept(page);
              } catch (e) {
                setError(String(e));
              }
            }
          },
          (e) => {
            if (token === scope.current) setError(String(e));
          },
        );
  }, [port, available, filters]);
  useEffect(() => {
    if (!available || !following) return;
    const token = scope.current;
    return port!.follow(
      queryFor(cursor.current, filters.level, filters.module),
      (page) => {
        if (token !== scope.current) return;
        try {
          accept(page);
        } catch (e) {
          setError(String(e));
          setFollowing(false);
        }
      },
      (e) => {
        if (token === scope.current) {
          setError(String(e));
          setFollowing(false);
        }
      },
    );
  }, [port, available, following, filters]);
  async function next(): Promise<void> {
    if (!port) return;
    const token = scope.current;
    setBusy(true);
    try {
      const before = cursor.current,
        page = decodeLogPage(
          await port.query(queryFor(before, filters.level, filters.module)),
        );
      if (token !== scope.current) return;
      if (page.has_more && page.next_cursor <= before)
        throw new Error("日志游标未推进，请重试");
      accept(page);
    } catch (e) {
      if (token === scope.current) setError(String(e));
    } finally {
      if (token === scope.current) setBusy(false);
    }
  }
  async function download(): Promise<void> {
    if (!port) return;
    const token = scope.current;
    setBusy(true);
    try {
      const page = decodeLogDownload(
        await port.download(
          queryFor(downloadCursor.current, filters.level, filters.module),
        ),
      );
      if (token !== scope.current) return;
      if (page.has_more && page.next_cursor <= downloadCursor.current)
        throw new Error("下载游标未推进");
      const metadata =
        JSON.stringify({
          kind: "export_metadata",
          gaps: page.gaps,
          has_more: page.has_more,
          next_cursor: page.next_cursor,
        }) + "\n";
      const bytes = new TextEncoder().encode(metadata + page.data);
      if (bytes.byteLength > 2 * 1024 * 1024)
        throw new Error("下载超出单段上限");
      const blob = new Blob([bytes], { type: "application/x-ndjson" });
      const url = URL.createObjectURL(blob),
        a = document.createElement("a");
      a.href = url;
      a.download = `vehicle-logs-${downloadCursor.current}.ndjson`;
      a.click();
      setTimeout(() => URL.revokeObjectURL(url), 0);
      downloadCursor.current = page.next_cursor;
      setDownloadMore(page.has_more);
      setWindow((prev) => ({
        ...prev,
        gaps: [...prev.gaps, ...page.gaps].slice(-32),
        trimmed: prev.trimmed || prev.gaps.length + page.gaps.length > 32,
      }));
    } catch (e) {
      if (token === scope.current) setError(String(e));
    } finally {
      if (token === scope.current) setBusy(false);
    }
  }
  return (
    <section className={ui.card} aria-label="车端日志">
      <header>
        <div>
          <h1>车辆日志</h1>
          <p>查看车端运行事件与原始记录。</p>
        </div>
        {onBack && <Button onClick={onBack}>返回控制 ↗</Button>}
      </header>
      <div className={ui.table}>
        <div className={ui.tools}>
          <input
            aria-label="搜索日志"
            placeholder="搜索日志内容或模块…"
            value={search}
            onChange={(event) => setSearch(event.target.value)}
          />
          <select
            aria-label="级别"
            disabled={!available}
            value={draft.level}
            onChange={(event) => {
              const next = { ...draft, level: event.target.value };
              setDraft(next);
              setFollowing(false);
              setFilters({ ...next, module: next.module.trim() });
            }}
          >
            <option value="">全部级别</option>
            {["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"].map((level) => (
              <option key={level}>{level}</option>
            ))}
          </select>
          <Button aria-label={downloadMore ? "下载下一段" : "下载当前条件"} disabled={!available || busy} onClick={() => void download()}>
            {downloadMore ? "下载下一段" : "导出"}
          </Button>
        </div>
        <div className={ui.caption}>
          <span>
            <i />
            当前设备的日志
          </span>
          <span>{window.entries.length} 条 · 最新在前</span>
        </div>
        <details className={ui.collection}>
          <summary>
            采集与服务端过滤 ·{" "}
            {following && available ? "正在跟随" : "跟随已暂停"}
          </summary>
          <div className={ui.collectionTools}>
            <label>
              模块
              <input
                disabled={!available}
                value={draft.module}
                maxLength={128}
                onChange={(event) =>
                  setDraft({ ...draft, module: event.target.value })
                }
              />
            </label>
            <Button
              disabled={!available || busy}
              onClick={() => {
                setFollowing(false);
                setFilters({ ...draft, module: draft.module.trim() });
              }}
            >
              应用过滤
            </Button>
            <Button
              disabled={!available}
              onClick={() => setFollowing((value) => !value)}
            >
              {following ? "暂停跟随" : "继续跟随"}
            </Button>
          </div>
        </details>
        {!available && <p>车端日志暂不可用，请等待车辆日志连接。</p>}
        {error && <p role="alert">日志读取失败：{error}</p>}
        {window.trimmed && (
          <p className={ui.gap}>
            浏览器窗口已裁剪，当前显示不包含全部历史；可按条件重新查询或分段下载。
          </p>
        )}
        {window.gaps.map((gap, index) => (
          <p key={index} className={ui.gap}>
            日志缺口：{gap.reason}{" "}
            {gap.after !== null ? `#${gap.after}–${gap.through ?? "?"}` : ""}
            {gap.count !== null ? `，数量 ${gap.count}` : "，数量未知"}
            {gap.durable ? "" : " · 尚未持久确认"}
          </p>
        ))}
        <div className={ui.stream} role="log" aria-label="车辆逐行日志">
          {[...window.entries]
            .reverse()
            .filter((entry) =>
              `${entry.summary} ${entry.detail} ${entry.module}`
                .toLowerCase()
                .includes(search.toLowerCase()),
            )
            .map((entry) => (
              <article
                key={entry.sequence}
                className={ui.entry}
                data-level={entry.level}
                data-module={entry.module}
              >
                <time>
                  {new Date(entry.source_utc * 1000).toLocaleTimeString()}
                </time>
                <span className={ui.level}>{entry.level}</span>
                <span className={ui.module}>[{entry.module}]</span>
                <span className={ui.message}>
                  {entry.kind === "gap"
                    ? `日志丢失：${entry.loss_reason} · ${entry.loss_count ?? "数量未知"}`
                    : entry.summary}
                </span>
                <details>
                  <summary>详情</summary>
                  <small>
                    接收{" "}
                    {new Date(entry.received_utc * 1000).toLocaleTimeString()} ·
                    #{entry.sequence}
                  </small>
                  {(entry.task_id || entry.request_id) && (
                    <small>
                      {entry.task_id} {entry.request_id}
                    </small>
                  )}

                  <pre>
                    {entry.detail}
                    {"\n"}
                    {entry.fields_json}
                  </pre>
                </details>
              </article>
            ))}
        </div>
        <footer>
          <span>
            当前 {window.entries.length} 条 · 游标 {window.cursor}
          </span>
          <button
            disabled={!available || busy || !window.more}
            onClick={() => void next()}
          >
            下一页历史
          </button>
        </footer>
      </div>
    </section>
  );
});
