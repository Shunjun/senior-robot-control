import { memo, useEffect, useRef, useState, type ReactElement } from 'react';
import { decodeLogDownload, decodeLogPage, type Level, type LogEntry, type LogGap, type LogPage, type LogQuery, type VehicleLogPort } from '../../api/vehicleState';
import styles from './Logs.module.css';
export interface VehicleLogsProps {
  port: VehicleLogPort | null;
  enabled: boolean;
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
  trimmed: false
};
const queryFor = (after: number, level: string, module: string): LogQuery => ({
  after,
  limit: 200,
  levels: level ? [level as Level] : [],
  module: module || null,
  request_id: null,
  task_id: null,
  since_utc: null,
  until_utc: null
});
function merge(previous: Window, page: LogPage): Window {
  const byId = new Map(previous.entries.map(entry => [entry.sequence, entry]));
  page.entries.forEach(entry => byId.set(entry.sequence, entry));
  const entries = [...byId.values()].sort((a, b) => a.sequence - b.sequence);
  const gapMap = new Map([...previous.gaps, ...page.gaps].map(gap => [`${gap.reason}:${gap.after}:${gap.through}:${gap.durable}`, gap]));
  const gaps = [...gapMap.values()];
  const high = Math.max(previous.high, page.high_water), cursor = Math.max(previous.cursor, page.next_cursor);
  return {
    entries: entries.slice(-1000),
    gaps: gaps.slice(-32),
    cursor,
    high,
    more: cursor < high,
    trimmed: previous.trimmed || entries.length > 1000 || gaps.length > 32
  };
}
export const VehicleLogs = memo(function VehicleLogs({ port, enabled }: VehicleLogsProps): ReactElement {
  const [window, setWindow] = useState(EMPTY), [error, setError] = useState<string | null>(null), [busy, setBusy] = useState(false);
  const [draft, setDraft] = useState({ level: '', module: '' }), [filters, setFilters] = useState({ level: '', module: '' });
  const [following, setFollowing] = useState(false), [downloadMore, setDownloadMore] = useState(false);
  const cursor = useRef(0), scope = useRef(0), downloadCursor = useRef(0);
  const available = enabled && port !== null;
  function accept(raw: LogPage): void {
    const page = decodeLogPage(raw);
    cursor.current = Math.max(cursor.current, page.next_cursor);
    setWindow(prev => merge(prev, page));
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
    if (available) void port!.query(queryFor(cursor.current, filters.level, filters.module)).then(page => {
      if (token === scope.current) {
        try {
          accept(page);
        } catch (e) {
          setError(String(e));
        }
      }
    }, e => {
      if (token === scope.current) setError(String(e));
    });
  }, [port, available, filters]);
  useEffect(() => {
    if (!available || !following) return;
    const token = scope.current;
    return port!.follow(queryFor(cursor.current, filters.level, filters.module), page => {
      if (token !== scope.current) return;
      try {
        accept(page);
      } catch (e) {
        setError(String(e));
        setFollowing(false);
      }
    }, e => {
      if (token === scope.current) {
        setError(String(e));
        setFollowing(false);
      }
    });
  }, [port, available, following, filters]);
  async function next(): Promise<void> {
    if (!port) return;
    const token = scope.current;
    setBusy(true);
    try {
      const before = cursor.current, page = decodeLogPage(await port.query(queryFor(before, filters.level, filters.module)));
      if (token !== scope.current) return;
      if (page.has_more && page.next_cursor <= before) throw new Error('日志游标未推进，请重试');
      accept(page);
    }
    catch (e) {
      if (token === scope.current) setError(String(e));
    }
    finally {
      if (token === scope.current) setBusy(false);
    }
  }
  async function download(): Promise<void> {
    if (!port) return;
    const token = scope.current;
    setBusy(true);
    try {
      const page = decodeLogDownload(await port.download(queryFor(downloadCursor.current, filters.level, filters.module)));
      if (token !== scope.current) return;
      if (page.has_more && page.next_cursor <= downloadCursor.current) throw new Error('下载游标未推进');
      const metadata = JSON.stringify({
        kind: 'export_metadata',
        gaps: page.gaps,
        has_more: page.has_more,
        next_cursor: page.next_cursor
      }) + '\n';
      const bytes = new TextEncoder().encode(metadata + page.data);
      if (bytes.byteLength > 2 * 1024 * 1024) throw new Error('下载超出单段上限');
      const blob = new Blob([bytes], { type: 'application/x-ndjson' });
      const url = URL.createObjectURL(blob), a = document.createElement('a');
      a.href = url;
      a.download = `vehicle-logs-${downloadCursor.current}.ndjson`;
      a.click();
      setTimeout(() => URL.revokeObjectURL(url), 0);
      downloadCursor.current = page.next_cursor;
      setDownloadMore(page.has_more);
      setWindow(prev => ({ ...prev, gaps: [...prev.gaps, ...page.gaps].slice(-32), trimmed: prev.trimmed || prev.gaps.length + page.gaps.length > 32 }));
    } catch (e) {
      if (token === scope.current) setError(String(e));
    }
    finally {
      if (token === scope.current) setBusy(false);
    }
  }
  return <section className={styles.card} aria-label="车端日志">
    <header>
      <h3>车端日志</h3>
      <span>{following && available ? '正在跟随' : '跟随已暂停'}</span>
    </header>{!available && <p>车端日志暂不可用，请等待车辆日志连接。</p>}<div className={styles.tools}>
      <label>级别<select disabled={!available} value={draft.level} onChange={e => setDraft({ ...draft, level: e.target.value })}>
        <option value="">全部</option>{['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'].map(x => <option key={x}>{x}</option>)}</select>
      </label>
      <label>模块<input disabled={!available} value={draft.module} maxLength={128} onChange={e => setDraft({ ...draft, module: e.target.value })} />
      </label>
      <button disabled={!available || busy} onClick={() => {
        setFollowing(false);
        setFilters({ ...draft, module: draft.module.trim() });
      }}>应用过滤</button>
      <button disabled={!available} onClick={() => setFollowing(value => !value)}>{following ? '暂停跟随' : '继续跟随'}</button>
      <button disabled={!available || busy} onClick={() => void download()}>{downloadMore ? '下载下一段' : '下载当前条件'}</button>
    </div>{error && <p role="alert">日志读取失败：{error}</p>}{window.trimmed && <p className={styles.gap}>浏览器窗口已裁剪，当前显示不包含全部历史；可按条件重新查询或分段下载。</p>}{window.gaps.map((gap, index) => <p key={index} className={styles.gap}>日志缺口：{gap.reason} {gap.after !== null ? `#${gap.after}–${gap.through ?? '?'}` : ''}{gap.count !== null ? `，数量 ${gap.count}` : '，数量未知'}{gap.durable ? '' : ' · 尚未持久确认'}</p>)}<div className={styles.table}>
      <table>
        <thead>
          <tr>
            <th>源时间 / 接收时间</th>
            <th>级别 / 模块</th>
            <th>事件 / 关联操作</th>
          </tr>
        </thead>
        <tbody>{window.entries.map(entry => <tr key={entry.sequence}>
          <td>{new Date(entry.source_utc * 1000).toLocaleTimeString()}<small>接收 {new Date(entry.received_utc * 1000).toLocaleTimeString()} · #{entry.sequence}</small>
          </td>
          <td>{entry.level}<small>{entry.module}</small>
          </td>
          <td>{entry.kind === 'gap' ? `日志丢失：${entry.loss_reason} · ${entry.loss_count ?? '数量未知'}` : entry.summary}<small>{entry.task_id} {entry.request_id}</small>{(entry.detail || entry.fields_json !== '{}') && <details>
            <summary>详情</summary>
            <pre>{entry.detail}{'\n'}{entry.fields_json}</pre>
          </details>}</td>
        </tr>)}</tbody>
      </table>
    </div>
    <footer>
      <span>当前 {window.entries.length} 条 · 游标 {window.cursor}</span>
      <button disabled={!available || busy || !window.more} onClick={() => void next()}>下一页历史</button>
    </footer>
  </section>;
});
