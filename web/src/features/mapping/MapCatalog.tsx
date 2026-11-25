import { useState, type ReactElement } from "react";
import { createPortal } from "react-dom";
import { Button } from "../../components/ui/Button";
import type { MapPhase } from "../../api/mapping";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  panel: [
    "mapping-panel pt-0 pr-0 pb-0 pl-0 bg-none bg-transparent border-0",
    "border-none border-current [&_h3]:text-base [&_h3]:font-medium [&_h3]:mt-0 [&_h3]:mr-0 [&_h3]:mb-[5px] [&_h3]:ml-0",
    "[&_label]:block [&_label]:text-xs [&_label]:text-field [&_label]:mt-[15px] [&_input]:w-full [&_input]:min-w-0 [&_input]:mt-[7px] [&_input]:pt-2.5",
    "[&_input]:pr-2.5 [&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:text-input-ink [&_input]:bg-none [&_input]:bg-input [&_input]:border",
    "[&_input]:border-solid [&_input]:border-login-input-border-color [&_input]:rounded-input [&_button:not([aria-busy])]:text-xs [&_button:not([aria-busy])]:pt-[7px] [&_button:not([aria-busy])]:pr-[9px] [&_button:not([aria-busy])]:pb-[7px]",
    "[&_button:not([aria-busy])]:pl-[9px] [&_button:not([aria-busy])]:border [&_button:not([aria-busy])]:border-solid [&_button:not([aria-busy])]:border-devices-item-border-color [&_button:not([aria-busy])]:rounded-control [&_button:not([aria-busy])]:text-inherit [&_button:not([aria-busy])]:bg-none [&_button:not([aria-busy])]:bg-control",
    "[&_button:not([aria-busy])]:cursor-pointer [&_.mapping-actions_button]:text-xs [&_button:disabled]:opacity-[0.36] [&_button:disabled]:cursor-not-allowed [&_.mapping-danger]:text-mapping-panel-color [&_.mapping-danger]:border-mapping-panel-border-color [&>details]:mt-2.5 [&>details]:text-xs",
    "[&>details]:text-muted [&>details_summary]:cursor-pointer max-[460.001px]:[&_h3]:text-sm"
  ].join(" "),
  head: [
    "mapping-head flex items-center justify-between gap-2.5"
  ].join(" "),
  english: [
    "mapping-english font-normal text-xs leading-[normal] font-mono tracking-[1px] text-english"
  ].join(" "),
  description: [
    "mapping-description text-muted leading-[1.8] mt-0 mr-0 mb-0 ml-0 text-xs"
  ].join(" "),
  warning: [
    "mapping-warning pt-2 pr-2 pb-2 pl-2 mt-2 mr-0 mb-2",
    "ml-0 bg-none bg-mapping-warning-background-color rounded-md text-xs leading-[1.8]"
  ].join(" "),
  catalogRow: [
    "mapping-catalog-row border-t [border-top-style:solid] border-t-line-soft pt-[13px] pr-0 pb-[13px] pl-0",
    "[&_strong]:text-sm [&_strong]:font-medium [&>button]:text-xs [&>button]:mt-[9px] [&>button]:pt-[7px] [&>button]:pr-[9px] [&>button]:pb-[7px] [&>button]:pl-[9px]",
    "[&_.mapping-actions]:mt-[9px] [&_.mapping-actions_button]:text-xs [&_.mapping-actions_button]:pt-[7px] [&_.mapping-actions_button]:pr-[9px] [&_.mapping-actions_button]:pb-[7px] [&_.mapping-actions_button]:pl-[9px]"
  ].join(" "),
  note: [
    "mapping-note text-muted text-xs leading-[1.8] mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  actions: [
    "mapping-actions flex flex-wrap gap-1.5 mt-3 [&_button]:text-xs"
  ].join(" "),
  danger: [
    "mapping-danger"
  ].join(" "),
  details: [
    "mapping-details [&_button:not([aria-busy])]:text-xs [&_button:not([aria-busy])]:pt-[7px] [&_button:not([aria-busy])]:pr-[9px] [&_button:not([aria-busy])]:pb-[7px] [&_button:not([aria-busy])]:pl-[9px] [&_button:not([aria-busy])]:border",
    "[&_button:not([aria-busy])]:border-solid [&_button:not([aria-busy])]:border-devices-item-border-color [&_button:not([aria-busy])]:rounded-control [&_button:not([aria-busy])]:text-inherit [&_button:not([aria-busy])]:bg-none [&_button:not([aria-busy])]:bg-control [&_button:not([aria-busy])]:cursor-pointer [&_button:disabled]:opacity-[0.36]",
    "[&_button:disabled]:cursor-not-allowed mt-2.5 text-xs text-muted [&_summary]:cursor-pointer [&_p]:[overflow-wrap:anywhere]"
  ].join(" "),
  modal: [
    "mapping-modal absolute inset-0 z-[40] bg-none bg-devices-modal-background-color grid place-items-center",
    "pt-5 pr-5 pb-5 pl-5 [body>&]:fixed"
  ].join(" "),
  dialog: [
    "mapping-dialog [&_label]:block [&_label]:text-xs [&_label]:text-field [&_label]:mt-[15px] [&_input]:w-full [&_input]:min-w-0 [&_input]:mt-[7px]",
    "[&_input]:pt-2.5 [&_input]:pr-2.5 [&_input]:pb-2.5 [&_input]:pl-2.5 [&_input]:text-input-ink [&_input]:bg-none [&_input]:bg-input",
    "[&_input]:border [&_input]:border-solid [&_input]:border-login-input-border-color [&_input]:rounded-input w-full max-w-[370px] max-h-full overflow-auto",
    "pt-[19px] pr-[19px] pb-[19px] pl-[19px] bg-none bg-card border border-solid",
    "border-line rounded-card shadow-ui-card backdrop-blur-[15px] [&_h2]:text-base [&_h2]:font-medium [&_h2]:mt-0 [&_h2]:mr-0",
    "[&_h2]:mb-3 [&_h2]:ml-0 [&_p]:text-xs [&_p]:text-muted [&_p]:leading-[1.8]"
  ].join(" "),
  dialogActions: [
    "mapping-dialog-actions flex justify-end gap-2.5 mt-5"
  ].join(" "),
} as const;

export interface MapCatalogEntry {
  map_id: string;
  name: string;
  saved_at: string | null;
}
interface MapCatalogProps {
  entries: readonly MapCatalogEntry[];
  activeId: string | null;
  status: MapPhase;
  busy: boolean;
  onCreate: (name: string) => Promise<boolean>;
  onActivate: (id: string) => void;
  onDelete: (id: string) => void;
  onExport?: (() => void) | undefined;
  onImport?: (() => void) | undefined;
  overlayTarget?: Element | null | undefined;
}

export function MapCatalog({
  entries,
  activeId,
  status,
  busy,
  onCreate,
  onActivate,
  onDelete,
  onExport,
  onImport,
  overlayTarget,
}: MapCatalogProps): ReactElement {
  const [name, setName] = useState("");
  const [creating, setCreating] = useState(false);
  const [deleting, setDeleting] = useState<string | null>(null);
  const labels: Record<MapPhase, string> = {
    loading: "正在切换地图，请等待…",
    idle: "尚无活动地图",
    unavailable: "车端建图运行时未就绪",
    failed: "建图状态异常",
    mapping: "持续建图 · 自动保存",
    saving: "正在保存最新地图",
    matching: "正在匹配车辆位置",
    pending_calibration: "等待校准车辆位置",
  };
  const current = entries.find((entry) => entry.map_id === activeId);
  const saved = entries.filter((entry) => entry.map_id !== activeId);
  return (
    <section className={ui.panel} aria-label="活动地图管理">
      <header className={ui.head}>
        <h3>地图管理</h3>
        <span className={ui.english}>MAPS</span>
      </header>
      <p className={ui.description}>地图自动保存，无手动保存步骤。</p>
      {status !== "mapping" && (
        <p className={ui.warning} role="status">
          {labels[status]}
        </p>
      )}
      {status === "pending_calibration" && (
        <p className={ui.warning}>
          车辆尚未匹配目标地图。Home 仅作为待校准参考，建图与导航尚未恢复。
        </p>
      )}
      {current && (
        <div
          className={ui.catalogRow}
          role="group"
          aria-label={`${current.name}地图`}
        >
          <strong>{current.name}</strong>
          <p className={ui.note}>当前地图 · Home 独立保存</p>
          {onExport && (
            <Button disabled={busy} onClick={onExport}>
              导出地图包
            </Button>
          )}
        </div>
      )}
      {saved.map((entry) => (
        <div
          className={ui.catalogRow}
          key={entry.map_id}
          role="group"
          aria-label={`${entry.name}地图`}
        >
          <strong>{entry.name}</strong>
          <p className={ui.note}>
            {entry.saved_at === null ? "未完成" : "已保存"} · 切换后重新定位
          </p>
          <div className={ui.actions}>
            <Button
              disabled={busy || entry.saved_at === null}
              onClick={() => onActivate(entry.map_id)}
            >
              切换
            </Button>
            <Button
              className={ui.danger}
              disabled={busy}
              onClick={() => setDeleting(entry.map_id)}
            >
              删除
            </Button>
          </div>
        </div>
      ))}
      <div className={ui.actions}>
        <Button
          variant="primary"
          disabled={busy}
          onClick={() => setCreating(true)}
        >
          ＋ 新建地图
        </Button>
        {onImport && (
          <Button disabled={busy} onClick={onImport}>
            导入地图包
          </Button>
        )}
      </div>
      <details className={ui.details}>
        <summary>地图详情</summary>
        <p className={ui.note}>
          手动驾驶时也会更新地图，切换视频或地图视图不影响建图。新地图的 Home
          默认设为开始建图时的车辆位置，每张地图独立保存。
        </p>
        {entries.map((entry) => (
          <p key={entry.map_id} className={ui.note}>
            {entry.name} · {entry.map_id} · {entry.saved_at ?? "等待首次保存"}
          </p>
        ))}
        {current && (
          <Button
            className={ui.danger}
            disabled={busy}
            onClick={() => setDeleting(current.map_id)}
          >
            删除当前地图
          </Button>
        )}
      </details>
      {(creating || deleting !== null) &&
        createPortal(
          <div
            className={ui.modal}
            onKeyDown={(event) => {
              if (event.key === "Escape" && !busy) {
                setCreating(false);
                setDeleting(null);
              }
            }}
          >
            <section
              className={ui.dialog}
              role={deleting ? "alertdialog" : "dialog"}
              aria-modal="true"
              aria-label={deleting ? "删除地图确认" : "新建地图"}
            >
              <h2>{deleting ? "删除地图" : "新建地图"}</h2>
              {deleting ? (
                <p>
                  删除“
                  {entries.find((entry) => entry.map_id === deleting)?.name}”？
                  {deleting === activeId ? "当前建图会话将先停止。" : ""}
                  此操作不可恢复。
                </p>
              ) : (
                <>
                  <p>输入地图名称，创建新的建图会话。</p>
                  <label>
                    新地图名称
                    <input
                      autoFocus
                      maxLength={64}
                      value={name}
                      onChange={(event) => setName(event.target.value)}
                    />
                  </label>
                </>
              )}
              <div className={ui.dialogActions}>
                <Button
                  disabled={busy}
                  onClick={() => {
                    setCreating(false);
                    setDeleting(null);
                  }}
                >
                  取消
                </Button>
                <Button
                  variant="primary"
                  disabled={busy || (creating && !name.trim())}
                  onClick={() => {
                    if (deleting !== null) {
                      onDelete(deleting);
                      setDeleting(null);
                    } else
                      void onCreate(name.trim()).then((success) => {
                        if (success) {
                          setCreating(false);
                          setName("");
                        }
                      });
                  }}
                >
                  {deleting ? "确认删除" : "新建并开始建图"}
                </Button>
              </div>
            </section>
          </div>,
          overlayTarget ?? document.body,
        )}
    </section>
  );
}
