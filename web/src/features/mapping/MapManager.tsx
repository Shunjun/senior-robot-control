import { newId } from "../../ids";
import { useEffect, useRef, useState, type ReactElement } from "react";

import {
  CommandError,
  describeCommandError,
  type RobotLink,
} from "../../api/client";
import { createPortal } from "react-dom";
import { Button } from "../../components/ui/Button";
import type {
  ResponsePayload,
  ResponseStatus,
  RobotSnapshot,
} from "../../api/protocol";
import { MapCatalogController } from "./MapCatalogController";
import { ImportDialog } from "./ImportDialog";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  title: [
    "mapping-title text-base font-medium mt-0 mr-0 mb-[5px] ml-0"
  ].join(" "),
  note: [
    "mapping-note text-muted text-xs leading-[1.8] mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
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
  actions: [
    "mapping-actions flex flex-wrap gap-1.5 mt-3 [&_button]:text-xs"
  ].join(" "),
  danger: [
    "mapping-danger"
  ].join(" "),
  warning: [
    "mapping-warning pt-2 pr-2 pb-2 pl-2 mt-2 mr-0 mb-2",
    "ml-0 bg-none bg-mapping-warning-background-color rounded-md text-xs leading-[1.8]"
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

const NAME = /^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$/;
const FINISHED = new Set<ResponseStatus>([
  "succeeded",
  "failed",
  "needs_review",
]);

interface MapManagerProps {
  robotId: string;
  snapshot: RobotSnapshot;
  link: RobotLink;
  compact?: boolean;
  overlayTarget?: Element | null | undefined;
}
export function MapManager({
  robotId,
  snapshot,
  link,
  compact = false,
  overlayTarget,
}: MapManagerProps): ReactElement {
  const [packageAction, setPackageAction] = useState<
    "export" | "import" | null
  >(null);
  const [name, setName] = useState("");
  const [confirmation, setConfirmation] = useState("");
  const [operation, setOperation] = useState<{
    id: string;
    status: ResponseStatus | null;
  } | null>(null);
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
    if (
      operation === null ||
      operation.status === null ||
      FINISHED.has(operation.status) ||
      !live
    )
      return;
    const timer = setTimeout(() => void query(operation.id), 1_000);
    return () => clearTimeout(timer);
  }, [operation, live]);

  async function mutate(
    type: "map.export" | "map.import" | "map.clear",
  ): Promise<void> {
    if (!live || (snapshot.map_ref === null && type !== "map.import")) return;
    const operationId = newId();
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
      const message =
        error instanceof CommandError
          ? describeCommandError(error.code)
          : "操作未确认";
      setOperation({
        id: `${operationId} · ${message}`,
        status: "needs_review",
      });
    }
  }

  return (
    <>
      {(snapshot.mapping_protocol_version ?? 0) < 1 && (
        <h3 className={ui.title}>地图管理</h3>
      )}
      {(snapshot.mapping_protocol_version ?? 0) >= 1 ? (
        <MapCatalogController
          robotId={robotId}
          live={live}
          link={link}
          onExport={() => setPackageAction("export")}
          onImport={() => setPackageAction("import")}
          overlayTarget={overlayTarget}
        />
      ) : (
        <p className={ui.note}>当前车端尚未提供新版地图管理接口。</p>
      )}
      <section className={ui.panel} aria-label="地图生命周期管理">
        {(snapshot.mapping_protocol_version ?? 0) < 1 && (
          <div className={ui.actions}>
            <Button
              disabled={!live || snapshot.map_ref === null}
              onClick={() => setPackageAction("export")}
            >
              导出地图包
            </Button>
            <Button disabled={!live} onClick={() => setPackageAction("import")}>
              导入地图包
            </Button>
          </div>
        )}
        <details open={!compact}>
          <summary>高级地图包与危险操作</summary>
          {(snapshot.mapping_protocol_version ?? 0) >= 1 && (
            <div className={ui.actions}>
              <Button
                disabled={!live || snapshot.map_ref === null}
                onClick={() => setPackageAction("export")}
              >
                导出地图包
              </Button>
              <Button
                disabled={!live}
                onClick={() => setPackageAction("import")}
              >
                导入地图包
              </Button>
            </div>
          )}
          <p className={ui.note}>清除当前地图前，请输入机器人编号确认。</p>
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
            className={ui.danger}
            disabled={
              !live || snapshot.map_ref === null || confirmation !== robotId
            }
            onClick={() => void mutate("map.clear")}
          >
            清除地图
          </button>
        </details>
        {!live && (
          <p className={ui.warning}>
            快照已过期或车辆断开，地图变更已锁定。
          </p>
        )}
        {snapshot.home?.status === "pending_confirmation" && (
          <p className={ui.warning}>
            跨车 Home 正在等待确认，当前不会采用，也不会发送替换指令。
          </p>
        )}
        {operation !== null && (
          <div role="status">
            {operation.status === "succeeded"
              ? "地图包操作已完成。"
              : "地图包操作等待确认，请在详情中核查。"}
            <details>
              <summary>操作详情</summary>
              {operation.id} · {operation.status ?? "等待车端确认"}
            </details>
          </div>
        )}
      </section>
      {packageAction !== null &&
        createPortal(
          <div
            className={ui.modal}
            onKeyDown={(event) => {
              if (event.key === "Escape") setPackageAction(null);
            }}
          >
            <section
              className={ui.dialog}
              role="dialog"
              aria-modal="true"
              aria-label={
                packageAction === "export" ? "导出地图包" : "导入地图包"
              }
            >
              <h2>
                {packageAction === "export" ? "导出地图包" : "导入地图包"}
              </h2>
              <p>
                使用车端受控目录中的地图包名称；此入口不上传或下载文件字节。
              </p>
              <label>
                地图包名称
                <input
                  autoFocus
                  aria-label="地图包名称"
                  value={name}
                  onChange={(event) => setName(event.target.value)}
                />
              </label>
              <div className={ui.dialogActions}>
                <Button onClick={() => setPackageAction(null)}>取消</Button>
                {packageAction === "export" ? (
                  <Button
                    variant="primary"
                    disabled={!live || !validName || snapshot.map_ref === null}
                    onClick={() => {
                      void mutate("map.export");
                      setPackageAction(null);
                    }}
                  >
                    导出地图包
                  </Button>
                ) : (
                  <ImportDialog
                    disabled={!live || !snapshot.localized || !validName}
                    onImport={() => {
                      void mutate("map.import");
                      setPackageAction(null);
                    }}
                  />
                )}
              </div>
              {!snapshot.localized && packageAction === "import" && (
                <p className={ui.warning}>完成定位后才可导入地图包。</p>
              )}
            </section>
          </div>,
          overlayTarget ?? document.body,
        )}
    </>
  );
}
