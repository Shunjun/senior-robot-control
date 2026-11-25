import { newId } from "../../ids";
import { useState, type ReactElement } from "react";

import {
  CommandError,
  describeCommandError,
  type RobotLink,
} from "../../api/client";
import type {
  MapRef,
  ResponsePayload,
  RobotSnapshot,
} from "../../api/protocol";
import { useVehicleView } from "../devices/useVehicleView";
import type { TaskAction } from "../../api/vehicleState";
import { Button } from "../../components/ui/Button";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  home: [
    "map-home relative bg-none bg-transparent [&_h3]:text-[15px] [&_h3]:font-medium [&_h3]:mt-0 [&_h3]:mr-0",
    "[&_h3]:mb-[5px] [&_h3]:ml-0 max-[460.001px]:[&_h3]:text-[14px]"
  ].join(" "),
  navigationHead: [
    "map-navigation-head flex items-center justify-between gap-2.5"
  ].join(" "),
  english: [
    "map-english font-normal text-[9px] leading-[normal] font-mono tracking-[1px] text-english"
  ].join(" "),
  instruction: [
    "map-instruction mt-3 mr-0 mb-3 ml-0 pt-2.5 pr-2.5 pb-2.5",
    "pl-2.5 bg-none bg-control rounded-md text-[11px] leading-[1.8]"
  ].join(" "),
  goalAction: [
    "map-goal-action w-full"
  ].join(" "),
  actions: [
    "map-actions flex flex-wrap gap-1.5 mt-3 [&_button]:text-[11px]"
  ].join(" "),
  positionActions: [
    "map-position-actions border-t [border-top-style:solid] border-t-line-soft pt-[13px] pr-0 pb-0 pl-0",
    "mt-4 [&_.map-actions]:mt-3 [&_button]:text-[10px] [&_button]:mt-[9px] [&_button]:px-[9px] [&_button]:py-[7px]"
  ].join(" "),
  quiet: [
    "map-quiet text-muted text-[11px] leading-[1.8] mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  navigationDetails: [
    "map-navigation-details text-muted text-[9px] [&_summary]:cursor-pointer [&[open]>p]:mt-2"
  ].join(" "),
  failure: [
    "map-failure mt-2 mr-0 mb-0 ml-0 pt-2 pr-2.5 pb-2",
    "pl-2.5 rounded-md text-[11px] text-warning-ink bg-none bg-red-wash"
  ].join(" "),
  confirmed: [
    "map-confirmed mt-2 mr-0 mb-0 ml-0 pt-2 pr-2.5 pb-2",
    "pl-2.5 rounded-md bg-none bg-control text-[11px]"
  ].join(" "),
} as const;

export interface HomeControlsProps {
  /** Null until the robot's first snapshot arrives. Nothing is enabled without a live one. */
  snapshot: RobotSnapshot | null;
  link: RobotLink;
  homeMapAvailable?: boolean;
  onPickGoal?: () => void;
  onCalibrate?: () => void;
  onEditHome?: () => void;
  onRefresh?: () => void;
  goalAvailable?: boolean;
  navigationUnavailable?: string;
  directNavigation?: {
    onReturnHome: () => void;
    pending: boolean;
  };
}

type Outcome = { kind: "ok"; text: string } | { kind: "error"; text: string };

type ActionName = "resume" | "cancel" | "home";

const ACTION_LABEL: Record<ActionName, string> = {
  resume: "继续导航",
  cancel: "取消导航",
  home: "设为 Home",
};

const STATUS_TEXT: Record<string, string> = {
  accepted: "已受理，等待执行",
  dispatching: "正在下发",
  running: "执行中",
  succeeded: "已完成",
  failed: "执行失败",
  needs_review: "需要人工核查",
};

function sameRef(a: MapRef, b: MapRef): boolean {
  return a.map_id === b.map_id && a.reference_revision === b.reference_revision;
}

function isLive(snapshot: RobotSnapshot | null): snapshot is RobotSnapshot {
  return snapshot !== null && !snapshot.stale && snapshot.connected;
}

/** Every reason navigation may not start. Empty means it may. */
function navigationBlocks(snapshot: RobotSnapshot | null): string[] {
  if (snapshot === null) return ["等待车端快照，收到之前不能下达导航指令。"];
  if (!isLive(snapshot))
    return ["缓存快照，不是车辆当前状态，不能下达导航指令。"];
  const reasons: string[] = [];
  if (snapshot.home === null) {
    reasons.push("Home 未设置，不能开始导航。");
  } else if (
    snapshot.map_ref === null ||
    !sameRef(snapshot.home.map_ref, snapshot.map_ref)
  ) {
    reasons.push("Home 不属于当前地图，不能开始导航。");
  } else if (snapshot.home.status === "pending_confirmation") {
    reasons.push("跨车 Home 尚未确认，不能开始导航。");
  }
  if (!snapshot.localized) reasons.push("导航需要先完成定位。");
  return reasons;
}

export function HomeControls({
  snapshot,
  link,
  homeMapAvailable = true,
  navigationUnavailable,
  directNavigation,
  onPickGoal,
  onCalibrate,
  onEditHome,
  onRefresh,
  goalAvailable = false,
}: HomeControlsProps): ReactElement {
  const [pending, setPending] = useState<ActionName | null>(null);
  const [outcome, setOutcome] = useState<Outcome | null>(null);
  const [confirmReplace, setConfirmReplace] = useState(false);

  const live = isLive(snapshot);
  const vehicle = useVehicleView(link.vehicle);
  const task = vehicle.frame?.task;
  const canTask = (action: TaskAction): boolean =>
    live &&
    vehicle.connected &&
    vehicle.available &&
    vehicle.frame?.capabilities.includes("task.execution.v1") === true &&
    task?.allowed_actions.includes(action) === true;

  async function runTask(action: "resume" | "cancel"): Promise<void> {
    if (!link.vehicle || !task || !canTask(action) || pending !== null) return;
    const current = link.vehicle.getSnapshot().frame?.task;
    if (current?.task_id !== task.task_id || current.version !== task.version)
      return;
    setPending(action);
    setOutcome(null);
    try {
      await link.vehicle.taskCommand(action);
      setOutcome({ kind: "ok", text: `车端已应答：${ACTION_LABEL[action]}` });
    } catch (error) {
      setOutcome({
        kind: "error",
        text: `${ACTION_LABEL[action]}未完成：${String(error)}`,
      });
    } finally {
      setPending(null);
    }
  }
  const blocks = navigationBlocks(snapshot);
  if (navigationUnavailable !== undefined) blocks.push(navigationUnavailable);
  const canNavigate = blocks.length === 0;
  const homeWaiting = snapshot?.home?.status === "pending_confirmation";
  const canSetHome =
    homeMapAvailable &&
    live &&
    snapshot.localized &&
    snapshot.map_ref !== null &&
    !homeWaiting;

  async function run(
    action: ActionName,
    type: string,
    payload: unknown,
  ): Promise<void> {
    setPending(action);
    setOutcome(null);
    try {
      const response = await link.request<ResponsePayload>(type, payload);
      const status = response.status ?? null;
      const detail =
        status === null ? "" : `（${STATUS_TEXT[status] ?? status}）`;
      setOutcome({
        kind: "ok",
        text: `车端已应答：${ACTION_LABEL[action]}${detail}`,
      });
    } catch (error) {
      const reason =
        error instanceof CommandError
          ? describeCommandError(error.code)
          : "无法发送指令";
      setOutcome({
        kind: "error",
        text: `${ACTION_LABEL[action]}未完成：${reason}`,
      });
    } finally {
      setPending((current) => (current === action ? null : current));
    }
  }

  function setHome(): void {
    if (!canSetHome || !isLive(snapshot)) return;
    if (snapshot.home != null && !confirmReplace) {
      setConfirmReplace(true);
      return;
    }
    setConfirmReplace(false);
    void run("home", "home.set", {
      operation_id: newId(),
      expected_revision: snapshot.home_revision,
      body: {},
    });
  }

  const home = snapshot?.home ?? null;
  const localization =
    snapshot === null
      ? "未知"
      : snapshot.localized
        ? "已定位"
        : "重新定位未完成：车辆还没有匹配到地图";

  return (
    <section className={ui.home} aria-label="Home 与导航">
      <header className={ui.navigationHead}>
        <h3>导航与定位</h3>
        <span className={ui.english}>NAVIGATE</span>
      </header>
      <div className={ui.instruction}>
        <span
          role="status"
          aria-label="定位状态"
          data-localized={snapshot?.localized === true}
        >
          {snapshot?.localized ? "● 定位正常" : localization}
        </span>
        <br />
        {directNavigation?.pending
          ? "导航操作进行中，请等待车辆确认。"
          : "在地图上选位置，再点击设置朝向。"}
      </div>
      <Button
        variant="primary"
        className={ui.goalAction}
        disabled={!live || !goalAvailable || directNavigation?.pending === true}
        onClick={onPickGoal}
      >
        在地图上设置目标
      </Button>
      <div className={ui.actions}>
        <Button
          disabled={
            !canNavigate || !directNavigation || directNavigation.pending
          }
          title={blocks.join(" ")}
          onClick={directNavigation?.onReturnHome}
        >
          返回 Home
        </Button>
        <Button
          disabled={!canTask("cancel") || pending !== null}
          onClick={() => void runTask("cancel")}
        >
          取消导航
        </Button>
        {canTask("resume") && (
          <Button
            disabled={pending !== null}
            onClick={() => void runTask("resume")}
          >
            继续导航
          </Button>
        )}
      </div>
      <div className={ui.positionActions}>
        <p className={ui.quiet}>车辆位置与 Home 独立编辑</p>
        <div className={ui.actions}>
          <Button
            disabled={!live || directNavigation?.pending === true}
            onClick={onCalibrate}
          >
            校准车辆位置
          </Button>
          <Button
            disabled={
              !canSetHome || !onEditHome || directNavigation?.pending === true
            }
            onClick={onEditHome}
          >
            编辑 Home
          </Button>
        </div>
      </div>
      <details className={ui.navigationDetails}>
        <summary>定位与 Home 详情</summary>
        <p className={ui.quiet}>
          Home：
          {home === null ? "未设置" : `${home.waypoint_id} · ${home.status}`}
        </p>
        {blocks.map((reason) => (
          <p className={ui.quiet} key={reason}>
            {reason}
          </p>
        ))}
        {homeWaiting && (
          <p className={ui.quiet}>
            该 Home 正在等待确认，控制台不会发送替换指令。
          </p>
        )}
        {live && !canSetHome && (
          <p className={ui.quiet}>
            {homeMapAvailable
              ? "设置 Home 需要车辆已定位并具有实时地图。"
              : "车端未开放当前地图的 Home 写入。"}
          </p>
        )}
        <div className={ui.actions}>
          {confirmReplace ? (
            <>
              <Button
                variant="outline"
                disabled={!canSetHome}
                onClick={setHome}
              >
                确认替换 Home
              </Button>
              <Button onClick={() => setConfirmReplace(false)}>
                保留原 Home
              </Button>
            </>
          ) : (
            <Button
              disabled={!canSetHome || pending === "home"}
              onClick={setHome}
            >
              设为 Home
            </Button>
          )}
          {!canTask("resume") && <Button disabled>继续导航</Button>}
          {onRefresh && <Button onClick={onRefresh}>刷新地图</Button>}
        </div>
      </details>
      {outcome && (
        <p
          className={
            outcome.kind === "error" ? ui.failure : ui.confirmed
          }
          role={outcome.kind === "error" ? "alert" : "status"}
        >
          {outcome.text}
        </p>
      )}
    </section>
  );
}
