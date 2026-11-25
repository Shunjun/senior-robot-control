import { useEffect, useRef, useState, type ReactElement } from "react";
import type { RobotSnapshot } from "../../api/protocol";
import type { VehiclePort } from "../../api/vehicleState";
import { useSpeedUnit, displaySpeed } from "../../display/speedUnits";
import { IconButton } from "../../components/ui/IconButton";
import { X, Navigation2, Gauge } from "lucide-react";
import { FloatingCard } from "../../components/ui/FloatingCard";
import { SegmentedControl } from "../../components/ui/SegmentedControl";
import { PerformancePanel } from "./PerformancePanel";
import { useMeasuredSpeed, useVehicleView } from "./useVehicleView";
import { init, chartTheme } from "./charts";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  panel: [
    "status-overlay-panel [&&]:pt-[17px] [&&]:pr-[17px] [&&]:pb-[17px] [&&]:pl-[17px] pt-[17px] pr-[17px] pb-[17px] pl-[17px] bg-none bg-card border",
    "border-solid border-line rounded-card shadow-ui-card [&>div[role=group]]:mt-3 [&>div[role=group]]:mr-0 [&>div[role=group]]:mb-3 [&>div[role=group]]:ml-0"
  ].join(" "),
  head: [
    "status-overlay-head flex items-center justify-between gap-2.5 [&_h2]:text-base [&_h2]:font-medium [&_h2]:mt-0",
    "[&_h2]:mr-0 [&_h2]:mb-[5px] [&_h2]:ml-0 [&_button]:pt-1 [&_button]:pr-[7px] [&_button]:pb-1 [&_button]:pl-[7px] [&_button]:bg-none",
    "[&_button]:bg-transparent [&_button]:border-0 [&_button]:border-none [&_button]:border-current [&_button]:text-quiet"
  ].join(" "),
  motionReadings: [
    "status-overlay-motion-readings flex items-center justify-between gap-2.5"
  ].join(" "),
  tiny: [
    "status-overlay-tiny text-xs text-muted"
  ].join(" "),
  speed: [
    "status-overlay-speed font-light text-4xl leading-[normal] font-number tracking-[-1.4px] [&_small]:font-normal [&_small]:text-xs",
    "[&_small]:leading-[normal] [&_small]:font-ui [&_small]:tracking-[0] [&_small]:text-muted"
  ].join(" "),
  turn: [
    "status-overlay-turn text-xs text-muted"
  ].join(" "),
  chart: [
    "status-overlay-chart w-full h-[56px] mt-2 mr-0 mb-2 ml-0"
  ].join(" "),
  attitude: [
    "status-overlay-attitude grid grid-cols-[1fr_1fr] gap-2.5 h-[112px] mt-0.5"
  ].join(" "),
  attitudeMetric: [
    "status-overlay-attitude-metric flex flex-col items-center justify-center gap-[5px]"
  ].join(" "),
  gauge: [
    "status-overlay-gauge grid place-items-center w-[72px] h-[72px] border border-solid border-status-overlay-gauge-border-color",
    "rounded-full text-status-overlay-gauge-color [&[data-unavailable=true]]:text-status-overlay-gauge-color-2 [&[data-unavailable=true]]:opacity-[0.65]"
  ].join(" "),
  attitudeCaption: [
    "status-overlay-attitude-caption grid grid-cols-[auto_auto] gap-[2px_4px] text-xs text-status-overlay-attitude-caption-color"
  ].join(" "),
  heading: [
    "status-overlay-heading text-xs text-status-overlay-attitude-caption-color"
  ].join(" "),
  cpu: [
    "status-overlay-cpu flex items-center justify-between border-t [border-top-style:solid] border-t-workspace-legend-border-color pt-2.5",
    "mt-3 text-xs"
  ].join(" "),
  track: [
    "status-overlay-track h-[4px] rounded-2xs bg-none bg-workspace-legend-border-color mt-2 mr-0 mb-2",
    "ml-0 [&_i]:block [&_i]:h-[4px] [&_i]:bg-none [&_i]:bg-amber [&_i]:rounded-2xs"
  ].join(" "),
} as const;

interface StatusOverlayProps {
  onClose?: (() => void) | undefined;
  snapshot: RobotSnapshot;
  port: VehiclePort | undefined;
  tab: "motion" | "performance";
  onTabChange: (tab: "motion" | "performance") => void;
  performanceExpired: boolean;
  visible: boolean;
}
interface Sample {
  sequence: number;
  value: number | null;
}
export function StatusOverlay({
  snapshot,
  onClose,
  port,
  tab,
  onTabChange,
  performanceExpired,
  visible,
}: StatusOverlayProps): ReactElement {
  const [samples, setSamples] = useState<Sample[]>([]);
  const chart = useRef<HTMLDivElement>(null);
  const chartInstance = useRef<ReturnType<typeof init> | null>(null);
  const view = useVehicleView(port);
  const unit = useSpeedUnit();
  const body = view.frame?.body_reference;
  const measured = useMeasuredSpeed(view);
  const turn =
    measured !== null && body && !body.stale ? body.average.wz : null;
  const cpu =
    snapshot.connected && !snapshot.stale && !performanceExpired
      ? (snapshot.performance?.cpu_percent ?? null)
      : null;
  useEffect(() => {
    if (!view.frame) return;
    setSamples((previous) =>
      previous.at(-1)?.sequence === view.frame!.sequence
        ? previous
        : [
            ...previous,
            { sequence: view.frame!.sequence, value: measured },
          ].slice(-60),
    );
  }, [view.frame?.sequence, measured]);
  useEffect(() => {
    if (measured === null)
      setSamples((previous) =>
        previous.length && previous.at(-1)?.value !== null
          ? [...previous, { sequence: -1, value: null }].slice(-60)
          : previous,
      );
  }, [measured]);
  useEffect(() => {
    if (!visible || !chart.current || tab !== "motion") return;
    // The canvas chart is only mounted in a browser with layout; no invented history.
    if (chart.current.clientWidth === 0) return;
    const instance = init(chart.current, chartTheme);
    chartInstance.current = instance;
    const observer = new ResizeObserver(() => instance.resize());
    observer.observe(chart.current);
    return () => {
      observer.disconnect();
      instance.dispose();
      chartInstance.current = null;
    };
  }, [tab, visible]);
  useEffect(() => {
    chartInstance.current?.setOption({
      animation: false,
      grid: { left: 4, right: 4, top: 8, bottom: 8 },
      xAxis: {
        type: "category",
        show: false,
        data: samples.map((_, index) => index),
      },
      yAxis: { type: "value", show: false, scale: true },
      series: [
        {
          type: "line",
          showSymbol: true,
          symbolSize: 3,
          connectNulls: false,
          data: samples.map((sample) =>
            sample.value === null ? null : displaySpeed(sample.value, unit),
          ),
        },
      ],
    });
  }, [samples, unit, tab, visible]);
  const yaw =
    snapshot.connected &&
    !snapshot.stale &&
    snapshot.localized &&
    snapshot.map_generation !== undefined
      ? snapshot.robot_pose?.yaw
      : undefined;
  return (
    <FloatingCard className={ui.panel} aria-label="车辆状态浮层">
      <header className={ui.head}>
        <h2>车辆状态</h2>
        <IconButton label="关闭状态" onClick={onClose}>
          <X />
        </IconButton>
      </header>
      <SegmentedControl
        label="状态分类"
        value={tab}
        onChange={onTabChange}
        options={[
          { value: "motion", label: "速度与姿态" },
          { value: "performance", label: "性能" },
        ]}
      />
      <div hidden={tab !== "performance"}>
        <PerformancePanel
          visible={visible && tab === "performance"}
          reading={snapshot.performance}
          live={snapshot.connected && !snapshot.stale}
          sampleExpired={performanceExpired}
        />
      </div>
      {tab === "motion" && (
        <div>
          <div className={ui.motionReadings}>
            <div>
              <span className={ui.tiny}>线速度</span>
              <div className={ui.speed}>
                {measured === null
                  ? "不可用"
                  : displaySpeed(measured, unit).toFixed(2)}{" "}
                <small>{unit}</small>
              </div>
            </div>
            <div className={ui.turn}>
              转向 <span>{turn === null ? "不可用" : turn.toFixed(2)}</span>
              <br />
              rad/s
            </div>
          </div>
          <div
            className={ui.chart}
            ref={chart}
            aria-label="已收到的速度样本"
          />
          <div className={ui.tiny}>最近速度 · 真实测量</div>
          <div className={ui.attitude}>
            <div className={ui.attitudeMetric}>
              <div className={ui.gauge} data-unavailable="true">
                <Gauge size={54} strokeWidth={1} />
              </div>
              <div className={ui.attitudeCaption}>
                <span>Pitch 俯仰</span>
                <span>不可用</span>
                <span>Roll 横滚</span>
                <span>不可用</span>
              </div>
            </div>
            <div className={ui.attitudeMetric}>
              <div
                className={ui.gauge}
                data-unavailable={yaw === undefined}
              >
                <Navigation2
                  size={39}
                  strokeWidth={1}
                  style={
                    yaw === undefined
                      ? undefined
                      : { transform: `rotate(${90 - (yaw * 180) / Math.PI}deg)` }
                  }
                />
              </div>
              <div className={ui.heading}>
                <span>Yaw 朝向</span>{" "}
                <span>
                  {yaw === undefined
                    ? "不可用"
                    : `${((yaw * 180) / Math.PI).toFixed(1)}°`}
                </span>
              </div>
            </div>
          </div>
          <div className={ui.cpu}>
            <span>整机 CPU</span>
            <span>{cpu === null ? "不可用" : `${cpu.toFixed(1)}%`}</span>
          </div>
          <div className={ui.track}>
            {cpu !== null && <i style={{ width: `${cpu}%` }} />}
          </div>
        </div>
      )}
    </FloatingCard>
  );
}
