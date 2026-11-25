import { useEffect, useRef, useState, type ReactElement } from "react";
import type { RobotSnapshot } from "../../api/protocol";
import type { VehiclePort } from "../../api/vehicleState";
import { useSpeedUnit, displaySpeed } from "../../display/speedUnits";
import { IconButton } from "../../components/ui/IconButton";
import { X } from "lucide-react";
import { FloatingCard } from "../../components/ui/FloatingCard";
import { SegmentedControl } from "../../components/ui/SegmentedControl";
import { PerformancePanel } from "./PerformancePanel";
import { useMeasuredSpeed, useVehicleView } from "./useVehicleView";
import { init, chartTheme } from "./charts";
import styles from "./StatusOverlay.module.css";
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
    <FloatingCard className={styles.panel} aria-label="车辆状态浮层">
      <header className={styles.head}>
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
          <div className={styles.speed}>
            {measured === null
              ? "不可用"
              : displaySpeed(measured, unit).toFixed(2)}{" "}
            <small>{unit}</small>
          </div>
          <p>
            真实测量速度 · {body && !body.stale ? body.source : "轮速里程计"}
          </p>
          <div
            className={styles.chart}
            ref={chart}
            aria-label="已收到的速度样本"
          />
          <dl>
            <dt>Yaw 朝向</dt>
            <dd>
              {yaw === undefined
                ? "不可用"
                : `${((yaw * 180) / Math.PI).toFixed(1)}°`}
            </dd>
            <dt>Roll 横滚</dt>
            <dd>不可用</dd>
            <dt>Pitch 俯仰</dt>
            <dd>不可用</dd>
            <dt>整机 CPU</dt>
            <dd>{cpu === null ? "不可用" : `${cpu.toFixed(1)}%`}</dd>
          </dl>
        </div>
      )}
    </FloatingCard>
  );
}
