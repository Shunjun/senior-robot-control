import { useEffect, useState, useRef, type ReactElement } from "react";
import type { PerformanceReading } from "../../api/protocol";
import { init, chartTheme } from "./charts";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  panel: [
    "performance-panel-panel pt-0 pr-0 pb-0 pl-0 text-xs"
  ].join(" "),
  machine: [
    "performance-panel-machine grid grid-cols-[1fr_1fr] gap-[15px] [&>div>span]:text-muted [&>div>span]:text-xs"
  ].join(" "),
  big: [
    "performance-panel-big font-light text-4xl leading-[normal] font-number tracking-[-1.4px] [&_small]:font-normal [&_small]:text-xs",
    "[&_small]:leading-[normal] [&_small]:font-ui [&_small]:tracking-[0] [&_small]:text-muted"
  ].join(" "),
  chart: [
    "performance-panel-chart h-[56px] w-full mt-2 mr-0 mb-2 ml-0"
  ].join(" "),
  metric: [
    "performance-panel-metric flex items-center justify-between gap-2.5 pt-2.5 pr-0 pb-2.5",
    "pl-0 text-xs border-t [border-top-style:solid] border-t-performance-panel-metric-border-top-color"
  ].join(" "),
  freshness: [
    "performance-panel-freshness text-performance-panel-freshness-color text-xs mt-2.5 mr-0 mb-0 ml-0"
  ].join(" "),
  details: [
    "performance-panel-details text-performance-panel-freshness-color text-xs mt-2.5 mr-0 mb-0 ml-0 [&_summary]:cursor-pointer",
    "[&_p]:text-xs [&_p]:leading-[1.8] [&_p]:[overflow-wrap:anywhere]"
  ].join(" "),
} as const;

interface PerformancePanelProps {
  reading: PerformanceReading | undefined;
  live: boolean;
  sampleExpired?: boolean;
  visible?: boolean;
}

const LABELS = { video: "视频采集/压缩", navigation: "导航", mapping: "建图" };
const percent = (value: number | null): string =>
  value === null ? "—" : `${value.toFixed(1)}%`;
const memory = (value: number | null): string =>
  value === null ? "—" : `${(value / 1048576).toFixed(0)} MiB`;

/** Sample freshness uses local receipt time, never the vehicle's wall clock. */
export function PerformancePanel({
  reading,
  live,
  sampleExpired = false,
  visible = true,
}: PerformancePanelProps): ReactElement {
  const [expired, setExpired] = useState(false);
  useEffect(() => {
    setExpired(false);
    if (reading === undefined) return;
    const timer = setTimeout(() => setExpired(true), 3500);
    return () => clearTimeout(timer);
  }, [reading?.sequence]);
  const [samples, setSamples] = useState<
    Array<{ sequence: number; cpu: number | null }>
  >([]);
  const chart = useRef<HTMLDivElement>(null);
  const instance = useRef<ReturnType<typeof init> | null>(null);
  useEffect(() => {
    if (!reading) return;
    setSamples((previous) =>
      previous.at(-1)?.sequence === reading.sequence
        ? previous
        : [
            ...previous,
            {
              sequence: reading.sequence,
              cpu: live ? reading.cpu_percent : null,
            },
          ].slice(-60),
    );
  }, [reading?.sequence]);
  useEffect(() => {
    if (!visible || !chart.current?.clientWidth) return;
    const next = init(chart.current, chartTheme);
    instance.current = next;
    const observer = new ResizeObserver(() => next.resize());
    observer.observe(chart.current);
    return () => {
      observer.disconnect();
      next.dispose();
      instance.current = null;
    };
  }, [visible, reading !== undefined]);
  useEffect(() => {
    instance.current?.setOption({
      animation: false,
      grid: { left: 4, right: 4, top: 8, bottom: 8 },
      xAxis: {
        type: "category",
        show: false,
        data: samples.map((s) => s.sequence),
      },
      yAxis: { type: "value", show: false, min: 0, max: 100 },
      series: [
        {
          type: "line",
          connectNulls: false,
          showSymbol: true,
          symbolSize: 3,
          data: samples.map((s) => s.cpu),
        },
      ],
    });
  }, [samples, visible]);
  const stale = !live || expired || sampleExpired;
  if (!visible) return <section hidden />;
  const cpu = stale ? null : (reading?.cpu_percent ?? null);
  const used = stale ? null : (reading?.memory_used_bytes ?? null);
  const total = stale ? null : (reading?.memory_total_bytes ?? null);
  return (
    <section className={ui.panel} aria-label="车辆性能">
      <div className={ui.machine}>
        <div>
          <span>整机 CPU</span>
          <div className={ui.big} aria-label="整机 CPU 读数">
            {cpu === null ? "不可用" : cpu.toFixed(1)}
            <small>{cpu === null ? "" : "%"}</small>
          </div>
        </div>
        <div>
          <span>内存</span>
          <div className={ui.big} aria-label="内存读数">
            {used === null ? "不可用" : (used / 1073741824).toFixed(1)}
            <small>{used === null ? "" : "GB"}</small>
          </div>
        </div>
      </div>
      <div
        className={ui.chart}
        ref={chart}
        aria-label="已收到的 CPU 样本"
      />
      {reading?.modules.map((row) => (
        <div className={ui.metric} key={row.name}>
          <span>{LABELS[row.name]}</span>
          <span>
            {stale
              ? "不可用"
              : row.status === "inactive"
                ? "未启动"
                : row.status === "unavailable"
                  ? "不可用"
                  : `${percent(row.cpu_percent)} CPU`}
          </span>
        </div>
      ))}
      <div className={ui.metric}>
        <span>内存预算</span>
        <span>
          {used === null || total === null
            ? "不可用"
            : `${(used / 1073741824).toFixed(1)} / ${(total / 1073741824).toFixed(1)} GB`}
        </span>
      </div>
      <p className={ui.freshness}>
        {!live
          ? "离线 · 最后样本"
          : expired || sampleExpired
            ? "样本已过期"
            : reading === undefined
              ? "等待数据"
              : `${reading.interval_s} 秒采样 · 车端真实数据`}
      </p>
      <details className={ui.details}>
        <summary>性能采样详情</summary>
        {reading === undefined ? (
          <p>车端尚未提供性能数据。</p>
        ) : (
          <>
            <p>
              {samples.length < 2
                ? "历史样本不足，等待后续真实采样。"
                : `页面已接收 ${samples.length} 个真实样本`}{" "}
              · sequence {reading.sequence}
            </p>
            <p>
              {reading.cpu_count || "—"} 核 · 内存 <span>{memory(used)}</span> /{" "}
              {memory(total)}
            </p>
            {reading.modules.map((row) => (
              <p key={row.name}>
                {LABELS[row.name]} · {row.process_count} 进程 · RSS{" "}
                {memory(stale ? null : row.rss_bytes)}
              </p>
            ))}
            <p>
              CPU 按全部可见核心归一。统计范围为系统 procfs；容器配额另计。RSS
              可能重复计算共享页；进程内线程共享内存。模块仅统计已登记进程，编码线程暂未拆分。
            </p>
          </>
        )}
      </details>
    </section>
  );
}
