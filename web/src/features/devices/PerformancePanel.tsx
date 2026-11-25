import { useEffect, useState, useRef, type ReactElement } from "react";
import type { PerformanceReading } from "../../api/protocol";
import { init, chartTheme } from "./charts";
import styles from "./PerformancePanel.module.css";

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
  return (
    <section className={styles.panel} aria-label="车辆性能">
      <header>
        <strong>车辆性能</strong>
        <span>
          {!live
            ? "离线 · 最后样本"
            : expired || sampleExpired
              ? "样本已过期"
              : reading === undefined
                ? "等待数据"
                : "1 秒采样"}
        </span>
      </header>
      {reading === undefined ? (
        <p>车端尚未提供性能数据。</p>
      ) : (
        <>
          <div className={styles.machine} data-stale={stale}>
            <span>
              整机 CPU <b>{percent(stale ? null : reading.cpu_percent)}</b>{" "}
              <small>{reading.cpu_count || "—"} 核</small>
            </span>
            <span>
              内存 <b>{memory(stale ? null : reading.memory_used_bytes)}</b> /{" "}
              {memory(stale ? null : reading.memory_total_bytes)}
            </span>
          </div>
          <div
            className={styles.chart}
            ref={chart}
            aria-label="已收到的 CPU 样本"
          />
          <p>
            {samples.length < 2
              ? "历史样本不足，等待后续真实采样。"
              : `页面已接收 ${samples.length} 个真实样本 · sequence ${reading.sequence}`}
          </p>
          <table data-stale={stale}>
            <thead>
              <tr>
                <th>模块</th>
                <th>CPU</th>
                <th>进程 RSS 合计</th>
              </tr>
            </thead>
            <tbody>
              {reading.modules.map((row) => (
                <tr key={row.name}>
                  <th>
                    {LABELS[row.name]}{" "}
                    <small>
                      {row.status === "inactive"
                        ? "未启动"
                        : row.status === "unavailable"
                          ? "不可用"
                          : `${row.process_count} 进程`}
                    </small>
                  </th>
                  <td>{percent(stale ? null : row.cpu_percent)}</td>
                  <td>{memory(stale ? null : row.rss_bytes)}</td>
                </tr>
              ))}
            </tbody>
          </table>
          <p>
            CPU 按全部可见核心归一。统计范围为系统 procfs；容器配额另计。RSS
            可能重复计算共享页；进程内线程共享内存。模块仅统计已登记进程，编码线程暂未拆分。
          </p>
        </>
      )}
    </section>
  );
}
