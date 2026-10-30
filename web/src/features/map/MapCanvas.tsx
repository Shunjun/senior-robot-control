import {
  useEffect,
  useId,
  useMemo,
  useRef,
  useState,
  type KeyboardEvent,
  type PointerEvent,
  type ReactElement,
} from "react";

import type { MapRef } from "../../api/protocol";
import styles from "./Map.module.css";

// ---------------------------------------------------------------- types

export interface Point {
  x: number;
  y: number;
}

/** A pose in the map frame: metres and radians, yaw counter-clockwise from +x. */
export interface Pose extends Point {
  yaw: number;
}

/** One cell of the occupancy layer (mirror of `CellEvidence`). */
export interface CellEvidence {
  /** Integer grid index (column, row). */
  cell: readonly [number, number];
  belief: number;
  hit_count: number;
  free_count: number;
  change_source: "geometric_confirmation" | "observed_free_space";
}

/**
 * The occupancy layer with its placement. `origin` is the map frame pose of grid cell (0, 0)'s
 * lower-left corner, as in a map.yaml `origin`; `resolution` is metres per cell. The grid axes
 * are rotated by `origin.yaw` against the map frame.
 */
export interface OccupancySnapshot {
  map_ref: MapRef | null;
  origin: Pose;
  resolution: number;
  cells: readonly CellEvidence[];
}

export interface SafetyZone {
  id: string;
  label: string;
  kind: "keep_out" | "slow" | "observe";
  points: readonly Point[];
}

/** The window onto the map frame. `zoom` is pixels per metre; screen y grows downwards. */
export interface MapView {
  center: Point;
  zoom: number;
  width: number;
  height: number;
}

type Layer = "compare" | "filtered" | "raw";

// ---------------------------------------------------------------- coordinates

export const MIN_ZOOM = 2;
export const MAX_ZOOM = 5_000;
/** Belief at or above this counts as occupied in the filtered layer. */
export const OCCUPIED_BELIEF = 0.5;

/** Grid coordinates (cells, fractional allowed) to the map frame. */
export function gridToWorld(map: OccupancySnapshot, gx: number, gy: number): Point {
  const { origin, resolution } = map;
  const cos = Math.cos(origin.yaw);
  const sin = Math.sin(origin.yaw);
  const lx = gx * resolution;
  const ly = gy * resolution;
  return { x: origin.x + cos * lx - sin * ly, y: origin.y + sin * lx + cos * ly };
}

/** The inverse of `gridToWorld`. */
export function worldToGrid(map: OccupancySnapshot, point: Point): Point {
  const { origin, resolution } = map;
  const cos = Math.cos(origin.yaw);
  const sin = Math.sin(origin.yaw);
  const dx = point.x - origin.x;
  const dy = point.y - origin.y;
  return { x: (cos * dx + sin * dy) / resolution, y: (-sin * dx + cos * dy) / resolution };
}

export function worldToScreen(view: MapView, point: Point): Point {
  return {
    x: view.width / 2 + (point.x - view.center.x) * view.zoom,
    y: view.height / 2 - (point.y - view.center.y) * view.zoom,
  };
}

export function screenToWorld(view: MapView, point: Point): Point {
  return {
    x: view.center.x + (point.x - view.width / 2) / view.zoom,
    y: view.center.y - (point.y - view.height / 2) / view.zoom,
  };
}

export function gridToScreen(view: MapView, map: OccupancySnapshot, gx: number, gy: number): Point {
  return worldToScreen(view, gridToWorld(map, gx, gy));
}

function clampZoom(zoom: number): number {
  return Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, zoom));
}

/** A view that shows every cell with a margin. An empty map is centred on its origin. */
export function fitView(map: OccupancySnapshot, width: number, height: number): MapView {
  if (map.cells.length === 0) {
    return { center: { x: map.origin.x, y: map.origin.y }, zoom: 50, width, height };
  }
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const entry of map.cells) {
    for (const [dx, dy] of [
      [0, 0],
      [1, 0],
      [0, 1],
      [1, 1],
    ] as const) {
      const corner = gridToWorld(map, entry.cell[0] + dx, entry.cell[1] + dy);
      minX = Math.min(minX, corner.x);
      maxX = Math.max(maxX, corner.x);
      minY = Math.min(minY, corner.y);
      maxY = Math.max(maxY, corner.y);
    }
  }
  const floor = map.resolution * 10;
  const spanX = Math.max(maxX - minX, floor);
  const spanY = Math.max(maxY - minY, floor);
  return {
    center: { x: (minX + maxX) / 2, y: (minY + maxY) / 2 },
    zoom: clampZoom(Math.min(width / spanX, height / spanY) * 0.85),
    width,
    height,
  };
}

function isRaw(entry: CellEvidence): boolean {
  return entry.hit_count > 0;
}

function isFiltered(entry: CellEvidence): boolean {
  return entry.belief >= OCCUPIED_BELIEF;
}

// ---------------------------------------------------------------- component

export interface MapCanvasProps {
  map: OccupancySnapshot;
  /** The map is a cached copy, not live. Picking is off. */
  stale: boolean;
  /** Initial viewport. Without it the map is fitted to the surface. */
  view?: MapView;
  robot?: Pose | null;
  home?: Pose | null;
  goal?: Pose | null;
  /** The pose being edited, drawn dashed. */
  draft?: Pose | null;
  trajectory?: readonly Point[];
  zones?: readonly SafetyZone[];
  /** A press (and optional drag for heading) on the map yields a pose instead of panning. */
  pickEnabled?: boolean;
  onPick?: (pose: Pose) => void;
}

const DEFAULT_WIDTH = 640;
const DEFAULT_HEIGHT = 420;
const ZOOM_STEP = 1.25;
const KEY_PAN_PX = 40;
const DRAG_THRESHOLD_PX = 4;
const SCALE_LENGTHS_M = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100] as const;

const INK = "#f2ead9";
const AMBER = "#ffb21a";
const RED = "#ec3a2f";

interface Gesture {
  pointerId: number;
  mode: "pan" | "pick";
  startScreen: Point;
  startCenter: Point;
  startWorld: Point;
  yaw: number | null;
}

function scaleBar(zoom: number): { meters: number; pixels: number } {
  const meters = SCALE_LENGTHS_M.find((length) => length * zoom >= 60) ?? 100;
  return { meters, pixels: meters * zoom };
}

function polar(from: Point, to: Point): number {
  // Screen y points down; the map's y points up.
  return Math.atan2(-(to.y - from.y), to.x - from.x);
}

export function MapCanvas({
  map,
  stale,
  view: initialView,
  robot = null,
  home = null,
  goal = null,
  draft = null,
  trajectory = [],
  zones = [],
  pickEnabled = false,
  onPick,
}: MapCanvasProps): ReactElement {
  const hintId = useId();
  const surfaceRef = useRef<HTMLDivElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const gestureRef = useRef<Gesture | null>(null);
  const [size, setSize] = useState({
    width: initialView?.width ?? DEFAULT_WIDTH,
    height: initialView?.height ?? DEFAULT_HEIGHT,
  });
  const [placement, setPlacement] = useState<{ center: Point; zoom: number }>(() => {
    const first = initialView ?? fitView(map, size.width, size.height);
    return { center: first.center, zoom: first.zoom };
  });
  const [layer, setLayer] = useState<Layer>("compare");
  const [pending, setPending] = useState<Pose | null>(null);

  const view: MapView = useMemo(
    () => ({ center: placement.center, zoom: placement.zoom, width: size.width, height: size.height }),
    [placement, size],
  );
  const picking = pickEnabled && !stale && onPick !== undefined;

  const mapId = map.map_ref?.map_id ?? null;
  const mapRevision = map.map_ref?.reference_revision ?? null;
  const fitted = useRef({ mapId, mapRevision });
  useEffect(() => {
    // A different map or revision starts from a fitted view, not the old one's framing.
    if (fitted.current.mapId === mapId && fitted.current.mapRevision === mapRevision) return;
    fitted.current = { mapId, mapRevision };
    const next = fitView(map, size.width, size.height);
    setPlacement({ center: next.center, zoom: next.zoom });
  }, [map, mapId, mapRevision, size.width, size.height]);

  // Follow the surface's real size where the browser can tell us.
  useEffect(() => {
    const element = surfaceRef.current;
    if (element === null || typeof ResizeObserver === "undefined") return;
    const observer = new ResizeObserver(() => {
      const rect = element.getBoundingClientRect();
      if (rect.width > 0 && rect.height > 0) {
        setSize({ width: Math.round(rect.width), height: Math.round(rect.height) });
      }
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  const counts = useMemo(() => {
    let raw = 0;
    let filtered = 0;
    let removed = 0;
    for (const entry of map.cells) {
      const inRaw = isRaw(entry);
      const inFiltered = isFiltered(entry);
      if (inRaw) raw += 1;
      if (inFiltered) filtered += 1;
      if (inRaw && !inFiltered) removed += 1;
    }
    return { raw, filtered, removed };
  }, [map.cells]);

  // Cells are drawn on a canvas; markers and zones are SVG on top.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (canvas === null) return;
    const context = canvas.getContext("2d");
    if (context === null) return;
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(size.width * ratio);
    canvas.height = Math.round(size.height * ratio);
    context.setTransform(1, 0, 0, 1, 0, 0);
    context.clearRect(0, 0, canvas.width, canvas.height);
    const p0 = gridToScreen(view, map, 0, 0);
    const px = gridToScreen(view, map, 1, 0);
    const py = gridToScreen(view, map, 0, 1);
    context.setTransform(
      (px.x - p0.x) * ratio,
      (px.y - p0.y) * ratio,
      (py.x - p0.x) * ratio,
      (py.y - p0.y) * ratio,
      p0.x * ratio,
      p0.y * ratio,
    );
    for (const entry of map.cells) {
      const inRaw = isRaw(entry);
      const inFiltered = isFiltered(entry);
      let color: string | null = null;
      if (layer === "raw") color = inRaw ? AMBER : null;
      else if (layer === "filtered") color = inFiltered ? INK : null;
      else if (inFiltered) color = INK;
      else if (inRaw) color = RED;
      if (color === null) continue;
      context.fillStyle = color;
      context.fillRect(entry.cell[0], entry.cell[1], 1, 1);
    }
  }, [map, view, layer, size.width, size.height]);

  // Wheel zoom about the cursor. React attaches wheel listeners as passive, so this is native.
  useEffect(() => {
    const element = surfaceRef.current;
    if (element === null) return;
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      const rect = element.getBoundingClientRect();
      const anchor = { x: event.clientX - rect.left, y: event.clientY - rect.top };
      const factor = Math.exp(-event.deltaY * 0.0015);
      setPlacement((current) => {
        const frame = { ...current, width: size.width, height: size.height };
        const before = screenToWorld(frame, anchor);
        const zoom = clampZoom(current.zoom * factor);
        const after = { ...frame, zoom };
        const moved = screenToWorld(after, anchor);
        return {
          zoom,
          center: {
            x: current.center.x + (before.x - moved.x),
            y: current.center.y + (before.y - moved.y),
          },
        };
      });
    };
    element.addEventListener("wheel", onWheel, { passive: false });
    return () => element.removeEventListener("wheel", onWheel);
  }, [size.width, size.height]);

  function localPoint(event: PointerEvent<HTMLDivElement>): Point {
    const rect = event.currentTarget.getBoundingClientRect();
    const scaleX = rect.width > 0 ? size.width / rect.width : 1;
    const scaleY = rect.height > 0 ? size.height / rect.height : 1;
    return { x: (event.clientX - rect.left) * scaleX, y: (event.clientY - rect.top) * scaleY };
  }

  function onPointerDown(event: PointerEvent<HTMLDivElement>): void {
    if (gestureRef.current !== null) return;
    const point = localPoint(event);
    gestureRef.current = {
      pointerId: event.pointerId,
      mode: picking ? "pick" : "pan",
      startScreen: point,
      startCenter: placement.center,
      startWorld: screenToWorld(view, point),
      yaw: null,
    };
    try {
      event.currentTarget.setPointerCapture?.(event.pointerId);
    } catch {
      // A synthetic or already-ended pointer cannot be captured; the gesture still works.
    }
    if (picking) setPending({ ...gestureRef.current.startWorld, yaw: draft?.yaw ?? 0 });
  }

  function onPointerMove(event: PointerEvent<HTMLDivElement>): void {
    const gesture = gestureRef.current;
    if (gesture === null || gesture.pointerId !== event.pointerId) return;
    const point = localPoint(event);
    if (gesture.mode === "pan") {
      const dx = point.x - gesture.startScreen.x;
      const dy = point.y - gesture.startScreen.y;
      setPlacement((current) => ({
        zoom: current.zoom,
        center: {
          x: gesture.startCenter.x - dx / current.zoom,
          y: gesture.startCenter.y + dy / current.zoom,
        },
      }));
      return;
    }
    if (Math.hypot(point.x - gesture.startScreen.x, point.y - gesture.startScreen.y) >= DRAG_THRESHOLD_PX) {
      gesture.yaw = polar(gesture.startScreen, point);
    }
    setPending({ ...gesture.startWorld, yaw: gesture.yaw ?? draft?.yaw ?? 0 });
  }

  function onPointerUp(event: PointerEvent<HTMLDivElement>): void {
    const gesture = gestureRef.current;
    if (gesture === null || gesture.pointerId !== event.pointerId) return;
    gestureRef.current = null;
    setPending(null);
    if (gesture.mode !== "pick" || onPick === undefined) return;
    const point = localPoint(event);
    const yaw =
      Math.hypot(point.x - gesture.startScreen.x, point.y - gesture.startScreen.y) >= DRAG_THRESHOLD_PX
        ? polar(gesture.startScreen, point)
        : (gesture.yaw ?? draft?.yaw ?? 0);
    onPick({ x: gesture.startWorld.x, y: gesture.startWorld.y, yaw });
  }

  function onPointerCancel(event: PointerEvent<HTMLDivElement>): void {
    if (gestureRef.current?.pointerId !== event.pointerId) return;
    gestureRef.current = null;
    setPending(null);
  }

  function zoomBy(factor: number): void {
    setPlacement((current) => ({ ...current, zoom: clampZoom(current.zoom * factor) }));
  }

  function fit(): void {
    const next = initialView ?? fitView(map, size.width, size.height);
    setPlacement({ center: next.center, zoom: next.zoom });
  }

  function onKeyDown(event: KeyboardEvent<HTMLDivElement>): void {
    const step = KEY_PAN_PX / placement.zoom;
    const moves: Record<string, Point> = {
      ArrowLeft: { x: -step, y: 0 },
      ArrowRight: { x: step, y: 0 },
      ArrowUp: { x: 0, y: step },
      ArrowDown: { x: 0, y: -step },
    };
    const move = moves[event.key];
    if (move !== undefined) {
      event.preventDefault();
      setPlacement((current) => ({
        zoom: current.zoom,
        center: { x: current.center.x + move.x, y: current.center.y + move.y },
      }));
    } else if (event.key === "+" || event.key === "=") {
      event.preventDefault();
      zoomBy(ZOOM_STEP);
    } else if (event.key === "-") {
      event.preventDefault();
      zoomBy(1 / ZOOM_STEP);
    } else if (event.key === "0") {
      event.preventDefault();
      fit();
    }
  }

  const bar = scaleBar(view.zoom);
  const preview = pending ?? draft;

  return (
    <section className={styles.mapPanel} data-stale={stale}>
      <div className={styles.toolbar}>
        <fieldset className={styles.layers}>
          <legend className={styles.legend}>显示层</legend>
          {(
            [
              ["compare", "对比"],
              ["filtered", "仅过滤层"],
              ["raw", "仅原始层"],
            ] as const
          ).map(([value, label]) => (
            <label key={value} className={styles.choice}>
              <input
                type="radio"
                name={`${hintId}-layer`}
                checked={layer === value}
                onChange={() => setLayer(value)}
              />
              <span>{label}</span>
            </label>
          ))}
        </fieldset>
        <div className={styles.zoomGroup}>
          <button type="button" className={styles.tool} onClick={() => zoomBy(ZOOM_STEP)}>
            放大
          </button>
          <button type="button" className={styles.tool} onClick={() => zoomBy(1 / ZOOM_STEP)}>
            缩小
          </button>
          <button type="button" className={styles.tool} onClick={fit}>
            适应窗口
          </button>
        </div>
      </div>

      {stale && (
        <p className={styles.staleBanner}>缓存地图 · 非实时。不能在这张图上下达目标。</p>
      )}

      <div
        ref={surfaceRef}
        role="application"
        aria-label="地图"
        aria-describedby={hintId}
        tabIndex={0}
        className={styles.surface}
        data-stale={stale}
        data-picking={picking}
        data-layer={layer}
        data-zoom={String(view.zoom)}
        data-center-x={String(view.center.x)}
        data-center-y={String(view.center.y)}
        style={{ aspectRatio: `${size.width} / ${size.height}` }}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerCancel}
        onKeyDown={onKeyDown}
      >
        <canvas ref={canvasRef} className={styles.canvas} width={size.width} height={size.height} />
        <svg
          role="img"
          aria-label="地图叠加层"
          className={styles.overlay}
          viewBox={`0 0 ${size.width} ${size.height}`}
        >
          {zones.map((zone) => {
            const points = zone.points.map((point) => worldToScreen(view, point));
            const label = points.reduce(
              (sum, point) => ({ x: sum.x + point.x / points.length, y: sum.y + point.y / points.length }),
              { x: 0, y: 0 },
            );
            return (
              <g key={zone.id}>
                <polygon
                  data-zone={zone.id}
                  data-kind={zone.kind}
                  className={`${styles.zone} ${styles[`zone_${zone.kind}`]}`}
                  points={points.map((point) => `${point.x},${point.y}`).join(" ")}
                />
                <text x={label.x} y={label.y} className={styles.zoneLabel} textAnchor="middle">
                  {zone.label}
                </text>
              </g>
            );
          })}
          {trajectory.length > 1 && (
            <polyline
              data-layer="trajectory"
              className={styles.trajectory}
              points={trajectory
                .map((point) => worldToScreen(view, point))
                .map((point) => `${point.x},${point.y}`)
                .join(" ")}
            />
          )}
          {home !== null && <Marker kind="home" pose={home} view={view} />}
          {goal !== null && <Marker kind="goal" pose={goal} view={view} />}
          {preview !== null && <Marker kind="draft" pose={preview} view={view} />}
          {robot !== null && <Marker kind="robot" pose={robot} view={view} />}
          <g className={styles.scale} transform={`translate(16 ${size.height - 18})`}>
            <line x1={0} y1={0} x2={bar.pixels} y2={0} />
            <line x1={0} y1={-4} x2={0} y2={4} />
            <line x1={bar.pixels} y1={-4} x2={bar.pixels} y2={4} />
            <text x={bar.pixels + 8} y={4}>
              {bar.meters} m
            </text>
          </g>
        </svg>
      </div>
      <p id={hintId} className={styles.hint}>
        方向键平移，+ / − 缩放，0 适应窗口
        {picking ? "；在地图上按下选点，拖动设定朝向" : "；拖动平移"}
      </p>

      <p role="status" aria-label="覆盖诊断" className={styles.diagnostics}>
        <span>原始 {counts.raw}</span>
        <span>过滤 {counts.filtered}</span>
        <span>被过滤掉 {counts.removed}</span>
      </p>
    </section>
  );
}

// ---------------------------------------------------------------- markers

const MARKER_LABEL = {
  robot: "车辆",
  home: "Home",
  goal: "目标",
  draft: "待确认",
} as const;

function Marker({
  kind,
  pose,
  view,
}: {
  kind: keyof typeof MARKER_LABEL;
  pose: Pose;
  view: MapView;
}): ReactElement {
  const at = worldToScreen(view, pose);
  // The screen's y axis is flipped, so a counter-clockwise yaw is a negative SVG rotation.
  const degrees = (-pose.yaw * 180) / Math.PI;
  return (
    <g
      data-marker={kind}
      className={`${styles.marker} ${styles[`marker_${kind}`]}`}
      transform={`translate(${at.x} ${at.y}) rotate(${degrees})`}
    >
      <title>{MARKER_LABEL[kind]}</title>
      <circle r={9} />
      <path d="M 0 0 L 22 0 M 14 -6 L 22 0 L 14 6" />
    </g>
  );
}
