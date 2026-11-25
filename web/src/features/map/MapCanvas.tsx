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

import { createPortal } from "react-dom";

import { ZoomIn, ZoomOut, LocateFixed, Maximize } from "lucide-react";
import type { MapRef } from "../../api/protocol";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  toolbar: [
    "map-toolbar absolute z-[6] bottom-3 left-4 flex pt-0 pr-0",
    "pb-0 pl-0 gap-1 items-center flex-wrap [&[data-docked=true]]:relative [&[data-docked=true]]:left-auto [&[data-docked=true]]:bottom-auto"
  ].join(" "),
  zoomGroup: [
    "map-zoom-group flex flex-wrap max-w-full items-center gap-1 [&_.map-tool]:grid [&_.map-tool]:place-items-center [&_.map-tool]:pt-[5px] [&_.map-tool]:pr-2.5",
    "[&_.map-tool]:pb-[5px] [&_.map-tool]:pl-2.5 [&_.map-tool]:bg-none [&_.map-tool]:bg-map-tool [&_.map-tool]:rounded-control [&_.map-tool_svg]:w-[12px] [&_.map-tool_svg]:h-[18px] max-[1050.001px]:[&_.map-tool]:w-[25px]",
    "max-[1050.001px]:[&_.map-tool]:h-[28px] max-[1050.001px]:[&_.map-tool]:pt-[3px] max-[1050.001px]:[&_.map-tool]:pr-[3px] max-[1050.001px]:[&_.map-tool]:pb-[3px] max-[1050.001px]:[&_.map-tool]:pl-[3px] max-[1050.001px]:gap-[3px] max-[760.001px]:[&_.map-tool]:w-[22px] max-[760.001px]:[&_.map-tool]:h-[27px]",
    "max-[460.001px]:gap-0.5 max-[460.001px]:[&_.map-tool]:w-[19px] max-[460.001px]:[&_.map-tool]:h-[26px] max-[460.001px]:[&_.map-tool]:pt-px max-[460.001px]:[&_.map-tool]:pr-px max-[460.001px]:[&_.map-tool]:pb-px max-[460.001px]:[&_.map-tool]:pl-px"
  ].join(" "),
  tool: [
    "map-tool pt-[9px] pr-[13px] pb-[9px] pl-[13px] text-ink bg-none",
    "bg-control border border-solid border-devices-item-border-color rounded-control cursor-pointer [&:disabled]:opacity-[0.36] [&:disabled]:cursor-not-allowed"
  ].join(" "),
  scaleHtml: [
    "map-scale-html inline-flex items-center gap-[7px] pt-[7px] pr-[9px] pb-[7px] pl-[9px]",
    "ml-[5px] rounded-sm bg-none bg-scale text-scale-ink whitespace-nowrap font-normal text-xs",
    "leading-[normal] font-mono [&_i]:block [&_i]:box-border [&_i]:shrink-0 [&_i]:min-w-0 [&_i]:h-[7px] [&_i]:border",
    "[&_i]:border-solid [&_i]:border-current [&_i]:border-t-0 [&_i]:[border-top-style:none] [&_i]:border-t-current [&_i]:[transform:translateY(-2px)] max-[1050.001px]:pt-1.5 max-[1050.001px]:pr-1.5",
    "max-[1050.001px]:pb-1.5 max-[1050.001px]:pl-1.5 max-[1050.001px]:text-xs max-[760.001px]:gap-[3px] max-[760.001px]:ml-0 max-[760.001px]:text-xs max-[760.001px]:pt-[5px] max-[760.001px]:pr-[5px]",
    "max-[760.001px]:pb-[5px] max-[760.001px]:pl-[5px] max-[460.001px]:pt-1 max-[460.001px]:pr-0.5 max-[460.001px]:pb-1 max-[460.001px]:pl-0.5 max-[460.001px]:gap-[3px]"
  ].join(" "),
  mapPanel: [
    "map-map-panel block relative border-0 border-none border-current pt-0 pr-0",
    "pb-0 pl-0 [border-radius:0] bg-none bg-map-unknown overflow-hidden min-h-0 [&_.map-surface]:w-full",
    "[&_.map-surface]:h-full [&_.map-surface]:min-h-0 [&_.map-surface]:bg-none [&_.map-surface]:bg-map-unknown [&_.map-surface]:border-0 [&_.map-surface]:border-none [&_.map-surface]:border-current [&_.map-surface[data-picking=true]]:[outline:1px_solid_var(--color-amber)]",
    "[&_.map-surface[data-picking=true]]:[outline-offset:-1px] [&>.map-hint]:absolute [&>.map-hint]:w-[1px] [&>.map-hint]:h-[1px] [&>.map-hint]:overflow-hidden [&>.map-hint]:[clip-path:inset(50%)] [&>.map-diagnostics]:absolute [&>.map-diagnostics]:w-[1px]",
    "[&>.map-diagnostics]:h-[1px] [&>.map-diagnostics]:overflow-hidden [&>.map-diagnostics]:[clip-path:inset(50%)] [&[data-mini=true]_.map-toolbar]:left-2 [&[data-mini=true]_.map-toolbar]:bottom-2 [&[data-mini=true]_.map-toolbar]:right-2 [&[data-mini=true]_.map-map-details]:hidden [&[data-mini=true]_.map-zoom-group]:gap-0.5",
    "[&[data-mini=true]_.map-tool]:w-[21px] [&[data-mini=true]_.map-tool]:min-w-[21px] [&[data-mini=true]_.map-tool]:h-[24px] [&[data-mini=true]_.map-tool]:pt-0.5 [&[data-mini=true]_.map-tool]:pr-0.5 [&[data-mini=true]_.map-tool]:pb-0.5 [&[data-mini=true]_.map-tool]:pl-0.5 [&[data-mini=true]_.map-tool]:rounded-input",
    "[&[data-mini=true]_.map-scale-html]:ml-0 [&[data-mini=true]_.map-scale-html]:pt-[3px] [&[data-mini=true]_.map-scale-html]:pr-[3px] [&[data-mini=true]_.map-scale-html]:pb-[3px] [&[data-mini=true]_.map-scale-html]:pl-[3px] [&[data-mini=true]_.map-scale-html]:gap-[3px] [&[data-mini=true]_.map-scale-html]:text-xs"
  ].join(" "),
  mapDetails: [
    "map-map-details absolute top-15.5 left-4 z-[4] text-map-map-details-color text-xs [&_summary]:cursor-pointer",
    "[&_summary]:bg-none [&_summary]:bg-map-map-details-background-color [&_summary]:pt-1.5 [&_summary]:pr-[9px] [&_summary]:pb-1.5 [&_summary]:pl-[9px] [&_summary]:border [&_summary]:border-solid",
    "[&_summary]:border-line [&_summary]:rounded-input [&_summary]:w-[max-content] [&[open]]:bg-none [&[open]]:bg-map-map-details-background-color-2 [&[open]]:border [&[open]]:border-solid [&[open]]:border-line",
    "[&[open]]:rounded-md [&[open]]:pt-2.5 [&[open]]:pr-2.5 [&[open]]:pb-2.5 [&[open]]:pl-2.5 [&[open]]:max-w-[260px] max-[760.001px]:top-15.5 max-[760.001px]:left-3",
    "max-[460.001px]:top-13 max-[460.001px]:left-3 max-[460.001px]:[&_summary]:text-xs max-[460.001px]:[&_summary]:pt-1.5 max-[460.001px]:[&_summary]:pr-1.5 max-[460.001px]:[&_summary]:pb-1.5 max-[460.001px]:[&_summary]:pl-1.5"
  ].join(" "),
  layers: [
    "map-layers flex flex-wrap gap-2 border-0 border-none border-current pt-2",
    "pr-0 pb-2 pl-0 mt-0 mr-0 mb-0 ml-0"
  ].join(" "),
  legend: [
    "map-legend text-muted font-normal text-xs leading-[normal] font-mono"
  ].join(" "),
  choice: [
    "map-choice inline-flex items-center gap-[5px] text-xs [&_input]:accent-amber"
  ].join(" "),
  geometryNote: [
    "map-geometry-note mt-2 mr-0 mb-2 ml-0 text-muted text-xs"
  ].join(" "),
  staleBanner: [
    "map-stale-banner absolute left-4 right-4 z-[2] top-27.5 pt-2.5 pr-2.5",
    "pb-2.5 pl-2.5 bg-none bg-workspace-map-message-background-color text-map-stale-banner-color rounded-md text-xs"
  ].join(" "),
  surface: [
    "map-surface relative w-full overflow-hidden bg-none bg-graphite-950 border border-solid",
    "border-line cursor-grab touch-none select-none [&[data-picking=true]]:[cursor:crosshair] [&[data-picking=true]]:border-amber [&[data-stale=true]_.map-canvas]:opacity-[0.45] [&[data-stale=true]_.map-canvas]:[filter:grayscale(0.8)]"
  ].join(" "),
  canvas: [
    "map-canvas absolute inset-0 w-full h-full"
  ].join(" "),
  overlay: [
    "map-overlay absolute inset-0 w-full h-full pointer-events-none"
  ].join(" "),
  zone: [
    "map-zone [stroke-width:2] [stroke-dasharray:8_5]"
  ].join(" "),
  zoneLabel: [
    "map-zone-label fill-ink font-data text-xs [paint-order:stroke] stroke-graphite-950 [stroke-width:3px]"
  ].join(" "),
  trajectory: [
    "map-trajectory [fill:none] stroke-teal [stroke-width:2.5] [stroke-linecap:round] [stroke-linejoin:round]"
  ].join(" "),
  taskRoute: [
    "map-task-route stroke-amber [stroke-width:2] [stroke-dasharray:4_3] [fill:none]"
  ].join(" "),
  waypointMarker: [
    "map-waypoint-marker [&_circle]:fill-graphite-900 [&_circle]:stroke-teal [&_circle]:[stroke-width:2] [&[data-selectable=true]]:pointer-events-auto [&[data-selectable=true]]:cursor-pointer [&[data-highlighted=true]_circle]:fill-amber [&[data-highlighted=true]_circle]:stroke-ink",
    "[&:focus-visible_circle]:fill-amber [&:focus-visible_circle]:stroke-ink"
  ].join(" "),
  waypointName: [
    "map-waypoint-name fill-ink text-xs font-data pointer-events-none"
  ].join(" "),
  routeDot: [
    "map-route-dot fill-amber stroke-map-route-dot-stroke [stroke-width:1]"
  ].join(" "),
  routeNumber: [
    "map-route-number fill-white font-normal text-xs leading-[normal] font-data"
  ].join(" "),
  scanHits: [
    "map-scan-hits fill-amber stroke-map-scan-hits-stroke [stroke-width:0.8] pointer-events-none"
  ].join(" "),
  hint: [
    "map-hint mt-0 mr-0 mb-0 ml-0 text-ink-dim font-data text-xs"
  ].join(" "),
  scanLegend: [
    "map-scan-legend text-map-scan-legend-color text-xs mt-0 mr-0 mb-0 ml-0 absolute",
    "w-[1px] h-[1px] overflow-hidden [clip-path:inset(50%)] before:content-[''] [&::before]:mr-[0.5em]"
  ].join(" "),
  diagnostics: [
    "map-diagnostics flex flex-wrap gap-[8px_16px] mt-0 mr-0 mb-0 ml-0",
    "font-data text-xs text-ink-soft"
  ].join(" "),
  marker: [
    "map-marker [&_circle]:[stroke-width:2.5] [&_path]:[fill:none] [&_path]:[stroke-width:3] [&_path]:[stroke-linecap:round] [&_path]:[stroke-linejoin:round]"
  ].join(" "),
  robotArrow: [
    "map-robot-arrow"
  ].join(" "),
  zone_keep_out: [
    "map-zone-keep-out fill-red-wash stroke-red"
  ].join(" "),
  zone_slow: [
    "map-zone-slow fill-amber-wash stroke-amber"
  ].join(" "),
  zone_observe: [
    "map-zone-observe fill-teal-wash stroke-teal"
  ].join(" "),
  marker_robot: [
    "map-marker-robot [&_circle]:fill-teal-wash [&_circle]:stroke-teal [&_path]:stroke-teal [&_.map-robot-arrow]:fill-amber [&_.map-robot-arrow]:stroke-map-marker-robot-stroke [&_.map-robot-arrow]:[stroke-width:1.6]"
  ].join(" "),
  marker_home: [
    "map-marker-home [&_circle]:fill-graphite-950 [&_circle]:stroke-ink [&_path]:stroke-ink"
  ].join(" "),
  marker_goal: [
    "map-marker-goal [&_circle]:fill-amber-wash [&_circle]:stroke-amber [&_path]:stroke-amber"
  ].join(" "),
  marker_draft: [
    "map-marker-draft [&_circle]:[fill:none] [&_circle]:stroke-amber [&_circle]:[stroke-dasharray:4_3] [&_path]:stroke-amber [&_path]:[stroke-dasharray:4_3]"
  ].join(" "),
} as const;
const zoneClasses = { keep_out: ui.zone_keep_out, slow: ui.zone_slow, observe: ui.zone_observe } as const;
const markerClasses = { robot: ui.marker_robot, home: ui.marker_home, goal: ui.marker_goal, draft: ui.marker_draft } as const;


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
export interface OccupancyGrid {
  width: number;
  height: number;
  values: Int8Array;
}

export interface OccupancySnapshot {
  map_ref: MapRef | null;
  origin: Pose;
  resolution: number;
  cells: readonly CellEvidence[];
  grid?: OccupancyGrid;
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

// A fixed reference viewport expresses the user's logical world rectangle. Only
// user zoom changes its px/m; resizing derives a separate effective view below.
function viewForSurface(logical: MapView, size: { width: number; height: number }): MapView {
  return {
    center: logical.center,
    zoom: logical.zoom * Math.min(size.width / logical.width, size.height / logical.height),
    ...size,
  };
}

type Layer = "compare" | "filtered" | "raw";

// ---------------------------------------------------------------- coordinates

// User limits apply in the fixed reference viewport, never to resize mapping.
export const MIN_ZOOM = 2;
export const MAX_ZOOM = 5_000;
/** Belief at or above this counts as occupied in the filtered layer. */
export const OCCUPIED_BELIEF = 0.5;

/** Grid coordinates (cells, fractional allowed) to the map frame. */
export function gridToWorld(
  map: OccupancySnapshot,
  gx: number,
  gy: number,
): Point {
  const { origin, resolution } = map;
  const cos = Math.cos(origin.yaw);
  const sin = Math.sin(origin.yaw);
  const lx = gx * resolution;
  const ly = gy * resolution;
  return {
    x: origin.x + cos * lx - sin * ly,
    y: origin.y + sin * lx + cos * ly,
  };
}

/** The inverse of `gridToWorld`. */
export function worldToGrid(map: OccupancySnapshot, point: Point): Point {
  const { origin, resolution } = map;
  const cos = Math.cos(origin.yaw);
  const sin = Math.sin(origin.yaw);
  const dx = point.x - origin.x;
  const dy = point.y - origin.y;
  return {
    x: (cos * dx + sin * dy) / resolution,
    y: (-sin * dx + cos * dy) / resolution,
  };
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

export function gridToScreen(
  view: MapView,
  map: OccupancySnapshot,
  gx: number,
  gy: number,
): Point {
  return worldToScreen(view, gridToWorld(map, gx, gy));
}

function clampZoom(zoom: number): number {
  return Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, zoom));
}

/** A view that shows every cell with a margin. An empty map is centred on its origin. */
export function fitView(
  map: OccupancySnapshot,
  width: number,
  height: number,
): MapView {
  if (map.cells.length === 0 && map.grid === undefined) {
    return {
      center: { x: map.origin.x, y: map.origin.y },
      zoom: 50,
      width,
      height,
    };
  }
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  function includeBounds(
    x: number,
    y: number,
    width: number,
    height: number,
  ): void {
    for (const [gx, gy] of [
      [x, y],
      [x + width, y],
      [x, y + height],
      [x + width, y + height],
    ] as const) {
      const corner = gridToWorld(map, gx, gy);
      minX = Math.min(minX, corner.x);
      minY = Math.min(minY, corner.y);
      maxX = Math.max(maxX, corner.x);
      maxY = Math.max(maxY, corner.y);
    }
  }
  if (map.grid !== undefined) {
    includeBounds(0, 0, map.grid.width, map.grid.height);
  } else {
    for (const {
      cell: [x, y],
    } of map.cells)
      includeBounds(x, y, 1, 1);
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

interface WaypointMarker extends Point {
  waypoint_id: string;
}

export interface MapCanvasProps {
  map: OccupancySnapshot;
  /** The map is a cached copy, not live. Picking is off. */
  stale: boolean;
  /** Initial viewport. Without it the map is fitted to the surface. */
  view?: MapView;
  robot?: Pose | null;
  /** Fresh laser hits already transformed into the map frame. */
  scanPoints?: readonly Point[];
  home?: Pose | null;
  goal?: Pose | null;
  /** The pose being edited, drawn dashed. */
  draft?: Pose | null;
  trajectory?: readonly Point[];
  zones?: readonly SafetyZone[];
  /** A press (and optional drag for heading) on the map yields a pose instead of panning. */
  pickEnabled?: boolean;
  onPick?: (pose: Pose) => void;
  /** When provided, a second click/Enter sets heading around the existing draft. */
  onPickHeading?: (pose: Pose) => void;
  compact?: boolean;
  mini?: boolean;
  toolbarTarget?: HTMLElement | null;
  taskRoute?: readonly Point[];
  waypointMarkers?: readonly WaypointMarker[];
  highlightedWaypoint?: string | null;
  onAddWaypoint?: (waypointId: string) => void;
}

const DEFAULT_WIDTH = 640;
const DEFAULT_HEIGHT = 420;
const ZOOM_STEP = 1.25;
const KEY_PAN_PX = 40;
const DRAG_THRESHOLD_PX = 4;

const INK = "#f2ead9";
const AMBER = "#ff7532";

interface Gesture {
  pointerId: number;
  mode: "pan" | "pick" | "heading";
  startScreen: Point;
  startCenter: Point;
  startWorld: Point;
  yaw: number | null;
}

function scaleBar(
  zoom: number,
  target = 45,
): { meters: number; pixels: number } {
  // Effective zoom can be below the user's reference MIN_ZOOM after contain.
  // Keep the same 1/2/5 scale choices without the old 1 mm / 100 m cutoffs.
  const unit = 10 ** Math.floor(Math.log10(target / zoom));
  const meters = [1, 2, 5, 10].map((multiple) => multiple * unit).reduce(
    (best, length) =>
      Math.abs(length * zoom - target) < Math.abs(best * zoom - target)
        ? length
        : best,
    unit,
  );
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
  scanPoints = [],
  home = null,
  goal = null,
  draft = null,
  trajectory = [],
  zones = [],
  pickEnabled = false,
  onPick,
  onPickHeading,
  compact = false,
  mini = false,
  toolbarTarget = null,
  taskRoute = [],
  waypointMarkers = [],
  highlightedWaypoint = null,
  onAddWaypoint,
}: MapCanvasProps): ReactElement {
  const hintId = useId();
  const surfaceRef = useRef<HTMLDivElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const gestureRef = useRef<Gesture | null>(null);
  const [size, setSize] = useState({
    width: initialView?.width ?? DEFAULT_WIDTH,
    height: initialView?.height ?? DEFAULT_HEIGHT,
  });
  const [placement, setPlacement] = useState<MapView>(
    () => initialView ?? fitView(map, size.width, size.height),
  );
  const fittedToSurface = useRef(initialView !== undefined);
  const [layer, setLayer] = useState<Layer>("compare");
  const [pending, setPending] = useState<Pose | null>(null);

  const view: MapView = useMemo(
    () => viewForSurface(placement, size),
    [placement, size],
  );
  const picking = pickEnabled && !stale && onPick !== undefined;

  const mapId = map.map_ref?.map_id ?? null;
  const mapRevision = map.map_ref?.reference_revision ?? null;
  const fitted = useRef({ mapId, mapRevision });
  useEffect(() => {
    // A different map or revision starts from a fitted view, not the old one's framing.
    if (
      fitted.current.mapId === mapId &&
      fitted.current.mapRevision === mapRevision
    )
      return;
    fitted.current = { mapId, mapRevision };
    const next = fitView(map, size.width, size.height);
    setPlacement(next);
  }, [map, mapId, mapRevision, size.width, size.height]);

  // Follow the surface's real size where the browser can tell us.
  useEffect(() => {
    const element = surfaceRef.current;
    if (element === null || typeof ResizeObserver === "undefined") return;
    const observer = new ResizeObserver(() => {
      const rect = element.getBoundingClientRect();
      if (rect.width > 0 && rect.height > 0) {
        const { width, height } = rect;
        setSize((current) => current.width === width && current.height === height ? current : { width, height });
        // ControlSurface first supplies an empty placeholder while tiles load.
        if (!fittedToSurface.current && (map.grid !== undefined || map.cells.length > 0)) {
          const next = fitView(map, width, height);
          setPlacement(next);
          fittedToSurface.current = true;
        }
      }
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, [initialView, map]);

  const occupancyImage = useMemo(() => {
    const grid = map.grid;
    if (grid === undefined) return null;
    const image = document.createElement("canvas");
    image.width = grid.width;
    image.height = grid.height;
    const context = image.getContext("2d");
    if (context === null) return null;
    const pixels = context.createImageData(grid.width, grid.height);
    grid.values.forEach((cell, index) => {
      const color =
        cell === -1
          ? [66, 68, 73]
          : cell === 0
            ? [216, 211, 203]
            : cell >= 50
              ? [85, 87, 91]
              : [174, 170, 166];
      const offset = index * 4;
      pixels.data[offset] = color[0]!;
      pixels.data[offset + 1] = color[1]!;
      pixels.data[offset + 2] = color[2]!;
      pixels.data[offset + 3] = 255;
    });
    context.putImageData(pixels, 0, 0);
    return image;
  }, [map.grid]);

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
    if (occupancyImage !== null) {
      context.imageSmoothingEnabled = false;
      context.drawImage(occupancyImage, 0, 0);
      return;
    }
    for (const entry of map.cells) {
      const inRaw = isRaw(entry);
      const inFiltered = isFiltered(entry);
      let color: string | null = null;
      if (layer === "raw") color = inRaw ? AMBER : null;
      else if (layer === "filtered") color = inFiltered ? INK : null;
      else if (inFiltered) color = "#55575b";
      else if (inRaw) color = "#aeaaa6";
      if (color === null) continue;
      context.fillStyle = color;
      context.fillRect(entry.cell[0], entry.cell[1], 1, 1);
    }
  }, [map, view, layer, occupancyImage, size.width, size.height]);

  // Wheel zoom about the cursor. React attaches wheel listeners as passive, so this is native.
  useEffect(() => {
    const element = surfaceRef.current;
    if (element === null) return;
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      const rect = element.getBoundingClientRect();
      const anchor = {
        x: event.clientX - rect.left,
        y: event.clientY - rect.top,
      };
      fittedToSurface.current = true;
      const factor = Math.exp(-event.deltaY * 0.0015);
      setPlacement((current) => {
        const frame = viewForSurface(current, size);
        const before = screenToWorld(frame, anchor);
        const zoom = clampZoom(current.zoom * factor);
        const moved = screenToWorld(viewForSurface({ ...current, zoom }, size), anchor);
        return {
          ...current,
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
    return {
      x: (event.clientX - rect.left) * scaleX,
      y: (event.clientY - rect.top) * scaleY,
    };
  }

  function onPointerDown(event: PointerEvent<HTMLDivElement>): void {
    if (gestureRef.current !== null) return;
    const point = localPoint(event);
    const heading = picking && onPickHeading !== undefined && draft !== null;
    gestureRef.current = {
      pointerId: event.pointerId,
      mode: heading ? "heading" : picking ? "pick" : "pan",
      startScreen: heading ? worldToScreen(view, draft) : point,
      startCenter: placement.center,
      startWorld: heading ? { x: draft.x, y: draft.y } : screenToWorld(view, point),
      yaw: null,
    };
    try {
      event.currentTarget.setPointerCapture?.(event.pointerId);
    } catch {
      // A synthetic or already-ended pointer cannot be captured; the gesture still works.
    }
    if (picking)
      setPending({ ...gestureRef.current.startWorld, yaw: draft?.yaw ?? 0 });
  }

  function onPointerMove(event: PointerEvent<HTMLDivElement>): void {
    const gesture = gestureRef.current;
    if (gesture === null || gesture.pointerId !== event.pointerId) return;
    const point = localPoint(event);
    if (gesture.mode === "pan") {
      fittedToSurface.current = true;
      const dx = point.x - gesture.startScreen.x;
      const dy = point.y - gesture.startScreen.y;
      setPlacement((current) => ({
        ...current,
        center: {
          x: gesture.startCenter.x - dx / view.zoom,
          y: gesture.startCenter.y + dy / view.zoom,
        },
      }));
      return;
    }
    if (
      Math.hypot(
        point.x - gesture.startScreen.x,
        point.y - gesture.startScreen.y,
      ) >= DRAG_THRESHOLD_PX
    ) {
      gesture.yaw = polar(gesture.startScreen, point);
    }
    setPending({ ...gesture.startWorld, yaw: gesture.yaw ?? draft?.yaw ?? 0 });
  }

  function onPointerUp(event: PointerEvent<HTMLDivElement>): void {
    const gesture = gestureRef.current;
    if (gesture === null || gesture.pointerId !== event.pointerId) return;
    gestureRef.current = null;
    setPending(null);
    if (gesture.mode === "pan" || onPick === undefined) return;
    const point = localPoint(event);
    const dragged = Math.hypot(point.x - gesture.startScreen.x, point.y - gesture.startScreen.y) >= DRAG_THRESHOLD_PX;
    const yaw = gesture.mode === "heading" || dragged
      ? polar(gesture.startScreen, point)
      : (gesture.yaw ?? draft?.yaw ?? 0);
    const commit = onPickHeading && (gesture.mode === "heading" || dragged) ? onPickHeading : onPick;
    commit({ x: gesture.startWorld.x, y: gesture.startWorld.y, yaw });
  }

  function onPointerCancel(event: PointerEvent<HTMLDivElement>): void {
    if (gestureRef.current?.pointerId !== event.pointerId) return;
    gestureRef.current = null;
    setPending(null);
  }

  function zoomBy(factor: number): void {
    fittedToSurface.current = true;
    setPlacement((current) => ({
      ...current,
      zoom: clampZoom(current.zoom * factor),
    }));
  }

  function fit(): void {
    fittedToSurface.current = true;
    // Fit the map in the same reference frame; mini's extra area is not selected.
    setPlacement((current) => fitView(map, current.width, current.height));
  }

  function onKeyDown(event: KeyboardEvent<HTMLDivElement>): void {
    const step = KEY_PAN_PX / view.zoom;
    const moves: Record<string, Point> = {
      ArrowLeft: { x: -step, y: 0 },
      ArrowRight: { x: step, y: 0 },
      ArrowUp: { x: 0, y: step },
      ArrowDown: { x: 0, y: -step },
    };
    const move = moves[event.key];
    if (event.key === "Enter" && picking && onPick) {
      event.preventDefault();
      if (onPickHeading && draft) {
        onPickHeading({ ...draft, yaw: Math.atan2(view.center.y - draft.y, view.center.x - draft.x) });
      } else {
        onPick({ ...view.center, yaw: draft?.yaw ?? 0 });
      }
    } else if (move !== undefined) {
      event.preventDefault();
      fittedToSurface.current = true;
      setPlacement((current) => ({
        ...current,
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

  const bar = scaleBar(
    view.zoom,
    mini ? 16 : size.width < 460 ? 15 : size.width < 760 ? 25 : 45,
  );
  const preview = pending ?? draft;

  const toolbar = (
    <div className={ui.toolbar} data-docked={toolbarTarget !== null}>
      <div className={ui.zoomGroup}>
        <button
          type="button"
          aria-label="放大"
          className={ui.tool}
          onClick={() => zoomBy(ZOOM_STEP)}
        >
          <ZoomIn size={18} />
        </button>
        <button
          type="button"
          aria-label="缩小"
          className={ui.tool}
          onClick={() => zoomBy(1 / ZOOM_STEP)}
        >
          <ZoomOut size={18} />
        </button>
        <button
          type="button"
          aria-label="适应窗口"
          className={ui.tool}
          onClick={fit}
        >
          <Maximize size={18} />
        </button>
        <span className={ui.scaleHtml} aria-label="地图比例尺">
          <i style={{ width: bar.pixels }} />
          {bar.meters} m
        </span>
      </div>
    </div>
  );

  return (
    <section
      className={ui.mapPanel}
      data-stale={stale}
      data-compact={compact}
      data-mini={mini}
    >
      {toolbarTarget ? createPortal(toolbar, toolbarTarget) : toolbar}

      <details className={ui.mapDetails}>
        <summary>地图显示详情</summary>
        <p className={ui.geometryNote}>地图 {map.map_ref ? `${map.map_ref.map_id} · r${map.map_ref.reference_revision}` : "尚未就绪"}</p>
        {map.grid === undefined && (
          <fieldset className={ui.layers}>
            <legend className={ui.legend}>显示层</legend>
            {(
              [
                ["compare", "对比"],
                ["filtered", "仅过滤层"],
                ["raw", "仅原始层"],
              ] as const
            ).map(([value, label]) => (
              <label key={value} className={ui.choice}>
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
        )}
        <button
          type="button"
          className={ui.tool}
          aria-label="居中车辆"
          disabled={stale || robot === null}
          onClick={() => {
            if (robot) {
              fittedToSurface.current = true;
              setPlacement((current) => ({
                ...current,
                center: { x: robot.x, y: robot.y },
              }));
            }
          }}
        >
          <LocateFixed size={18} />
        </button>
        {!stale && robot !== null && (
          <p className={ui.geometryNote}>
            实际轮廓尺寸不可用，车辆位置以箭头表示。
          </p>
        )}
      </details>
      {stale && (
        <p className={ui.staleBanner}>
          缓存地图 · 非实时。不能在这张图上下达目标。
        </p>
      )}

      <div
        ref={surfaceRef}
        role="application"
        aria-label="地图"
        aria-describedby={hintId}
        tabIndex={0}
        className={ui.surface}
        data-stale={stale}
        data-picking={picking}
        data-layer={layer}
        data-zoom={String(view.zoom)}
        data-center-x={String(view.center.x)}
        data-center-y={String(view.center.y)}
        style={
          compact
            ? undefined
            : { aspectRatio: `${size.width} / ${size.height}` }
        }
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerCancel}
        onKeyDown={onKeyDown}
      >
        <canvas
          ref={canvasRef}
          className={ui.canvas}
          width={size.width}
          height={size.height}
        />
        <svg
          role="img"
          aria-label="地图叠加层"
          className={ui.overlay}
          viewBox={`0 0 ${size.width} ${size.height}`}
        >
          {zones.map((zone) => {
            const points = zone.points.map((point) =>
              worldToScreen(view, point),
            );
            const label = points.reduce(
              (sum, point) => ({
                x: sum.x + point.x / points.length,
                y: sum.y + point.y / points.length,
              }),
              { x: 0, y: 0 },
            );
            return (
              <g key={zone.id}>
                <polygon
                  data-zone={zone.id}
                  data-kind={zone.kind}
                  className={`${ui.zone} ${zoneClasses[zone.kind]}`}
                  points={points
                    .map((point) => `${point.x},${point.y}`)
                    .join(" ")}
                />
                <text
                  x={label.x}
                  y={label.y}
                  className={ui.zoneLabel}
                  textAnchor="middle"
                >
                  {zone.label}
                </text>
              </g>
            );
          })}
          {trajectory.length > 1 && (
            <polyline
              data-layer="trajectory"
              className={ui.trajectory}
              points={trajectory
                .map((point) => worldToScreen(view, point))
                .map((point) => `${point.x},${point.y}`)
                .join(" ")}
            />
          )}
          {taskRoute.length > 1 && (
            <polyline
              aria-label="任务航点预览"
              className={ui.taskRoute}
              points={taskRoute
                .map((point) => worldToScreen(view, point))
                .map(({ x, y }) => `${x},${y}`)
                .join(" ")}
            />
          )}
          {waypointMarkers.map((waypoint) => {
            const at = worldToScreen(view, waypoint);
            const selectable = onAddWaypoint !== undefined && !stale;
            return (
              <g
                key={waypoint.waypoint_id}
                className={ui.waypointMarker}
                data-highlighted={waypoint.waypoint_id === highlightedWaypoint}
                data-selectable={selectable}
                transform={`translate(${at.x} ${at.y})`}
                role={selectable ? "button" : undefined}
                tabIndex={selectable ? 0 : undefined}
                aria-label={`添加地图航点 ${waypoint.waypoint_id}`}
                onPointerDown={(event) => event.stopPropagation()}
                onClick={
                  selectable
                    ? () => onAddWaypoint(waypoint.waypoint_id)
                    : undefined
                }
                onKeyDown={(event) => {
                  if (
                    selectable &&
                    (event.key === "Enter" || event.key === " ")
                  ) {
                    event.preventDefault();
                    onAddWaypoint(waypoint.waypoint_id);
                  }
                }}
              >
                <title>{waypoint.waypoint_id}</title>
                <circle r={12} />
                <text
                  textAnchor="middle"
                  dominantBaseline="central"
                  className={ui.waypointName}
                >
                  {waypoint.waypoint_id.slice(0, 3)}
                </text>
              </g>
            );
          })}
          {taskRoute.map((point, index) => {
            const at = worldToScreen(view, point);
            return (
              <g key={index} data-route-index={index + 1}>
                <circle
                  cx={at.x}
                  cy={at.y}
                  r={11}
                  className={ui.routeDot}
                />
                <text
                  x={at.x}
                  y={at.y + 4}
                  textAnchor="middle"
                  className={ui.routeNumber}
                >
                  {index + 1}
                </text>
              </g>
            );
          })}
          {home !== null && <Marker kind="home" pose={home} view={view} />}
          {goal !== null && <Marker kind="goal" pose={goal} view={view} />}
          {preview !== null && (
            <Marker kind="draft" pose={preview} view={view} />
          )}
          {!stale && scanPoints.length > 0 && (
            <g aria-label="实时激光命中点" className={ui.scanHits}>
              <title>实时激光命中点 · 独立于静态地图</title>
              {scanPoints.map((point, index) => {
                const at = worldToScreen(view, point);
                return <circle key={index} cx={at.x} cy={at.y} r={3} />;
              })}
            </g>
          )}
          {!stale && robot !== null && (
            <Marker kind="robot" pose={robot} view={view} />
          )}
        </svg>
      </div>
      <p id={hintId} className={ui.hint}>
        方向键平移，+ / − 缩放，0 适应窗口
        {picking ? onPickHeading ? "；先选位置，再点朝向；也可拖动设定朝向" : "；在地图上按下选点，拖动设定朝向" : "；拖动平移"}
      </p>

      {!stale && scanPoints.length > 0 && (
        <p className={ui.scanLegend}>实时激光命中点 · 独立于静态地图</p>
      )}
      {taskRoute.length > 0 && (
        <p className={ui.hint}>任务航点顺序连线 · 不代表车辆实际避障路径</p>
      )}
      {map.grid === undefined && (
        <p role="status" aria-label="覆盖诊断" className={ui.diagnostics}>
          <span>原始 {counts.raw}</span>
          <span>过滤 {counts.filtered}</span>
          <span>被过滤掉 {counts.removed}</span>
        </p>
      )}
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
      className={`${ui.marker} ${markerClasses[kind]}`}
      transform={`translate(${at.x} ${at.y}) rotate(${degrees})`}
    >
      <title>{MARKER_LABEL[kind]}</title>
      {kind === "robot" ? (
        <path
          d="M 13 0 L -11 -9 L -6 0 L -11 9 Z"
          className={ui.robotArrow}
        />
      ) : (
        <>
          <circle r={9} />
          <path d="M 0 0 L 22 0 M 14 -6 L 22 0 L 14 6" />
        </>
      )}
    </g>
  );
}
