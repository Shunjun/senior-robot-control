import type { ReactElement } from "react";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  overlay: [
    "video-overlay absolute inset-0 pointer-events-none"
  ].join(" "),
  box: [
    "video-box absolute block text-teal font-normal text-xs leading-[normal] font-data",
    "border-[2px] border-solid border-teal [background:rgb(16_15_12_/_.55)]"
  ].join(" "),
} as const;

export interface Detection {
  frameTimestamp: number;
  label: string;
  score: number;
  x: number;
  y: number;
  width: number;
  height: number;
}

export function DetectionOverlay({ frameTimestamp, detections }: {
  frameTimestamp: number | null;
  detections: Detection[];
}): ReactElement {
  const matching =
    frameTimestamp === null ? [] : detections.filter((detection) => detection.frameTimestamp === frameTimestamp);
  return (
    <div className={ui.overlay} aria-label="检测结果">
      {matching.map((detection, index) => (
        <span
          className={ui.box}
          key={`${detection.label}-${index}`}
          style={{
            left: `${detection.x * 100}%`,
            top: `${detection.y * 100}%`,
            width: `${detection.width * 100}%`,
            height: `${detection.height * 100}%`,
          }}
        >
          {detection.label} {Math.round(detection.score * 100)}%
        </span>
      ))}
    </div>
  );
}
