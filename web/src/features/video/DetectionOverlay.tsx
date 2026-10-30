import type { ReactElement } from "react";
import styles from "./Video.module.css";

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
    <div className={styles.overlay} aria-label="检测结果">
      {matching.map((detection, index) => (
        <span
          className={styles.box}
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
