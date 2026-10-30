import { cleanup, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it } from "vitest";

import { DetectionOverlay, type Detection } from "./DetectionOverlay";
import { VideoPanel } from "./VideoPanel";

afterEach(cleanup);

describe("video", () => {
  it("test_detection_frame_mismatch_hidden", () => {
    const detection: Detection = {
      frameTimestamp: 10,
      label: "person",
      score: 0.9,
      x: 0.1,
      y: 0.2,
      width: 0.3,
      height: 0.4,
    };
    const { rerender } = render(<DetectionOverlay frameTimestamp={11} detections={[detection]} />);
    expect(screen.queryByText("person 90%")).toBeNull();
    rerender(<DetectionOverlay frameTimestamp={10} detections={[detection]} />);
    expect(screen.getByText("person 90%")).toBeTruthy();
  });

  it("shows unavailable without a camera answer and labels inference honestly", () => {
    render(<VideoPanel />);
    expect(screen.getByText("视频不可用")).toBeTruthy();
    expect(screen.getByText("推理未启用")).toBeTruthy();
  });
});
