import { cleanup, render, screen, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

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

it("starts signaling when the agent becomes live and sends the offer before trickle ICE", async () => {
  class Peer {
    onicecandidate: ((event: { candidate: { candidate: string } }) => void) | null = null;

    addTransceiver() {}
    close() {}

    async createOffer() {
      return { type: "offer", sdp: "offer" };
    }

    async setLocalDescription() {
      this.onicecandidate?.({ candidate: { candidate: "candidate:1" } });
    }
  }

  const sendVideoSignal = vi.fn();
  const link = { sendVideoSignal, subscribeVideo: () => () => {} };
  vi.stubGlobal("RTCPeerConnection", Peer);
  try {
    const { rerender } = render(<VideoPanel link={link} live={false} />);
    expect(sendVideoSignal).not.toHaveBeenCalled();

    rerender(<VideoPanel link={link} live />);
    await waitFor(() =>
      expect(sendVideoSignal.mock.calls).toEqual([
        ["video.offer", { sdp: "offer" }],
        ["video.candidate", { candidate: "candidate:1" }],
      ]),
    );
  } finally {
    vi.unstubAllGlobals();
  }
});
