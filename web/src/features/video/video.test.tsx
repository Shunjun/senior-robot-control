import { act, cleanup, render, screen, waitFor } from "@testing-library/react";
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

it("releases hidden previews and resumes with a fresh peer across page lifecycle", async () => {
  let finishRetiredOffer!: (offer: { type: string; sdp: string }) => void;
  const retiredOffer = new Promise<{ type: string; sdp: string }>((resolve) => { finishRetiredOffer = resolve; });
  const peers: Peer[] = [];
  class Peer {
    close = vi.fn();
    setLocalDescription = vi.fn(async () => {});
    constructor() { peers.push(this); }
    addTransceiver() {}
    async createOffer() {
      return peers[0] === this ? retiredOffer : { type: "offer", sdp: "offer" };
    }
  }
  const sendVideoSignal = vi.fn();
  const link = { sendVideoSignal, subscribeVideo: () => () => {} };
  vi.stubGlobal("RTCPeerConnection", Peer);
  let visibility = "hidden";
  vi.spyOn(document, "visibilityState", "get").mockImplementation(() => visibility as DocumentVisibilityState);
  try {
    const { unmount } = render(<VideoPanel link={link} />);
    expect(peers).toHaveLength(0);
    visibility = "visible";
    document.dispatchEvent(new Event("visibilitychange"));
    await waitFor(() => expect(peers).toHaveLength(1));
    window.dispatchEvent(new Event("pagehide"));
    await waitFor(() => expect(peers[0]!.close).toHaveBeenCalledTimes(1));
    expect(sendVideoSignal).not.toHaveBeenCalled();
    window.dispatchEvent(new Event("pageshow"));
    await waitFor(() => expect(sendVideoSignal).toHaveBeenCalledWith("video.offer", { sdp: "offer" }));
    expect(peers).toHaveLength(2);
    await act(async () => finishRetiredOffer({ type: "offer", sdp: "retired-offer" }));
    expect(peers[0]!.setLocalDescription).not.toHaveBeenCalled();
    visibility = "hidden";
    document.dispatchEvent(new Event("visibilitychange"));
    await waitFor(() => expect(peers[1]!.close).toHaveBeenCalledTimes(1));
    expect(sendVideoSignal).toHaveBeenCalledWith("video.close", {});
    unmount();
    expect(sendVideoSignal.mock.calls.filter(([type]) => type === "video.offer")).toHaveLength(1);
  } finally {
    vi.restoreAllMocks();
    vi.unstubAllGlobals();
  }
});
