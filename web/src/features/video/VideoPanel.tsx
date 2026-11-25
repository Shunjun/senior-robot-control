import { useEffect, useState, type ReactElement } from "react";
import type { RobotLink } from "../../api/client";
import { DetectionOverlay, type Detection } from "./DetectionOverlay";

// Complete utility strings keep the finite presentation states visible to Tailwind.
const ui = {
  panel: [
    "video-panel grid gap-3 pt-4 pr-4 pb-4 pl-4 bg-none",
    "bg-graphite-850 border border-solid border-line [&_h3]:mt-0 [&_h3]:mr-0 [&_h3]:mb-0 [&_h3]:ml-0",
    "[&_p]:mt-0 [&_p]:mr-0 [&_p]:mb-0 [&_p]:ml-0 [&[data-compact=true]]:h-full [&[data-compact=true]]:min-h-0 [&[data-compact=true]]:[grid-template-rows:minmax(0,1fr)] [&[data-compact=true]]:pt-0",
    "[&[data-compact=true]]:pr-0 [&[data-compact=true]]:pb-0 [&[data-compact=true]]:pl-0 [&[data-compact=true]]:border-0 [&[data-compact=true]]:border-none [&[data-compact=true]]:border-current [&[data-compact=true]]:gap-0 [&[data-compact=true]>header]:hidden",
    "[&[data-compact=true]>.video-facts]:hidden [&[data-compact=true]_.video-viewport]:h-full [&[data-compact=true]_.video-viewport]:min-h-0 [&[data-compact=true]_.video-viewport]:[aspect-ratio:auto] [&[data-compact=true]_.video-viewport]:border-0 [&[data-compact=true]_.video-viewport]:border-none [&[data-compact=true]_.video-viewport]:border-current [&[data-compact=true]_video]:absolute",
    "[&[data-compact=true]_video]:inset-0 [&[data-compact=true]_video]:w-full [&[data-compact=true]_video]:h-full [&[data-compact=true]_video]:object-contain"
  ].join(" "),
  kicker: [
    "video-kicker text-ink-dim font-normal text-xs leading-[normal] font-data"
  ].join(" "),
  viewport: [
    "video-viewport relative grid min-h-[220px] place-items-center overflow-hidden bg-none bg-video-viewport-background-color",
    "border border-solid border-line [&_video]:block [&_video]:w-full [&_video]:h-auto"
  ].join(" "),
  unavailable: [
    "video-unavailable text-ink-dim font-data"
  ].join(" "),
  facts: [
    "video-facts grid grid-cols-[repeat(2,_1fr)] gap-px mt-0 mr-0 mb-0 ml-0",
    "bg-none bg-line-soft [&_div]:pt-2 [&_div]:pr-2 [&_div]:pb-2 [&_div]:pl-2 [&_div]:bg-none [&_div]:bg-graphite-900",
    "[&_dt]:text-ink-dim [&_dt]:text-xs [&_dd]:mt-0 [&_dd]:mr-0 [&_dd]:mb-0 [&_dd]:ml-0 [&_dd]:font-data"
  ].join(" "),
} as const;

const MAX_PENDING_CANDIDATES = 64;

interface VideoPanelProps {
  link?: Pick<RobotLink, "sendVideoSignal" | "subscribeVideo">;
  live?: boolean;
  compact?: boolean;
  stream?: MediaStream | null;
  frameTimestamp?: number | null;
  detections?: Detection[];
}

export function VideoPanel({
  link,
  compact = false, live = true,
  stream: suppliedStream = null,
  frameTimestamp = null,
  detections = [],
}: VideoPanelProps): ReactElement {
  const [remoteStream, setRemoteStream] = useState<MediaStream | null>(suppliedStream);
  const stream = suppliedStream ?? remoteStream;
  const [page, setPage] = useState(() => ({ visible: document.visibilityState === "visible", generation: 0 }));

  useEffect(() => {
    const update = (visible: boolean) => setPage((previous) => previous.visible === visible
      ? previous : { visible, generation: previous.generation + 1 });
    const visible = () => update(document.visibilityState === "visible");
    const hidden = () => update(false);
    document.addEventListener("visibilitychange", visible);
    window.addEventListener("pagehide", hidden);
    window.addEventListener("pageshow", visible);
    return () => {
      document.removeEventListener("visibilitychange", visible);
      window.removeEventListener("pagehide", hidden);
      window.removeEventListener("pageshow", visible);
    };
  }, []);

  useEffect(() => {
    if (
      !live || !page.visible ||
      link?.sendVideoSignal === undefined ||
      link.subscribeVideo === undefined ||
      typeof RTCPeerConnection === "undefined"
    ) return;
    const peer = new RTCPeerConnection();
    let active = true;
    let offered = false;
    const pendingCandidates: string[] = [];

    function finish() {
      if (!active) return;
      active = false;
      peer.close();
      setRemoteStream(null);
      if (offered) {
        try {
          link?.sendVideoSignal?.("video.close", {});
        } catch {
          // The control socket may already be gone; media cleanup remains local.
        }
      }
    }

    function sendCandidate(candidate: string) {
      try {
        link?.sendVideoSignal?.("video.candidate", { candidate });
      } catch {
        finish();
      }
    }

    const onVisibility = () => { if (document.visibilityState !== "visible") finish(); };
    // Release synchronously before a hidden page can be frozen in the back-forward cache.
    document.addEventListener("visibilitychange", onVisibility);
    window.addEventListener("pagehide", finish);

    peer.addTransceiver("video", { direction: "recvonly" });
    peer.ontrack = (event) => {
      if (active) setRemoteStream(event.streams[0] ?? new MediaStream([event.track]));
    };
    peer.onconnectionstatechange = () => {
      if (peer.connectionState === "failed") finish();
    };
    peer.onicecandidate = (event) => {
      if (!active || event.candidate === null) return;
      const candidate = event.candidate.candidate;
      if (offered) sendCandidate(candidate);
      else if (pendingCandidates.length < MAX_PENDING_CANDIDATES) pendingCandidates.push(candidate);
    };
    const off = link.subscribeVideo(async (frame) => {
      if (!active) return;
      try {
        if (frame.type === "video.answer") {
          await peer.setRemoteDescription({ type: "answer", sdp: frame.payload.sdp });
        } else if (frame.type === "video.candidate") {
          await peer.addIceCandidate({ candidate: frame.payload.candidate, sdpMLineIndex: 0 });
        } else if (frame.type === "video.close") {
          offered = false;
          finish();
        }
      } catch {
        finish();
      }
    });
    void peer.createOffer()
      .then(async (offer) => {
        if (!active) return;
        await peer.setLocalDescription(offer);
        if (!active || !offer.sdp) return;
        link.sendVideoSignal?.("video.offer", { sdp: offer.sdp });
        offered = true;
        for (const candidate of pendingCandidates.splice(0)) {
          if (!active) break;
          sendCandidate(candidate);
        }
      })
      .catch(finish);
    return () => {
      document.removeEventListener("visibilitychange", onVisibility);
      window.removeEventListener("pagehide", finish);
      off();
      finish();
    };
  }, [link, live, page]);

  return (
    <section className={ui.panel} data-compact={compact} aria-label="实时视频">
      <header><p className={ui.kicker}>VIDEO / WEBRTC</p><h3>实时视频</h3></header>
      <div className={ui.viewport}>
        {stream === null ? (
          <p className={ui.unavailable}>视频不可用</p>
        ) : (
          <video
            autoPlay
            muted
            playsInline
            ref={(element) => {
              if (element !== null && element.srcObject !== stream) element.srcObject = stream;
            }}
          />
        )}
        <DetectionOverlay frameTimestamp={frameTimestamp} detections={detections} />
      </div>
      <dl className={ui.facts}>
        <div><dt>帧龄 / 延迟</dt><dd>{stream === null ? "无可用视频帧" : "由 WebRTC 媒体统计提供"}</dd></div>
        <div><dt>视觉推理</dt><dd>推理未启用</dd></div>
      </dl>
    </section>
  );
}
