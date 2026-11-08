import { useEffect, useState, type ReactElement } from "react";
import type { RobotLink } from "../../api/client";
import { DetectionOverlay, type Detection } from "./DetectionOverlay";
import styles from "./Video.module.css";

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

  useEffect(() => {
    if (
      !live ||
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
      off();
      finish();
    };
  }, [link, live]);

  return (
    <section className={styles.panel} data-compact={compact} aria-label="实时视频">
      <header><p className={styles.kicker}>VIDEO / WEBRTC</p><h3>实时视频</h3></header>
      <div className={styles.viewport}>
        {stream === null ? (
          <p className={styles.unavailable}>视频不可用</p>
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
      <dl className={styles.facts}>
        <div><dt>帧龄 / 延迟</dt><dd>{stream === null ? "无可用视频帧" : "由 WebRTC 媒体统计提供"}</dd></div>
        <div><dt>视觉推理</dt><dd>推理未启用</dd></div>
      </dl>
    </section>
  );
}
