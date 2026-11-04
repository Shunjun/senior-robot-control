import { useEffect, useState, type ReactElement } from "react";
import type { RobotLink } from "../../api/client";
import { DetectionOverlay, type Detection } from "./DetectionOverlay";
import styles from "./Video.module.css";

export function VideoPanel({ link, stream: suppliedStream = null, frameTimestamp = null, detections = [] }: {
  link?: RobotLink;
  stream?: MediaStream | null;
  frameTimestamp?: number | null;
  detections?: Detection[];
}): ReactElement {
  const [remoteStream, setRemoteStream] = useState<MediaStream | null>(suppliedStream);
  const stream = suppliedStream ?? remoteStream;

  useEffect(() => {
    if (
      link?.sendVideoSignal === undefined ||
      link.subscribeVideo === undefined ||
      typeof RTCPeerConnection === "undefined"
    ) return;
    const peer = new RTCPeerConnection();
    let active = true;
    peer.addTransceiver("video", { direction: "recvonly" });
    peer.ontrack = (event) => {
      if (active) setRemoteStream(event.streams[0] ?? new MediaStream([event.track]));
    };
    peer.onicecandidate = (event) => {
      if (event.candidate !== null) {
        link.sendVideoSignal?.("video.candidate", { candidate: event.candidate.candidate });
      }
    };
    const off = link.subscribeVideo(async (frame) => {
      if (!active) return;
      if (frame.type === "video.answer") {
        await peer.setRemoteDescription({ type: "answer", sdp: frame.payload.sdp });
      } else if (frame.type === "video.candidate") {
        await peer.addIceCandidate({ candidate: frame.payload.candidate });
      } else if (frame.type === "video.close") {
        setRemoteStream(null);
        peer.close();
      }
    });
    void peer.createOffer()
      .then(async (offer) => {
        await peer.setLocalDescription(offer);
        if (active && offer.sdp) link.sendVideoSignal?.("video.offer", { sdp: offer.sdp });
      })
      .catch(() => setRemoteStream(null));
    return () => {
      active = false;
      off();
      peer.close();
      try {
        link.sendVideoSignal?.("video.close", {});
      } catch {
        // The control socket may already be gone; media cleanup remains local.
      }
    };
  }, [link]);

  return (
    <section className={styles.panel} aria-label="实时视频">
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
