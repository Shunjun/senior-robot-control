# Video demand and 15 fps implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. The handoff explicitly preserves direct implementation by the coordinator.

**Goal:** Limit browser preview to 15 fps and allocate camera/encoding resources only to active consumers.

**Architecture:** Keep capture timestamps and latest-frame slots. The service owns demand leases and camera processes; each preview owns its lease, subscriber and encoder. On the first consumer, capture opens the vendor camera as a whole (color/depth/IR/cloud configuration intact); adding or releasing a consumer never reconfigures capture. JPEG and depth compression are independent leased components, both paced to at most 15 fps before encoding.

**Tech Stack:** Python asyncio, ROS 2 Humble/Astra, aiortc/PyAV, React/TypeScript.

**Spec:** `docs/operations/video-optimization-handoff.md` (user-authorized continuation).

## Global constraints

- Stay on `feat/vehicle-architecture-refactor`; preserve the pre-existing PATH change in `platform_runtime.py`.
- Do not restart the vehicle service or send motion commands to obtain measurements.
- Preview requests color only; capture opens as a whole per the later user decision. JPEG and depth compression start only for their consumers and encode at most 15 fps.
- Limit before AV conversion/encoding; retain real capture-time RTP gaps.
- Hide/unmount/disconnect releases preview resources; other consumer leases survive.
- Hardware encoding depends on measured device capability, never codec preference alone.
- Do not start a development server except for debugging.

## Review focus

- Closing during camera start or SDP negotiation must release only that session's resources.
- A replacement preview must not be rejected merely because the previous preview is cleaning up.
- Losing an Agent must expire its resource leases even if explicit release fails.
- Changing the consumer union must preserve resources still needed by another consumer.
- Browser visibility and pagehide/pageshow must not revive stale peer callbacks.

## Task 1: Baseline and frame pacing

- [ ] Read car CPU/GPU/video devices, running camera configuration, raw receive fps, preview RTP fps and interval CPU without restarting services.
- [x] Extend `VideoLimits` in `senior_control/src/vehicle_manager/media/stream.py` with a 15 fps ceiling; pace the latest slot before `peer.push`, using monotonic scheduling, and preserve image stamps.
- [x] Verify real aiortc output timestamps and bounded processing with existing `tests/integration/test_video.py` fixtures.

## Task 2: Shared demand ownership

- [x] Add a focused service-side camera demand owner and a client lease, using the existing private IPC protocol. Define capture and optional compression ownership once; renew leases every 3 seconds and expire abandoned requests after 12 seconds.
- [x] Remove unconditional camera startup from `service.py`; serialize component start/stop and child teardown with shutdown; never restart capture on consumer changes.
- [x] Add a project ROS launch file selecting the installed vendor camera mode, preserving whole capture, and launching separate compressed republishers only when requested. Add a serialized latest-image gate ahead of each compressor, capped at 15 fps. Keep bounded leaf recovery.
- [x] Bind each production video session to a color lease, ROS source and encoder factory; release all three after failure/cancellation/close.
- [x] Make signaling wait for previous cleanup before opening a replacement without sharing candidate queues or overwriting the new viewer's ownership.
- [x] Migrate the existing camera recovery test to acquire demand explicitly; retain real process recovery/cleanup assertions.

## Task 3: Browser and acceptance

- [x] Default manual view to map so entering the page alone does not request video.
- [x] Add visibility/page lifecycle handling to `VideoPanel`; suppress offers while hidden and close old negotiation before resuming.
- [x] Reuse video and devices tests, add only the selected scenarios below, run frontend typecheck/build and relevant Python regressions.
- [x] Review the full diff and record measured vs unverified vehicle results in operations documentation.
- [ ] Commit/push the code as a Draft PR; field acceptance is still pending. Do not merge.

## Selected verification scenarios

The following requirement scenarios are approved for implementation. One independently reproduced signaling bug has a separate targeted regression. Direct implementation follows the handoff's no-delegation instruction.

1. 30 fps input yields at most 15 fps peer submissions, latest frame wins and actual timestamp gaps survive. Reuse video source/peer and real track fixtures.
2. No demand starts no camera; acquisition starts it; last release stops it. Reuse service real-child fixture.
3. Capture remains the same process while consumers join/leave; optional JPEG/depth compression starts and stops independently. Releasing preview preserves the remaining consumer.
4. Abandoned lease expires and releases its owned processes. Reuse service IPC/child fixture.
5. Production preview acquires a service lease and closes its source, encode worker and demand together. Reuse real IPC/aiortc and process fixtures.
6. Retain actual camera leaf recovery and owner teardown, migrating its setup to acquire camera demand.
7. Hidden/pagehide closes preview; visible/pageshow creates one fresh peer and stale negotiation cannot offer. Reuse `web/src/features/video/video.test.tsx`.
8. Control page initially requests no video, explicit video switch starts it, map/page exit releases it. Reuse `web/src/features/devices/devices.test.tsx`.
9. Preserve the urgent-stop epoch assertion when the new initial map view also requests a manifest; select the actual estop message and retain its identity assertion.
10. Serialized 30 fps input emits at most 15 fps and replaces stale pending bytes without changing the original header/pixels; reuse the same gate for JPEG and depth compression.

**Actual bug regression:** closing a preview while its cleanup is pending caused an immediate replacement offer to receive `video.close`. The test was observed failing with that response, then passed after waiting for predecessor cleanup and keeping per-offer candidate queues/ownership.

Existing valid auth, congestion, real aiortc loopback, ICE, recovery and process teardown regressions remain. Avoid extra codec matrices or hypothetical hardware fallbacks; current hardware determines the implemented path.

**Dependency order:** Task 1 pacing can be independent of Task 2. Service demand API precedes production binding; Task 3 lifecycle can be independent, but final verification follows all changes. Execute inline, as requested in the handoff.


## Execution rulings and evidence

- User clarified that whole capture stays open until no consumers remain, avoiding driver restarts when another consumer joins. Optional JPEG/depth compressors remain on demand; the user then explicitly requested 15 fps for vehicle compression too.
- Keep vendor point-cloud and stream parameters intact; do not infer live settings from local defaults. The car's live parameter services were read directly.
- Whole ROS launch registration in `ros2/src/senior_robot/setup.py` is required so deployed colcon builds can find the new launch and limiter executable.
- Direct implementation and review are preserved from the handoff; no Orca or temporary subagents were created.
- Validation output and remaining field checks are recorded in the operations handoff. No existing vehicle service was restarted and no motion command was sent.

- Final related Python run: 61 passed, 2 skipped, one unchanged LearningClient monkeypatch failure. Field lifecycle acceptance and the last car-side compression run remain unverified because the SSH connection ended with No route to host.
