# Remaining dependency and failed-test cleanup implementation plan

> Execute directly with superpowers:executing-plans. The user authorized current Codex execution when Orca was unavailable and now explicitly requested completing these remaining items. Continue `feat/vehicle-architecture-refactor`, commit in batches and update existing Draft PR38.

**Goal:** Remove perception/estimation mutual imports and perception's concrete control-profile dependency, then reconcile failing tests with the approved current behavior without restoring retired driving gates.

**Architecture:** Move spatial estimation and RGB-D body-motion interpretation to motion estimation. Estimation may consume perception output; perception must not import estimation or control. LearningPorts accepts a narrow read-only structural profile protocol containing only the four attributes actually read. Keep the current implementations and numeric behavior, no new runtime providers or compatibility facades.

**Spec:** `docs/superpowers/specs/2026-11-12-vehicle-motion-redesign.md`, especially the approved2026-11-18 revision, plus the user's explicit2026-11-23 continuation. Prior ROS ownership migration: `docs/superpowers/plans/2026-11-22-ros-package-cleanup.md`. Baseline `ed42e0ac526f223500528472d21970f4990e7b3e`.

## Constraints and review focus

- No development/vehicle services, simulation, deployment or motion commands. Local test child processes and socket listeners are test resources only.
- Do not restore controller/scheduler/qualification/ScanGeometry APIs to satisfy stale tests. Do not reintroduce independent-velocity, learning, web-latency or stopping-certificate driving gates.
- Retain valid software-estop revision/latch, source/epoch fencing, original command deadline, local250ms lease, task terminal identity/release and real sensor-age protections.
- Tests asserting removed sweep/permit/settling/admission behavior may be retired with an explicit requirement mapping. Mixed files keep valid cases. No blanket skip/xfail, weakened assertions or removal based only on failure.
- Existing raw source/algorithm, wire framing, IPC exclusive bind and real child-process cleanup regressions remain and are run. Environment and pre-existing failures are not reported as passes.
- File forecast before edits: product add1–3/mod10–18/delete2–3; tests/data add0–2/mod35–55/delete5–12; build/config mod2–4; tools/docs add/mod5–8; total60–105 paths. Reforecast if this materially expands.

## Task1: Capture and classify current failures

- [x] Run each current test file in a bounded subprocess, preserving output, exit code and timeout status under `/tmp/ros-cleanup-followup/baseline`. Do not mutate product/tests while this baseline runs.
- [x] Classify each failure as retired requirement, moved name/fixture, environment, or reproduced product defect. Record exact cases and evidence before changing them.
- [x] Read-only candidate review: reuse existing interface/algorithm protections; select necessary current behavior scenarios before implementation. User has already approved completing cleanup; do not add product behaviors.

## Task2: Finish dependency ownership

- [x] Move `senior_perception/space_estimator.py` to `senior_motion_estimation/space_estimator.py`; move `senior_perception/vision/rgbd_body_motion.py` to `senior_motion_estimation/rgbd_body_motion.py`; rewrite all live consumers, tests and public docs. No forwarding facades.
- [x] Introduce read-only `LearningProfile` protocol in existing perception learning_runtime with length/width and two startup-learning occlusion thresholds. Change only LearningPorts annotation; preserve actual profile object injection and attribute reads.
- [x] Declare estimation's actual perception dependency and corresponding shapely runtime dependency. Add an actual Python import-boundary assertion to existing package ownership check, not just manifest DAG.
- [x] Run existing pose/space, RGB-D transform, optional motion inputs, learning lifecycle and canonical ownership checks. Review exact move diffs and commit independently.

## Task3: Migrate retired test surfaces and fix real bugs

- [x] Update resource/fixture/import targets to current owners; preserve valid assertions.
- [x] Retire only explicitly superseded sweep/certificate/settling/admission scenarios; retain independent estimation/geometry algorithm tests even when those algorithms are no longer driving gates.
- [x] Adapt valid control tests to the current raw-scan/collision/speed pipeline and arbitration owner. Use existing real MotionCoreOutput composition; do not mock away its decisions.
- [x] Reproduce each remaining real bug, explain cause and fix at its owner; record targeted before/after regression evidence. Fixes must not expand current behavior.
- [x] Resolve collection errors and test resource leaks/timeouts. Preserve process/socket cleanup and exclusivity assertions. Commit coherent groups as they pass.

## Task4: Verify and deliver

- [x] Rerun affected files and final broad collection/suite with bounded resources. Explicitly report every remaining failure/skip/timeout, including unavailable ROS/native gates.
- [x] Build affected wheels and verify imports from isolated installation without source editable paths. Run Ruff/changed-file formatting/diff checks.
- [x] Personally review product diffs, retained behavioral coverage and every test deletion. Update current ownership/testing/handoff docs, push feature branch and update Draft PR38, without main merge.

## Selected new/materially rewritten behavioral scenarios

Cross-module cleanup cap20, selected10 (not a target to fill). Import/fixture/path-only changes that preserve scenario assertions are not new behaviors. Confirmed real-bug regressions are recorded separately.

1. Runtime Python dependencies have no perception import of control/estimation, and package manifests remain acyclic (extend existing ownership scenario).
2. Independent pose/space and RGB-D interpretation remain numerically unchanged at their canonical estimation imports (reuse existing scenarios; no new matrix).
3. Learning consumes the original injected profile through its narrow port without loading control (one import/port scenario, reuse lifecycle checks).
4. Fresh raw scan and manual target reach real continuous speed output without independent-body or learning qualification.
5. Missing/expired raw scan emits zero and valid data restores output while preserving source.
6. Forward/reverse obstacle clearance applies the current directional limit, without persistent fault.
7. Release/expiry emits zero immediately; zero-stick retains authority and old targets do not resume.
8. Software estop remains latched until explicit matching revision reset; no old-command replay.
9. Current output retains original target/sensor deadline and local250ms lease, using actual emitted candidate.
10. Existing task retirement/collection identity and replay guards remain valid through the current arbitration owner.

Selection rationale: real current driving/control boundaries and the requested dependency cleanup. Omit new hypothetical DDS/hardware faults, parameter matrices and duplicate per-facade identity tests. Preserve existing valid regression even when outside these new/materially changed scenarios.

## Execution ledger

- Read approved design revision and remaining imports. `space_estimator` and RGB-D body interpretation are estimation algorithms currently under perception. LearningRuntime reads only four concrete profile attributes. No algorithm rewrite needed.
- Baseline requested with socket permission to distinguish sandbox bind failures from actual IPC failures; per-file timeout75s plus8s SIGINT cleanup, then kill only that test subprocess group. No pass claimed for abnormal exit.


### First implementation checkpoint

- Completed spatial estimation/RGB-D body-motion moves and canonical caller imports; implementations unchanged apart from import organization. LearningProfile is a four-property read-only Protocol; original profile object is still injected. Declared estimation dependency on perception and shapely. Ownership test now checks actual perception Python imports for both forbidden upper-layer packages.
- Retired test_continuous_scan_control (old ScanGeometry/ConstraintEngine continuous scheduling, two parameterized scenarios) and test_scan_geometry_scope (removed geometry job pending/proof lifecycle). Both exclusively require the geometry scheduler explicitly removed by the approved2026-11-18 revision. Independent spatial/pose algorithm tests are retained.
- Replaced obsolete automatic complete-learning/visibility/snapshot cases in test_learning_core_surface_review; preserved its real-obstacle protection as one actual MotionCoreOutput raw-scan scenario with both learning stop switches disabled. Assert zero output, source retained and no fault latch. No product decision mocked.
- Targeted verification: 18 passed in0.79s (ownership, RGB-D transforms, pose/space, optional motion inputs and obstacle output). Unit collection:1198 tests in0.72s, no errors. Ruff/diff checks passed. No whole-suite pass claimed.
- Original bounded per-file baseline attempt was interrupted by runtime/tool restart before producing retained logs. Only current collection failure reproduction and targeted results are evidence. Broad runtime failures, isolated installation, native ROS gates and final delivery remain open; do not mark overall cleanup complete.

### Continuous cleanup checkpoint

- User clarified that checkpoint commits are not stopping points; continue until the requested cleanup converges.
- Current full unit snapshot before this batch:65 failures/101 errors/976 passes; after canonical fixture migration:74 failures/25 errors/1028 passes. Error-to-failure changes are not claimed fixes; exact remaining results are retained under `/tmp/ros-followup`.
- Reception helper now returns the actual reception-owned estimate rather than call deleted output `_inputs` or estimate again in the read-only control snapshot. Boundary/backlog fixtures patch both actual arbitration and output clocks, use `_last_tick`, and inspect reception-owned tracking rather than nonexistent reader tracking. Original timestamp/frontier assertions retained.
- Modernized shared task fixture with explicit synthetic open raw scans (no geometry/scheduler fake permit). Terminal proof/collection tests use canonical arbitration owner for adversarial state injection. Actual fault preservation is triggered with EXECUTION_FAULT, not a peer close packet. Task identity assertions remain unchanged.
- Two current output tests replace retired controller propose_tick/permit/braking-proof cases: original source bound and250ms local lease, and expiry producing zero with no target replay. Removed test_motion_core_stop_fallback and test_motion_core_event_time: exclusively retired prepare_stop/transactional-controller fallback; rollback protection remains exercised by existing task identity release checks.
- Retired degraded-manual tests that require output `_snapshot`, independent-body mode admission and ConstraintEngine candidates. Kept original metadata validation, wire metadata rejection and standalone controller acceleration tests. Stop is no longer a permanent permission close; release/revoke remain in permanent-close matrix.
- Current report fixture models an already established output incarnation, initializing the fields from the actual constructor instead of triggering an unrelated first-boot handoff. Output incarnation/handoff tests are separate existing coverage. Software estop reset assertion now observes actual `_output`, retaining no-replay and revision checks.
- Added deterministic unit teardown for actual core instances even if a replay fixture fails before yield; no product service is started or modified. Test case failures no longer leave live diagnostic resources.
- Verified89 tests passed in2.18s (metadata,deadline,authority,reference,terminal identity/collection);31 passed in0.71s (runtime reports,software-estop,core handoff). Ruff passed. Remaining full suite is still being repaired.

### Product defects and retained current behavior

- Confirmed bug: state-machine clock rollback latched execution_fault but retained previous-coordinate stationary evidence. Existing observation-clock regression reproduced stationary=True after rollback; `_advance_clock` now clears `_evidence` before projecting the unchanged fault.59 state/estop/terminal/handoff tests passed after fix.
- Confirmed bug: early learning-motion withdrawal set validation_after before the original action end; service correctly requires a post-window stationary watermark, so capture stayed open indefinitely. After the fixed original window, a stationary validating consumer now advances that watermark once. Existing withdrawn-window test failed before fix and passes afterward, preserving original3s deadline, actual nonzero-duration and no-motion-replay assertions. Learning chain19 passed after fixture protocol corrections.
- Confirmed config drift: hardware-motion JSON Schema listed length/width optional while the actual profile requires both. Regenerated the schema; only these two fields/defaults and required entries changed. Original schema-equality regression retained.
- Retired only real-core integration with the removed stop-model envelope/settling/sweep/certificate and pose-aligned scan qualification. `test_stopping_fused_input` retains hardware assumptions and full profile schema/loader checks. `test_tick_evaluation` retains21 standalone engine/controller/scheduler scenarios;23 tests across those files pass. `test_estimation_review_contracts` retains independent numerical, origin/dedup, space/pose/error and diagnostics checks; seven replay functions coupling those values to removed control `_observation`/`_space_estimate` were retired. Relative-interval standalone numerical scenarios remain; the retired control scan-alignment adapter scenario was removed.
- Current startup raw-scan actual-output journey verifies no independent-motion qualification, zero on missing scan, preserved manual source/no fault, and recovery when scan returns. Navigation overlimit output retains all four original input groups and cap/deadline assertions; removed only scheduler waiting. State reset explicitly matches revision without old motion-evidence qualification; replay cannot revive the former source. Legacy five-sample feedback readiness and automatic feedback replanning tests were retired; explicit task pause/done/resume protections remain.
- Current unit suite:1059 passed,10 skipped,2 socket-permission failures (both runtime_identity),33.95s. These two are included in the permitted-socket integration rerun, not treated as passes. First permitted-socket integration baseline:1050 passed,38 failed,11 errors,29 skipped. Remaining repairs are fixture ABI fields, IPC learning-capability response, updated capture-wire gap injection and output trace metadata. No blanket skip/xfail was added.
- Fixed the existing default-collection test_home basename collision by renaming only the e2e file to test_home_persistence; its contents and assertions are unchanged.


### Final verification and delivery

- Completed normal-exit default full-tree run (socket permission, existing Docker CLI on PATH for compose config only):2453 passed,116 environment/native-gated skips,1 existing forkpty deprecation warning,113.91s. No failure/collection error/xfail. Final output-failure safety scenario appended after that run:cadence/output file4 passed in0.32s, including all three unchanged timing cases. Product code did not change between these verifications.
- Initial default-order run hung in test_learning_cleanup:its termination condition waited for eager camera spawn, intentionally removed by demand-driven capture. Ended only the identified pytest PID94412; changed fixture stop condition to completion of its cleanup assertions.16 cleanup cases passed; full rerun exited normally. This was a test harness bug, not a product service hang.
- Integration snapshot1101 passed/29 skipped includes both socket-identity cases that failed inside the sandbox. Default collection has no import collision after the byte-identical e2e file rename.
- Reused valid algorithm/process/IPC/identity/lease/estop regressions. Final retirement review list includes71 removed/renamed functions across17 paths; one is the byte-identical e2e rename and two are direct behavior-renamed current tests. Remaining removals correspond to retired control qualification/sweep/stop-certificate/geometry-queue/automatic-learning paths, not independent algorithm protection. The valid failed-output zero/revocation case was explicitly preserved under the current SafetyGate timer and actual core.
- New/materially rewritten current scenarios conservatively counted14 under the cross-module cap20; three confirmed bug/config regressions use existing tests and are recorded separately. No new hypothetical failure matrix, skip or xfail was introduced. Module/fixture/clock/trace/protocol-shape-only changes retain their behavioral assertions.
- Five final wheels built offline using existing uv cache:core/contracts/perception/estimation/control. Isolated installed import with `python -I -S` and installed/third-party paths only passed:perception learning import loads neither control nor estimation; new algorithm locations resolve from wheel; old perception algorithm modules absent; shared RecoveryObservation type identity unchanged; no rclpy loaded. Fresh isolated build cache lacked hatchling, so reused pre-existing user cache offline rather than fetching dependencies.
- Personally reviewed product diff and deletion inventory. Ruff, changed Python formatting and diff whitespace checks passed. No independent worker review claimed (user authorized direct execution). Native ROS/ament/typesupport/installed C++ header and car acceptance remain unavailable, not counted as passes.
- Delivery uses existing feature branch and Draft PR38, with incremental commits; no main merge, deployment, dev service, simulator or vehicle motion.
