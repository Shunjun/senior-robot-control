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

- [ ] Run each current test file in a bounded subprocess, preserving output, exit code and timeout status under `/tmp/ros-cleanup-followup/baseline`. Do not mutate product/tests while this baseline runs.
- [ ] Classify each failure as retired requirement, moved name/fixture, environment, or reproduced product defect. Record exact cases and evidence before changing them.
- [ ] Read-only candidate review: reuse existing interface/algorithm protections; select necessary current behavior scenarios before implementation. User has already approved completing cleanup; do not add product behaviors.

## Task2: Finish dependency ownership

- [ ] Move `senior_perception/space_estimator.py` to `senior_motion_estimation/space_estimator.py`; move `senior_perception/vision/rgbd_body_motion.py` to `senior_motion_estimation/rgbd_body_motion.py`; rewrite all live consumers, tests and public docs. No forwarding facades.
- [ ] Introduce read-only `LearningProfile` protocol in existing perception learning_runtime with length/width and two startup-learning occlusion thresholds. Change only LearningPorts annotation; preserve actual profile object injection and attribute reads.
- [ ] Declare estimation's actual perception dependency and corresponding shapely runtime dependency. Add an actual Python import-boundary assertion to existing package ownership check, not just manifest DAG.
- [ ] Run existing pose/space, RGB-D transform, optional motion inputs, learning lifecycle and canonical ownership checks. Review exact move diffs and commit independently.

## Task3: Migrate retired test surfaces and fix real bugs

- [ ] Update resource/fixture/import targets to current owners; preserve valid assertions.
- [ ] Retire only explicitly superseded sweep/certificate/settling/admission scenarios; retain independent estimation/geometry algorithm tests even when those algorithms are no longer driving gates.
- [ ] Adapt valid control tests to the current raw-scan/collision/speed pipeline and arbitration owner. Use existing real MotionCoreOutput composition; do not mock away its decisions.
- [ ] Reproduce each remaining real bug, explain cause and fix at its owner; record targeted before/after regression evidence. Fixes must not expand current behavior.
- [ ] Resolve collection errors and test resource leaks/timeouts. Preserve process/socket cleanup and exclusivity assertions. Commit coherent groups as they pass.

## Task4: Verify and deliver

- [ ] Rerun affected files and final broad collection/suite with bounded resources. Explicitly report every remaining failure/skip/timeout, including unavailable ROS/native gates.
- [ ] Build affected wheels and verify imports from isolated installation without source editable paths. Run Ruff/changed-file formatting/diff checks.
- [ ] Personally review product diffs, retained behavioral coverage and every test deletion. Update current ownership/testing/handoff docs, push feature branch and update Draft PR38, without main merge.

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
