# ROS package ownership cleanup implementation plan

> **Execution:** Use superpowers:executing-plans. The user explicitly authorized direct Codex implementation because Orca CLI is unavailable; do not dispatch workers.

**Goal:** Retire the three obsolete package boundaries and historical robot facades, preserving actual algorithms and transport fields.

**Architecture:** Seven functional packages, parallel real/simulation composition packages, and one interface-only `senior_interfaces` package. Capture timing and shared ROS lifecycle belong to sensor ingress. The estimation package owns scan matching. Runtime nodes live with their functional algorithms; robot composition injects hardware-profile loading where otherwise a reverse dependency on control would arise.

**Spec:** User-approved ownership inventory in this chat and `docs/architecture/ROS2与车端架构重构确认稿.md`.

**Baseline:** `da3ef99728544b566fd29adc55fdb2cac16c8833`; continue `feat/vehicle-architecture-refactor`, update existing PR #38. No development/vehicle services, motion commands, or deployment.

## Review focus

- ROS generated interfaces must remain independent of Python runtime installation; preserve every field, bound, constant and service section.
- Move handwritten timing headers to sensor ingress with a real installed CMake export; do not leave source-tree include fallbacks.
- No functional package may depend on the real/simulation composition package; avoid estimation/control or ingress/interfaces cycles.
- Remove old facades only after rewriting static imports, dynamic module names, patch targets, launch executable ownership and install verification.
- Keep real and simulated composition behavior and cleanup, native wire frames, calibration inputs and algorithm assertions unchanged.

## Task 1: Interface and native algorithm ownership

- [x] Capture existing broad test failures before product edits; keep them separate from migration regressions.
- [x] Merge 25 motion messages plus Keyframe and five SLAM services into `ros2/src/senior_interfaces/{msg,srv}`. Keep fields intact; rewrite nested type package names and all consumers.
- [x] Move sample_time.hpp/sensor_timing.hpp and the installed-header consumer to `senior_sensor_ingress`; export the header target. Retire unused latest_task.hpp only after confirming no callers.
- [x] Move the scan matcher headers, sources, replay tool and tests to `senior_motion_estimation`; merge CMake targets, update runtime/package dependencies. Namespace implementation as motion estimation, generated messages as interfaces.
- [x] Remove the three old package build manifests; verify package discovery/dependency DAG and standalone native tests.

## Task 2: Functional nodes and composition boundary

- [x] Move generic ROS lifecycle from motion control to sensor ingress; rewrite consumers to that public boundary.
- [x] Move MotionFusion to estimation; inject profile loading from a small robot composition entry rather than import control into estimation.
- [x] Move perception/depth/semantic/RGB-D nodes and public scripts to perception. Move ROS video/rate limiter to ingress; update manager media binding and camera launch.
- [x] Move simulation transport helper and profile to senior_sim; register its Python package and resources.
- [x] Rewrite all callers of the 29 import facades by actual symbol owner, including combined historical facades. Remove old camera/OwnedState facades and obsolete simulation forwarding launches. Keep real preflight/service/adapter/whole-vehicle launch composition.
- [x] Update package scripts, setup, CI, tools, public README ownership, and affected existing tests without weakening algorithm assertions.

## Task 3: Verification and delivery

- [x] Add a small structural migration check: ten packages, no retired references in active source, interface field preservation, valid dependency DAG and node executable targets.
- [ ] Native installed-header/ROS scan matcher checks still need Linux; portable regressions and exact broad failure comparison completed, see testing record.
- [x] Built ten wheels and performed isolated import checks; recorded unavailable ROS/ament/DDS validation explicitly.
- [ ] Personally reviewed moved source/build registration and actual diffs; incremental commits created. Push and Draft PR #38 update are the remaining delivery actions.

**Order:** Task 1 establishes interface/header names before Task 2 rewrites runtime entrypoints. Native algorithm relocation and facade inventory are independent conceptually, but execute sequentially in this one checkout under the user's direct-execution authorization. No parallel worktrees needed.

## Approved verification scenarios

1. Exactly the intended ten ROS packages remain, with no package dependency cycle.
2. All interface fields/bounds/constants and service request/response sections survive package-qualified type renaming.
3. Installed consumer can discover timing headers and generated messages from their actual packages.
4. Existing scan matcher replay/point-index/shutdown checks use estimation-owned sources and executable.
5. Real/sim composition targets resolve to functional executable owners; lifecycle exits remain bounded.
6. Production modules and isolated wheel imports do not need retired modules/package names.
7. Existing control/perception/estimation behavior and frame contracts keep their assertions under canonical imports.
8. Camera source, serialized compression limiter and production video lease/encoder lifecycle remain valid after relocation.

## Execution ledger

- Baseline scan: 220 current paths reference affected ownership; 29 import-only robot facades. Package manifests and launch references inspected. Baseline suite runs before edits.
- Ruling: consolidate ROS contracts into a dedicated interface-only package, rather than generating Python interfaces inside existing ament_python_install_package runtime projects, per Humble build constraints.
- Ruling: keep senior_robot/senior_sim as real composition owners; removal means retired package boundaries and facade files, not deleting still-required hardware/simulation entrypoints.

### Checkpoint 1

- User requested incremental commits while implementation continues. Save the current coherent migration before remaining verification fixes; this is not completion/field acceptance.
- Baseline: 2248 passed, 200 failed, 112 errors, 115 skipped (519.05 seconds), log `/tmp/ros-cleanup/baseline.log`. New structure/field preservation checks: 2 passed.
- Canonical launch/profile/lifecycle/perception/native framing selection after fixing moved test paths: 69 passed, 38 skipped.
- Full repository Ruff lint passed. No new collection errors: the same three retired ScanGeometry/full-cycle imports remain from baseline.
- Docker daemon is unavailable; do not start it. Car reachability query is pending. Broad comparison, install/build proof, dynamic ROS fixture paths, package metadata cleanup and PR update remain in progress.

### Checkpoint 2

- Fixed remaining dynamic ROS test module/executable/config references, including subprocess scripts. MotionFusion fixtures now inject the profile factory through its explicit composition port.
- Reproduced the moved optional-camera import bug: a missing `senior_sensor_ingress` escaped instead of disabling preview (one regression failed before fix); corrected the exact package guard, regression now passes.
- Reconciled existing mapping manifest `slam_toolbox 2.6.10` with its CMake `EXACT 2.6.8`; removed duplicate ingress dependency declaration.
- Verification: Ruff passed; focused ownership, launch, lifecycle, native framing, camera regression: 72 passed, 43 skipped (macOS lacks ROS). Broad suite still waiting in asyncio after its faulthandler timeout; no broad pass claimed.

### Checkpoint 3

- Relocated the unchanged RecoveryObservation data value from control to estimation so the estimation producer no longer imports its control consumer. The existing public control import remains the same class (`//Compatible`).
- Existing raw wheel/IMU/stillness/recovery scenarios: 36 passed. Larger startup/rearm selection: 42 passed, 2 failed, 13 errors; every failure/error node is already present in the pre-migration baseline (retired admission expectations and missing controller attribute), not removed or weakened.
- Reviewed package build registration and installed-header consumer. Normalized estimation manifest formatting. Ten portable Python wheels built and installed into `/tmp/ros-cleanup/installed`; final wheels must be rebuilt after this checkpoint.
- Import audit also finds pre-existing perception/estimation references and a perception learning-profile reference to control; the manifest DAG check does not certify complete Python dependency independence. These are separate from removal of retired package/facade names.

### Checkpoint 4

- Broader portable selection completed: 1217 passed, 116 failed, 104 errors, 11 skipped (962.85 seconds; it required SIGINT to unblock an asyncio wait before finishing). Compared exact failing node names with the original baseline: one new failure, localization script registration still asserted the removed robot setup entry.
- Updated that existing assertion to validate the actual localization CMake install, executable permission and canonical script import; all 18 localization scenarios now pass. No algorithm assertion removed. All remaining broad failing/error nodes were already in baseline.
- Isolated final estimation/control wheel reinstall and imports passed with `python -I -S`, installed directory and third-party site-packages only: retired facades absent, canonical module paths, recovery type identity, navigation contracts and read-only preflight verified. ROS installation/assets/typesupport remain unverified.
- Default whole-tree collection also exposes an unchanged `test_home` basename collision between e2e and manager tests; original comparison roots omit e2e/deploy. Preserve and report it, do not rename unrelated tests.

### Checkpoint 5

- Actual media/camera loopback tests with localhost UDP/Unix socket access: 48 passed in12.29s. No development or vehicle services started.
- Final same-root collection: 2672 tests collected, exactly the three original collection errors.
- Current layout/rebuild instructions and test evidence documented; historical B-stage READMEs explicitly marked as historical where they mention removed facades/packages.
- Final code review removed redundant alias-to-itself checks in install verifier. Scope remains within the announced 280–350 file-path range. No independent worker review claimed; Orca bypass was explicitly authorized.
