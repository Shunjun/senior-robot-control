# Five-frame startup learning verification

Version: commit containing this report, based on `71a8ea8`, branch `feat/scan-matcher-performance`. The prior scan-matcher optimization remains intact. This change touches 3 product files, 2 test files and 4 documentation files (9 total).

## Implemented behavior

- Capture and the existing fixed three-second motion window proceed without running classification. Once capture and the existing stationary-tail lifecycle are complete, select exactly five distinct motion-window frames nearest temporal quartiles of the actual first/last acquisitions. Preserve original frame objects, transforms, clocks and source identity. Fewer than five real acquisitions cannot be duplicated to produce a completed model.
- Feed only selected frames to the existing CPU child, in batches of four and one. Decode at most five observations; classify only upon the final batch. Match first-frame reference positions to the other four hit clouds with the original 1cm position policy and spatial indices. Do one reference connectivity pass after stability filtering, without the streaming candidate fixpoint loop.
- Include stable surfaces connected to strict interior seeds even outside the body. Split stable but spatially distant surfaces, including valid hits at consecutive beam indices. Connection threshold is actual adjacent-beam chord length plus two existing 1cm position allowances; no obstacle-budget or user-configurable margin is added.
- Preserve action timing, takeover, source/capture admission, FIFO limits, durable single-writer results, explicit recalibration and frozen runtime filtering. Raw frame transport is unchanged; this is a computation reduction, not a redesign of the entire learning IPC system. Legacy streaming-model APIs remain available for compatibility, but production `LearningKernel` no longer invokes their `observe`/`validate` path.

## Selected scenarios

Five selected scenarios: **3 new, 2 migrated**, below the planned medium-change limit of ten. No actual bug-specific regression is claimed.

| Scenario | Verified assertion | Result |
|---|---|---|
| Migrated raw-frame/kernel scenario | Nonfinite values and sample times survive; no result before final batch; exactly 5 observations; isolated interior echo cannot pull in a distant wall | PASS |
| Migrated actual-transform scenario | Each decoded observation uses its original TF; structural geometry changes remain rejected | PASS |
| Temporal selection | Five distinct ordered quartiles inside motion; original objects retained; insufficient acquisitions rejected | PASS |
| Service/owner/persistence chain | No CPU flight while capture is open; after closure exactly 5 frames compute and the model becomes durable; action facts unchanged | PASS |
| Attached extension versus wall | Stable interior-connected exterior equipment retained, a separate stable wall and moving isolated return remain obstacles; filtering does not mutate the model | PASS |

The selected service test uses the actual service, record owner, classification kernel and filesystem persistence with deterministic in-process transport/commit resources. It does not demonstrate Linux socket/process or real ROS/robot behavior.

## Executed commands and results

```sh
.venv/bin/pytest -q \
  ros2/src/senior_perception/test/test_learning_compute.py \
  ros2/src/senior_perception/test/test_sampled_learning.py \
  ros2/src/senior_perception/test/test_learning_record_owner.py \
  ros2/src/senior_perception/test/test_learning_dual_clock.py \
  tests/unit/test_startup_learning_review.py \
  ros2/src/senior_perception/test/test_learning_client.py \
  ros2/src/senior_perception/test/test_learning_cleanup.py \
  ros2/src/senior_perception/test/test_learning_service_linux.py
```

**52 passed, 1 skipped.** The skip is the existing Linux-only native-service check on macOS. Existing cancellation/late-batch ordering, overflow, durable owner, recalibration, cleanup and clock regressions are retained.

```sh
.venv/bin/pytest -q \
  tests/unit/test_startup_learning_timed.py \
  tests/unit/test_learning_frozen_reference.py::test_adjacent_valid_returns_extend_outside_without_connection_distance_threshold \
  tests/unit/test_learning_frozen_reference.py::test_invalid_gap_and_no_interior_seed_keep_disconnected_points_external \
  tests/unit/test_learning_frozen_reference.py::test_unstable_connected_bridge_excludes_disconnected_outside_point \
  tests/unit/test_learning_frozen_reference.py::test_neighbor_direction_and_changed_beam_indices_match_same_surface_position \
  tests/unit/test_learning_frozen_reference.py::test_learning_checks_early_instability_even_after_bounded_capture_window_moves \
  tests/unit/test_learning_frozen_reference.py::test_fixed_one_centimeter_boundary_applies_inside_and_outside_body
```

**18 passed, 1 failed.** The failure is `test_completed_result_restores_same_run_without_replaying_action`: unchanged test line 160 calls `InProcessClient.complete`, which does not exist in the current test support. The same failure was reproduced before any product modification in this task (baseline run: 38 passed, 1 failed). It is not hidden or repaired as part of this optimization. Direct streaming-model tests exercise compatibility APIs, not the new sampled production policy.

Combined post-change evidence: **70 passed, 1 skipped, 1 pre-existing failure**. The five selected new/migrated scenarios are included in those totals; all other passes are reused regressions.

Ruff format/check and `git diff --check` pass for the changed Python files. Actual diff review covered temporal-selection bounds, unique samples, four-frame transport limits, capture closure, partial-batch behavior, reference position stability, spatial/wrap connection splitting, durable commits and unchanged runtime filtering.

## Limits

No ROS/robot deployment or real lidar recording was tested. Full acquisition transport and existing raw FIFO limits are unchanged. Raw frames may continue to be sent outside the learning interval under the previous lifecycle; this change does not claim to eliminate that independent transport overhead. No wall-clock acceleration figure is claimed: the changed input and connectivity policy intentionally differ from the old full-stream algorithm. The production classifier now processes five frames regardless of how many motion-window frames are present within the existing capture budget.

## Follow-up: forwarding modules and recording lifetime

The follow-up commit containing this section deletes six repository-unused production forwarding modules: `senior_control.runtime.learning_cleanup`, `learning_compute`, `learning_owner`, `learning_service`, `learning_store`, and `senior_control.motion.startup_learning`. Three test modules now import their canonical `senior_perception` implementations. Existing logger names are intentionally preserved as log-filter identities, not import references.

The service now reports whether the current attempt still accepts recordings. The raw sender returns before decoding or copying a new scan after capture closes or the attempt fails, clears pending raw frames, and polls only lightweight control metadata. A new explicit attempt reopens recording. The existing four-frame transport, full-stream tracking and old streaming-model APIs are not removed by this follow-up.

Selected scenarios: no new general scenarios for the mechanical alias deletion; existing import/functional checks are reused. One targeted regression covers the confirmed recording-lifetime defect: after a completed durable model the sender ignores even an input without message/header/ranges (proving no conversion occurs), and an explicit new attempt reopens recording. The real sender thread communicates with the actual service/owner using the existing in-process transport fixture. The defect was the absence of any completed-attempt stop condition, causing each incoming scan to be copied and transmitted even after the service discarded it.

Validation at the follow-up commit: the prior eight-module 52-pass command plus this new client regression and `tests/unit/test_startup_learning.py::test_program_start_context_is_explicit_and_run_owned` produces **54 passed, 1 skipped**. The three test files whose imports migrated collect successfully (**41 scenarios**, collection only, not counted as executed passes). Ruff check and `git diff --check` pass. Previously reported stale-test failures and unavailable native ROS/Linux validation remain unchanged; they are not claimed fixed.

## Final cleanup: no streaming learner or full-frame proof

Version: the follow-up commit containing this section, based on `d2e57ea` on `feat/scan-matcher-performance`.

Removed production streaming `SelfReturnModel.observe/validate/retain_recent/begin_segment`, 64-frame history, capture-history metadata and candidate fixpoint loop. Five frames now execute as one `ComputeRequest` through a lazy dedicated generic `ProcessWorker`; there is no learning-specific streaming kernel, finish flag, cumulative batch count or custom child polling loop. The shared worker implementation is unchanged and may stay alive for subsequent explicit calibration; this is not a claim of zero idle-process overhead.

Removed raw transport index/stream IDs, per-batch ACK frontiers and the requirement to deliver every unselected scan. Recording remains bounded: the sender keeps 16 recent frames, and the service compacts a dense 128-frame sampling buffer to 64 time-ordered candidates preserving endpoints. The five-frame selector is exact over retained candidates; dense-input compaction may approximate the nearest quartile among the full original stream. Sample origin, monotonic clock/sequence, geometry and unique acquisitions are still checked. Failed optional recordings do not clear ordinary safety scans. Capture completion/failure still stops raw recording until the next explicit attempt.

Removed startup's extra five-preframe gate and five-post-stopframe wait. The fixed three-second action, raw scan admission/TTL, final output limit, takeover and stationary fact remain. A stop fact can close a window after the reception watermark covers it, provided five in-window acquisitions were received before takeover. A later takeover cannot cancel already closed data; a late arrival recorded after takeover cannot be used to complete an earlier window. Durable commit and frozen-model restoration still use the existing owner/storage chain.

Old tests were migrated rather than using compatibility methods: five-observation classification replaces sequential model mutation; source/duplicate/clock checks move to the actual job boundary; spatially adjacent surfaces replace the retired unlimited-gap connection policy; dense full-resolution reference retention, periodic/inclusive/float32 seam handling, alternate stable connection paths, one-centimeter boundaries, masking/occlusion and immutable filtering remain covered. History eviction/reordering tests tied solely to the deleted API were retired. Former tests calling nonexistent `complete`, legacy mutable core properties or retired `ScanGeometry` were repaired to use the current owner and actual composed learning consumer.

Executed command:

```sh
.venv/bin/pytest -q \
  ros2/src/senior_perception/test/test_learning_compute.py \
  ros2/src/senior_perception/test/test_sampled_learning.py \
  ros2/src/senior_perception/test/test_learning_client.py \
  ros2/src/senior_perception/test/test_learning_record_owner.py \
  ros2/src/senior_perception/test/test_learning_dual_clock.py \
  ros2/src/senior_perception/test/test_learning_cleanup.py \
  ros2/src/senior_perception/test/test_learning_service_linux.py \
  tests/unit/test_startup_learning_review.py \
  tests/unit/test_startup_learning.py \
  tests/unit/test_startup_learning_timed.py \
  tests/unit/test_learning_frozen_reference.py \
  tests/unit/test_learning_surface_review.py \
  tests/unit/test_body_return_geometry_only.py
```

Result: **124 passed, 1 skipped**. The skip is Linux-only native socket/service integration on macOS. The passing set includes a real spawned background process computing one complete five-frame task, service/owner/filesystem delivery, composed control-consumer frozen filtering, takeover/late-data handling, dense sampling, explicit recalibration and lifecycle timing. In-process transport remains substituted in service tests; this does not establish real ROS/lidar or Linux socket correctness. The previously reported stale fixture failures in this set are now repaired, not excluded from the final command.

All 21 changed Python files pass Ruff lint, Ruff format check and `git diff --check`. Diff review checked source and clock boundaries, one-job delivery, stale-result identity, bounded sampling and temporal endpoints, stop/takeover ordering, retired wire fields, frozen-result compatibility and test assertions. Private IPC schema changes require a coordinated process restart; existing disk stream fields are accepted only for old-record decoding and no longer drive the new code.
