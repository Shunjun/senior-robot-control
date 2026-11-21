# Sampled startup learning implementation plan

**Goal:** Replace continuous learning computation with one classification over five evenly timed motion-window samples.
**Spec:** `docs/superpowers/specs/2026-11-21-sampled-startup-learning-design.md`.
**Execution:** Direct implementation, continuing `feat/scan-matcher-performance`; no workers or parallel worktrees. Tasks are sequential because service delivery depends on the kernel contract.

## Scope

Product: `learning_service.py` selects five frames after capture completion; `learning_compute.py` accumulates only selected observations then classifies once; `self_learning.py` provides indexed sampled classification and spatially separated connected runs. Preserve existing streaming-model compatibility APIs and their valid identity/filter regressions, but the production compute kernel no longer uses their streaming observe/validate path.

Tests: migrate `test_learning_compute.py` kernel expectations; add `test_sampled_learning.py` for timing selection, independent computation and connected-surface behavior. No transport, UI or motion-policy changes.

## Selected scenarios (5 total: 2 migrated, 3 new; medium-change upper bound 10)

1. Migrate original kernel/raw-identity scenario: five distinct raw frames survive reversible nonfinite encoding; no model before finish; exactly five observations classify once and far disconnected hits remain external.
2. Preserve per-frame actual TF decoding and structural-geometry rejection in the existing TF scenario; update only assertions tied to the retired streaming-model internals.
3. Select five distinct, ordered, evenly timed motion-window frames; exclude pre-motion and post-motion frames; reject insufficient acquisitions rather than duplicating them.
4. Real service/owner/storage with delayed compute: no compute work while capture is open, then exactly five frames compute and persist after closure; motion action facts stay unchanged.
5. Real sampled classifier: keep stable interior-connected exterior equipment, exclude a separate stable wall and a moving disconnected return; frozen view still filters only matched positions.

Existing cancel/order/overflow, single-writer/durable result, re-calibration, clocks and motion lifecycle tests are reused. No actual bug regression is claimed. Old full-stream computation and unlimited-gap connection expectations in production-kernel tests are intentionally superseded, not silently excluded.

## Tasks

- [x] Add one-shot sampled model classification and preserve immutable runtime filtering.
- [x] Select five frames only after capture closure and feed the existing four-frame IPC batches.
- [x] Run the five selected scenarios and relevant existing regressions; inspect actual diff and report any baseline failures separately.
- [x] Prepare implementation and evidence for the same commit; branch push and full ROS/hardware validation are subject to environment availability, not assumed successful.
