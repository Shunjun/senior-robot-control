# Scan matcher performance implementation plan

**Goal:** Optimize exact residual nearest-neighbor search and reuse the fixed-anchor correlation grid without changing matching or qualification behavior.

**Execution:** Direct implementation in this session, as requested. Branch `feat/scan-matcher-performance`, based on `main` at `deac988`. No Orca workers.

## Design

- Use a balanced two-dimensional KD-tree over valid reference endpoints. Search exact Euclidean distance with the same `std::hypot` calculation as the baseline. Do not downsample, approximate distances, or change the residual/inlier thresholds.
- Retain the existing reference point order for grid rasterization. Rebuild the index and correlation grid lazily together when an anchor changes; reuse them otherwise. Clear/reanchor/geometry discontinuity must invalidate both.
- Keep both Karto correlation searches and their parameters unchanged. This change removes repeated reference preparation, not matching candidates.
- Alternatives: squared-distance brute force still scales quadratically; approximate lookup or downsampling can change acceptance. A KD-tree avoids those tradeoffs without adding a dependency.

## Tasks (sequential, because both optimizations share reference lifetime)

- [x] Implement private `detail::PointIndex` in `ros2/src/senior_scan_matcher/src/point_index.hpp` and exact-oracle standalone tests.
- [x] Integrate lazy reference preparation in `scan_matcher.cpp`; add cache/reinitialization checks to existing matcher tests.
- [x] Add a reproducible dependency-free C++ residual benchmark. Use identical input, compiler flags and repeated runs, checking numerical sums/inlier counts. Reuse the existing `scan_matcher_replay` full-update timing interface for later ROS-environment measurement; no new full-matcher harness is needed after the user deferred full measurement.
- [x] Review actual diff; run available native checks and existing relevant tests; record measurements and unavailable validation honestly before committing.

## Selected test scenarios

The local optimization uses five new scenarios (below the medium-change limit of ten):

1. Deterministic radar-like point cloud and query sequence: every nearest distance and accumulated residual/inliers equal the brute-force oracle, including cutoff boundaries.
2. Collinear and duplicate points: exact nearest distance remains equal to brute force.
3. Index reset to another/empty cloud: no stale reference point remains.
4. Reused reference grid: raw match metrics equal a fresh matcher built from the same anchor and query. Existing `AccumulatesSubcellTranslationAndYawAcrossManyScans` and `LongStationaryAnchorThenSubcellMovement` also remain relevant.
5. Explicit reset after a populated cache: matching in a different room-relative reference agrees with a fresh matcher. Existing `ReanchorKeepsMeasuredContinuity`, turning, invalid/masked beam, rollback and geometry regressions remain relevant.

No real bug has been claimed or fixed; these are optimization equivalence checks. No speculative fault tests or timing assertions in regression tests.

## Validation

Native kernel: compile the standalone test and benchmark with C++17 and `-O3 -DNDEBUG`, then run the test with ASan/UBSan separately. Full ROS/Karto (deferred): build `senior_scan_matcher` and run its existing and new CTest cases. Build the existing replay executable on baseline and optimized versions; compare deterministic result output before comparing timing.

Current environment limitations: macOS arm64, clang available; ROS/Karto and CMake absent, Docker socket access denied and shell DNS unavailable. Full-match performance cannot be inferred from kernel performance. Do not install substitute/stub matching algorithms to claim full-chain validation.
