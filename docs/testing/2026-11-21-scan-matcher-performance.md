# Scan matcher performance change

Branch: `feat/scan-matcher-performance`, based on `main` at `deac988`.
Code version: the commit containing this report (all eight changed files are committed together).

## Behavior and review

The residual calculation uses a private balanced 2D KD-tree instead of testing every query against every reference point. Nearest distance still uses `std::hypot`; inlier comparison, accumulation order, thresholds, diagnostics and acceptance remain unchanged. The index owns a copy of the reference endpoints, so rearranging its medians cannot change grid rasterization order or create dangling references.

The correlation grid and index are prepared lazily once per anchor. `set_reference` invalidates preparation at initialization/reanchor, and `clear` also clears the index and invalidates preparation. Geometry/time discontinuity and failed qualification already call `clear`. Both Karto correlation calls and all their search parameters are unchanged. There is no downsampling or approximate distance lookup.

Diff review covered median/branch bounds, equal split coordinates, reference ownership, grid fill ordering, invalidation paths and original qualification calculations. Full Karto integration remains unverified locally, as stated below.

## Measured residual kernel performance

Environment: macOS arm64; Apple clang 21.0.0 (`clang-2100.0.123.102`), target `arm64-apple-darwin25.4.0`. Build flags `-std=c++17 -O3 -DNDEBUG -Wall -Wextra -Werror`; no fast-math.

Synthetic asymmetric-room anchor endpoints, with twelve precomputed translated/rotated query frames per anchor. Both methods consume exactly the same points. Seven repetitions after warmup; timing order alternates. Results below are medians. Indexed cost includes one index build amortized over those twelve frames. It does not include coordinate transformation, Karto matching or grid preparation. Timing is descriptive, not a test assertion.

| Points/frame | Brute force ms/frame | Index query ms/frame | Index build ms/anchor | Index incl. build ms/frame | Speedup incl. build |
|---|---:|---:|---:|---:|---:|
| 720 | 0.980958 | 0.138069 | 0.030958 | 0.140691 | 6.97x |
| 1440 | 3.960212 | 0.400931 | 0.052959 | 0.405184 | 9.77x |
| 2048 | 7.888035 | 0.669962 | 0.074125 | 0.676139 | 11.67x |

Squared-distance sums and inlier counts are exactly equal between both algorithms in every measured repetition. The independent test also compares every nearest distance exactly against brute force. These numbers are **not full-frame speedups** and do not quantify the grid-cache benefit. Grid caching removes repeated clear/rasterize/smear work when the reference stays fixed; both correlation searches still execute per matched frame.

Reproduce from repository root:

```sh
clang++ -std=c++17 -O3 -DNDEBUG -Wall -Wextra -Werror \
  -Iros2/src/senior_scan_matcher/src \
  tools/scan_matcher_residual_benchmark.cpp -o /tmp/senior-scan-residual-benchmark
/tmp/senior-scan-residual-benchmark
```

## Verification evidence

Selected new/rewritten scenarios: **5 new, 0 rewritten**. No actual bug fix or bug-specific regression is claimed.

| Scenario | Assertion | Local result |
|---|---|---|
| Radar-like point cloud, transformed queries and residual cutoff | Every nearest distance, sum and inlier count equal brute force | PASS |
| Collinear/duplicate endpoints | Exact nearest distance equals brute force | PASS |
| Reference replacement/clear/empty | Old reference points cannot remain queryable | PASS |
| Cached versus freshly prepared reference | Match response, residual, inliers, pose and covariance agree | Added; requires Karto, not run |
| Reset after cache preparation | New reference produces same result as a fresh matcher | Added; requires Karto, not run |

Native standalone test command (passed with Release checks active):

```sh
clang++ -std=c++17 -O3 -DNDEBUG -Wall -Wextra -Werror \
  -Iros2/src/senior_scan_matcher/src \
  ros2/src/senior_scan_matcher/test/test_point_index.cpp -o /tmp/senior-test-point-index
/tmp/senior-test-point-index
```

The same three scenarios passed with AddressSanitizer and UndefinedBehaviorSanitizer, using `-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer` instead of `-O3 -DNDEBUG`.

Reused regressions:

```sh
.venv/bin/pytest -q tests/unit/test_scan_matcher_replay.py tests/ros/test_scan_matcher_process_shutdown.py
```

Result: **3 passed, 4 skipped**. All skips are existing process-level ROS checks; `rclpy` is absent. The replay tests check input preparation/identity, not native matching.

An initial broader command also included `tests/unit/test_scan_geometry_scope.py`, which failed during collection: that unchanged test imports `ScanGeometry`/`prepare_scan_geometry` from an unchanged compatibility module that no longer exports them. `git diff main --` for both files was empty, and `git show main:ros2/src/senior_robot/senior_robot/motion_core_perception.py` confirms the exports are already absent on the baseline. This pre-existing failure is reported, not fixed or hidden in this optimization.

## Deferred full integration and timing

Local ROS/Karto/CMake are unavailable and Docker socket access is denied. No native matcher build, full matcher C++ test suite, ROS chain or grid-cache timing was run. The user explicitly deferred full measurement and requested the code change first. No real-recording comparison is claimed.

In a configured ROS Humble workspace, build/run the package tests:

```sh
colcon build --packages-select senior_scan_matcher --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select senior_scan_matcher --event-handlers console_direct+
colcon test-result --verbose
```

For later before/after measurements, build baseline `deac988` and this branch in separate workspaces with identical compiler/dependencies/Release flags. Run each existing `scan_matcher_replay` executable on the same `tools.scan_matcher_replay` prepared input. Compare all output fields except `seconds` before comparing medians/p95 of matched-frame `seconds`; preserve initialization and reanchor costs rather than silently discarding them. Keep this change unmerged until full matcher tests and equivalence checks pass.
