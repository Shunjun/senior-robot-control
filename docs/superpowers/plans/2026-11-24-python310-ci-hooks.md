# Python 3.10 CI and commit hooks implementation plan

> Implement inline with executing-plans; user authorized direct Codex execution.

**Goal:** Check only deployed Python 3.10, repair CI failures, sort staged Python imports before formatting, and document one-command development initialization.

**Architecture:** Keep Ruff and Lefthook as the existing owners. The hook fixes import order then formats the staged version with stage_fixed; development setup installs locked dependencies and the local hook. Production entry points do not install development hooks. Keep strict CI checks, tests and wheel-install verification.

**Spec:** User instructions in this chat (2026-11-24), existing lefthook.yml and README development setup.

## Constraints and selected verification

- Worktree fix/python310-ci from merged main af03f088; original feature branch remains unchanged.
- Do not install a real shared hook during fixture validation. No services or Docker.
- One substantially rewritten functional scenario: the existing fully staged root Python case includes unsorted imports and unformatted code; successful commit must contain sorted/formatted content. Reuse nested/Unicode, non-Python skip, partial staging, added-file append and syntax-error refusal protections without new scenarios.
- Bootstrap is a small shell entry point; verify syntax and its existing dependency/hook commands through the real isolated fixture. Keep worktree support and filenames with spaces.
- Baseline GitHub run 45313762004 fails on two I001 import blocks. Fresh Python 3.10 additionally exposes missing PEP 561 package metadata and real annotation errors; no blanket ignore or removal of CI steps.
- Scope initially 9–10 paths, expanded to approximately 20–25 after full type-check evidence; report further material expansion before edits.

## Tasks

- [x] Reproduce sorted-import requirement with the existing real-commit fixture before changing the hook.
- [x] Restrict Actions to 3.10 and fix existing import/format failures.
- [x] Supply missing type metadata and repair strict typing errors without weakening the gate.
- [x] Extend the hook with ordered import-fix and format commands; add tools/setup-dev.sh and update README.
- [x] Run real hook fixture, full CI checks on Python 3.10, wheel installation checks, review diff, commit and create a fix PR.

## Evidence

Initial local type-check reports were invalidated by writes denied to caches under the app-managed worktree. Repeated with caches in /private/tmp: 92 reports in 56 files. Most are repeated imports of the unmarked shared contracts package. App-managed worktree file edits use explicitly approved commands; diagnostic caches remain temporary.


## Implementation and final local evidence

The real staged-import case failed with the original format-only hook (`/private/tmp/ci310-hook-red.log`), then all five existing commit workflows passed with the ordered import-fix/format pipeline (`/private/tmp/ci310-hook-green.log`). Only the root-file positive input gained unsorted imports; nested/Unicode paths, non-Python skip, partial staging, added-file append and parse-error refusal assertions remain. The fixture installs only the dev dependency group, since its disposable repository intentionally lacks the production workspace packages. Real shared hooks and hooksPath hashes are unchanged.

Fresh locked CPython 3.10.22 with uv 0.12.23 initially reported 92 mypy errors; a shared contracts PEP 561 marker removed repeated untyped-import propagation. Robot/motion-control markers expose their API types. Source lookup includes the colcon-owned sensor-ingress package; only the external optional ROS Launch SDK's absent stubs use an explicit mypy boundary. `strict = true` and all 152 existing type-check targets remain. Fixed terminal-phase variable narrowing, aggregate-row invariant, process ancestry/status types, snapshot protocol/version/grid fields, learning envelope annotation and explicit compatibility facade exports.

Ruff continues checking all product Python and tracked design docs. Generated `.superpowers` plan workspaces are excluded from source formatting, preserving another plan's workspace instead of editing it. Fixed the two launch import blocks and existing formatting issues. No blanket suppression of first-party typing or removal of runtime tests.

New tools/setup-dev.sh selects Python 3.10, syncs locked dev dependencies, then installs the local Lefthook. README documents first-clone initialization, shared worktree hooks, and local-only generated hook files. The real common hook is deliberately not activated by verification.

Local workflow replay: `/private/tmp/ci310-full-checks.py` runs the exact workflow run commands on 3.10, with disposable install directories relocated under `/private/tmp/ci310-full-9037krp3`. Steps 0–17 passed (Ruff check, format check, strict mypy; 2430 passed/17 existing platform/feature skips in 191.02s; all 11 wheels; base/safety installation). Step 18 then exposed an outdated assertion: nine independent clients were still expected as eight 1s plus one 3s. The approved dedicated mapping lane now has 30s, so the verifier was updated to seven 1s, one 3s and one 30s; uniqueness, shared ownership, reporting and shutdown checks remain. This is a second substantially rewritten existing verification scenario, directly tied to the reproduced failure. `/private/tmp/ci310-resume-patch.py` reran affected static checks and steps 18–20; manager base/agent wheels, cloud lifespan and frozen cloud closure all passed. Earlier passed business tests were not repeated after only a verifier expectation/comment change. Both original failure and resumed reports are preserved.

Selected new/substantially rewritten functional coverage totals two scenarios (staged root import sorting and installed Agent deadline verification); all other valid regressions are reused. No new skip/xfail. Shell syntax checks pass; all three typed markers are present in built wheels. The reviewer inspected the actual diff, invariant guards, imports/exports, hook staging behavior and CI run list. Final scope: 12 product paths, four verification/test paths, six configuration/build/type-marker paths, four tool/document paths, 26 total. Plans are committed with implementation. Remote Linux Actions remains the final CI confirmation after PR creation.


## Remote Linux follow-up

PR #39 at fba45c3 reached pytest on both remote runs. Push run 45322319362 failed on two short-admission timing artifact paths and a dedicated-child helper import; PR run 45322328207 additionally reproduced the corrected-map publication race (manifest read-only at cut 4). Full logs are retained in /private/tmp/ci39-*-clean.log. These are test fixture defects, not grounds to relax production navigation qualification.

Scope was reported before expansion: three more existing test files, 29 total paths. Short-admission artifacts now use pytest tmp_path and the peer explicitly reports the absent SLAM capability; all timing/admission assertions remain. Linux parent-death subprocesses use current repository/test module paths instead of retired src paths, retaining real death/reaping assertions. The corrected-map fixture now models a 20ms raster RPC; wait for its qualified published view and corrected yaw instead of treating the receipt counter as a publication barrier. Original map, Home, generation and navigation assertions remain. No timeout increase, skip, xfail, or production permission change. These corrections update existing failing scenarios rather than adding a parameter matrix.


The 20ms raster RPC deterministically reproduced the original read-only-manifest failure on macOS (ci310-corrected-publication-red.log). After waiting for the coherent published corrected view, the three affected files yield 26 passed/1 Linux-only skip in 15.60s (ci310-linux-fixtures-green.log). The dedicated-process helper also imports a sibling fixture from senior_control/tests; that path is now explicitly inherited by both isolated subprocess levels. An isolated interpreter import probe passes with these exact paths. Linux parent-death behavior remains protected by the existing unmodified process/signal/reaping assertions and will be verified by remote Actions. Original remote failures are retained; no new cases, skip markers, timeout increases or production changes.
