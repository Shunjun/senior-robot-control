# Control Standard Typography and Logout Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans; implement inline without helper workers as explicitly authorized.

**Goal:** Use standard Tailwind typography throughout web and expose the existing logout action beside settings return.

**Architecture:** Keep local static utility variants and the existing App logout handler. Pass optional logout props through Shell → RobotPanel → VehicleTools → VehicleSettings; reuse VehicleSettings in the existing scene for empty selection.

**Tech Stack:** React, TypeScript, Tailwind v4, Vitest, Playwright.

**Spec:** Dispatched task task_6ea176174a69 and `/tmp/control-ui-standard-typography-logout-audit-2026-11-25.md`.

## Global Constraints

Keep branch feat/control-workspace, a follow-up Draft PR after merged PR41 (coordinator ruling msg_aeefd2dd31d0), fixture data and static LAN preview at port 4149. No backend, navigation URL, dependencies or chart helpers. Minimum text-xs; no arbitrary font sizes or root-size override. Preserve settings category revisions and action alignment, existing auth/CSRF/pending/release behavior.

## Review Focus

Small descendants and SVG labels must resolve to at least standard xs. Empty/loading/unknown/revoked selection must reach settings. Pairing overlays must close before settings interaction. Failed logout retains session, 401 expires it, pending rejects duplicate actions. Larger text must remain reachable at all three existing viewport sizes.

### Task 1: Standard typography

**Files:** Existing 28 TSX typography consumers and web/src/theme.css; badge in web/e2e/fixtures/control-fixture.js.
**Interfaces:** Standard Tailwind xs/sm/base/lg/xl/2xl/3xl/4xl classes; browser root scaling preserved.
- [x] Replace arbitrary size classes by specified semantic mapping; use body sm, h1 2xl, h2 xl, h3 lg, small xs base.
- [x] Read every changed style and adapt only necessary local wrapping/sizing.
- [x] Static scan src and inspect computed sizes in real expanded browser views.

### Task 2: Logout wiring and empty settings

**Files:** App.tsx, RobotPanel.tsx, VehicleTools.tsx, VehicleSettings.tsx; devices.test.tsx.
**Interfaces:** Optional `onLogout?: () => void`, `loggingOut?: boolean` preserve standalone mounts; Shell supplies existing required props.
- [x] Migrate existing logout tests to real settings entry, closing pairing first; confirm failing behavior before implementation.
- [x] Pass existing action and busy state through named props interfaces; add shared Button with LogOut beside return; remove footer logout.
- [x] Offer settings in empty Shell with existing scene dimensions and enabled=false; return calls workspace.setPage(control).
- [x] Run existing devices/settings/drive regressions and inspect errors.

### Task 3: Existing regressions and three-size verification

**Files:** web/e2e/control.spec.ts, shared fixture, delivery report in /tmp (outside git).
**Interfaces:** DELETE /api/session fixture responds 204; calls remain recorded; production API unchanged.
- [x] Update original measurement assertions to browser-resolved standard tokens; add one real header logout at end of each existing continuous viewport run.
- [x] Run frontend unit suite once, typecheck/build, and original 1440×1000, 390×844, 844×390 e2e against updated production bundle.
- [x] Inspect new screenshots and computed font/bounds evidence, retain original/failure evidence; report actual changes and real-service limits.
- [x] Commit and push original branch, create the authorized follow-up Draft PR to main, retain preview carrier.

Execution notes: initial settings-entry tests failed before wiring; final frontend suite 221 passed, targeted post-review suite 101 passed. Typography probes cover small/SVG and browser root scaling. Local wrapping reserves room for miniature-map scale and drive readings; mobile manual feedback and settings recovery notices no longer obscure readings or logout. The existing logout regression now includes success, failure and 401 through real settings, and pending logout rejects late list data. Author performed the diff and screenshot review without helper workers, as requested.
