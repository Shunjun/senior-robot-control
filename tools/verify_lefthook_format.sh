#!/bin/sh
# Five real commits in one temporary linked worktree. The hook is installed
# only inside that fixture. Outcomes are Git state and Ruff's own output.
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BASE=$(mktemp -d "${TMPDIR:-/tmp}/lefthook-fixture.XXXXXX")
PRIMARY="$BASE/primary"
LINKED="$BASE/linked"
LOG="$BASE/log"
mkdir -p "$LOG"

unset GIT_DIR GIT_WORK_TREE GIT_INDEX_FILE LEFTHOOK

printf 'fixture %s\n' "$BASE"

die() {
  printf 'FAIL: %s\n' "$1" >&2
  exit 1
}

write_bytes() {
  mkdir -p "$(dirname -- "$1")"
  cat >"$1"
}

sha256() {
  shasum -a 256 "$1" | awk '{print $1}'
}

snapshot_real_hooks() {
  dest=$1
  : >"$dest"
  find "$REAL_HOOKS" -maxdepth 1 -type f -print | sort | while read -r file; do
    shasum -a 256 "$file" >>"$dest"
  done
}

format_copy() {
  src=$1
  dest=$2
  cp "$src" "$dest"
  (
    cd "$LINKED"
    uv run --no-sync ruff check --select I --fix --config "$LINKED/pyproject.toml" -- "$dest" &&
      uv run --no-sync ruff format --config "$LINKED/pyproject.toml" -- "$dest"
  ) >"$LOG/ruff-oracle.out" 2>&1 || die "ruff oracle failed for $src"
}

names_in_head() {
  git -C "$LINKED" -c core.quotepath=false diff-tree --no-commit-id --name-only -r HEAD | sort
}

canon() {
  python3 -c 'import pathlib, sys; print(pathlib.Path(sys.argv[1]).resolve())' "$1"
}

REAL_HOOKS=$(canon "$(git -C "$ROOT" rev-parse --path-format=absolute --git-path hooks)")
snapshot_real_hooks "$LOG/real-hooks-before.txt"
BEFORE_HOOKS_PATH=$(git -C "$ROOT" config --local --get core.hooksPath || true)

git init -b main "$PRIMARY" >"$LOG/git-init.out" 2>&1
# Override inherited hooksPath inside the disposable repository before any commit.
git -C "$PRIMARY" config core.hooksPath "$PRIMARY/.git/hooks"
git -C "$PRIMARY" config user.name "Lefthook Fixture"
git -C "$PRIMARY" config user.email "lefthook-fixture@example.invalid"
git -C "$PRIMARY" config commit.gpgsign false
git -C "$PRIMARY" config core.quotepath false

cp "$ROOT/pyproject.toml" "$ROOT/uv.lock" "$ROOT/lefthook.yml" "$ROOT/.gitignore" "$PRIMARY/"
write_bytes "$PRIMARY/tracked.py" <<'EOF'
def alpha() -> int:
    return 1


def omega() -> int:
    return 1
EOF
write_bytes "$PRIMARY/B.py" <<'EOF'
value = 1
EOF
git -C "$PRIMARY" add -- pyproject.toml uv.lock lefthook.yml .gitignore tracked.py B.py
git -C "$PRIMARY" commit -m "fixture base" >"$LOG/base-commit.out" 2>&1

git -C "$PRIMARY" worktree add -b fixture "$LINKED" HEAD >"$LOG/worktree-add.out" 2>&1
GIT_DIR=$(git -C "$LINKED" rev-parse --absolute-git-dir)
COMMON_DIR=$(git -C "$LINKED" rev-parse --path-format=absolute --git-common-dir)
[ "$GIT_DIR" != "$COMMON_DIR" ] || die "expected a linked worktree"
printf 'git-dir %s\ncommon-dir %s\n' "$GIT_DIR" "$COMMON_DIR"

(
  cd "$LINKED"
  uv sync --frozen --only-group dev
) >"$LOG/uv-sync.out" 2>&1 || {
  tail -n 40 "$LOG/uv-sync.out" >&2
  die "uv sync failed"
}
(
  cd "$LINKED"
  uv run --no-sync lefthook validate
  uv run --no-sync lefthook install --force
) >"$LOG/install.out" 2>&1 || {
  cat "$LOG/install.out" >&2
  die "lefthook install failed"
}

HOOKS=$(canon "$(git -C "$LINKED" rev-parse --path-format=absolute --git-path hooks)")
COMMON_HOOKS=$(canon "$COMMON_DIR")/hooks
BASE_CANON=$(canon "$BASE")
case "$HOOKS" in
  "$BASE_CANON"/*) ;;
  *) die "hook directory escaped the fixture: $HOOKS" ;;
esac
[ "$HOOKS" = "$COMMON_HOOKS" ] || die "linked worktree hooks are not the common Git directory: $HOOKS"
[ "$HOOKS" != "$REAL_HOOKS" ] || die "fixture reused the user hook directory"
[ -f "$HOOKS/pre-commit" ] || die "pre-commit was not installed in the fixture"
printf 'fixture-hooks %s\n' "$HOOKS"
{
  uv --version
  printf 'lefthook %s\n' "$(cd "$LINKED" && uv run --no-sync lefthook version)"
  (cd "$LINKED" && uv run --no-sync ruff version)
  git --version
} | tee "$LOG/versions.txt"

# 1. Fully staged root, nested, and spaced Unicode Python.
write_bytes "$LINKED/root_mod.py" <<'EOF'
import sys
import os

value=1+2
EOF
write_bytes "$LINKED/pkg/nested_mod.py" <<'EOF'
value=3+4
EOF
write_bytes "$LINKED/中文 目录/示例 模块.py" <<'EOF'
value=5+6
EOF
cp "$LINKED/root_mod.py" "$LOG/orig-root.py"
cp "$LINKED/pkg/nested_mod.py" "$LOG/orig-nested.py"
cp "$LINKED/中文 目录/示例 模块.py" "$LOG/orig-unicode.py"
git -C "$LINKED" add -- root_mod.py pkg/nested_mod.py "中文 目录/示例 模块.py"
if ! git -C "$LINKED" commit -m "format staged python" >"$LOG/scenario-1.out" 2>&1; then
  cat "$LOG/scenario-1.out" >&2
  die "scenario 1 commit failed"
fi
format_copy "$LOG/orig-root.py" "$LOG/expect-root.py"
format_copy "$LOG/orig-nested.py" "$LOG/expect-nested.py"
format_copy "$LOG/orig-unicode.py" "$LOG/expect-unicode.py"
cmp -s "$LOG/orig-root.py" "$LOG/expect-root.py" && die "scenario 1 root input was already formatted"
git -C "$LINKED" show "HEAD:root_mod.py" >"$LOG/head-root.py"
git -C "$LINKED" show "HEAD:pkg/nested_mod.py" >"$LOG/head-nested.py"
git -C "$LINKED" show "HEAD:中文 目录/示例 模块.py" >"$LOG/head-unicode.py"
cmp -s "$LOG/head-root.py" "$LOG/expect-root.py" || die "scenario 1 root HEAD is not ruff output"
cmp -s "$LOG/head-nested.py" "$LOG/expect-nested.py" || die "scenario 1 nested HEAD is not ruff output"
cmp -s "$LOG/head-unicode.py" "$LOG/expect-unicode.py" || die "scenario 1 unicode HEAD is not ruff output"
got_names=$(names_in_head)
expect_names=$(printf '%s\n' pkg/nested_mod.py root_mod.py "中文 目录/示例 模块.py" | sort)
[ "$got_names" = "$expect_names" ] || die "scenario 1 commit files: $got_names"
[ -z "$(git -C "$LINKED" status --porcelain)" ] || die "scenario 1 worktree is dirty: $(git -C "$LINKED" status --porcelain)"
printf 'scenario 1 ok\n'

# 2. Stage only a non-Python file. Leave an unformatted Python file unstaged.
write_bytes "$LINKED/notes.txt" <<'EOF'
plain note
EOF
write_bytes "$LINKED/root_mod.py" <<'EOF'
value=7+8
EOF
dirty_before=$(sha256 "$LINKED/root_mod.py")
git -C "$LINKED" add -- notes.txt
if ! git -C "$LINKED" commit -m "note only" >"$LOG/scenario-2.out" 2>&1; then
  cat "$LOG/scenario-2.out" >&2
  die "scenario 2 commit failed"
fi
[ "$(sha256 "$LINKED/root_mod.py")" = "$dirty_before" ] || die "scenario 2 changed root_mod.py"
[ "$(git -C "$LINKED" status --porcelain -- root_mod.py)" = " M root_mod.py" ] || die "scenario 2 changed the unstaged state of root_mod.py"
[ "$(names_in_head)" = "notes.txt" ] || die "scenario 2 committed extra files"
grep -q "ruff-format (skip) no files for inspection" "$LOG/scenario-2.out" || die "scenario 2 did not report the formatter skipped"
printf 'scenario 2 ok\n'

# 3. Stage one formatted-needed edit. Keep a distant edit, B.py, and C.py unstaged.
write_bytes "$LINKED/tracked.py" <<'EOF'
def alpha() -> int:
    value=1+2
    return value


def omega() -> int:
    return 1
EOF
git -C "$LINKED" add -- tracked.py
python3 - "$LINKED/tracked.py" <<'PY'
import pathlib, sys
path = pathlib.Path(sys.argv[1])
text = path.read_text()
old = "def omega() -> int:\n    return 1\n"
new = "def omega() -> int:\n    return 2\n"
if old not in text:
    raise SystemExit("omega anchor missing before commit")
path.write_text(text.replace(old, new, 1))
PY
write_bytes "$LINKED/B.py" <<'EOF'
value=9+10
EOF
write_bytes "$LINKED/C.py" <<'EOF'
value=11+12
EOF
b_before=$(sha256 "$LINKED/B.py")
c_before=$(sha256 "$LINKED/C.py")
git -C "$LINKED" show :tracked.py >"$LOG/staged-tracked.py"
if ! git -C "$LINKED" commit -m "partial tracked" >"$LOG/scenario-3.out" 2>&1; then
  cat "$LOG/scenario-3.out" >&2
  die "scenario 3 commit failed"
fi
format_copy "$LOG/staged-tracked.py" "$LOG/expect-tracked.py"
python3 - "$LOG/expect-tracked.py" "$LOG/expect-tracked-worktree.py" <<'PY'
import pathlib, sys
formatted = pathlib.Path(sys.argv[1]).read_text()
old = "def omega() -> int:\n    return 1\n"
new = "def omega() -> int:\n    return 2\n"
if old not in formatted:
    raise SystemExit("formatted staged file lost the omega anchor")
pathlib.Path(sys.argv[2]).write_text(formatted.replace(old, new, 1))
PY
git -C "$LINKED" show "HEAD:tracked.py" >"$LOG/head-tracked.py"
cmp -s "$LOG/head-tracked.py" "$LOG/expect-tracked.py" || die "scenario 3 HEAD is not the formatted staged file"
cmp -s "$LINKED/tracked.py" "$LOG/expect-tracked-worktree.py" || die "scenario 3 worktree lost the unstaged edit or the formatting"
grep -q "return 2" "$LOG/head-tracked.py" && die "scenario 3 committed the unstaged business edit"
[ "$(names_in_head)" = "tracked.py" ] || die "scenario 3 committed extra files"
[ "$(sha256 "$LINKED/B.py")" = "$b_before" ] || die "scenario 3 changed B.py"
[ "$(sha256 "$LINKED/C.py")" = "$c_before" ] || die "scenario 3 changed C.py"
[ "$(git -C "$LINKED" status --porcelain -- B.py)" = " M B.py" ] || die "scenario 3 B.py was staged or restored"
[ "$(git -C "$LINKED" status --porcelain -- C.py)" = "?? C.py" ] || die "scenario 3 C.py was tracked"
[ -z "$(git -C "$LINKED" diff --cached --name-only)" ] || die "scenario 3 left staged changes"
printf 'scenario 3 ok\n'

# 4. New Python file staged, then appended (AM).
write_bytes "$LINKED/added.py" <<'EOF'
def staged() -> int:
    value=1+2
    return value


def anchor() -> int:
    return 1
EOF
git -C "$LINKED" add -- added.py
printf '\ndef appended() -> int:\n    value=3+4\n    return value\n' >>"$LINKED/added.py"
[ "$(git -C "$LINKED" status --porcelain -- added.py)" = "AM added.py" ] || die "scenario 4 was not AM before commit"
git -C "$LINKED" show :added.py >"$LOG/staged-added.py"
python3 - "$LOG/staged-added.py" "$LINKED/added.py" "$LOG/added-suffix.py" <<'PY'
import pathlib, sys
staged = pathlib.Path(sys.argv[1]).read_bytes()
work = pathlib.Path(sys.argv[2]).read_bytes()
if not work.startswith(staged):
    raise SystemExit("append is not a suffix of the staged file")
pathlib.Path(sys.argv[3]).write_bytes(work[len(staged):])
PY
if ! git -C "$LINKED" commit -m "added python" >"$LOG/scenario-4.out" 2>&1; then
  cat "$LOG/scenario-4.out" >&2
  die "scenario 4 commit failed"
fi
format_copy "$LOG/staged-added.py" "$LOG/expect-added.py"
python3 - "$LOG/expect-added.py" "$LOG/added-suffix.py" "$LOG/expect-added-worktree.py" <<'PY'
import pathlib, sys
pathlib.Path(sys.argv[3]).write_bytes(pathlib.Path(sys.argv[1]).read_bytes() + pathlib.Path(sys.argv[2]).read_bytes())
PY
git -C "$LINKED" show "HEAD:added.py" >"$LOG/head-added.py"
cmp -s "$LOG/head-added.py" "$LOG/expect-added.py" || die "scenario 4 HEAD is not the formatted staged version"
cmp -s "$LINKED/added.py" "$LOG/expect-added-worktree.py" || die "scenario 4 append was not preserved unstaged"
grep -q "def appended" "$LOG/head-added.py" && die "scenario 4 committed the append"
[ "$(names_in_head)" = "added.py" ] || die "scenario 4 committed extra files"
[ "$(git -C "$LINKED" status --porcelain -- added.py)" = " M added.py" ] || die "scenario 4 append is not an unstaged modification"
printf 'scenario 4 ok\n'

# 5. Staged syntax error fails the commit and keeps the file.
head_before=$(git -C "$LINKED" rev-parse HEAD)
write_bytes "$LINKED/broken.py" <<'EOF'
def broken(:
    return 1
EOF
broken_before=$(sha256 "$LINKED/broken.py")
git -C "$LINKED" add -- broken.py
if git -C "$LINKED" commit -m "broken python" >"$LOG/scenario-5.out" 2>&1; then
  die "scenario 5 commit succeeded"
fi
[ "$(git -C "$LINKED" rev-parse HEAD)" = "$head_before" ] || die "scenario 5 moved HEAD"
[ "$(sha256 "$LINKED/broken.py")" = "$broken_before" ] || die "scenario 5 changed broken.py"
git -C "$LINKED" show :broken.py >"$LOG/staged-broken.py"
cmp -s "$LINKED/broken.py" "$LOG/staged-broken.py" || die "scenario 5 changed the staged content"
grep -Eq "Failed to parse|invalid-syntax" "$LOG/scenario-5.out" || die "scenario 5 error is not readable"
grep -q "broken.py" "$LOG/scenario-5.out" || die "scenario 5 error does not name the file"
[ "$(git -C "$LINKED" status --porcelain -- broken.py)" = "A  broken.py" ] || die "scenario 5 did not keep the staged file"
printf 'scenario 5 ok\n'

snapshot_real_hooks "$LOG/real-hooks-after.txt"
cmp -s "$LOG/real-hooks-before.txt" "$LOG/real-hooks-after.txt" || die "real hook directory changed"
AFTER_HOOKS_PATH=$(git -C "$ROOT" config --local --get core.hooksPath || true)
[ "$BEFORE_HOOKS_PATH" = "$AFTER_HOOKS_PATH" ] || die "real core.hooksPath changed"
printf 'real hooks unchanged\n'
printf 'OK %s\n' "$BASE"
