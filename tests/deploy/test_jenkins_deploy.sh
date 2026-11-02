#!/usr/bin/env bash
set -eu
root=$(cd "$(dirname "$0")/../.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/bin"
cat > "$tmp/bin/docker" <<'MOCK'
#!/usr/bin/env bash
if [[ "$1 $2 $3" == 'context inspect --format' ]]; then echo 'tcp://other-host:2375'; exit 0; fi
exit 0
MOCK
chmod +x "$tmp/bin/docker"
printf "SENIOR_SECRET='%s'\nADMIN_PASSWORD_HASH='scrypt\$16384\$8\$1\$%s\$%s'\n" "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" "abababababababababababababababab" "cdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcd" > "$tmp/env"
if PATH="$tmp/bin:$PATH" DEPLOY_ROOT="$tmp/deploy" RELEASE_ID=0123456789abcdef0123456789abcdef01234567-1 ENV_FILE="$tmp/env" ACME_EMAIL=ops@example.com bash "$root/deploy/jenkins-deploy.sh" >"$tmp/out" 2>&1; then
  echo 'remote Docker daemon was accepted' >&2; exit 1
fi
if ! grep -qi 'local docker' "$tmp/out"; then cat "$tmp/out"; exit 1; fi
cat > "$tmp/bin/docker" <<'MOCK'
#!/usr/bin/env bash
if [[ "$1 $2 $3" == 'context inspect --format' ]]; then echo 'unix:///var/run/docker.sock'; fi
exit 0
MOCK
cat > "$tmp/bin/curl" <<'MOCK'
#!/usr/bin/env bash
case "${*: -1}" in
  */api/robots) printf '%s' "${MOCK_API:-401}" ;;
  *) echo '<!doctype html><html></html>' ;;
esac
MOCK
cat > "$tmp/bin/sleep" <<'MOCK'
#!/usr/bin/env bash
exit 0
MOCK
cat > "$tmp/bin/flock" <<'MOCK'
#!/usr/bin/env bash
exit 0
MOCK
cat > "$tmp/bin/mv" <<'MOCK'
#!/usr/bin/env bash
python3 - "$2" "$3" <<'PYMOVE'
import os
import sys
os.replace(sys.argv[1], sys.argv[2])
PYMOVE
MOCK
chmod +x "$tmp/bin/"*
export PATH="$tmp/bin:$PATH" DEPLOY_ROOT="$tmp/deploy" ENV_FILE="$tmp/env" ACME_EMAIL=ops@example.com
export RELEASE_ID=0123456789abcdef0123456789abcdef01234567-1
cd "$root"
bash deploy/jenkins-deploy.sh >/dev/null
expected="$DEPLOY_ROOT/releases/$RELEASE_ID"
[[ $(readlink "$DEPLOY_ROOT/current") == "$expected" ]] || { echo 'Healthy release was not activated' >&2; exit 1; }
export RELEASE_ID=0123456789abcdef0123456789abcdef01234567-2 MOCK_API=200
if bash deploy/jenkins-deploy.sh >"$tmp/out" 2>&1; then echo 'SPA fallback accepted as API health' >&2; exit 1; fi
[[ $(readlink "$DEPLOY_ROOT/current") == "$expected" ]] || { echo 'Failed release changed current' >&2; exit 1; }
