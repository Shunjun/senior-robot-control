#!/usr/bin/env bash
set -Eeuo pipefail
set +x
umask 077
: "${DEPLOY_ROOT:?}" "${RELEASE_ID:?}" "${ENV_FILE:?}"
[[ "$DEPLOY_ROOT" =~ ^/[a-zA-Z0-9_./-]+$ && "$DEPLOY_ROOT" != *..* && "$DEPLOY_ROOT" != / ]] || { echo 'Invalid deployment directory' >&2; exit 1; }
[[ "$RELEASE_ID" =~ ^[a-f0-9]{40}-[0-9]+$ ]] || { echo 'Invalid release ID' >&2; exit 1; }
[[ -r "$ENV_FILE" ]] || { echo 'Secret file is not readable' >&2; exit 1; }
settings=$(python3 deploy/validate-jenkins-env.py "$ENV_FILE")
read -r domain enable_turn https_port <<< "$settings"
[[ -z "${DOCKER_HOST:-}" || "$DOCKER_HOST" == unix:///var/run/docker.sock ]] || { echo 'A local Docker daemon is required' >&2; exit 1; }
endpoint=$(docker context inspect --format '{{.Endpoints.docker.Host}}')
[[ "$endpoint" == unix:///var/run/docker.sock ]] || { echo 'A local Docker daemon is required' >&2; exit 1; }
docker compose version >/dev/null
command -v flock >/dev/null
mkdir -p "$DEPLOY_ROOT/releases"
exec 9>"$DEPLOY_ROOT/deploy.lock"
flock -n 9 || { echo 'Another deployment is running' >&2; exit 1; }
unset COMPOSE_PROFILES ACME_EMAIL SENIOR_SECRET ADMIN_PASSWORD_HASH TURN_LISTEN_IP TURN_EXTERNAL_IP TURN_REALM TURN_SHARED_SECRET
unset DOMAIN HTTP_PORT HTTPS_PORT
export DEPLOY_ROOT
export RELEASE_DIR="$DEPLOY_ROOT/releases/$RELEASE_ID"
export CLOUD_IMAGE="senior-control-cloud:$RELEASE_ID"
mkdir "$RELEASE_DIR"
mkdir -p "$RELEASE_DIR/deploy/coturn"
cp deploy/compose.jenkins.yaml deploy/Caddyfile deploy/redact-logs.py "$RELEASE_DIR/deploy/"
cp deploy/coturn/turnserver.conf.template "$RELEASE_DIR/deploy/coturn/"
mkdir "$RELEASE_DIR/web"
cp -R web/dist/. "$RELEASE_DIR/web/"
compose=(docker compose --project-name senior-control --env-file "$ENV_FILE" -f "$RELEASE_DIR/deploy/compose.jenkins.yaml")
if [[ "$enable_turn" == true ]]; then
  [[ -r "$DEPLOY_ROOT/turn/fullchain.pem" && -r "$DEPLOY_ROOT/turn/privkey.pem" ]] || { echo 'TURN certificates are missing' >&2; exit 1; }
  compose+=(--profile turn)
fi
"${compose[@]}" config --quiet
docker build -f deploy/Dockerfile.cloud -t "$CLOUD_IMAGE" .
"${compose[@]}" up -d --force-recreate --remove-orphans
ok=false
for attempt in {1..36}; do
  api=$(curl --silent --output /dev/null --write-out '%{http_code}' --max-time 5 --noproxy "*" --connect-to "$domain:443:127.0.0.1:$https_port" "https://$domain/api/robots" || true)
  if [[ "$api" == 401 ]] && curl --fail --silent --show-error --max-time 5 --noproxy "*" --connect-to "$domain:443:127.0.0.1:$https_port" "https://$domain/" | grep -qi '<!doctype html'; then ok=true; break; fi
  sleep 5
done
if [[ "$ok" != true ]]; then
  echo 'Health check failed; current remains unchanged. Inspect service logs and database before recovery.' >&2
  "${compose[@]}" ps >&2 || true
  "${compose[@]}" logs --no-color --tail=40 cloud caddy 2>&1 | python3 "$RELEASE_DIR/deploy/redact-logs.py" "$ENV_FILE" >&2 || true
  exit 1
fi
ln -s "$RELEASE_DIR" "$DEPLOY_ROOT/.current-$RELEASE_ID"
mv -Tf "$DEPLOY_ROOT/.current-$RELEASE_ID" "$DEPLOY_ROOT/current"
echo "Deployment healthy: $RELEASE_ID"
