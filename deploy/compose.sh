#!/usr/bin/env bash
set -euo pipefail
deploy_dir=$(cd "$(dirname "$0")" && pwd)
env_file="$deploy_dir/.env"
settings=$(python3 "$deploy_dir/validate-jenkins-env.py" "$env_file")
read -r _ enable_turn _ <<< "$settings"
unset COMPOSE_PROFILES HTTP_PORT HTTPS_PORT DOMAIN ACME_EMAIL SENIOR_SECRET ADMIN_PASSWORD_HASH \
  TURN_LISTEN_IP TURN_EXTERNAL_IP TURN_REALM TURN_SHARED_SECRET
compose=(docker compose --env-file "$env_file" -f "$deploy_dir/compose.yaml")
if [[ "$enable_turn" == true ]]; then
  compose+=(--profile turn)
fi
exec "${compose[@]}" "$@"
