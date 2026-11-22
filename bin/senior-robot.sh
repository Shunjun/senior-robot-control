#!/bin/bash
# Operator syntax lives here. Python receives one normalized action, never CLI argv.
# Install this single file as senior-robot; no source-relative helper files are needed.
set -u

help() {
    while IFS= read -r line; do printf '%s\n' "$line"; done <<'HELP'
Usage: senior-robot [docker] COMMAND [OPTIONS]

Commands:
  pair       Exchange a one-time pairing code over TLS (hidden terminal prompt)
  doctor     Check the selected runtime without starting services
  deploy     Install the selected service; --dry-run previews without writes
  start      Start the selected project service
  stop       Stop project processes, leaving shared containers running
  restart    Stop successfully, then start
  status     Show process, verified ROS IPC and cloud status
  logs       Read the project's journal directly

Options:
  --control-dir PATH   Config/state directory (default: ~/senior-control)
  --profile PATH       Explicit legacy profile; excludes --control-dir
  --cloud-url URL      Required for pair (https:// or wss://)
  --name NAME          Required for pair
  --headless true|false  start/restart simulation mode (default: true)
  --dry-run            deploy only
  --container [NAME]   Docker group only (default: ros2)
  --follow             logs only
  --lines COUNT        logs only (default: 100)
  -h, --help           Show help without Python

Native: senior-robot start. Docker: senior-robot docker start.
Native Python: SENIOR_ROBOT_PYTHON or config project_dir/.venv/bin/python.
Docker runtime actions require a running container; stop/logs are host-only.
The CLI never starts/stops the shared container itself.
Host tools: jq; Docker runtime actions need docker; lifecycle/deploy need Linux systemd.
HELP
}
fail() { printf 'senior-robot: %s\n' "$1" >&2; exit "${2:-2}"; }
need_value() {
    [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || fail "$1 requires a value"
}
mode=native
if [[ "${1:-}" == docker ]]; then mode=docker; shift; fi
case "${1:-}" in
    -h|--help) help; exit 0 ;;
    pair|doctor|deploy|start|stop|restart|status|logs) action=$1; shift ;;
    '') fail 'a command is required (use --help)' ;;
    *) fail "unknown command: $1 (use --help)" ;;
esac
selection=control-dir
selected=${HOME:-}/senior-control
selection_set=false
headless=true
dry_run=false
cloud_url=
name=
follow=false
lines=100
container=ros2
container_set=false
while [[ $# -gt 0 ]]; do
    if [[ "$1" == --*=* ]]; then set -- "${1%%=*}" "${1#*=}" "${@:2}"; fi
    option=$1
    case "$option" in
        -h|--help) help; exit 0 ;;
        --profile|--control-dir)
            need_value "$@"
            [[ "$selection_set" == false ]] || fail 'select --profile or --control-dir only once'
            selection=${option#--}; selected=$2; selection_set=true; shift 2 ;;
        --cloud-url|--name)
            [[ "$action" == pair ]] || fail "$option is only valid for pair"
            need_value "$@"
            if [[ "$option" == --cloud-url ]]; then cloud_url=$2; else name=$2; fi
            shift 2 ;;
        --headless)
            [[ "$action" == start || "$action" == restart ]] || fail '--headless is only valid for start/restart'
            need_value "$@"
            [[ "$2" == true || "$2" == false ]] || fail '--headless must be true or false'
            headless=$2; shift 2 ;;
        --dry-run)
            [[ "$action" == deploy ]] || fail '--dry-run is only valid for deploy'
            dry_run=true; shift ;;
        --container)
            [[ "$mode" == docker ]] || fail 'use senior-robot docker for --container'
            container_set=true
            shift
            if [[ $# -gt 0 && "$1" != -* ]]; then container=$1; shift; fi
            [[ "$container" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}$ ]] || fail 'invalid container name' ;;
        --follow)
            [[ "$action" == logs ]] || fail '--follow is only valid for logs'
            follow=true; shift ;;
        --lines)
            [[ "$action" == logs ]] || fail '--lines is only valid for logs'
            need_value "$@"
            [[ "$2" =~ ^[0-9]+$ ]] || fail '--lines requires a nonnegative integer'
            lines=$2; shift 2 ;;
        *) fail "unknown option: $option" ;;
    esac
done
if [[ "$action" == pair ]]; then
    [[ -n "$cloud_url" && -n "${name//[[:space:]]/}" ]] || fail 'pair requires --cloud-url and --name'
    case "$cloud_url" in https://*|wss://*) ;; *) fail 'pair requires a TLS cloud URL' ;; esac
fi
# Shared host selection and installed-unit checks do not depend on Docker/Python.
normalize_docker_directory() {
    [[ -d "$selected" && ! -L "$selected" ]] || fail 'control directory is not a real directory' 1
    logical=$(cd -L -- "$selected" && pwd -L) || exit 1
    selected=$(cd -- "$selected" && pwd -P) || exit 1
    [[ "$selected" == "$logical" ]] || fail 'control directory must not contain symlink aliases' 1
}
select_docker_unit() {
    command -v jq >/dev/null 2>&1 || fail 'jq is required for project unit routing' 127
    [[ $(jq -er '.kind' "$selected/config.json") == real ]] || fail 'Docker simulation is unsupported' 1
    if command -v sha256sum >/dev/null 2>&1; then
        digest=$(printf '%s' "$selected" | sha256sum) || exit $?
    elif command -v shasum >/dev/null 2>&1; then
        digest=$(printf '%s' "$selected" | shasum -a 256) || exit $?
    else
        fail 'sha256sum or shasum is required to identify the project unit' 127
    fi
    unit=senior-agent-${digest:0:16}.service
}
require_linux() {
    [[ $(uname -s) == Linux ]] || fail 'systemd lifecycle/deployment requires Linux' 1
    command -v timeout >/dev/null 2>&1 || fail 'timeout is required for bounded host commands' 127
}
require_deployed() {
    [[ /etc/systemd/system/$unit -ef "$selected/$unit" ]] || fail 'deploy the selected project service first' 1
    grep -Fx 'Requires=docker.service' "$selected/$unit" >/dev/null || fail 'selected unit is not Docker-managed' 1
    if [[ "$1" == start ]]; then
        expected=$(docker_invoke unit) || return $?
        actual=$(< "$selected/$unit")
        [[ "$actual" == "$expected" ]] || fail 'service mode/configuration changed; redeploy the Shell-managed unit explicitly' 1
    fi
}
docker_lifecycle() {
    require_linux
    require_deployed "$1" || return $?
    # start must not invoke an old unit that auto-starts a shared container.
    timeout 30s systemctl "$1" "$unit"
}
# Runtime-dependent Docker commands inspect before exec; stop/logs stay on host.
if [[ "$mode" == docker && ( "$action" == logs || "$action" == stop ) ]]; then
    [[ "$selection" == control-dir ]] || fail 'Docker requires --control-dir' 1
    if [[ "$action" == stop ]]; then
        normalize_docker_directory
    else
        selected=$(cd -- "$selected" && pwd -P) || exit 1
    fi
    select_docker_unit
    if [[ "$action" == stop ]]; then
        invoke() { docker_lifecycle stop; }
    else
        invoke() { printf '{"unit":"%s","user":false}\n' "$unit"; }
    fi
elif [[ "$mode" == docker ]]; then
    [[ "$selection" == control-dir ]] || fail 'Docker requires --control-dir' 1
    command -v jq >/dev/null 2>&1 || fail 'jq is required for Docker routing' 127
    docker_bin=$(command -v docker) || fail 'docker is unavailable' 127
    [[ "$docker_bin" == /* ]] || fail 'docker must resolve to an absolute executable' 127
    # Container selection is deliberately independent of Python availability.
    if [[ "$container_set" == false ]]; then
        configured=$(jq -er '.container // "ros2"' "$selected/config.json") || exit 1
        container=$configured
    fi
    [[ "$container" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}$ ]] || fail 'invalid container name'
    command -v timeout >/dev/null 2>&1 || fail 'timeout is required for bounded Docker commands' 127
    inspection=$(timeout 10s "$docker_bin" inspect "$container") || exit $?
    printf '%s' "$inspection" | jq -e 'length == 1 and (.[0] | type == "object")' >/dev/null || fail 'invalid Docker inspection' 1
    printf '%s' "$inspection" | jq -e '.[0].State.Running == true' >/dev/null || fail "container $container is stopped; start it explicitly before this command" 1
    inspection=$(printf '%s' "$inspection" | jq -c '.[0]') || exit 1
    normalize_docker_directory
    config=$selected/config.json
    [[ $(jq -er '.kind' "$config") == real ]] || fail 'Docker requires a real profile; Docker simulation is unsupported' 1
    project=$(jq -er '.project_dir | select(type == "string" and startswith("/"))' "$config") || fail 'set an absolute container project_dir' 1
    # Preliminary routing only: canonical container_directory revalidates the
    # exact same inspected mount inside Python before any action or file write.
    mapped=$(printf '%s' "$inspection" | jq -er --arg root "$selected" '
        [.Mounts[] | select(.Type == "bind") |
         . + {source: (.Source | rtrimstr("/"))} |
         select(.source as $source | $root == $source or ($root | startswith($source + "/")))] |
        reduce .[] as $mount (null;
            if . == null or (.source | length) < ($mount.source | length) then $mount else . end) |
        select(.RW == true) |
        (.Destination | rtrimstr("/")) + $root[(.source | length):]
    ') || fail 'control directory needs a writable bind mount' 1
    receipt='[]'
    properties=
    event=
    docker_request() {
        jq -cn --arg action "$1" --arg host_root "$selected" --arg runtime_root "$mapped" \
            --arg container "$container" --arg docker "$docker_bin" --arg cloud_url "$cloud_url" \
            --arg name "$name" --arg properties "$properties" --arg event "$event" \
            --argjson inspection "$inspection" --argjson receipt "$receipt" \
            '{action:$action, host_root:$host_root, runtime_root:$runtime_root, container:$container,
              docker:$docker, cloud_url:$cloud_url, name:$name, inspection:$inspection,
              receipt:$receipt, properties:$properties, event:$event}'
    }
    remote() {
        command=(timeout 15s "$docker_bin" exec)
        if [[ "${1:-}" == pair || "${1:-}" == pair-preflight ]]; then
            # Original pairing runs as the invoking operator, not container root.
            # Preserve credential UID/GID and the directory's OS access checks.
            command+=(--user "$(id -u):$(id -g)")
        fi
        "${command[@]}" -i "$container" "$project/.venv/bin/python" -m vehicle_manager.operator_entry docker-request
    }
    docker_invoke() { docker_request "$1" | remote "$1"; }
    metadata=
    unit=
    if [[ "$action" != pair ]]; then
        metadata=$(docker_invoke metadata) || exit $?
        unit=$(printf '%s' "$metadata" | jq -er '.unit') || exit 1
    fi
    invoke() {
        case "$1" in
            start|stop)
                docker_lifecycle "$1" ;;
            log-route) printf '%s\n' "$metadata" ;;
            status)
                require_linux
                require_deployed status || return $?
                properties=$(timeout 10s systemctl show -p ActiveState -p InvocationID "$unit") || return $?
                active=unknown; invocation=
                while IFS='=' read -r key value; do
                    case "$key" in ActiveState) active=$value ;; InvocationID) invocation=$value ;; esac
                done <<< "$properties"
                if [[ "$active" == active && -n "$invocation" && "$invocation" != 00000000000000000000000000000000 ]]; then
                    event=$(timeout 10s journalctl --no-pager -n 1 -o cat '--grep="event": "connection_(opened|closed)"' "_SYSTEMD_INVOCATION_ID=$invocation") || event=
                fi
                docker_invoke status ;;
            deploy)
                if [[ "$dry_run" == true ]]; then docker_invoke preview; return $?; fi
                require_linux
                [[ "$EUID" == 0 ]] || fail 'deployment requires root on the Linux systemd host' 1
                [[ ! -e /etc/systemd/system/$unit && ! -L /etc/systemd/system/$unit ]] || fail 'a service with this identity already exists' 1
                receipt=$(docker_invoke install) || return $?
                linked=false
                rollback_deployment() {
                    # Same ownership fence as the original deployment rollback.
                    if [[ "$linked" == true && -L /etc/systemd/system/$unit && /etc/systemd/system/$unit -ef "$selected/$unit" ]]; then
                        rm -- "/etc/systemd/system/$unit" || return 1
                    fi
                    docker_invoke rollback || printf 'Rollback incomplete; inspect project artifacts before retrying.\n' >&2
                }
                # Keep cancellation inside the same artifact/link transaction.
                trap rollback_deployment EXIT
                trap 'exit 1' INT
                trap 'exit 143' TERM
                if timeout 10s systemctl link "$selected/$unit"; then
                    linked=true
                    if timeout 10s systemctl daemon-reload; then
                        trap - EXIT INT TERM
                        printf 'Deployed %s. Start explicitly with senior-robot docker start --control-dir %s\n' "$unit" "$selected"
                        return 0
                    fi
                fi
                rollback_deployment
                trap - EXIT INT TERM
                return 1 ;;
            pair) { docker_request pair; cat; } | remote pair ;;
            *) docker_invoke "$1" ;;
        esac
    }
else
    python=${SENIOR_ROBOT_PYTHON:-}
    if [[ -z "$python" ]]; then
        command -v jq >/dev/null 2>&1 || fail 'jq is required to locate project .venv; or set SENIOR_ROBOT_PYTHON' 127
        if [[ "$selection" == control-dir ]]; then config=$selected/config.json; else config=$selected; fi
        project=$(jq -er '.project_dir | select(type == "string" and startswith("/"))' "$config") || fail 'set an absolute project_dir or SENIOR_ROBOT_PYTHON' 1
        python=$project/.venv/bin/python
    fi
    [[ "$python" == /* && -x "$python" ]] || fail 'project .venv Python is unavailable; set an absolute SENIOR_ROBOT_PYTHON' 127
    invoke() {
        "$python" -m vehicle_manager.operator_entry "$1" "$selection" "$selected" "$headless" "$dry_run" "$cloud_url" "$name"
    }
fi
case "$action" in
    restart)
        invoke stop || exit $?
        if [[ "$mode" == docker ]]; then
            running=$(timeout 10s "$docker_bin" inspect "$container") || exit $?
            printf '%s' "$running" | jq -e 'length == 1 and .[0].State.Running == true' >/dev/null || fail "container $container is stopped; not starting the shared container" 1
        fi
        invoke start; exit $? ;;
    pair)
        invoke pair-preflight || exit $?
        # Bash owns the interaction; the secret travels only through stdin.
        set +x  # Never echo the credential through a caller-enabled trace.
        tty_state=$(stty -g </dev/tty) || fail 'pair requires a terminal' 1
        restore_terminal() { stty "$tty_state" </dev/tty; }
        trap restore_terminal EXIT
        trap 'restore_terminal; exit 1' INT
        trap 'restore_terminal; exit 143' TERM
        stty -echo </dev/tty || exit 1
        code=
        if ! IFS= read -r -p 'Pairing code: ' code </dev/tty; then
            printf '\nPairing input cancelled.\n' >&2; exit 1
        fi
        restore_terminal || exit 1
        trap - EXIT INT TERM
        printf '\n' >&2
        printf '%s\n' "$code" | invoke pair
        result=$?; unset code; exit "$result" ;;
    logs)
        command -v jq >/dev/null 2>&1 || fail 'jq is required to read the project journal route' 127
        route=$(invoke log-route) || exit $?
        unit=$(printf '%s' "$route" | jq -er '.unit | select(type == "string")') || exit 1
        user=$(printf '%s' "$route" | jq -er '.user | tostring') || exit 1
        scope=(journalctl)
        [[ "$user" != true ]] || scope+=(--user)
        options=("--unit=$unit" --no-pager "--lines=$lines")
        [[ "$follow" != true ]] || options+=(--follow)
        exec "${scope[@]}" "${options[@]}" ;;
    *) invoke "$action"; exit $? ;;
esac
