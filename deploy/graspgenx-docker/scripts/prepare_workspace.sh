#!/usr/bin/env bash
# Run on remote host only after confirming this private path is permitted.
set -euo pipefail
workspace=/home/ircgroup1/rwh_lab/graspgenx_workspace
umask 077
mkdir -p "$workspace"/{models,cache,tensorrt_artifacts,logs}
printf 'Workspace: %s\nUID=%s GID=%s\n' "$workspace" "$(id -u)" "$(id -g)"
