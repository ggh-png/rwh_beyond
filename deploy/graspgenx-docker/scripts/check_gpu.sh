#!/usr/bin/env bash
# READ ONLY: no pulls, containers, package installs, sudo, or filesystem changes.
set -uo pipefail
for command in hostname 'uname -sr' id 'cat /etc/os-release' nvidia-smi \
  'docker --version' 'docker compose version' \
  'docker info --format "{{json .Runtimes}}"' \
  'docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"' \
  'df -h /home/ircgroup1/rwh_lab' 'free -h' 'ss -ltn'; do
  printf '\n$ %s\n' "$command"
  bash -c "$command" || printf 'Diagnostic failed (status %s)\n' "$?"
done
# GPU visibility != permission to use it. Obtain assigned UUID and limits separately.
