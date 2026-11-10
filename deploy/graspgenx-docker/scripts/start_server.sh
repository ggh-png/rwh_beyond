#!/usr/bin/env bash
set -euo pipefail
umask 077
cd /opt/GraspGenX
log=/data/logs/server-$(date -u +%Y%m%dT%H%M%S)-$$.log
{
  # Only the official container code downloads models/assets into the private ext bind.
  python -c 'from graspgenx import get_checkpoints_root, get_gripper_descriptions_root; print(get_checkpoints_root()); print(get_gripper_descriptions_root())'
  python -u /opt/deploy/scripts/run_server.py
} 2>&1 | tee -a "$log"
