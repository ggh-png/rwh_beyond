#!/usr/bin/env bash
set -euo pipefail
: "${GPU_DEVICE:?Export administrator-approved GPU UUID/index}"
: "${CPU_LIMIT:?Export approved CPU limit}"
: "${MEMORY_LIMIT:?Export approved memory limit}"
: "${CUDA_IMAGE:?Export approved CUDA image}"
# This starts a container/pulls an image; run only after Phase 1 approval.
docker run --rm --gpus "device=${GPU_DEVICE}" \
  --cpus "$CPU_LIMIT" --memory "$MEMORY_LIMIT" --pids-limit 64 \
  --cap-drop ALL --security-opt no-new-privileges --network none \
  --name rwh-graspgenx-gpu-probe "$CUDA_IMAGE" nvidia-smi
