#!/usr/bin/env bash
set -euo pipefail
workspace=${SAM3_WORKSPACE:-/home/ggh/sam3_workspace}
robot=${SAM3_ROBOT_CONTAINER:-moma_mj_lab_1.1.2_gui}
image=localhost/rwh-sam3:f1647e8-cu130
container=rwh-sam3-local
case "${1:-status}" in
 build) podman build --cpu-period 100000 --cpu-quota 200000 --memory 4g -t "$image" "$workspace/deploy" ;;
 start)
  if podman container exists "$container"; then podman start "$container"; exit; fi
  mkdir -p "$workspace"/{models,cache,logs} /home/ggh/graspgenx_workspace/cache/huggingface
  podman run -d --name "$container" --device nvidia.com/gpu=0 --network "container:$robot" --user 0:0 \
   --cpus 2 --memory 8g --memory-swap 8g --pids-limit 256 --shm-size 256m \
   --cap-drop ALL --security-opt no-new-privileges --read-only --tmpfs /tmp:rw,size=1g,mode=1777 \
   -e NVIDIA_VISIBLE_DEVICES=void -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=2 \
   -e HF_HOME=/data/cache/huggingface -e SAM3_MEMORY_FRACTION=0.65 \
   -v "$workspace/models:/data/models:rw" -v "$workspace/cache:/data/cache:rw" \
   -v /home/ggh/graspgenx_workspace/cache/huggingface:/auth:ro \
   -v "$workspace/logs:/data/logs:rw" "$image" ;;
 download)
  podman exec "$container" python -c 'from pathlib import Path; from huggingface_hub import hf_hub_download; p=Path("/auth/token"); hf_hub_download("facebook/sam3", "sam3.pt", local_dir="/data/models", token=p.read_text().strip() if p.is_file() else None)' ;;
 model-start) podman exec -it "$container" bash -c 'set -o pipefail; python -u /opt/segmentation/server.py 2>&1 | tee -a /data/logs/server.log' ;;
 health) podman exec "$container" python -c 'from wire import request; print(request("tcp://127.0.0.1:5567", {"action":"health"}, 3))' ;;
 stop) podman stop --time 20 "$container" ;;
 status) podman ps -a --filter "name=^${container}$" ;;
 *) echo 'Usage: local_podman.sh build|start|download|model-start|health|stop|status'; exit 2 ;;
esac
