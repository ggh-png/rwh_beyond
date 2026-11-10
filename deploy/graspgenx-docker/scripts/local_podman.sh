#!/usr/bin/env bash
# Dedicated local inference container; no changes to the ROS container or host runtime.
set -euo pipefail
workspace=${GRASPGENX_LOCAL_WORKSPACE:-/home/ggh/graspgenx_workspace}
image=localhost/rwh-graspgenx:44eb1d7-cu130
container=rwh-graspgenx-local
robot=${GRASPGENX_ROBOT_CONTAINER:-moma_mj_lab_1.1.2_gui}
case "${1:-status}" in
  build)
    podman build --cpu-period 100000 --cpu-quota 200000 --memory 4g -t "$image" "$workspace/docker"
    ;;
  start)
    if podman container exists "$container"; then
      podman start "$container"
      exit
    fi
    test "$(podman inspect --format '{{.State.Running}}' "$robot")" = true
    mkdir -p "$workspace"/{models,cache,tensorrt_artifacts,logs}
    podman run -d --name "$container" --device nvidia.com/gpu=0 \
      --network "container:$robot" --user 0:0 \
      --cpus 2 --memory 8g --memory-swap 8g --pids-limit 256 --shm-size 256m \
      --cap-drop ALL --security-opt no-new-privileges --read-only \
      --tmpfs /tmp:rw,size=1g,mode=1777 --env NVIDIA_VISIBLE_DEVICES=void \
      --env OMP_NUM_THREADS=2 --env OPENBLAS_NUM_THREADS=2 \
      --env GRASPGENX_SERVER_CONFIG=/opt/deploy/config/server.local.yaml \
      --env GRASPGENX_CUDA_MEMORY_FRACTION=0.6 \
      --env GRASPGENX_CPP_RUNTIME_ARTIFACTS=/data/trt \
      -v "$workspace/models:/opt/GraspGenX/ext:rw" \
      -v "$workspace/cache:/data/cache:rw" \
      -v "$workspace/tensorrt_artifacts:/data/trt:rw" \
      -v "$workspace/logs:/data/logs:rw" \
      --log-opt max-size=10mb --entrypoint /usr/bin/tini "$image" -g -- sleep infinity
    ;;
  model-start)
    # Foreground model only; Ctrl+C ends this exec session, not the idle container.
    podman exec -it "$container" /bin/bash /opt/deploy/scripts/start_server.sh
    ;;
  bridge-start)
    # A runner-owned bridge must not be duplicated by this standalone helper.
    if podman exec "$robot" python3 -c '
from pathlib import Path
for p in Path("/proc").glob("[0-9]*/cmdline"):
    try: command=p.read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError): continue
    if any(arg.endswith(b"/rwh_beyond_grasp_bridge/inference_action_server") for arg in command.split(bytes([0]))):
        raise SystemExit(0)
raise SystemExit(1)
'; then
      echo 'A ROS inference bridge is already running; manage it through its owning runner/session'
      exit
    fi
    if podman exec "$robot" python3 -c '
from pathlib import Path
p=Path("/tmp/rwh-graspgenx-bridge.pid")
try:
    command=Path("/proc/"+p.read_text().strip()+"/cmdline").read_bytes()
except (FileNotFoundError, ProcessLookupError):
    raise SystemExit(1)
raise SystemExit(0 if b"ros2\0launch\0rwh_beyond_grasp_bridge" in command else 1)
'; then
      echo 'Tracked GraspGenX ROS bridge is already running'
    else
      podman exec -d "$robot" bash -lc 'source /root/ros2_ws/install/setup.bash; echo $$ > /tmp/rwh-graspgenx-bridge.pid; exec ros2 launch rwh_beyond_grasp_bridge inference.launch.py endpoint:=tcp://127.0.0.1:5566 > /tmp/rwh-graspgenx-bridge.log 2>&1'
    fi
    ;;
  bridge-stop)
    podman exec "$robot" python3 -c '
import os, signal
from pathlib import Path
p=Path("/tmp/rwh-graspgenx-bridge.pid")
try:
    pid=int(p.read_text().strip())
    command=Path(f"/proc/{pid}/cmdline").read_bytes()
except (FileNotFoundError, ProcessLookupError):
    print("Tracked bridge is not running")
    raise SystemExit(0)
if b"ros2\0launch\0rwh_beyond_grasp_bridge" not in command:
    raise SystemExit("PID does not belong to the tracked bridge; refusing to signal")
os.kill(pid, signal.SIGINT)
print("Sent SIGINT to tracked GraspGenX ROS bridge")
'
    ;;
  stop) podman stop --time 20 "$container" ;;
  health) podman exec "$container" python /opt/deploy/scripts/health_check.py --endpoint tcp://127.0.0.1:5566 ;;
  logs) podman logs --tail 80 "$container" ;;
  status) podman ps -a --filter "name=^${container}$" ;;
  *) echo 'Usage: local_podman.sh build|start|stop|model-start|health|logs|status|bridge-start|bridge-stop' >&2; exit 2 ;;
esac
