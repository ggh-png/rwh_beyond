# Private GraspGenX inference deployment

Remote work root: `/home/ircgroup1/rwh_lab`. Proposed new private subtree:
`/home/ircgroup1/rwh_lab/graspgenx_workspace`. Existing `pose2actionLLM-isolated`
is unrelated and must remain untouched.

**Status:** Dockerfile and Compose are prepared locally. Remote build/inference
has not been validated: the supplied account cannot access the Docker daemon,
interactive `sudo -v` reports that ircgroup1 may not run sudo on BB-Jr,
and GPU/CPU/RAM allocation is not yet confirmed. Do not treat this as a running
service. No remote images, containers, host settings or files have been changed.

Local RTX 3060 deployment is now validated separately. See
[LOCAL_PODMAN.md](LOCAL_PODMAN.md) for the running local service, resource limits,
network namespace arrangement and actual ROS Action results. The remote access
block above remains unresolved.

`assets_dir: auto` uses the official checkout-aware helper; the source checkout
is selected via PYTHONPATH so mounted model data is not resolved under site-packages.

## Reviewed baseline

- Official source pinned to `44eb1d77d6d27bc97ee361e3afb717a2115e30a9`.
- Candidate: CUDA 13.0.2 runtime / Ubuntu 24.04, Python 3.12,
  PyTorch 2.9.1 cu130, official `.[serve]`, NumPy 1.26.4.
- CUDA 13.x requires a compatible R580+ driver. Observed driver: 595.91.07.
  `nvidia-smi CUDA Version` describes driver capability, not installed toolkit.
- Reuses the official ZMQ server without patches; no training/rendering extras.
  Official full Dockerfile includes training, rendering, dataset and mesh tools
  that are omitted here. `pip check` runs during build.
- No ROS on GPU server; no host Python installs or Docker/runtime changes.
- Fast inference disabled. First prove eager PyTorch inference. A separate devel
  image with nvcc and matched TensorRT headers/runtime is required for the native
  path; the runtime-only image intentionally cannot compile TensorRT engines.

Official references:
[README](https://github.com/NVlabs/GraspGenX/blob/44eb1d77d6d27bc97ee361e3afb717a2115e30a9/README.md),
[wire protocol](https://github.com/NVlabs/GraspGenX/blob/44eb1d77d6d27bc97ee361e3afb717a2115e30a9/client-server/README.md),
[fast inference](https://github.com/NVlabs/GraspGenX/blob/44eb1d77d6d27bc97ee361e3afb717a2115e30a9/FAST_INFERENCE.md).

## Phase 1: read-only diagnostics and policy gate

On the remote server, run `bash scripts/check_gpu.sh`. Confirm the target is
`BB-Jr` with the expected GPU. Docker version/CLI availability alone does not
prove socket permission or GPU runtime functionality. Installed `nvidia-ctk`
alone does not prove the daemon runtime is configured. Obtain administrator
confirmation of the account's Docker access method, allocated GPU UUID, CPU,
RAM, allowed build load, disk budget and use window. Current free disk ~139 GB
on a 93%-full root filesystem: do not clean other users' data to make room.

Sudo use was explicitly requested, but the server denies it for this account.
Do not add groups, change sudoers, reconfigure runtimes, prune Docker, stop existing
containers, or claim an idle GPU without allocation. Check existing containers
and port 5556 after Docker access is granted. Use a different `HOST_PORT` if busy.

## Phase 2: stage and build (only after policy gate)

Copy this directory into a new personal
`/home/ircgroup1/rwh_lab/graspgenx_workspace/docker/` directory. No other host
paths are mounted. Run:

```bash
bash scripts/prepare_workspace.sh
cp .env.example .env
# Edit .env: numeric RUN_UID/RUN_GID from id, approved GPU_DEVICE, CPU_LIMIT,
# MEMORY_LIMIT, WORKSPACE and any nonconflicting HOST_PORT.
docker compose config --quiet
# Apply administrator-approved CPU/RAM/build limits through the site's build
# service or builder policy. Runtime cpus/mem_limit do NOT limit image builds.
docker compose build
```

The empty allocation fields intentionally make Compose fail closed. Never set
`GPU_DEVICE=all`. The observed UUID is recorded in the phase report only; it is
not authorization to use that device. GPU memory is not capped by the Docker
CPU/RAM limits; use the agreed scheduling policy and monitor nvidia-smi.

If an approved standalone CUDA probe is required, export `CUDA_IMAGE`,
`GPU_DEVICE`, `CPU_LIMIT`, `MEMORY_LIMIT` and run `bash scripts/gpu_probe.sh`.
This pulls an image if absent and starts only a project-named disposable
container. It performs no inference or host configuration.

## Phase 3: model load, server and persistence

```bash
docker compose up -d
# First start downloads official checkpoints/assets inside the container.
# This can take substantial time/disk; inspect disk and logs before continuing.
docker compose logs --tail=100 -f inference
docker compose exec inference python /opt/deploy/scripts/health_check.py
docker compose exec inference python -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available()); print(torch.cuda.get_device_name(0))'
```

Port binding is container `0.0.0.0:5556` → host `127.0.0.1:${HOST_PORT}` only.
No privileged mode, host networking, extra capabilities, host Python changes,
or unrestricted GPU exposure. Uses the caller's numeric UID/GID, read-only root,
private writable mounts, bounded CPU/RAM/PIDs and bounded Docker log rotation.

Persistent directories:

| Host subtree | Container path | Contents |
|---|---|---|
| models | /opt/GraspGenX/ext | graspgenx_checkpoints, gripper_descriptions including assets |
| cache | /data/cache | Hugging Face and runtime caches |
| tensorrt_artifacts | /data/trt | future GPU/version-specific TRT plans |
| logs | /data/logs | per-start server log files |

Startup verifies generator/discriminator directories and the piper_hand config.
Interrupted downloads are not silently deleted; inspect that project's paths
and recover only the affected personal data. `docker compose down` removes the
project containers/network, retaining all four bind-mounted directories. Avoid
`down -v`, pruning and unrelated Docker operations. Restart only this service:
`docker compose restart inference`. Archive/remove only your own old log files
as needed under the agreed disk quota.

The official server serializes requests; health requests can wait behind a long
inference. A health timeout is not proof of a failed CUDA model. Check logs/GPU
activity before restarting; restarts interrupt active requests.

## Phase 4–5: tunnel and real Action verification

From the robot PC (inside its ROS network namespace if containerized):

```bash
ssh -N -p 5022 -o IdentitiesOnly=yes -i ~/.ssh/id_ed25519 \
  -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:5556:127.0.0.1:5556 ircgroup1@166.104.138.121
```

The final remote port must match `HOST_PORT`. The first local port must match
the Bridge endpoint. Never store SSH passwords in .env, scripts or logs. If
forwarding is denied, request the administrator's permitted method; do not
change sshd. The endpoint is always `tcp://127.0.0.1:5556` within that namespace.

Use the [bridge README](../../rwh_beyond_grasp_bridge/README.md) to send a measured
segmented cloud. Save the real Action result, metadata, torch/CUDA versions,
input frame/stamp, request counts and health result to the phase report.
The model's transforms are gripper-base poses; no guessed PiPER TCP correction.

## TensorRT follow-up

After eager inference is validated, build a separately tagged **devel** image
with CUDA nvcc, native build tools, matched TensorRT 10.13.3.9 headers/runtime,
and official fast-inference extras. Execute the official installer inside that
image with `--diffusion-grasps 100 --obb-grasps 50` and
`--artifact-directory /data/trt`. Native library/build outputs must also be kept
in a project volume or rebuilt in that image. Do not enable `fast_inference`
until its runtime and fixed counts are validated on this exact GPU. Preserve
the eager image/config for rollback. No host CUDA Toolkit is required or
installed by this project.
