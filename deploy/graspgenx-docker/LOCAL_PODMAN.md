# Local GraspGenX — RTX 3060

## Container stays up; model starts manually

The container now runs `tini -g -- sleep infinity`. It stays running when the model
stops. Model state is independent; check health before starting another model process. Its persistent data mounts are unchanged.
The former auto-start container is retained, stopped, as `rwh-graspgenx-autostart-backup`.

```bash
# Start/restart the idle container (already created).
bash ~/graspgenx_workspace/docker/scripts/local_podman.sh start
# Run the model in the foreground; Ctrl+C stops only this model execution.
podman exec -it rwh-graspgenx-local /bin/bash /opt/deploy/scripts/start_server.sh
```

Equivalent model helper: `bash ~/graspgenx_workspace/docker/scripts/local_podman.sh model-start`.
Run `health` only after the model has loaded. `stop` stops the entire container.
`podman logs` reflects the idle PID 1; model output appears in the exec terminal
and in the persistent `logs/server-*.log` files.


This workspace belongs to ggh. Model code/dependencies and inference run only in
the dedicated rootless Podman container, never in host Python or the ROS venv.

- Inference container: `rwh-graspgenx-local`
- Image: `localhost/rwh-graspgenx:44eb1d7-cu130`
- Official source: `44eb1d77d6d27bc97ee361e3afb717a2115e30a9`
- Robot container: `moma_mj_lab_1.1.2_gui` (existing)
- Model endpoint **inside the robot container**: `tcp://127.0.0.1:5566`
- ROS package: `rwh_beyond_grasp_bridge`
- ROS interfaces: `rwh_beyond_msgs/action/InferGrasps`

The two containers share the robot container's network namespace. Their files,
processes and Python environments remain separate. No host inference port is
published; the 5566 listener exists in that private namespace. The existing 5556
listener is untouched. Restart the inference container if the ROS container's
network namespace is recreated. Do not assume the host's localhost is the same
as the robot container's localhost.

## Lifecycle

```bash
bash /home/ggh/graspgenx_workspace/docker/scripts/local_podman.sh build
bash /home/ggh/graspgenx_workspace/docker/scripts/local_podman.sh start
bash /home/ggh/graspgenx_workspace/docker/scripts/local_podman.sh logs
bash /home/ggh/graspgenx_workspace/docker/scripts/local_podman.sh health
bash /home/ggh/graspgenx_workspace/docker/scripts/local_podman.sh bridge-start
bash /home/ggh/graspgenx_workspace/docker/scripts/local_podman.sh bridge-stop
bash /home/ggh/graspgenx_workspace/docker/scripts/local_podman.sh stop
```

No privileged mode, host network, host driver/runtime changes, all-GPU selection,
or changes to existing containers. Selects GPU 0 via NVIDIA CDI. Resource limits:
build CPU 2/RAM 4 GiB; server CPU 2/RAM 8 GiB/no additional swap, PIDs 256;
PyTorch allocator fraction 0.6. This fraction limits PyTorch allocations, not all
GPU allocations made by the driver or other libraries. Existing desktop,
MuJoCo and RViz processes remain running. TensorRT is disabled.

`models`, `cache`, `tensorrt_artifacts`, `logs` are personal bind mounts. Container
removal does not remove those directories. Normal startup reuses checkpoints and
assets. Never run a prune command or remove unrelated containers/images.

## ROS Action Bridge

Model startup is now manual; the container itself stays running independently. Use `bridge-start` /
`bridge-stop` above to manage this Bridge without signaling unrelated processes.
Do not launch a second copy with the same `/infer_grasps` Action name.

For a manual foreground Bridge instead, first use `bridge-stop`, then run inside
the existing robot container once the model health request succeeds:

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch rwh_beyond_grasp_bridge inference.launch.py endpoint:=tcp://127.0.0.1:5566
```

In another terminal of the robot container:

```bash
source /root/ros2_ws/install/setup.bash
ros2 run rwh_beyond_grasp_bridge infer_cloud /tmp/graspgenx_sample.npy \
  --frame graspgenx_sample_frame --num-grasps 20 --top-k 5
```

The sample file is populated during validation. It is an official object-cloud
sample, not a calibrated PiPER scene. Results retain that input frame and are
model candidates only: no TF/TCP alignment, IK, collision checks or robot motion.

## Verified local results (2026-11-10)

- Official model and `piper_hand` loaded; ZMQ health/metadata succeeded.
- Model backend PyTorch FP32, CUDA 13.0, torch 2.9.1+cu130; TensorRT off.
- Official sample `assets/sample_data/object_pc/1740787815_545011.json`,
  finite-filtered and deterministically sampled to 2048 points.
- Small request 20 / top-k 5: real candidates returned through the ROS Action.
- Default request 100 / top-k 20: **20 sorted PoseStamped candidates**, success,
  no mock marker, input frame/stamp preserved.
- Client command wall time 1.265 s (includes CLI startup/ROS discovery), one
  measured request only; not a latency distribution or guaranteed rate.
- Model process GPU memory observed 964–972 MiB over 10 nvidia-smi samples;
  this is sampled usage, not a precise allocator high-water mark.
- Server container memory observed ~1.311 GB. Image ~8.16 GB; persistent model
  data ~3.3 GiB including Git LFS storage. Existing desktop/ROS processes retained.
- Official named-gripper infer can return more than requested top-k; the Bridge
  validates all returned matrices/scores and enforces its requested top-k locally.
- No PiPER motion, MoveIt goal, TF/TCP alignment or IK/collision checks performed.

Rootless user mapping: container UID 0 maps to host UID 1000 (ggh). The local
network-sharing mode uses the default rootless user namespace with capabilities
dropped; it does not grant host root. No user/group/sudo/runtime configuration
was changed. Checkpoints/assets survived recreation of this project container.

Artifacts:

- `logs/local-runtime-validation.json`: default-size Action and memory measurements
- `logs/ros-action-default-output.txt`: actual candidates/positions/quaternions/scores
- `logs/local-inference-summary.json`, `logs/local-inference-result.npz`: official ZMQ client run
- `logs/requirements-resolved.txt`: resolved container Python dependencies
- `logs/server-*.log`: server logs (personal bind mount)
- `/tmp/rwh-graspgenx-bridge.log` inside robot container: Bridge launch log

Source revisions:

- GraspGenX: `44eb1d77d6d27bc97ee361e3afb717a2115e30a9`
- Gripper descriptions: `19a03c00d19aeaf052d0f6801f0041982d676e8a`
- Checkpoints: `7c834043c11a11417e31d6d5ea9355801e40a2c1`

These are official model predictions on an offline sample. They are not measured
PiPER grasp success, and the sample frame is not calibrated to the real robot.
