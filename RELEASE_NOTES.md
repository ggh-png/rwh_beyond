# v2.0.0 — 2026-11-26

- Add file inference for SAM 3 and GraspGenX. Set `inference_transport` in `ros_runner/config/runners.yaml` to `direct`, `ssh`, or `shared`.
- Run direct inference with `RunDemo mode: 0`, file inference with `mode: 1`, and place with `mode: 2`. Update existing direct clients that sent `mode: 1`.
- Create one request folder at action start, named with the robot's local time at second precision. Keep model inputs, results, and optional multiview files together under `~/beyond_demo/com_msgs` on the robot host.
- Update the ROS inference actions, Runner, and BEYOND together. Existing installations of the earlier `v2.0.0` must fetch the updated tags and rebuild these packages.

- Pin the BEYOND dependency releases in `moma-mj-lab.repos` and keep a development manifest in `moma-mj-lab.latest.repos`.
- Share the PiPER speed setting between the real and simulation profiles, with an optional simulation override.
- Add a full-size labeled can to MuJoCo and vary both object poses on viewer reset. Clear stale simulated grasp state on reset.
- Support a zero-distance pregrasp and tune the default joint and Cartesian motion scales. The default multiview tree currently uses a 0.20 m pregrasp on both branches and returns home after closing.
- Preserve the real robot's payload guards. Configure local inference credentials before running the released runner.

## Validation

Real SAM 3 and GraspGenX completed a file-mode MuJoCo Coca-Cola can pick over SSH. The robot and server request files matched. The `shared` profile activated the simulation Navigator and both bridges with SSH configuration absent. See [File inference setup](docs/file-inference-exchange.md).

The MuJoCo left/right observation sequence passed ten transitions with a joint velocity scale of 0.7 and acceleration scale of 1.0. A Coca-Cola can pick completed after resetting to the default scene. Verify hardware speed, grasp depth, and the return path on the deployed arm before enabling physical operation.

Two existing regression tests remain failing: standalone RViz and simulation report different arm planning limits, and the default wrist pose shows too few box pixels for the RGB-D geometry test. Both failures were reproduced on the preceding revisions.

# v1.0.0 — 2026-11-12

- Add RunDemo mode 1 (pick), mode 2 (guarded place/release), and legacy mode 0 support.
- Compute placement relative to the configured home TCP position using base-frame +X 0.20 m and +Z 0.15 m; preserve home orientation.
- Retain attached payloads after motion failures; release only after successful placement.
- Configure the default place tree in bringup and report stage-specific navigator failures.

## Integration

Use the accompanying ros_runner startup wrapper for the configurable, single-attempt /exit_teach_mode call (10 s delay, 15 s client timeout). This service may home and enable the arm.
The ros2-docker camera mounts and recovery documentation cover stale device nodes after USB reconnection.
RunDemo action schema changed: rebuild dependent packages and restart nodes together.

## Validation and known limitation

Motion and bringup regression tests pass. Camera-only recovery verified RGB, aligned depth, CameraInfo and point-cloud delivery.
Physical placement is still unresolved: the base-frame target Cartesian path stopped at fraction 0.12698412698412698. This release records the current implementation, not a successful end-to-end physical place test. No incomplete trajectory is executed and no automatic release occurs after failure.
