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
