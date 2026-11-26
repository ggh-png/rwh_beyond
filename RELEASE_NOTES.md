# Legacy Real Robot Baseline — 2026-11-26

PiPER 실물 운용에서 되돌아갈 수 있도록 2026-11-23 기준 코드(`dfae156`)와 당시 의존성을 고정한 스냅샷입니다.

- 실물 PiPER 드라이버 속도 30%.
- 일반·직선 MoveIt 속도 및 가속도 배율 0.5.
- 정면·멀티뷰 사전 접근 10cm, 모델→TCP Z 오프셋 13cm, 그리퍼 닫기 2초.
- 파지 후 수직 리프트를 실행하고 홈으로 복귀합니다.

별도 컨테이너 또는 새 ROS 워크스페이스에서 이 태그를 체크아웃한 후 `moma-mj-lab.repos`의 의존성을 가져오세요. 기존 워크스페이스에 `vcs import --skip-existing`만 실행하면 기존 의존 저장소의 리비전이 바뀌지 않습니다. 별도 컨테이너에서는 프로젝트 기본 경로인 `/root/ros2_ws`를 사용하면 됩니다.

PiPER 드라이버와 URDF 서브모듈은 기존 배포 리비전을 사용합니다. MuJoCo와 러너는 커밋 SHA로 고정하며, RealSense는 공식 `v4.58.4` 태그를 사용합니다. 비공개 의존 저장소에 대한 GitHub 접근과 로컬 추론 설정이 필요합니다.

이 릴리즈에서는 의존성 가져오기와 설정값을 검증했습니다. 실물 PiPER의 집기·배치 성공 여부는 별도 현장 검증이 필요합니다.

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
