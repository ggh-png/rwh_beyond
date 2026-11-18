# 속도·계획·실행 시간 제한

BT 계획 설정은 `rwh_beyond_bringup/config/bt_demo.yaml`의
`bt_navigator.ros__parameters.bt_motion`에서 관리합니다.
아래 값은 저장소의 실행 YAML 기준입니다.

## 계획 속도와 계산 시간

| 설정 | 현재 값 | 적용 대상·허용 범위 |
|---|---:|---|
| `moveit_velocity_scaling_factor` | 0.5 | 관측·home·사전 접근·배치 OMPL, 0.001~1 |
| `moveit_acceleration_scaling_factor` | 0.5 | 같은 관절 경로, 0.001~1 |
| `moveit_cartesian_velocity_scaling_factor` | 0.5 | 접근·리프트·직선 배치, 0.001~1 |
| `moveit_cartesian_acceleration_scaling_factor` | 0.5 | 같은 Cartesian 경로, 0.001~1 |
| `moveit_planning_time` | 10초 | 일반 관절 경로 계획, 0.01~120초 |
| `observation_motion_mode` | cartesian | `MoveObservation` 이동 방식: ompl / cartesian. `MoveJointPose`에는 적용되지 않음 |
| `moveit_observation_ik_timeout` | 1초 | OMPL 관측 목표 IK 탐색, 0.001~10초 |
| `moveit_observation_planner_id` | RRTConnect | 관측 전용 OMPL 플래너 ID |
| `moveit_observation_planning_time` | 10초 | 관측 관절 경로 계획, 0.01~120초 |
| `moveit_observation_num_planning_attempts` | 5 | 관측 계획 시도 횟수, 1~10 |
| `moveit_candidate_planning_time` | 4초 | 후보별 사전 접근 계획, 0.01~60초 |
| `moveit_ik_timeout` | 0.2초 | IK 요청, 0.001~10초 |

속도·가속도 배율은 로봇 모델의 관절 제한에 적용합니다. 배율을 두 배로 해도 전체 작업 시간이
절반이 되지는 않습니다. 계획 시간은 계산 예산이며 대기 시간이나 실제 이동 시간이 아닙니다.
수정 후 BT를 재활성화합니다.

## 실물 driver와 그리퍼 시간

`ros_runner/config/robot_real.yaml`의 `robot_settings.ros__parameters.piper.speed_percent`는
현재 30이며 MoveIt 배율과 별개입니다.
실물 driver가 읽는 값이므로 변경 후 `piper_moveit`과 `bt_demo`를 다시 활성화합니다.
같은 프로필의 `pub_rate=200`Hz는 피드백 발행 주기이며 관절 이동 속도와 다릅니다.
`fast_mode=false`와 `gripper_default_effort=1.0`은 기존 driver 기본값을 유지합니다.
fast mode와 그리퍼 effort는 장치 사양과 안전 조건을 확인하기 전 변경하지 마세요.

- 닫기: `gripper_close_duration=2.0`초. 명령 완료를 사용하며 실제 잡힘은 판정하지 않습니다.
- 열기: 기본 3초 명령 후 개방 피드백을 확인합니다. XML `duration`으로 바꿀 수 있습니다.
- 관측 안정화: `observation_settle_time=0.5`초.
- 등록 관절 자세 안정화: `joint_pose_settle_time=1.0`초. BT의 `settle_time`이 우선합니다.

후보 수·모델 추론·반복 home 이동도 전체 작업 시간에 영향을 줍니다.

## 병목 측정과 조정 순서

실물 설정을 바꾸기 전에 동일한 물체·관측 자세로 여러 번 실행해 아래 로그를 비교합니다.
`wall_s`는 서비스 대기와 응답을 포함한 경과 시간이며, ICP는 멀티뷰 경로에서만 실행합니다.

- `TIMING selection`: 후보 선택 전체 시간, 검사한 자세 수와 IK·Cartesian·계획 요청의
  누적 시간과 횟수. 선택 실패나 중단 시에도 `outcome=not_selected`로 기록합니다.
- `TIMING moveit_plan`: 후보 선택 외 관절 계획의 요청 경과 시간과 MoveIt 내부 계획 시간.
- `TIMING moveit_cartesian`: 실제 접근·리프트 등의 Cartesian 계산 경과 시간과 경로 비율.
- `TIMING moveit_execution`: 계획된 궤적 시간, 실행 action 대기, 이후 BT 피드백 확인 시간.
- `TIMING piper`: feedback transport의 gate 열기 시간, 계획 시간, 시작 상태 연결 시간,
  실행 경과 시간과 궤적 종료 이후 도착·안정 대기. 그리퍼 timed close는 목표 닫기 시간을 기록합니다.
- `ICP view ... elapsed_s`: 이전 점군 준비를 포함한 ICP 정렬 시간. 품질 기준 미달 시 TF를 사용합니다.

ICP와 MoveIt·PiPER 시간이 어디에서 커지는지 확인한 후 다음을 하나씩 비교합니다.
계획이 느리면 `bt_perception.top_k: 500 → 200`, `bt_motion.max_candidates: 1000 → 200`,
`moveit_candidate_planning_time: 4.0 → 2.0`, `moveit_pregrasp_num_planning_attempts: 3 → 2`를
순차적으로 시험할 수 있습니다. 제한이 작으면 가능한 파지를 놓치거나 계획이 실패할 수 있습니다.
이동 자체가 느리면 계획된 궤적 시간과 실제 실행 시간을 비교하고, PiPER 운용 한계가 확인된
안전한 환경에서 `speed_percent: 30 → 35`를 단독으로 시험합니다. 실제 속도는 MoveIt 배율,
관절 제한, 컨트롤러와 부하에도 좌우되므로 향상을 보장하지 않습니다.

실물에서는 저속·빈 작업 공간에서 비상 정지 가능 상태로 시작하고, 성공률·오차·진동과
실제 이동 시간을 함께 기록합니다. 충돌 검사, 시작 상태 확인 및 도착 후 안정 검사를 끄거나
완화해서 속도를 높이지 않습니다.

## 실물 MoveIt 실행 감시

같은 YAML의 `move_group.ros__parameters.trajectory_execution`에서 설정합니다.

```yaml
allowed_execution_duration_scaling: 3.0
allowed_goal_duration_margin: 20.0
```

MoveIt의 실행 감시 상한은 **계획 궤적 시간 × 3 + 20초**입니다.
실물 전달기는 별도로 시작 상태 연결 구간·피드백 도착·정지 확인을 수행하며,
자체 시간 제한도 갖습니다. MoveIt 감시 시간만 늘려도 모든 timeout이 해결되지는 않습니다.

변경 후 `bt_demo`와 `piper_moveit`을 중지하고 `piper_moveit` → `bt_demo` 순으로 재활성화합니다.
실행 실패 시 [문제 해결](troubleshooting.md)에서 오류 발생 계통과 피드백을 먼저 확인합니다.

## move_group의 OMPL 설정

`MoveObservation`은 `observation_motion_mode: ompl`에서 목표 TCP 자세의 IK를 구하고 관절 경로로 이동합니다. TCP 직선 이동은
요구하지 않으므로 `/compute_cartesian_path`의 fraction=0 때문에 관측 이동이 실패하지는
않습니다. 대신 목표 자체가 도달 불가이거나 충돌 상태이면 `/compute_ik` 또는
`/plan_kinematic_path`에서 실패할 수 있습니다. 파지 접근·리프트는 Cartesian 검사를 유지합니다.

같은 `bt_demo.yaml`의 `move_group.ros__parameters.ompl`에서 다음을 조절합니다.

| 설정 | 초기값 | 의미 |
|---|---|---|
| `planner_configs.RRTConnect.range` | 0.0 | 상태 공간의 트리 확장 거리; 0은 자동 |
| `planner_configs.RRTstar.range` | 0.0 | 동일한 확장 거리 설정 |
| `arm.planner_configs` | RRTConnect, RRTstar | 사용할 등록 플래너 목록 |
| `arm.default_planner_config` | RRTConnect | 요청에 planner_id가 없을 때 기본값 |
| `arm.enforce_joint_model_state_space` | true | 관절 상태 공간에서 계획 |
| `arm.longest_valid_segment_fraction` | 0.005 | 경로 충돌 검사 분해능; 작으면 촘촘하게 검사 |

BT는 관측·사전 접근의 planner_id를 명시하므로 `arm.default_planner_config`보다 BT 설정이
우선합니다. `range`는 속도나 미터 단위 TCP 이동 거리가 아닙니다. 등록되지 않은 플래너,
음수·비유한 range, 잘못된 기본 플래너는 launch 단계에서 거부합니다.

`bt_motion`만 바꾸면 `bt_demo`를 재시작합니다. `move_group` 설정을 바꾸면 기존 BT 작업을
종료한 뒤 `piper_moveit`과 `bt_demo`를 다시 시작합니다. 이 변경은 XML 선택값을 바꾸지 않습니다.

## 관측 이동 모드 전환과 IK 실패

같은 YAML의 `bt_navigator.ros__parameters.bt_motion`에서 변경합니다.

```yaml
observation_motion_mode: ompl  # ompl 또는 cartesian
moveit_observation_ik_timeout: 1.0  # OMPL 관측 목표 IK 제한 시간(초)
```

`cartesian`은 별도의 목표 IK·OMPL 요청 없이 `/compute_cartesian_path`를 사용하고,
경로 fraction이 0.999 미만이면 실행하지 않습니다. 다른 방식으로 자동 전환하지 않습니다.
모드·관측 IK 시간만 변경하면 `bt_demo`를 재시작합니다.

`NO_IK_SOLUTION`은 OMPL 경로 탐색 이전의 목표 IK 실패입니다. 도달 범위, 관절 한계,
목표 방향 또는 충돌이 원인일 수 있고 오류 코드만으로 구분할 수 없습니다. 관측 IK 시간은
일반 파지 IK의 0.2초와 분리해 기본 1초로 두었습니다. 현재 YAML의 `MoveObservation` 모드는
`cartesian`이며 `MoveJointPose`는 별도로 관절 계획을 사용합니다. 시간을 늘려도 불가능한 자세가
가능해지는 것은 아닙니다. 모드 변경도 물리적으로 불가능한 목표를 해결하지는 않습니다.

관측 시작 로그에 선택한 모드, base 기준 target XYZ·quaternion, XML의 offset·tilt·yaw가
나옵니다. 마지막으로 출력된 관측 목표를 확인해 어느 시점에서 실패했는지 구분할 수 있습니다.
