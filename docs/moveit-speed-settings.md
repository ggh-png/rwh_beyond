# 속도·계획·실행 시간 제한

BT 계획 설정은 `rwh_beyond_bringup/config/bt_demo.yaml`의
`bt_navigator.ros__parameters.bt_motion`에서 관리합니다.
아래 값은 저장소의 실행 YAML 기준입니다.

## 계획 속도와 계산 시간

| 설정 | 현재 값 | 적용 대상·허용 범위 |
|---|---:|---|
| `moveit_velocity_scaling_factor` | 0.5 | home·사전 접근·배치 OMPL, 0.001~1 |
| `moveit_acceleration_scaling_factor` | 0.5 | 같은 관절 경로, 0.001~1 |
| `moveit_cartesian_velocity_scaling_factor` | 0.5 | 관측·접근·리프트·직선 배치, 0.001~1 |
| `moveit_cartesian_acceleration_scaling_factor` | 0.5 | 같은 Cartesian 경로, 0.001~1 |
| `moveit_planning_time` | 10초 | 일반 관절 경로 계획, 0.01~120초 |
| `moveit_candidate_planning_time` | 4초 | 후보별 사전 접근 계획, 0.01~60초 |
| `moveit_ik_timeout` | 0.2초 | IK 요청, 0.001~10초 |

속도·가속도 배율은 로봇 모델의 관절 제한에 적용합니다. 배율을 두 배로 해도 전체 작업 시간이
절반이 되지는 않습니다. 계획 시간은 계산 예산이며 대기 시간이나 실제 이동 시간이 아닙니다.
수정 후 BT를 재활성화합니다.

## 실물 driver와 그리퍼 시간

`ros_runner/config/runners.yaml`의 `piper.speed_percent`는 현재 30이며 MoveIt 배율과 별개입니다.
실물 driver가 읽는 값이므로 변경 후 `piper_moveit`과 `bt_demo`를 다시 활성화합니다.

- 닫기: `gripper_close_duration=2.0`초. 명령 완료를 사용하며 실제 잡힘은 판정하지 않습니다.
- 열기: 기본 3초 명령 후 개방 피드백을 확인합니다. XML `duration`으로 바꿀 수 있습니다.
- 관측 안정화: `observation_settle_time=0.5`초.

후보 수·모델 추론·반복 home 이동도 전체 작업 시간에 영향을 줍니다.

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
