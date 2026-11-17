# 파지 후보 탈락·오류 진단

`GRASP_STAGE_FAILED (-1201)`은 파지 단계 실패를 묶어 표현합니다.
`error_source`, `failed_stage`, `error_msg`를 함께 확인합니다.
`select_collision_free_grasp`의 `No feasible model grasp`는 후보를 선택하지 못했다는 뜻이며,
이 메시지만으로 SAM·GraspGenX 서버가 꺼졌다고 판단할 수 없습니다.

## invalid_observed_width만 발생하는 경우

현재 폭 추정기는 서로 다른 입력 오류를 모두 `invalid_observed_width`로 집계합니다.
1000처럼 큰 숫자는 대칭 자세를 포함한 검사 탈락 횟수이며 서버 연결 실패 횟수가 아닙니다.
폭 검사가 다른 필터보다 먼저 수행되므로 이 검사에서 모두 탈락하면 MoveIt 계획은 요청하지 않습니다.

먼저 `grasp_coordinator.ros__parameters.grasp_width_closing_inset`을 확인합니다.

```yaml
grasp_coordinator:
  ros__parameters:
    # 전체 손가락 간격에서 뺄 양(m), 허용 범위 0~0.005.
    grasp_width_closing_inset: 0.004
```

이 예시는 4mm이며, `0.05`는 50mm로 허용 상한 5mm를 초과합니다.
현재 YAML은 `0.005`m로 설정되어 있습니다. `0.05`처럼 허용 범위를 넘는 값으로
바꾸면 후보가 모두 탈락하므로 단위를 확인하고, 변경 후 `bt_demo`를 재시작합니다.
현재 코드에서는 이 값의 범위 검사를 후보별 폭 추정에서 수행하기 때문에
노드가 시작됐더라도 모든 후보가 같은 오류로 탈락할 수 있습니다.

설정이 유효한데 계속 실패하면 아래 입력 조건을 확인합니다.

- 분할된 물체의 3차원 점이 최소 100개이고 모두 유한한 값인지
- 파지 변환 행렬이 4×4이며 유한한 값인지
- `gripper_open_width`가 0 초과~0.10m 이하인지
- 파지 로컬 X축의 2~98백분위 폭이 최소 8mm인지
- 점군이 파지 중심축의 양쪽에 걸쳐 있는지

후보 좌표 변환·분할·깊이 입력을 확인하며, 설정값이 유효한지부터 구분합니다.
[폭 추정의 의미와 제한](width-aware-bt.md)을 함께 참고하세요.

## 나머지 후보 탈락 사유

| 사유 | 의미 | 확인할 설정·입력 |
|---|---|---|
| `not_top_down` | 수직 하향 접근 각도 조건 불만족 | `grasp_top_down_max_angle_deg` |
| `table_clearance` | 최종 TCP가 추정 지지면에 너무 가까움 | 지지면 추정, `grasp_table_clearance_m` |
| `off_target` | 접촉점이 허용 물체 경계 밖 | 분할·TF, `grasp_target_margin_m` |
| `grasp_height_above_object` | 물체 상단 대비 TCP 높이 범위 불만족 | 높이 필터와 최소·최대값 |
| `pre_ik` | 사전 접근 자세의 충돌 없는 IK 실패 | 도달 범위·카메라 TF·로봇 모델 |
| `wrist_goal_rotation` | IK 목표가 손목 허용 편차 밖 | 손목 회전 제한 |
| `wrist_approach_rotation`, `wrist_lift_rotation` | 접근·리프트가 손목 허용 편차 밖 | 손목 회전 제한·파지 방향 |
| `pre_plan` | MoveIt 사전 접근 계획 실패 | 플래너·계획 시간·충돌·제약 |
| `wrist_path_rotation` | 반환 경로가 손목 제한을 벗어남 | MoveIt 경로·손목 제약 |
| `pre_plan_goal_mismatch` | 성공 반환 경로 끝이 요청한 IK 목표와 다름 | 목표·경로 제약 충돌 여부 |

높이 필터를 켠 상태에서 최소·최대값을 모두 0으로 두면 필터가 꺼지는 것이 아니라
물체 상단과 같은 TCP 높이만 허용합니다. 해제는 `grasp_height_filter_enabled: false`입니다.
후보를 위로 이동시키는 기능이 아닙니다.

## 이동 중 시간 초과

`move_pregrasp`에서 MoveIt `TIMED_OUT (-6)`가 나오면 후보 선택 이후 실행 단계의 문제입니다.
실제 driver 속도, 목표·피드백 관절 오차와 실행 완료 판정을 확인합니다.
`move_group.trajectory_execution`은 실행 감시 시간이고 손목 회전 선호도가 아닙니다.
[속도·시간 제한](moveit-speed-settings.md), [손목 회전 튜닝](grasp-motion-tuning.md)을 참고하세요.

## 물체가 박스로 보이는 경우

대상 충돌 형상은 관측 범위를 base 축 정렬 박스로 근사합니다.
파지 폭과 모델 후보는 실제 분할 점군을 사용하며, 박스는 MoveIt 충돌 검사와 부착 물체에 사용합니다.
빈 공간을 과하게 막으면 우회·계획 실패가 늘고, 가려진 부분을 놓치면 실제 형상을 과소 추정할 수 있습니다.
박스의 위치·크기를 실제 점군과 비교하고 장착 TF·분할 결과부터 확인합니다.
