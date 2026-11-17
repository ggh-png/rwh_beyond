# 파지 접근 중 손목 뒤집기 감소

`rwh_beyond_bringup/config/bt_demo.yaml`의 `bt_navigator.ros__parameters.bt_motion`에서 설정합니다.
BT가 모델 좌표 변환·대칭 파지 선택을 수행하고 MoveIt 서비스를 직접 호출합니다.
OMPL 경로 생성, IK, 충돌 검사와 관절 경로 제약 판정은 MoveIt에서 수행합니다.
MoveIt 실행 감시 설정인 `move_group.trajectory_execution`은 경로 형태를 바꾸지 않습니다.

## 현재 설정

```yaml
moveit_candidate_planning_time: 4.0
moveit_pregrasp_planner_id: RRTConnect
moveit_pregrasp_num_planning_attempts: 3
moveit_prefer_current_grasp_orientation: true
moveit_pregrasp_wrist_constraint_enabled: true
moveit_pregrasp_wrist_max_deviation_deg: 100.0
```

현재 관절값의 MoveIt FK로 TCP 방향을 계산하고 모델→TCP 변환을 적용한
모든 후보·180도 대칭 자세를 TCP 회전 차이가 작은 순으로 검사합니다.
이는 모델 점수보다 회전 차이를 우선하는 정책이며, 전체 관절 이동이나 궤적 길이의
전역 최적화를 보장하지 않습니다. 수직 접근에 필요한 자세 변화는 남을 수 있습니다.

사전 접근은 MoveIt `MotionPlanRequest.path_constraints`의 `JointConstraint`로
후보 선택 시작 시점 joint4·joint6에서 각각 ±100도 이내로 제한합니다.
이 값은 TCP 총 회전각이나 전체 누적 관절 이동량이 아닙니다. joint5는 기울기에
필요해 제한하지 않으며 관절의 URDF 한계·충돌 검사는 유지합니다.
접근·리프트 Cartesian 경로는 계산 후 동일한 관절 범위를 검사하고,
실행 직전 재계획된 접근·리프트에도 검사합니다. 홈·관측·배치에는 적용하지 않습니다.

플래너 ID는 MoveIt에 등록된 값을 사용해야 합니다. 시도 횟수 3은 후보당
계획 시간 4초 내에서의 시도입니다. 긴 계획 시간 자체가 뒤집기를 줄이지는 않습니다.

## 조정과 확인

- 더 엄격한 손목 제한: `100.0`에서 `90.0`, 필요하면 `75.0`으로 낮춥니다.
- 도달 가능 후보가 너무 적으면 `120.0` 등으로 완화합니다(0 초과, 180 미만).
- 제한을 해제하려면 `moveit_pregrasp_wrist_constraint_enabled: false`로 설정합니다.
- 모델 순서를 우선하려면 `moveit_prefer_current_grasp_orientation: false`로 설정합니다.

제약을 만족하는 후보가 없으면 실패하며 제약 없는 계획으로 자동 재시도하지 않습니다.
`wrist_goal`은 IK 목표의 손목 제한 위반입니다. `approach_path`·`lift_path`는
부분 경로·MoveIt 오류·궤적 유효성·손목 제한 검사 실패를 함께 집계하므로 원본 오류도 확인합니다.
`pre_plan`은 MoveIt 경로 계획 실패입니다. 선택 로그의 `flipped`는 대칭 자세 사용 여부,
`rotation_deg`는 선택 시작 TCP 대비 목표 TCP 회전 차이입니다(방향 우선 설정 시).
탈락 횟수는 대칭 자세를 포함한 검사 횟수입니다.

`move_group.ros__parameters.ompl.arm`도 같은 YAML에서 조절합니다.
`planner_configs: [RRTConnect, RRTstar]`, `enforce_joint_model_state_space: true`,
`longest_valid_segment_fraction: 0.005`를 실물 move_group에 전달합니다.
사전 접근 요청은 등록된 `RRTConnect`를 명시합니다.
플래너를 바꾸려면 등록 목록과 `moveit_pregrasp_planner_id`를 함께 맞춥니다.

MoveIt이 성공을 반환해도 궤적 끝 관절값이 요청한 IK 목표와 0.002rad 이상 다르면
해당 후보를 `pre_plan`으로 집계해 제외합니다. 제약 병합 과정에서 목표가 바뀌는 경우도
실행 전에 감지합니다.

MoveIt 그룹 설정 적용은 `bt_demo`, `piper_moveit` 중지 후
`piper_moveit` → `bt_demo` 순서로 재활성화합니다.
후보 선택 설정만 바꾸면 `bt_demo`만 재시작합니다. 효과는 실제 장면·도달 범위에 따라 달라지므로 선택 로그와 경로를 확인합니다.
공식 MoveIt 경로 제약 설명: https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html
