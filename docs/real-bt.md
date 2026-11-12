# 심·실물 공통 BT와 설정

심 runner는 `bt_demo_sim`, 실물 runner는 `bt_demo`입니다.
두 모드는 `rwh_beyond_bt/behavior_trees/demo.xml` 하나를 기본 트리로 사용합니다.

| 항목 | sim | real |
|---|---|---|
| coordinator | `grasp_coordinator` | `hardware_grasp_coordinator` |
| 시간 | `use_sim_time=true` | `use_sim_time=false` |
| 피드백·controller | MuJoCo | PiPER CAN + physical transport |
| 기본 XML | `demo.xml` | `demo.xml` |

## 시나리오

준비 확인 → Octomap 갱신 → home → 열기 → 관측 → Octomap 갱신 → SAM → 점군 →
GraspGenX → 후보 선택 → pregrasp → 접근 → 닫기·계획용 부착 → 리프트 → 관측 복귀 → home.

실패하면 후속 단계를 실행하지 않습니다. 이미 부착한 대상의 충돌 모델은 유지하며 자동 해제하지 않습니다.
실제 잡힘 여부·폭·힘에 의한 파지 성공 판정은 수행하지 않습니다.

## XML에서 관측 자세 조정

`demo.xml`의 두 `move_observation` 호출을 수정합니다. 최초 관측과 복귀 값을 같게 유지하려면
두 위치에 동일한 값을 지정합니다.

```xml
<GraspStep command="move_observation" rise_m="0.10" tilt_deg="40.0" server_timeout="1000"/>
```

- `rise_m`: home TCP 기준 base +Z 상승량(m).
- `tilt_deg`: home TCP 로컬 Y축 상대 회전(도).
- 위치와 회전은 하나의 Cartesian 경로로 계산합니다.
- 부분 경로·충돌·계획 실패는 실행하지 않습니다.
- XML 값은 YAML의 관측 기본값보다 우선합니다. 둘 다 생략하면 YAML을 사용합니다.

XML만 수정하면 재빌드는 필요 없으며, BT를 비활성화·재활성화해 다시 읽습니다.
home은 `home_joint_positions`에 정의한 6개 관절 목표입니다. 이미 home 허용 오차 안이면 이동을 생략합니다.

## 공통 YAML

`rwh_beyond_bringup/config/bt_demo.yaml`을 navigator·lifecycle manager·SAM bridge·
GraspGenX bridge·coordinator가 함께 읽습니다.

| 설정 | 현재 값 | 의미 |
|---|---|---|
| `target_text` | `yellow block` | Action에 대상이 없을 때의 기본값 |
| `gripper_open_width` | 0.10 m | 열기 목표 |
| `gripper_close_width` | 0.04 m | 닫기 목표 |
| `gripper_close_duration` | 4.0 s | 실물 닫기 완료 기준 시간 |
| `lift_distance` | 0.08 m | TCP 리프트 거리 |
| `home_joint_positions` | 6축 모두 0 rad | home 목표 |
| `home_joint_tolerance` | 0.01 rad | home 도착 판정 |
| `planning_start_bounds_tolerance` | 0.05 rad | 실물 계획 시작 상태의 작은 경계 초과 허용량 |

실물 닫기는 목표 폭을 4초간 명령하면 완료합니다. 열기·팔 이동은 위치 기반 확인을 유지합니다.
심에서는 설정한 duration으로 controller Action을 요청하고 Action 완료 결과를 확인합니다.
설정 변경 후 BT를 재시작합니다. 실물 전달기도 읽는 닫기 폭·시간이나 시작 오차 허용량을 변경하면
`piper_moveit`도 재시작합니다.

[실행](grasp-pipeline-guide.md) · [Action 대상 지정](bt-operation.md)
