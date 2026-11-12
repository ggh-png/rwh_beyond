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

`demo.xml`의 두 `MoveObservation` 호출을 수정합니다. 최초 관측과 복귀 값을 같게 유지하려면
두 위치에 동일한 값을 지정합니다.

```xml
<MoveObservation x="0.0" y="0.0" z="0.10" tilt_deg="40.0" server_timeout="1000"/>
```

- `x`, `y`, `z`: home TCP 기준 base 축 방향 이동량(m). 양수·0·음수 모두 가능합니다.
- 절대 좌표나 현재 위치에서 누적하는 이동량이 아니며, 매번 home을 기준으로 계산합니다.
- `tilt_deg`: home TCP 로컬 Y축 상대 회전(도).
- 위치와 회전은 하나의 Cartesian 경로로 계산합니다.
- 부분 경로·충돌·계획 실패는 실행하지 않습니다.
- XML 값은 YAML의 관측 기본값보다 우선합니다. 각 입력은 독립적이며 생략한 X·Y는 0, Z·기울임은 YAML을 사용합니다.

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
| `grasp_top_down_max_angle_deg` | 35° | 수직 하향에서 허용하는 최대 파지 각도 |
| `grasp_table_clearance_m` | 0.004 m | TCP의 지지면 위 최소 여유 높이 |
| `grasp_target_margin_m` | 0.03 m | 물체 경계 바깥 접촉점 허용 여유 |
| `lift_distance` | 0.08 m | TCP 리프트 거리 |
| `home_joint_positions` | 6축 모두 0 rad | home 목표 |
| `home_joint_tolerance` | 0.01 rad | home 도착 판정 |
| `planning_start_bounds_tolerance_deg` | 10° | 계획·MoveIt·실물 궤적의 시작 상태 허용량(도) |

실물 닫기는 목표 폭을 4초간 명령하면 완료합니다. 열기·팔 이동은 위치 기반 확인을 유지합니다.
심에서는 설정한 duration으로 controller Action을 요청하고 Action 완료 결과를 확인합니다.
설정 변경 후 BT를 재시작합니다. 실물 전달기도 읽는 닫기 폭·시간이나 시작 오차 허용량을 변경하면
`piper_moveit`도 재시작합니다.

## 파지 후보 필터 조정

세 필터는 공통 YAML의 `grasp_coordinator.ros__parameters`에서 관리하며 심·실물이 공유합니다.

```yaml
grasp_coordinator:
  ros__parameters:
    grasp_top_down_max_angle_deg: 35.0
    grasp_table_clearance_m: 0.004
    grasp_target_margin_m: 0.03
```

- `not_top_down`: 허용각을 키우면 더 기울어진 파지 후보도 검사합니다. 범위는 0~180°입니다.
- `table_clearance`: 여유 높이를 줄이면 지지면에 더 가까운 TCP도 검사합니다. 단위는 m입니다.
- `off_target`: 여유를 키우면 물체 경계 바깥쪽 접촉점도 더 넓게 허용합니다. 단위는 m입니다.

거리 여유는 0 이상의 유한한 값이어야 합니다. 필터를 통과해도 이후 IK·경로·충돌 검사는 수행합니다.
YAML 수정 후 BT runner를 비활성화·재활성화하면 적용되며 YAML 값 변경만으로 재빌드할 필요는 없습니다.

[실행](grasp-pipeline-guide.md) · [Action 대상 지정](bt-operation.md)

## Octomap 장애물 회피 켜기·끄기

`rwh_beyond_bringup/config/bt_demo.yaml`의 `/**.ros__parameters.octomap_enabled`로
실물 Octomap 장애물 회피를 제어합니다. 현재 시험 설정은 `false`입니다.
ROS 공통 파라미터가 navigator와 coordinator에 같은 값을 전달합니다. 실물 MoveIt은 이 설정이 `false`이면
점군 지도 updater를 시작하지 않으며, BT와 후보 선택 단계는 지도 갱신 대기를 건너뜁니다.
카메라·SAM·GraspGenX, 로봇 자체 충돌 검사와 대상 물체 충돌 형상은 유지됩니다.
주변 테이블·장애물은 Octomap 충돌 검사에서 제외됩니다.
변경 후 실물 장치 runner(MoveIt)와 BT를 모두 중지·재활성화해야 기존 지도까지 제거됩니다.
`true`로 복구할 때 runner의 `piper.octomap`도 `true`여야 합니다.
이 스위치는 시뮬레이터의 MoveIt 지도 updater를 제어하지 않습니다.
