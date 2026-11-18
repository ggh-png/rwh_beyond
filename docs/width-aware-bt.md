# 점군 추정 폭으로 닫기

기본 집기 트리 `demo_width.xml`은 분할된 물체 점군에서 닫기 폭을 추정합니다.
GraspGenX 응답은 자세와 점수이며 폭은 BT가 계산합니다.

## 폭 계산과 출력

후보의 모델 로컬 X축(손가락 닫힘 방향)으로 점군을 변환하고 2·98백분위 범위를 구합니다.
파지 중심축 양쪽의 범위를 대칭으로 감싸는 전체 폭을 `grasp_width`로 사용합니다.
최소 100개의 유한한 점, 8mm 이상의 유효 폭과 중심축 양쪽의 관측이 필요합니다.

```text
close_width = min(grasp_width - grasp_width_closing_inset, gripper_open_width)
```

`gripper_open_width`의 상한은 0.10m입니다. 추정 폭이 상한을 넘더라도 `grasp_width`에는
원래 추정값을 저장하고 명령 폭만 제한합니다. 폭을 제한했다는 사실이 실제 파지 가능성을 뜻하지는 않습니다.
폭 추정에 실패한 후보는 탈락하며 고정 폭으로 대체하지 않습니다.

| 출력 | 위치 | 의미 |
|---|---|---|
| `grasp_width` | SelectFeasibleGrasp 출력·Blackboard | 관측 전체 폭(m) |
| `close_width` | SelectFeasibleGrasp 출력·GraspPlan | 보정·상한 적용 후 명령 간격(m) |

`AttachObject`는 선택된 GraspPlan의 폭으로 닫습니다. `/run_demo` Result에는 폭 필드가 없습니다.
선택 로그에는 `close_width`가 출력됩니다. 부분 깊이 관측은 가려진 형상을 놓칠 수 있으며,
이 추정과 시간 기반 닫기는 힘 제어나 잡힘 검출이 아닙니다.

백분위수 보간·대칭 전체 폭의 수식과 수치 예시는
[BT 흐름의 폭 계산](bt-flows.md#grasp-width-equations)에서 확인할 수 있습니다.

## 닫기 보정값

`rwh_beyond_bringup/config/bt_demo.yaml`에서 설정합니다.

```yaml
bt_navigator:
  ros__parameters:
    bt_motion:
      grasp_width_closing_inset: 0.005
      gripper_open_width: 0.10
      gripper_close_duration: 2.0
```

inset은 전체 간격에서 빼는 양이며 허용 범위는 0~0.005m입니다.
현재 실행 YAML은 0.005m, 코드의 생략 시 기본값은 0.002m입니다.
추정 폭이 40mm라면 현재 설정의 명령 간격은 35mm입니다.
범위를 벗어난 설정은 후보 검사 전에 실패합니다. 수정 후 BT를 재활성화합니다.

## 실행과 반복 집기

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{mode: 1, target_text: "yellow block"}' --feedback
```

기본 XML의 `CheckGraspReady release_previous="true"`는 남은 대상을 **현재 위치에서**
개방하고, 개방 피드백 확인 후 부착을 해제합니다. 알 수 없는 부착 물체는 거부하며,
개방 실패 시 새 집기를 중단합니다. 배치 위치로 이동하는 기능은 `mode: 2`를 사용합니다.

고정 폭이 필요하면 현재 지원되는 대체 트리 `demo.xml`을 `behavior_tree`에 지정합니다.
이 트리는 `gripper_close_width=0.04m`를 사용하고, 기존 부착이 있으면 새 집기를 거부합니다.
[트리 선택과 차이](bt-flows.md#fixed-width-pick) · [오류 진단](grasp-troubleshooting.md)

## 다중 시점 점군으로 집기 { #multiview }

`demo_multiview.xml`은 먼저 정면에서 집기를 시도하고, 실패하면 좌측·우측 관측을
융합해 다시 시도합니다. 각 시점의 물체를 SAM 3로 분할하고 촬영 시각 TF로 점군을
base 좌표계에 모읍니다. 관측 이동은
`MoveJointPose`를 사용하므로 TCP 목표 IK를 계산하지 않습니다. 현재 위치에서 저장한
관절 목표까지 충돌을 검사하는 OMPL 경로를 계획합니다. 그리퍼 목표는 변경하지 않습니다.

### 관측 자세 등록

실물 팔을 원하는 자세로 잡고 정지한 상태에서 각 이름으로 캡처합니다.
아래 명령은 `feedback/joint_states`를 읽어 파일에 저장하며 이동 명령을 보내지 않습니다.
실행 중인 로봇과 같은 ROS_DOMAIN_ID·RMW·DDS 환경을 사용합니다.

```bash
# 정면 관측 자세로 잡은 뒤
ros2 run rwh_beyond_bt capture_joint_pose front
# 좌측 관측 자세로 잡은 뒤
ros2 run rwh_beyond_bt capture_joint_pose left
# 우측 관측 자세로 잡은 뒤
ros2 run rwh_beyond_bt capture_joint_pose right

# 등록된 값 확인
ros2 run rwh_beyond_bt capture_joint_pose --list
# 특정 자세를 다시 등록할 때만 명시적으로 덮어쓰기
ros2 run rwh_beyond_bt capture_joint_pose front --overwrite
```

기본 저장 파일은 `/root/.ros/rwh_beyond/joint_poses.yaml`입니다. `joint1~joint6`만 고정
순서의 rad 단위로 저장하며 그리퍼는 제외합니다. `bt_motion.joint_poses_file`을 바꾸면
CLI의 `--file /원하는/경로.yaml`도 같은 경로로 지정합니다. 파일을 백업하거나 호스트
데이터 디렉터리에 연결하면 컨테이너 교체 후에도 보존할 수 있습니다.

저장 형식은 다음과 같습니다. 아래 빈 목록은 초기 상태이며, 예시 관절값을 임의로
등록하지 않습니다.

```yaml
version: 1
units: radians
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
poses: {}
```

`CaptureJointPose` BT 노드도 같은 파일에 저장합니다. 자세 등록용 트리에서만 사용하고,
일반 집기 트리는 기존 자세를 덮어쓰지 않습니다.

```xml
<CaptureJointPose pose_name="front" timeout="5.0"/>
<MoveJointPose pose_name="front" settle_time="1.0"/>
```

집기 XML의 첫 `CheckJointPoses pose_names="front;left;right"`는 세 자세가 모두 등록돼
있는지 검사합니다. 하나라도 없거나 파일 형식이 잘못되면 home·그리퍼 이동 전에 실패합니다.
정면 집기 실패 시 그리퍼를 열고 left → right 순서로 촬영합니다. 멀티뷰 집기 후에는
front 관절 자세를 거쳐 home으로 이동합니다. 모든 집기 시도가 실패하면 home 복귀를
시도합니다.

`MoveJointPose`는 관측용 `moveit_observation_planner_id`, 계획 시간·시도 횟수와 일반
관절 계획의 속도·가속도 배율을 사용합니다. `observation_motion_mode`와 관측 IK 시간은
이 노드에 적용되지 않습니다. 목표에 이미 도착했다면 실행 명령을 생략하고 안정화만
기다립니다. 실행 후 실제 관절 피드백을 `joint_pose_tolerance`(기본 0.01rad)로 최대
10초 확인합니다. 관절 한계·충돌은 MoveIt 계획에서, 반환 궤적과 목표 일치는 BT에서
검사합니다. 등록된 자세도 현재 장면에서 이동 가능한 경로가 없으면 실패할 수 있습니다.

### 점군 융합 흐름

1. `ResetObjectCloudFusion`: 매 실행마다 이전 누적을 비우고 3mm voxel 설정으로 시작합니다.
2. 각 `MoveJointPose` 도착·안정화 후 새로운 `CaptureRgbd`와 `SegmentObject`를 수행합니다.
3. `AccumulateObjectCloud`: 촬영 데이터·마스크·`base_T_camera`를 보관하고,
   TF로 변환한 현재 점군을 이전 누적 점군에 ICP로 정렬합니다.
4. `BuildFusedObjectCloud`: base 좌표계에서 같은 voxel의 점을 평균내 중복을 줄입니다.
5. 마지막 촬영의 optical 좌표계와 stamp로 변환해 기존 `InferGrasps`에 전달합니다.
6. `SelectFeasibleGrasp fusion="{fusion}"`이 원본 관측으로 융합 결과를 재계산해 검증한 후,
   기존 폭 추정·사전 접근·파지·부착·리프트 흐름을 실행합니다.

base 좌표계 디버그 점군은 `/grasp/fused_object_cloud`에서 확인합니다.
GraspGenX에는 마지막 카메라 좌표계 점군을 전달하므로 모델의 기존 입력·출력 규약과
Action 메시지는 그대로 사용합니다. 지지면은 마지막 촬영의 배경 깊이로 추정합니다.

ICP는 PCL point-to-point 방식이며 기본적으로 각 정렬에 최대 10,000개 voxel 점을
사용합니다. 대응 거리는 15mm, 최대 반복은 40회입니다. 수렴하고 대응점 50개 이상, source 겹침
35% 이상, RMSE 4mm 이하, 물체 중심 보정 10mm 이하 및 회전 5도 이하일 때만 적용합니다.
조건을 만족하지 못하면 해당 관측은 원래 TF를 사용하며 경고 로그에 사유와 지표를
남깁니다. 첫 관측이 기준이므로 ICP는 공통 TF 오차를 보정하지 못하고, 평면이나
대칭 물체에서 잘못 정렬될 수 있습니다. 실물 적용 전 시각화와 저속 검증이 필요합니다.
`bt_demo.yaml`의 `bt_motion.icp_*`에서 위 기준과 점·반복 수를 바꿀 수 있습니다.
`icp_enabled`로 ICP를 켜고 끄며, XML의 `ResetObjectCloudFusion use_icp`가 명시되면
XML이 우선합니다. 설정은 새 fusion을 시작할 때 읽습니다. 유효 범위를 벗어나거나
RMSE 한도가 대응 거리보다 크면 해당 단계에서 실패합니다. TSDF·통계적 outlier
제거·FPS는 포함하지 않습니다.

### 속도 조정 제안

`rwh_beyond_bringup/config/bt_demo.yaml`에서 기본값은 그대로 유지했습니다.
아래는 측정 후 하나씩 시험할 시작점이며 성공률이나 안전을 보장하지 않습니다.

| 항목 | 현재값 | 시험할 값과 영향 |
|---|---:|---|
| `bt_perception.top_k`, `bt_motion.max_candidates` | 500, 1000 | 각각 200으로 시작. 후보 검사 횟수는 줄지만 가능한 파지를 놓칠 수 있습니다. |
| `bt_motion.moveit_candidate_planning_time` | 4초 | 2초부터 시험. 실패 후보의 시간 상한이며 모든 계획이 빨라지는 것은 아닙니다. |
| `bt_motion.moveit_pregrasp_num_planning_attempts` | 3 | 1~2부터 시험. 계획 성공률이 떨어지면 복원합니다. |
| `bt_motion.moveit_observation_planning_time` / `moveit_observation_num_planning_attempts` | 10초 / 5 | 5초 / 2~3부터 시험. `MoveJointPose`의 OMPL 관측 계획에 적용되며 도달하기 어려우면 실패가 늘 수 있습니다. |
| `bt_motion.joint_pose_settle_time` | 1초 | 0.7초부터 시험. 흔들림·깊이 영상과 분할 품질을 확인합니다. XML의 `settle_time`이 우선합니다. |
| `bt_motion.icp_max_points` / `icp_max_iterations` | 10000 / 40 | 각각 5000 / 30부터 시험. 정렬 시간이 줄 수 있지만 겹침·정확도·수렴률을 확인합니다. |

`moveit_velocity_scaling_factor`와 `moveit_acceleration_scaling_factor`는 일반 관절 계획,
`moveit_cartesian_velocity_scaling_factor`와 `moveit_cartesian_acceleration_scaling_factor`는
접근·리프트 등 Cartesian 경로에 적용됩니다. 모두 현재 0.5이며 실제 속도는 관절 한계,
시간 파라미터화와 하드웨어 컨트롤러에도 좌우됩니다. 무작정 높이지 말고 시뮬레이션과
저속 실물 시험으로 여유·충돌·진동을 확인하세요. 충돌 검사와 손목 제한은 끄지 않습니다.

### 선택할 XML

두 파일 모두 `rwh_beyond_bt/behavior_trees/`에 설치됩니다.

- `scan_multiview.xml`: 관측 이동·분할·융합·후보 생성까지 수행합니다. 파지·닫기·리프트는
  수행하지 않지만, 시작 시 home 이동과 그리퍼 열기를 실행합니다.
- `demo_multiview.xml`: 위 과정에 실행 가능한 후보 선택과 실제 집기·리프트·home 복귀를 더합니다.

사용할 파일은 YAML의 `default_demo_bt_xml` 또는 RunDemo의 `behavior_tree`로 선택합니다.
선택값은 자동으로 바꾸지 않습니다. 기본 install 경로는 다음과 같습니다.

```text
/root/ros2_ws/install/rwh_beyond_bt/share/rwh_beyond_bt/behavior_trees/scan_multiview.xml
/root/ros2_ws/install/rwh_beyond_bt/share/rwh_beyond_bt/behavior_trees/demo_multiview.xml
```

### 실패 조건과 적용 범위

한 번에 3개 시점을 요구하며 최대 8개까지 지원합니다. 전체 스캔은 180초 안에 끝나야
하고 중복되거나 역행하는 촬영 시각, 다른 frame/stamp의 마스크, 잘못된 TF는 거부합니다.
각 관측은 누적 당시 팔 관절과 일치해야 하고 최신 촬영은 후보 선택 시에도 30초 이내여야
합니다. 융합 뒤에는 마지막 촬영 자세를 유지한 채 추론·후보 선택을 진행합니다.

시점별 물체 중심이 첫 관측보다 8cm 넘게 달라지면 물체 이동이나 대상 전환으로 간주해
중단합니다. 이는 같은 물체임을 완전히 보장하는 추적기는 아닙니다. 동일한 대상 설명을
사용하고 물체·로봇 베이스·작업대를 스캔 중 움직이지 않아야 합니다. 시점마다 변하는
마스크와 외부 캘리브레이션 오차는 형상 겹침·두께 증가를 만들 수 있습니다.

최종 점이 100개 미만이거나 200,000개를 넘으면 실패합니다. 이 경우 XML의
`voxel_size`(0.001~0.020m)를 조절합니다. 어느 시점에서도 보이지 않은 표면을 생성하거나
후면을 완전히 복원하는 기능은 아닙니다. 실패하면 남은 집기 단계와 자동 복귀를 실행하지
않으며, 다음 실행의 Reset에서 이전 누적을 버립니다.
