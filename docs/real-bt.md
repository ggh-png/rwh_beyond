# 실물 PiPER BT 실행

`bt_demo` runner는 실물용이고, `bt_demo_sim`은 기존 MuJoCo 시뮬용입니다.
이번 변경은 코드·빌드·단위 검사만 검증했으며 실물 실행 실험은 하지 않았습니다.
장착 TF는 여전히 실측 보정 전의 명목값입니다. 실제 카메라 점군/물체 위치 정합을 확인하세요.

## 시작

기존 runner와 실물 MoveIt을 종료하고 새 runner를 실행해야 새 transport가 적용됩니다.
카메라 단독 실행도 종료합니다. 모든 명령은 `moma_mj_lab_1.1.2_hostcan` 안에서 실행합니다.

```bash
source /root/ros2_ws/install/setup.bash
export RWH_PIPER_CAN_PORT=can0
export RWH_PIPER_EXECUTION_TRANSPORT=feedback
export RWH_GRASP_TARGET_TEXT="red box"
ros2 launch ros_runner runner.launch.py
```

다른 터미널:

```bash
source /root/ros2_ws/install/setup.bash
ros2 service call /ros_runner/activate_bt_demo std_srvs/srv/Empty '{}'
```

위 명령은 의존하는 `piper_moveit`(D405·TF·MoveIt·실물 궤적 전달 포함), 원격 모델 bridge,
실물 coordinator, BT navigator를 시작합니다. 이 단계에서는 동작 목표를 보내지 않습니다.
원격 endpoint는 기존 `/root/rwh-inference/remote.env`를 자동 로드합니다.
모터 자동 활성화는 기본 false를 유지합니다. 사용자가 준비됐을 때:

```bash
ros2 service call /enable_agx_arm std_srvs/srv/SetBool '{data: true}'
```

RGB-D, TF, 실제 물체 위치와 home/관측 이동 공간을 확인한 뒤 실행합니다.
실물 BT는 home으로 이동한 후 상승과 기울임을 동시에 수행하고 새 프레임에서 관측합니다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo '{}' --feedback
```

준비 검사 → Octomap 갱신 → home 이동 → 그리퍼 열기(100 mm) → 상승·기울임 관측 이동 → Octomap 재갱신 → 관측·SAM 분할 → 점군 → GraspGenX →
충돌·IK·경로 검사 → pregrasp → 접근 → 그리퍼 닫기(목표 40 mm) → 계획용 attachment 추가 → 8 cm 리프트로 진행합니다.
기본 파지 후 시퀀스는 XML에서 조정합니다. 현재 XML은 리프트 뒤 관측 위치와 home으로 복귀하며 놓지는 않습니다. 대상의 planning scene attachment는 실제 파지 여부와 무관하게 계획용으로 유지하며
다음 집기는 거부합니다. 사용자가 물체를 내려놓은 뒤 attachment/장면을 정리해야 합니다.

## 실물 실행 경로

runner의 기본 `execution_transport=feedback`은 모의 ros2_control 대신 물리
`FollowJointTrajectory` 서버를 사용합니다. 실제 관절 위치에서 보간 명령을 시작하고,
실제 위치 피드백을 기준으로 완료를 판단합니다. arm/gripper 동시 목표는 거부합니다.
명령 전달 gate는 해당 서버가 열고 닫으며 기존 status 기반 gate는 함께 실행하지 않습니다.
관절 피드백이 1초 이상 끊기면 실패하고 gate 닫기를 요청합니다.
coordinator는 `grasp/hardware_transport_ready`를 확인하므로 이전 mock transport로는 실행하지 않습니다.
일반 수동 launch의 기본 transport는 기존 `official`을 유지하고 runner는 `feedback`을 명시합니다.

사용자 요청에 따라 물체를 집었는지 판단하지 않습니다. 닫기 목표 폭을 `gripper_close_duration`(현재 4초) 동안 명령한 뒤 폭/힘/접촉
대기 없이 TARGET을 계획용으로 attach하고 리프트·복귀를 진행합니다. attachment는 실제 파지 성공을
뜻하지 않습니다. 리프트·복귀에서도 파지 여부를 검사하지 않습니다.
팔 이동과 그리퍼 열기는 위치 피드백으로 완료를 확인합니다. 그리퍼 닫기는 4초 경과로 완료 처리하며,
닫는 도중에도 피드백 통신·취소 처리는 유지합니다. 충돌 검사도 유지합니다.
피드백/동작 실패 시 자동 놓기나 자동 복귀는 하지 않습니다.

`target_cloud_filter`는 원본 점군을 `/camera/depth/planning_points`로 전달합니다.
선택한 관측 물체의 작은 base_link AABB만 제외하며 테이블·외부 장애물과
MoveIt self-filter·Octomap·충돌 검사는 유지합니다. 제외 활성화 후 TF가 없으면 해당 프레임을 버립니다.
시뮬의 `simulation/*` 서비스를 실물 실행에서 사용하지 않습니다.

## 검증 범위

하드웨어를 실행하지 않고 build, launch 인자/구성, 실제 피드백 필수 조건,
궤적 입력·완료·피드백 소실과 gate 종료, 영역 제한과 TF 실패 시 점군 차단을 검사했습니다.
실물 파지 성공률·힘 임계값·장착 보정·운동 성능은 검증하지 않았습니다.

## BT 설정 파일

BT 실행 설정은 `rwh_beyond_bringup/config/bt_demo.yaml` 하나에서 관리합니다.
같은 파일을 navigator, lifecycle manager, SAM bridge, GraspGenX bridge,
coordinator에 전달합니다. 패키지별 bridge/grasp YAML은 단독 launch용이며 BT에서 읽지 않습니다.
기존 `params/bt_demo_params.yaml`은 이 파일로 통합했습니다.
`params_file:=/absolute/path/custom.yaml`로 전체 설정을 교체할 수 있습니다.
실물 runner는 대상 텍스트를 강제하지 않습니다. 명시한 launch 인자/환경변수는 YAML보다 우선합니다.
모델 endpoint 환경변수는 기존 원격 환경 파일에서 계속 읽습니다.

`grasp_coordinator.ros__parameters`의 주요 관측 설정:

- `home_joint_positions`: joint1..6 순서의 rad. 기본은 기존 home인 모두 0.
- `observation_z_offset`: home TCP에서 base_link +Z 방향 이동량, 현재 0.20 m. X/Y는 유지.
- `observation_tilt_rpy_deg`: home TCP 로컬 XYZ축 기준 상대 Euler 회전, 기본 [0, 30, 0]도.
- `observation_settle_time`: 이동 후 새 관측 전 안정화 시간, 기본 0.5초.
- `home_joint_tolerance`: 실제 home 도착 확인 허용 오차, 기본 0.01 rad.

이미 실제 관절 피드백이 home 허용 오차 안이면 home 이동을 생략합니다.
home 이동이 필요하면 현재 실측 관절 상태에서 충돌 검사를 포함해 계획합니다. 관측 이동은
home FK로 계산한 목표까지 하나의 Cartesian 경로에서 위치와 회전을 함께 보간합니다.
부분 경로나 충돌·도달 실패는 실행하지 않으며 BT를 실패 처리합니다.
두 이동 모두 기존 물리 피드백 transport와 속도/가속도 배율 0.1을 사용합니다.
실물 관측 경로의 도달성과 카메라 시야는 현장에서 확인해야 합니다.

## XML에서 관측 자세 조정

심/실물 공통 BT의 관측 이동은 `rwh_beyond_bt/behavior_trees/demo.xml`의
다음 한 줄을 수정합니다. 이 값은 coordinator YAML의 상승량/Y축 기울임보다 우선합니다.

```xml
<GraspStep command="move_observation" rise_m="0.10" tilt_deg="40.0" server_timeout="1000"/>
```

`rise_m`은 home TCP에서 base +Z 방향 상승량(m), `tilt_deg`는 home TCP 로컬 Y축
상대 기울임(도)입니다. 두 값을 함께 지정하며, 둘 다 생략하면 기존 YAML 설정을 사용합니다.
XML 변경 후 `bt_demo`를 비활성화/재활성화하면 다시 읽습니다. XML 값 수정만으로는 빌드가 필요 없습니다.
목표가 도달 가능한지는 별도의 경로 검사 결과에 따르며, 부분 경로는 실행하지 않습니다.

## 관절 한계 근처의 작은 영점 오차

`planning_start_bounds_tolerance`는 계획 시작 상태에서 허용하는 작은 경계 초과량입니다.
기본/최대 0.05 rad(약 2.86도)이며, 초과한 관절만 MoveIt의 실제 `robot_description` 한계값에 맞춰
계획용 복사본에서 보정합니다. 원본 관절 피드백·URDF 관절 한계·충돌 검사는 유지합니다.
허용량보다 큰 초과나 누락/잘못된 한계값은 실패 처리합니다.
실물 transport의 명령 시작·완료 확인은 계속 원본 실측값을 사용합니다.

같은 `planning_start_bounds_tolerance`를 실물 MoveIt의 실행 시작점 검사와 physical transport에도
전달합니다. transport는 원본 실측값에서 계획 시작점까지 최소 0.5초·최대 0.1 rad/s의
보간 구간을 붙여 0초 명령 점프를 방지합니다. 원래 궤적 시간은 이 구간만큼 뒤로 이동합니다.
이 허용량을 바꾼 뒤에는 `piper_moveit`과 `bt_demo`를 모두 재시작해야 합니다.

## 파지 후 관측/home 복귀

XML에서 리프트 뒤 `move_observation` 또는 `move_home`을 사용할 수 있습니다.
알려진 TARGET의 계획용 attachment가 있어야 하며,
물체의 attachment와 collision geometry를 유지해 계획합니다.
파지 상태의 관측 복귀는 home에 먼저 가지 않고 현재 자세에서 home 기준 관측 목표로 이동합니다.
빈 그리퍼의 최초 관측은 기존대로 home 선행을 요구합니다.
복귀 중 파지 여부는 판정하지 않습니다. 자동 그리퍼 열기나 attachment 제거는 하지 않습니다.

그리퍼 목표 폭은 공통 YAML의 `gripper_open_width`(현재 0.10 m),
`gripper_close_width`(현재 0.04 m)로 설정합니다. 닫기 목표 0.04 m는
양손가락 사이의 최종 개방 폭 40 mm를 뜻합니다. 물체 파지 여부를 판단하는 폭/힘 검사는 하지 않습니다.

`gripper_close_duration: 4.0`은 닫기 완료 판정 시간(초)입니다. gate가 열린 뒤 목표 폭
40 mm를 4초 동안 명령하면 실제 도달 폭·힘과 무관하게 닫기를 완료 처리합니다.
그리퍼 열기와 팔 이동의 위치 기반 완료 판정은 유지합니다. 설정 변경 후
`piper_moveit`과 `bt_demo`를 모두 재시작해야 합니다.


## 심/실물 공통 시나리오

`bt_demo.launch.py`는 `mode:=sim`과 `mode:=real` 모두 동일한 `demo.xml`을 읽습니다.
`hardware_demo.xml`은 `demo.xml`을 가리키는 호환 심볼릭 링크이며 별도 시나리오가 아닙니다.
기존 경로에서 수정해도 같은 파일에 반영됩니다.

공통 순서: 준비 확인 → Octomap 갱신 → home → 그리퍼 열기 → 관측 자세 →
Octomap 갱신 → 분할 → 물체 점군 → 파지 추론/선택 → 접근 → 닫기/계획용 attachment →
리프트 → 관측 자세 복귀 → home 복귀.

home/관측/attachment/리프트 구현도 공유합니다. 두 환경 모두 잡힘 여부를 판정하지 않고
attachment는 충돌 계획을 위한 가정으로 사용합니다. backend 준비 확인과 그리퍼 명령 전달은
각 환경의 controller를 사용하며, 실물의 4초 닫기 완료 처리는 physical transport가 담당합니다.
심에서는 시뮬레이터 trajectory action의 완료 결과를 확인합니다. 실물 상태/통신/충돌 검사는 유지됩니다.
이전 심 전용 고정 관측 자세는 공통 데모에서 사용하지 않습니다.
