# BT 단계·포트·직접 호출

coordinator 프로세스와 `/grasp/*` 단계 중계 액션은 제거했습니다.
기본 집기는 `demo_width.xml`, 고정 폭 집기는 `demo.xml`, 배치는 `place.xml`입니다.

| BT 노드 | 직접 호출/연산 | 입력 → 출력 |
|---|---|---|
| CheckGraspReady | 피드백·실물 전송·SAM/모델 health 확인 | release_previous |
| CaptureRgbd | 동기화 RGB-D·CameraInfo·관절 상태 수집 | → capture, observation_id |
| SegmentObject | `/segment_image` | capture → mask |
| BuildObjectCloud | BT 내부 역투영 | capture, mask → cloud |
| InferGrasps | `/infer_grasps` | cloud → candidates |
| SelectFeasibleGrasp | 촬영 TF·TCP 변환·폭·형상 추정, MoveIt IK/Cartesian/OMPL | capture, mask, cloud, candidates → selected_plan, pregrasp, grasp_pose, grasp_width, close_width |
| MoveToPregrasp | `/execute_trajectory` | plan |
| ApproachGrasp | `/compute_cartesian_path`, `/execute_trajectory` | plan |
| AttachObject | 그리퍼 FollowJointTrajectory, `/apply_planning_scene` | plan |
| LiftObject | `/compute_fk`, Cartesian, ExecuteTrajectory | plan |
| MoveHome | `/plan_kinematic_path`, ExecuteTrajectory | home_joint_positions(선택) |
| MoveObservation | `/compute_fk`, Cartesian, ExecuteTrajectory | home_joint_positions, x/y/z, tilt_deg, settle_time(선택) |
| MovePlace | FK, Cartesian 또는 IK/OMPL, ExecuteTrajectory | x, z, home_joint_positions(선택) |
| OpenGripper / ReleaseObject | `/gripper_controller/follow_joint_trajectory` | opening, duration(선택) |
| ClearOctomap | `/clear_octomap`, `/get_planning_scene` | timeout, fresh_clouds, once |

```xml
<CaptureRgbd capture="{capture}" observation_id="{observation}"/>
<SegmentObject capture="{capture}" mask="{mask}"/>
<BuildObjectCloud capture="{capture}" mask="{mask}" cloud="{cloud}"/>
<InferGrasps cloud="{cloud}" candidates="{candidates}"/>
<SelectFeasibleGrasp capture="{capture}" mask="{mask}" cloud="{cloud}"
  candidates="{candidates}" use_estimated_width="true"
  pregrasp_distance="0.10" lift_distance="0.08"
  selected_plan="{plan}" grasp_pose="{goal}"/>
<MoveToPregrasp plan="{plan}"/>
<ApproachGrasp plan="{plan}"/>
<AttachObject plan="{plan}"/>
<LiftObject plan="{plan}"/>
```

`plan`은 Blackboard의 `shared_ptr<GraspPlan>`입니다. 문자열 plan_id나 서버 세션은 없습니다.
계획은 선택 → 사전 접근 → 접근 → 부착 → 리프트 순서를 검사하며 재실행·단계 생략을 거부합니다.
관측은 30초 제한과 촬영 후 팔 관절 0.02 rad 변화 제한을 유지합니다.
마스크·점군·후보의 frame/stamp와 점군이 실제 마스크·깊이에 대응하는지도 검사합니다.

파지 목표·접근·리프트 목표는 BT에서 생성합니다. 모델은 후보 자세·점수를 반환합니다.
MoveIt이 IK·충돌·경로 계획을 수행하고 BT가 반환 경로의 완전성·손목 제한·목표 도달을 검사합니다.
`pregrasp_distance`는 0.02~0.20m, `lift_distance`는 0.065~0.20m이며 생략하면 YAML 기본값입니다.

설정은 `bt_navigator.ros__parameters.bt_perception`과 `bt_motion`입니다.
MoveObservation의 위치는 home TCP 기준 base 축 오프셋(m), tilt_deg는 home 로컬 Y 회전(도)입니다.
관절 배열은 `home_joint_positions="0;0;0;0;0;0"` 형식입니다.
열기·해제의 opening은 전체 간격 0~0.10m, duration은 0 초과~30초입니다.
모션 노드에는 중계용 server_name/server_timeout 포트가 없습니다. 모델 브릿지 노드는 기존 Nav2 액션 포트를 유지합니다.

BT halt는 실행 중인 MoveIt/그리퍼 액션을 취소합니다. 실패·취소 시 미부착 장면을 정리합니다.
닫기를 시작했거나 이미 부착된 물체는 자동 개방·제거하지 않습니다. 실제 파지 성공은 판정하지 않습니다.

`RunGraspStage`, `RunGraspStep`과 이전 플러그인 라이브러리는 제거했습니다.
사용자 XML은 plan_id를 plan으로 바꾸고 SelectFeasibleGrasp 출력은 selected_plan으로 바꿔야 합니다.
메시지·BT·navigator·bringup을 함께 빌드하고 bt_demo를 재시작합니다.

[구조](bt-architecture.md) · [공통 설정](real-bt.md)
