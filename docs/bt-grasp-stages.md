# 역할별 파지 BT 플러그인

2026-11-11부터 기본 `demo.xml`은 역할별 노드를 사용한다. 각 노드는 고정된 ROS Action
endpoint를 호출하며 XML에 `command`를 지정하지 않는다. SAM 마스크 → 로컬 ROS CPU
점군 → GraspGenX 후보 → MoveIt 후보 선택 → 접근·접촉 확인·리프트 흐름은 유지한다.

## 기본 트리와 데이터

```xml
<SegmentObject observation_id="{observation}" mask="{object_mask}"/>
<BuildObjectCloud observation_id="{observation}" mask="{object_mask}" cloud="{object_cloud}"/>
<InferGrasps observation_id="{observation}" cloud="{object_cloud}" candidates="{grasps}"/>
<SelectFeasibleGrasp observation_id="{observation}" cloud="{object_cloud}"
                     candidates="{grasps}" plan_id="{grasp_plan}" pregrasp="{pregrasp}"/>
<MoveToPregrasp plan_id="{grasp_plan}"/>
<ApproachGrasp plan_id="{grasp_plan}"/>
<CloseGripper/>
<AttachObject plan_id="{grasp_plan}"/>
<LiftObject plan_id="{grasp_plan}"/>
```

완전한 트리는 앞에 `CheckGraspReady`, `ClearOctomap`, `MoveHome`, `OpenGripper`,
`MoveObservation`을 실행하고 마지막에 `MoveHome`으로 복귀한다. `Sequence`이므로
실패하면 다음 단계는 실행하지 않는다. 그리퍼를 열거나 내려놓는 단계는 포함하지 않는다.

| BT 노드 | 고정 endpoint | 입력 → 출력 |
|---|---|---|
| CheckGraspReady | grasp/check_ready | MuJoCo·실제 모델·기존 파지 상태 확인 |
| SegmentObject | grasp/segment_object | 설정된 대상 텍스트/박스/픽셀 → observation_id, mask |
| BuildObjectCloud | grasp/build_object_cloud | observation_id, mask → cloud |
| InferGrasps | grasp/infer_grasps | observation_id, cloud → candidates |
| SelectFeasibleGrasp | grasp/select_feasible_grasp | observation_id, cloud, candidates → plan_id, pregrasp |
| MoveToPregrasp | grasp/move_pregrasp | plan_id → 검증된 사전 접근 경로 실행 |
| ApproachGrasp | grasp/approach | plan_id → Cartesian 최종 접근 |
| AttachObject | grasp/attach | plan_id → 양쪽 접촉 확인·MoveIt 부착 |
| LiftObject | grasp/lift | plan_id → 상승 경로·물리 리프트 확인 |

Blackboard에는 이름 문자열만 저장되는 것이 아니라 실제 `sensor_msgs/Image`,
`PointCloud2`, `vector<GraspCandidate>`, `TcpPose` 메시지가 저장된다. ROS Action 요청에도
그 데이터를 전달한다. 메시를 생성하지 않으며 GraspGenX에는 XYZ 점군을 보낸다.
`server_timeout="1000"`은 Nav2 Action 응답 대기 시간(ms)이며 모델 실행 전체 제한이 아니다.

## 저장 상태와 재사용 범위

원본 RGB-D·CameraInfo·촬영 시각 TF·관절 상태는 ROS 측 관측 context에 보관한다.
`observation_id`로 이 context와 Blackboard 마스크·점군을 연결한다. 서버는 입력 마스크와
점군이 해당 관측에서 나온 값과 일치하는지 확인한다. 30초가 지났거나 촬영 후 팔 관절이
0.02 rad 넘게 바뀌면 이동 전에 재관측을 요구한다. 후보의 frame/stamp도 점군과 일치해야 한다.
따라서 새 노드는 별도 프로세스에서 임의의 마스크/점군을 주입하는 무상태 API는 아니다.

추론은 `InferGrasps`에서만 수행한다. `SelectFeasibleGrasp`는 받은 후보로 관측 기반
충돌 모델·선택 영역 제외를 준비하고 TF/TCP, IK, 충돌, 접근·상승 경로와 사전 접근 경로를
검사한다. 선택한 경로는 서버가 보관하며 Blackboard의 `plan_id`로 참조한다.
`pregrasp`는 표시·다른 노드 연결용이며 `MoveToPregrasp`는 저장한 경로를 실행한다.

계획 ID와 `selected → pregrasp → approached → attached → lifted` 순서를 확인한다.
다른 계획의 ID, 단계 생략, 같은 이동의 재실행을 거부한다. 실패·취소 시 임시 대상 제외와
접촉 허용 규칙을 복원하고 미부착 계획을 무효화한다. 이미 확인한 부착 물체는 유지하며
자동으로 그리퍼를 열지 않는다. 모든 이동 경로는 MuJoCo 검증 서비스를 요구한다.

## 구현 경계

- `rwh_beyond_bt/include/rwh_beyond_bt/plugins/action/*_action.hpp`: 역할별 공개 클래스·포트 선언.
- `rwh_beyond_bt/plugins/action/*_action.cpp`: 각 클래스 구현과 독립 플러그인 등록.
- `rwh_beyond_bt/src/bt_grasp_action_node.cpp`: 공통 Action 전송·진단.
- `rwh_beyond_msgs/action/RunGraspStage.action`: command 없는 공통 전송 형식. endpoint마다 사용하는 필드가 다르다.
- `perception_stages.py`: RGB-D·TF 획득, SAM 마스크, 로컬 Depth 역투영.
- `inference_stages.py`: 로컬 점군을 기존 `InferGrasps` Action으로 전달.
- `selection_stages.py`: 충돌 장면과 실행 가능한 후보·경로 선택.
- `motion_stages.py`: 이동, 접촉 부착, 물리 리프트 검증.
- `grasp_coordinator.py`: ROS 연결·관측/계획 수명·단계별 endpoint·공유 락·취소/정리.

노드는 각각 `librwh_beyond_<name>_action_bt_node.so`로 빌드하며 공개 헤더와 CPP를 분리한다.
기존 `librwh_beyond_bt_nodes.so`는 이들을 불러오는 호환 로더다. [Nav2 스타일 구성](bt-code-style.md)을 참고한다.
백엔드는 한 ROS 프로세스에 유지한다. MoveIt 장면과 부착 상태를 공유하고 취소를 직렬화하기
위한 구성이다. 모델 컨테이너는 계속 SAM과 GraspGenX로 분리된다.

기존 `GraspStep`, `/grasp_step`, `RunGraspStep`은 이전 XML·클라이언트 호환용으로 남긴다.
legacy `prepare`는 분할·점군·추론·선택을 묶어 호출한다. 기본 트리는 이를 호출하지 않는다.
다른 노드에 `server_name`을 재지정할 때도 같은 역할의 endpoint 규약을 지켜야 한다.

## 빌드와 Groot2

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --packages-select rwh_beyond_msgs rwh_beyond_bt rwh_beyond_grasp_bridge \
  rwh_beyond_navigator rwh_beyond_bringup --symlink-install
source install/setup.bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml \
  src/rwh_beyond/rwh_beyond_bt/behavior_trees/rwh_beyond_tree_nodes.xml
```

Action 타입을 추가했으므로 기존 ROS Bridge/navigator는 빌드 후 재시작한다.
Groot2에서도 갱신된 노드 모델 XML을 다시 불러온다. 모델 서버를 재시작할 필요는 없다.
실행은 [최종 가이드](grasp-pipeline-guide.md)를 따른다.

## 2026-11-11 검증

- 관련 패키지 빌드, Bridge Python 테스트 57개, BT GTest 28개, navigator pytest 5개 통과.
- BT의 실제 ROS Action 테스트에서 마스크 → 점군 → 후보 → 계획 ID 전달과 실패 시 뒤 단계 미실행 확인.
- 잘못된 관측/계획 ID, 수정된 마스크·점군, 오래된 관측, 단계 생략·중복 실행, 취소 후 정리 검사 통과.
- 첫 실제 모델 시험: 로컬 점군 2,775개, 접근·닫기·양쪽 접촉까지 성공했으나 리프트 Cartesian
  fraction=0.95238로 실행 거부. 충돌/경로 판정 기준은 완화하지 않았다.
- 두 번째 전체 시험: 2,777개 점, 물리 리프트 7.70 cm, 마지막 홈 복귀·양쪽 접촉 유지.
- 최종 모듈 분리 후 전체 시험: 2,772개 점, 물리 리프트 7.58 cm, 홈 복귀·양쪽 접촉 유지.
- 별도 runner 시험: 후보 선택 중 cancel → 선택 영역 제외 해제, 임시 물체 제거, 양쪽 손가락의
  충돌 허용 복원, 미부착 상태 확인. `SAM_RUNNER_CANCEL_ONLY=1` 모드이며 재집기·5초 유지 시험은 제외했다.
- 문서 strict 빌드와 Groot2 노드 모델 재생성 통과.

성공 2회와 실패 1회의 제한된 회귀 시험이며 일반 물체·실물 파지 성공률을 뜻하지 않는다.
검증용 ROS 프로세스는 종료하고 기존 두 GPU 모델 서버는 유지한다.
[요약 JSON](validation/bt-split-2026-11-11.json)에 실제 단계·결과를 기록했다.

전체 BT 시험은 컨테이너 안에서 모델 서버가 실행 중일 때 다음처럼 수행한다.

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=94 SAM_PICK_FULL_TREE=1 SAM_PICK_LOG_PREFIX=/tmp/bt-split-check \
  python3 /root/ros2_ws/src/rwh_beyond/rwh_beyond_grasp_bridge/test/sam_pick_integration.py
# 취소와 장면 복원만 검증하는 별도 실행
ROS_DOMAIN_ID=95 SAM_RUNNER_CANCEL_ONLY=1 \
  python3 /root/ros2_ws/src/rwh_beyond/rwh_beyond_grasp_bridge/test/sam_runner_pick_integration.py
```
