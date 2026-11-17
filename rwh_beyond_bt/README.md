# rwh_beyond_bt

PiPER 파지 시나리오의 BehaviorTree.CPP 플러그인과 실행 XML을 제공합니다.
`rwh_beyond_navigator`의 `/run_demo` Action이 트리를 실행합니다.

## 기본 실행

심·실물 기본 집기는 `behavior_trees/demo_width.xml`을 사용합니다.
고정 폭 집기 `demo.xml`과 배치·해제 `place.xml`도 제공합니다.
`bt_demo_sim`은 `piper_sim`, `bt_demo`는 `piper_moveit`을 의존 실행합니다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{mode: 1, target_text: "yellow block"}' --feedback
```

`mode: 1`은 pick(기본 `demo_width.xml`), `mode: 2`는 place/release(`place.xml`)를 실행합니다.
`place.xml`은 물체를 든 상태에서 home → home TCP 위치에서 base +X 0.20 m 및 base +Z 0.15 m →
개방·attachment 해제 → home 순으로 실행합니다. 배치 직선 경로가 불완전하면 동일한 목표 위치·자세에 대해 충돌 검사 IK와 OMPL 우회 계획을 시도합니다. 완전한 계획만 실행하며 계획·실행 실패 시 물체 부착을 유지합니다. `behavior_tree` 경로를 명시하면 mode 대신
그 사용자 지정 트리를 실행합니다. mode 생략 시 기본 pick을 선택합니다.

home·열기·관측은 `MoveHome`·`OpenGripper`·`MoveObservation`, 인식·추론·선택·접근·부착·리프트는 역할별 Action 노드입니다.
home 관절값·관측 높이/각도/안정화 시간·그리퍼 열림 폭/시간은 XML 입력 포트로 조정할 수 있습니다.
입력을 생략하면 공통 설정의 기본값을 사용합니다.
기본 트리는 닫기를 `AttachObject` 안에서 수행하고 관측 자세와 home으로 복귀합니다.
실제 잡힘은 판정하지 않습니다. pick 종료 시 부착을 유지하지만 기본 폭 추정 트리의 다음 pick은
남은 대상을 현재 위치에서 개방·해제합니다. 지정 배치 위치에 놓을 때는 먼저 mode 2를 사용합니다.

## 플러그인 구조

사용자 정의 BT 노드의 구현은 `plugins/action/`에 있고, 공개 선언은
`include/rwh_beyond_bt/plugins/action/`에 있습니다. 현재 Condition 노드는 없습니다.
`CheckGraspReady`는 설정에 따라 이전 물체를 해제할 수 있는 Action입니다.
공통 로더는 `plugins/register_nodes.cpp`에 있으며 특정 노드 유형에 속하지 않습니다.
공유 진단과 모션 클라이언트는 `src/`에 둡니다.

- `rwh_beyond_perception_bt_nodes`: CaptureRgbd·SegmentObject·BuildObjectCloud·InferGrasps
- `rwh_beyond_motion_bt_nodes`: 후보 선택·모션·그리퍼·부착 등 11개 노드
- `rwh_beyond_clear_octomap_action_bt_node`: 지도 초기화
- `rwh_beyond_bt_nodes`: 전체 노드 로더

모션 노드는 `include/rwh_beyond_bt/plugins/action/`에 노드별 헤더를,
`plugins/action/`에 대응하는 소스를 둡니다. `motion_action.cpp`에서 공통 실행·취소
처리를 공유하고 `motion_context.cpp`는 MoveIt/그리퍼
클라이언트·TF·피드백·장면 자원을 공유합니다.
관측과 typed GraspPlan은 Blackboard 포트로 전달합니다.
설정은 bt_navigator.ros__parameters의 bt_perception·bt_motion에 있습니다.

Groot 노드 모델 재생성:

```bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/node_models/rwh_beyond_tree_nodes.xml
```

## 검증

`test_bt_actions.cpp`는 인식 브릿지·점군·취소를, `test_motion.cpp`는 모의 MoveIt/controller로
파지 선택·접근·닫기·부착·리프트·경로 거부·취소를 검증합니다.
`test_clear_octomap.cpp`는 지도 초기화와 신선한 점군 대기를 검증합니다.
실물 장치는 테스트에서 실행하지 않습니다.

[구조](../docs/bt-architecture.md) · [집기·배치 흐름](../docs/bt-flows.md) ·
[16개 플러그인 기능·포트](../docs/bt-grasp-stages.md) · [설정](../docs/real-bt.md)
