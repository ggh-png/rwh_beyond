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
그 사용자 지정 트리를 실행합니다. mode를 생략한 기존 호출은 pick으로 호환됩니다.

home·열기·관측은 `MoveHome`·`OpenGripper`·`MoveObservation`, 인식·추론·선택·접근·부착·리프트는 역할별 Action 노드입니다.
home 관절값·관측 높이/각도/안정화 시간·그리퍼 열림 폭/시간은 XML 입력 포트로 조정할 수 있습니다.
입력을 생략하면 공통 설정의 기본값을 사용합니다.
기본 트리는 닫기를 `AttachObject` 안에서 수행하고 관측 자세와 home으로 복귀합니다.
실제 잡힘 여부를 판정하거나 자동으로 물체를 놓지 않습니다.

## 플러그인 구조

모션 노드는 `include/rwh_beyond_bt/plugins/action/*_action.hpp`와
`plugins/action/*_action.cpp`에 선언·구현합니다.
RGB-D 획득·SAM 직접 호출·로컬 점군·그래스핑 직접 호출은 `perception_actions.hpp/.cpp`에 묶어
`rwh_beyond_perception_bt_nodes` 라이브러리로 제공합니다.
`rwh_beyond_bt_nodes` 로더가 전체 노드를 등록합니다. 같은 factory에 전체 로더와
개별 라이브러리를 중복 등록하지 않습니다.

공통 코드는 `bt_scenario_action_node.cpp`, `bt_grasp_action_node.cpp`, `diagnostics.cpp`입니다.
`tcp_pose.hpp`는 파지 단계가 전달하는 typed blackboard 포트의 자료형을 정의합니다.
Groot2 노드 모델을 재생성하려면 다음을 실행합니다.

```bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/node_models/rwh_beyond_tree_nodes.xml
```

배포하는 실행 트리는 `behavior_trees/`의 집기 2개와 배치 1개입니다.
이 트리에서 사용하는 16개 Action만 빌드·등록합니다. 범용 TCP 이동·자세 생성·독립 닫기 플러그인은 제거했습니다.
사용자 정의 트리에서 해당 노드를 사용했다면 `MoveHome`, `MoveObservation`, `MovePlace`,
`OpenGripper`와 파지 단계 노드로 전환해야 합니다. 닫기는 `AttachObject`가 수행합니다.
`node_models/rwh_beyond_tree_nodes.xml`은 Groot2 편집용 노드 정의이며 실행 트리가 아닙니다.

[실행](../docs/grasp-pipeline-guide.md) · [공통 설정](../docs/real-bt.md) ·
[Action](../docs/bt-operation.md) · [포트](../docs/bt-grasp-stages.md)

## 테스트 구조

- `test/test_bt_actions.cpp`: 플러그인 등록, 단계별 데이터·폭 전달, 실패 시 중단, 취소와 XML 입력
- `test/test_clear_octomap.cpp`: 지도 초기화·새 점군 대기·비활성화 동작

navigator 통합 테스트는 `rwh_beyond_navigator/test/`에 있으며 coordinator mock을 사용합니다.
삭제한 플러그인 전용 테스트와 직접 MoveIt/controller mock은 함께 제거했습니다.

## BT가 직접 수행하는 인식 경로

`CaptureRgbd → SegmentObject → BuildObjectCloud → InferGrasps → SelectFeasibleGrasp`로 연결합니다.
앞의 네 단계는 coordinator를 거치지 않으며 원본 촬영·마스크·점군·후보를 Blackboard에 저장합니다.
파지 선택은 촬영 묶음과 후보를 모션 backend에 한 번 전달합니다.
접근·리프트 거리도 XML 입력으로 지정하고 최종 선택 TCP 목표를 `grasp_pose` 포트로 받습니다.
MoveIt 계획·충돌 장면·부착·실물 실행은 현재 별도의 모션 backend에 남아 있습니다.
