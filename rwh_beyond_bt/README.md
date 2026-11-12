# rwh_beyond_bt

PiPER 파지 시나리오의 BehaviorTree.CPP 플러그인과 실행 XML을 제공합니다.
`rwh_beyond_navigator`의 `/run_demo` Action이 트리를 실행합니다.

## 기본 실행

심·실물 모두 `behavior_trees/demo.xml`을 사용합니다.
`bt_demo_sim`은 `piper_sim`, `bt_demo`는 `piper_moveit`을 의존 실행합니다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{mode: 1, target_text: "yellow block"}' --feedback
```

`mode: 1`은 pick(`demo.xml`), `mode: 2`는 place/release(`place.xml`)를 실행합니다.
`place.xml`은 물체를 든 상태에서 home → home TCP 위치에서 base +X 0.20 m 및 base +Z 0.15 m →
개방·attachment 해제 → home 순으로 실행합니다. `behavior_tree` 경로를 명시하면 mode 대신
그 사용자 지정 트리를 실행합니다. mode를 생략한 기존 호출은 pick으로 호환됩니다.

home·열기·관측은 `MoveHome`·`OpenGripper`·`MoveObservation`, 인식·추론·선택·접근·부착·리프트는 역할별 Action 노드입니다.
home 관절값·관측 높이/각도/안정화 시간·그리퍼 열림 폭/시간은 XML 입력 포트로 조정할 수 있습니다.
입력을 생략하면 공통 설정의 기본값을 사용합니다.
기본 트리는 닫기를 `AttachObject` 안에서 수행하고 관측 자세와 home으로 복귀합니다.
실제 잡힘 여부를 판정하거나 자동으로 물체를 놓지 않습니다.

## 플러그인 구조

각 노드는 `include/rwh_beyond_bt/plugins/action/*_action.hpp`와
`plugins/action/*_action.cpp`에 선언·구현하며 개별 shared library로 빌드합니다.
`rwh_beyond_bt_nodes` 로더가 전체 노드를 등록합니다. 같은 factory에 전체 로더와
개별 라이브러리를 중복 등록하지 않습니다.

공통 Action·진단은 `src/bt_*.cpp`, `diagnostics.cpp`, pose 자료형은 `tcp_pose.cpp`에 있습니다.
Groot2 노드 모델을 재생성하려면 다음을 실행합니다.

```bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/node_models/rwh_beyond_tree_nodes.xml
```

범용 `MakeTcpPose`, `MoveTcp`, `SetGripper`는 사용자 정의 트리에서 사용할 수 있습니다.
배포하는 실행 트리는 심·실물 공통 `behavior_trees/demo.xml`과 `behavior_trees/place.xml`입니다.
`node_models/rwh_beyond_tree_nodes.xml`은 Groot2 편집용 노드 정의이며 실행 트리가 아닙니다.

[실행](../docs/grasp-pipeline-guide.md) · [공통 설정](../docs/real-bt.md) ·
[Action](../docs/bt-operation.md) · [포트](../docs/bt-grasp-stages.md)
