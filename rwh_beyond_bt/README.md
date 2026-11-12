# rwh_beyond_bt

PiPER 파지 시나리오의 BehaviorTree.CPP 플러그인과 실행 XML을 제공합니다.
`rwh_beyond_navigator`의 `/run_demo` Action이 트리를 실행합니다.

## 기본 실행

심·실물 모두 `behavior_trees/demo.xml`을 사용합니다.
`bt_demo_sim`은 `piper_sim`, `bt_demo`는 `piper_moveit`을 의존 실행합니다.

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{target_text: "yellow block"}' --feedback
```

home·열기·관측은 `GraspStep`, 인식·추론·선택·접근·부착·리프트는 역할별 Action 노드입니다.
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
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/behavior_trees/rwh_beyond_tree_nodes.xml
```

범용 `MakeTcpPose`, `MoveTcp`, `SetGripper`는 사용자 정의 트리에서 사용할 수 있습니다.
`perception_preview.xml`과 `grasp_debug.xml`은 심 전용 부분 실행 트리입니다.

[실행](../docs/grasp-pipeline-guide.md) · [공통 설정](../docs/real-bt.md) ·
[Action](../docs/bt-operation.md) · [포트](../docs/bt-grasp-stages.md)
