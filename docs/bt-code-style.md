# Nav2 방식의 BT C++ 구성

2026-11-11 리팩토링은 Nav2 Jazzy의 `WaitAction`처럼 공개 헤더·구현·플러그인 등록을
노드별로 나눈다. XML 노드 ID, 포트, ROS Action endpoint와 파지 동작은 유지한다.
기준 구현은 [Nav2 WaitAction 헤더](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/wait_action.hpp)와
[CPP](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_behavior_tree/plugins/action/wait_action.cpp)다.

## 파일과 클래스

```text
rwh_beyond_bt/
├── include/rwh_beyond_bt/
│   ├── plugins/action/segment_object_action.hpp
│   ├── plugins/action/build_object_cloud_action.hpp
│   ├── plugins/action/...                    # 19개 노드의 공개 선언
│   ├── bt_grasp_action_node.hpp              # 파지 Action 공통 베이스
│   ├── bt_diagnostic_action_node.hpp         # MoveIt/controller 진단 베이스
│   ├── bt_move_group_node.hpp
│   ├── bt_gripper_command.hpp
│   ├── diagnostics.hpp
│   └── tcp_pose.hpp
├── plugins/action/                           # 각 헤더와 짝인 CPP 19개
│   ├── segment_object_action.cpp
│   ├── build_object_cloud_action.cpp
│   └── ...
├── src/                                     # 공통 구현·호환 로더·모델 생성기
│   ├── bt_grasp_action_node.cpp
│   ├── bt_diagnostic_action_node.cpp
│   ├── bt_move_group_node.cpp
│   ├── bt_gripper_command.cpp
│   ├── diagnostics.cpp
│   ├── tcp_pose.cpp
│   ├── register_nodes.cpp
│   └── generate_tree_nodes_xml.cpp
├── behavior_trees/
└── test/
```

예를 들어 `SegmentObjectAction`은 헤더에서 생성자, `providedPorts()`, `on_success()`를
선언하고 CPP에서 구현한다. 생성자는 `(xml_tag_name, action_name, conf)`를 받아
`GraspActionNode`로 전달한다. 기본 Action 이름은 CPP 아래의 `BT_REGISTER_NODES`에서
`BT::NodeBuilder`와 `registerBuilder<SegmentObjectAction>()`로 지정한다.
`server_name` 포트로 이를 재지정하는 Nav2 동작도 유지한다.

로컬 동기 노드인 `MakeTcpPoseAction`과 Stateful 노드인 `ClearOctomapAction`은
ROS Action 이름을 받지 않으며 `registerNodeType()`으로 등록한다.

- 클래스는 PascalCase와 `Action` 접미사, 파일과 자체 함수는 snake_case를 사용한다.
- `providedPorts`, `onStart`, `onRunning`, `onHalted`는 BehaviorTree.CPP API 이름을 유지한다.
- Nav2 Action 콜백은 `on_tick`, `on_success`, `on_aborted`, `on_cancelled`를 사용한다.
- 헤더에는 include guard, 직접 사용하는 타입의 include와 공개 선언을 둔다.
- 일반 함수·비템플릿 메서드 구현은 CPP에 둔다. 포트 타입에 의존하는 작은
  `required_input<T>`, future 준비 검사, BT 문자열 변환 특수화는 헤더에 남긴다.
- `DiagnosticActionNode<ActionT>`도 헤더에는 선언만 두고 CPP에서 두 지원 타입
  (`MoveGroup`, `FollowJointTrajectory`)을 명시적으로 인스턴스화한다. 다른 Action 타입을
  추가하려면 CPP 인스턴스화와 결과 해석을 함께 추가한다.
- 공통 진단·TCP 검증과 MoveIt/gripper 코드는 `rwh_beyond_bt_common`에 한 번 컴파일한다.

## 플러그인 라이브러리

각 CPP는 `BT_REGISTER_NODES`를 가지며 별도 공유 라이브러리로 빌드한다.
예: `rwh_beyond_segment_object_action_bt_node`, `rwh_beyond_build_object_cloud_action_bt_node`.
`CMakeLists.txt`의 `add_bt_action_plugin(name)`으로 추가하고 헤더·라이브러리는 설치·export한다.

기존 `plugin_lib_names: [rwh_beyond_bt_nodes]`는 그대로 동작한다. 이 라이브러리는
19개 개별 라이브러리를 factory에 등록하는 호환 로더다. 새 설정은 필요한 개별 라이브러리만
나열할 수도 있다. **같은 factory에 호환 로더와 그 내부 개별 라이브러리를 함께 나열하지 않는다.**
중복 노드 등록이 발생할 수 있다. 기존 runner·navigator 설정은 변경하지 않았다.

원래 enum 템플릿 `GraspStage<Stage::...>`는 제거했다. 새 노드는 각각 이름 있는 클래스이며
공통 베이스는 ROS 전송·진단·결과 성공 검사만 담당한다. typed 데이터와 파지 순서는
[역할별 파지 플러그인](bt-grasp-stages.md)을 따른다. 백엔드 Python 처리와 서버 배치는 그대로다.

## 검사

ROS 2 Jazzy 기본 설정의 `ament_uncrustify`, `ament_cpplint`를 CMake 테스트에 추가했다.
`include`, `plugins`, `src`, `test` 모두 검사하며 필터나 규칙 예외는 추가하지 않았다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --packages-up-to rwh_beyond_bt rwh_beyond_navigator rwh_beyond_bringup \
  --symlink-install
source install/setup.bash
colcon test --packages-select rwh_beyond_bt rwh_beyond_navigator \
  --event-handlers console_direct+
colcon test-result --verbose
```

`test_plugin_registration`은 19개 라이브러리를 직접 로드한 factory와 호환 로더의
노드 ID·포트 방향·타입이 같은지 확인한다. Action 전송, 실패 전파, Octomap·TF·그리퍼
변환 검사는 기존 테스트를 재사용한다. Groot2 모델도 실제 factory에서 다시 생성해 비교한다.

## 2026-11-11 검증 결과

- BT·navigator·bringup 빌드 통과.
- 기본 규칙의 cpplint·uncrustify 검사 모두 통과.
- BT GTest 29개(플러그인 로딩 1개 포함), navigator pytest 5개 통과.
- 생성한 Groot2 모델은 리팩토링 전 파일과 byte 단위로 동일하다.
- 실제 SAM 3·GraspGenX와 별도 ROS 도메인의 MuJoCo 전체 BT 시험 성공:
  로컬 점군 2,778개 → 접근·파지 → 물리 리프트 7.44 cm → 홈 복귀,
  복귀 후에도 양쪽 손가락 접촉 유지.
- 별도 runner 시험에서 후보 선택 중 취소 후 선택 영역 제외·임시 대상·손가락 충돌
  허용 규칙 복원 확인. 이 실행은 취소 검증 전용이며 장시간 유지·재집기 시험은 포함하지 않는다.
- 문서 strict 빌드와 변경 공백 검사 통과. 검증용 ROS 프로세스는 종료하고 모델 서버는 유지한다.

[검증 결과 JSON](validation/nav2-bt-refactor-2026-11-11.json)에 실제 상태와 실행 단계를 기록했다.
이는 구조 변경의 회귀 검사이며 실물 로봇 동작을 검증한 결과는 아니다.
