# 테스트와 개발

이 문서는 코드 변경을 검증하는 절차입니다. 모델 서버·센서·실물 동작 검증은
[빠른 시작](grasp-pipeline-guide.md)과 장치별 안내를 따릅니다.

## 변경할 위치

[코드 안내](project-code-map.md)에서 진입점을 찾고,
[BT 실행 구조](bt-architecture.md)와 [노드 포트](bt-grasp-stages.md)를 함께 확인합니다.

| 변경 | 함께 확인할 항목 |
|---|---|
| BT 노드·포트 | 선언·구현·등록, 실행 XML, Groot 노드 모델 |
| Action·메시지 | bridge·BT·navigator·외부 호출 측, 구조화된 오류 필드 |
| 모션·장면 처리 | 후보 선택, 실행 전 검사, 취소, 닫기·부착 이후 상태 보존 |
| launch·YAML | runner 전달값, sim/real 시간, MoveIt과 BT 설정 일치 |
| 카메라 장착 | robot_state_publisher·MoveIt·RViz의 같은 변환 사용 |

## 빌드와 자동 검사

ROS 컨테이너에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-up-to rwh_beyond_navigator
source install/setup.bash
colcon test --packages-select rwh_beyond_bt rwh_beyond_navigator --return-code-on-test-failure
colcon test-result --test-result-base build/rwh_beyond_bt --verbose
colcon test-result --test-result-base build/rwh_beyond_navigator --verbose
```

bridge·장치 전송·bringup도 수정했다면 해당 패키지를 빌드·검사합니다.
메시지 변경은 이를 사용하는 모든 패키지의 재빌드가 필요합니다.
`colcon test`는 이미 빌드된 테스트를 실행하므로 소스 변경 후 빌드를 먼저 수행합니다.

| 위치 | 검사 대상 |
|---|---|
| `rwh_beyond_bt/test/` | 모의 RGB-D·모델·MoveIt·controller, 노드 포트, 단계 전이·취소·오류·Octomap |
| `rwh_beyond_navigator/test/` | XML filter, 요청별 대상·오류 Result·Groot 모니터링 |
| `rwh_beyond_grasp_pipeline/test/` | 실물 전달기·피드백·점군 필터 |
| `rwh_beyond_sam3_bridge/test/` | SAM bridge 입력·오류·프로토콜 |
| `rwh_beyond_graspgenx_bridge/test/` | 모델 bridge·점군·프로토콜·CLI |
| `rwh_beyond_bringup/test/` | launch 구성과 공통 설정 |
| `tests/integration/` | 실제 SAM·MuJoCo를 사용하는 수동 통합 검사 |

모의·정적 검사는 실제 물체 잡힘이나 보정 정확도를 증명하지 않습니다.
수동 통합 검사는 필요한 모델·시뮬레이터와 실행 범위를 확인한 뒤 별도로 수행합니다.

## Groot 노드 모델 갱신

```bash
ros2 run rwh_beyond_bt generate_tree_nodes_xml \
  /root/ros2_ws/src/rwh_beyond/rwh_beyond_bt/node_models/rwh_beyond_tree_nodes.xml
```

노드 모델은 참조 정의입니다. 실행 트리는 `behavior_trees/demo_multiview.xml`, `demo_width.xml`, `demo.xml`, `place.xml`입니다.
Groot에서 저장한 XML은 [자동 정리 규칙](bt-operation.md#groot-xml-filter)에 따라 로드합니다.

## 문서와 변경 검토

```bash
cd /root/ros2_ws/src/rwh_beyond
mkdocs build --strict
git diff --check
```

명령·설정값은 실행 소스와 대조합니다. 변경 당시의 시험 로그·개인 작업 경로는
현재 사용 가이드에 넣지 않고 Git 이력이나 별도 이슈에 기록합니다.
[문서 관리](documentation.md)
