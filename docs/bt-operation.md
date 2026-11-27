# RunDemo 인터페이스

`/run_demo`는 집기 또는 배치를 요청하는 ROS 2 액션입니다.

- **협업자가 설치할 GitHub 저장소**는 [ggh-png/rwh_beyond_interfaces](https://github.com/ggh-png/rwh_beyond_interfaces)입니다. 저장소의 설치 안내에 따라 `rwh_beyond_msgs`를 빌드하고 `source install/setup.bash`를 실행하세요.
- **CLI와 코드에서 사용하는 ROS 액션 타입**은 `rwh_beyond_msgs/action/RunDemo`입니다.

서버와 같은 ROS 도메인에서 `ros2 action info /run_demo`를 실행해 서버가 보이는지 확인한 뒤 호출하세요.

## 입력 (Goal)

| 필드 | 타입 | 입력값 |
|---|---|---|
| `mode` | `uint8` | `0` 또는 생략은 직접 추론 집기, `1`은 파일 추론 집기, `2`는 앞서 집은 물체의 배치·해제. |
| `target_text` | `string` | 집을 물체의 설명. 예를 들어 `"blue cup"`. 생략하면 설정된 기본 대상을 사용. 이번 요청에만 적용. |
| `behavior_tree` | `string` | 로봇의 액션 서버가 읽을 수 있는 BT XML 경로. 지정하면 기본 트리 대신 이 XML을 사용합니다. 추론 방식은 `mode`로 선택합니다. |

## CLI 예시

Runner의 `inference_transport`가 `ssh` 또는 `shared`이면 집기에 `mode: 1`을 사용합니다. `direct`에서는 `mode: 0`을 사용합니다. [파일 추론 설정](file-inference-exchange.md)

```bash
# 파일 추론으로 파란 컵 집기
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{mode: 1, target_text: "blue cup"}' --feedback

# 앞서 집은 물체 배치·해제
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{mode: 2}' --feedback
```

`--feedback`을 사용하면 진행 중인 `current_stage`와 `current_node`도 볼 수 있습니다.

## 결과 (Result) { #feedback-result }

- **성공**은 ROS Action 상태가 `SUCCEEDED`이고 `error_code`가 `0`일 때입니다. 이 값은 시퀀스 완료를 뜻합니다. 실제 파지 여부는 별도로 판정해야 합니다.
- **실패했을 때는** `ABORTED` 등 ROS Action 상태와 `error_code`, `error_domain`, `error_name`, `error_source`, `error_msg`를 함께 확인하세요. `failed_stage`와 `failed_node`에는 실패 위치가 담깁니다.
- **취소**는 ROS Action 상태 `CANCELED`로 구분합니다.

| 필드 | Feedback | Result | 의미 |
|---|---|---|---|
| `current_stage`, `current_node` | 사용 | — | 실행 중인 단계·BT 노드 |
| `failed_stage`, `failed_node` | — | 사용 | 실패한 단계·BT 노드 |
| `moveit_state` | 사용 | 사용 | 최신 MoveIt 호출·단계 문자열; 해당 호출이 없으면 비어 있을 수 있음 |
| `backend_stage` | 사용 | 사용 | 최근 백엔드 액션 또는 전송 작업 단계 |
| `error_code` | 사용 | 사용 | `0`은 오류 없음. 성공 여부는 최종 ROS Action 상태와 함께 확인 |
| `error_domain` | 사용 | 사용 | 코드의 발생 계통. 예: `bt`, `moveit`, `controller`, `transport`, `navigator` |
| `error_name`, `error_source`, `error_msg` | 사용 | 사용 | 오류 분류·발생 위치·설명 |

BT는 root Blackboard의 `execution_state`에 최신 진단을 보관합니다. 새 요청과 새 단계
시작 시 진단을 초기화하고, 실패 기록 이후의 정리 작업이 그 오류를 덮어쓰지 않도록 합니다.
Navigator의 `onLoop()`는 스냅샷을 Feedback으로 변환하며, 완료 시 Result와 마지막
Feedback을 만듭니다. ROS Feedback 객체 자체나 오류 이력을 Blackboard에 저장하지 않습니다.

SAM·GraspGenX 브릿지의 구조화된 오류와 MoveIt·controller의 원본 오류 코드는 전달 가능한
경로에서 domain·source와 함께 보존합니다. 실패 응답에 필수 오류 정보가 없으면
`INVALID_RESPONSE`로 보고합니다. 숫자만으로는 발생 계통을 구분할 수 없으므로
`error_domain`, `error_source`, `error_name`, `error_msg`와 ROS Action status를 함께 확인합니다.
후보별 검사 실패가 누적된 경우에는 원본 MoveIt 오류가 보존될 수 있으며,
그렇지 않으면 `NO_FEASIBLE_GRASP`로 보고합니다.

프로젝트 오류 상수는 `rwh_beyond_msgs/msg/ErrorCode.msg`에 있습니다.
진단이 없는 BT 실패·취소는 Navigator가 각각 `-1002 / BT_FAILED`,
`-1004 / CANCELED`로 보완합니다.

## 기본 트리와 대상 선택

서버는 BT Navigator의 `rwh_beyond_navigator::DemoNavigator`입니다.
각 단계의 연결은 [실행 흐름](bt-flows.md)과 [플러그인 기능](bt-grasp-stages.md)에 정리했습니다.

현재 YAML의 기본 pick은 `demo_multiview.xml`, place는 `place.xml`입니다.
고정 폭 `demo.xml`은 `behavior_tree`로 명시할 수 있습니다.
대상 텍스트를 생략하면 XML·launch·YAML의 대상 설정을 사용합니다.
전달 경로는 Navigator → Blackboard → SegmentObject → SAM 브릿지입니다.
현재 인스턴스 선택 정책은 `bt_perception.selection_policy=highest_score`입니다.
place는 인식을 다시 수행하지 않습니다. 이동은 별도 Navigation 인터페이스에서 실행합니다.

## Groot 저장 XML 자동 정리 { #groot-xml-filter }

Navigator는 트리 로드 직전에 `attr=""`처럼 **값이 정확히 빈 문자열인 속성**을 자동으로 제거합니다.
Groot가 생략 가능한 숫자 포트를 `opening=""`, `duration=""` 등으로 저장해도
해당 포트를 생략한 것으로 처리해 노드 기본값을 사용합니다.

```xml
<!-- Groot 저장 파일 -->
<OpenGripper opening="" duration=""/>
<!-- Navigator에 전달하는 실행 사본 -->
<OpenGripper/>
```

- 기본 pick XML은 navigator lifecycle 설정·활성화 전에 정리합니다.
- pick·place·사용자 `behavior_tree`는 매 요청의 로드 전에 정리합니다.
- 상대 경로 `<include path="..."/>`와 `ros_pkg` include도 재귀적으로 정리하고,
  실행 사본 안의 include 경로를 정리된 파일의 절대 경로로 연결합니다.
- 원본 XML과 `.bak`은 수정·생성하지 않습니다. 전용 임시 디렉터리에 실행 사본을 만들고 navigator 객체 종료 시 정리합니다.
- `"  "`처럼 공백만 있는 값, `"0"`, `"false"`, `"{capture}"`, 요소의 텍스트·주석은 유지합니다.
  필요한 입력을 빈 값으로 두었다면 속성 제거 뒤에도 필수 입력 검사에서 실패합니다.
- 잘못된 XML·누락된 include·순환 include는 로드를 거부하며 실행하지 않습니다.

원본 또는 include의 내용이 바뀌면 새 실행 사본 경로를 사용하므로
`always_reload_bt_xml=false`여도 **다음 요청**에서 저장한 변경을 읽습니다.
실행 중인 트리는 저장만으로 바뀌지 않습니다. 내용이 같으면 같은 사본 경로를 재사용해 기존 트리 캐시를 유지합니다.
빈 속성을 제거한 경우 로그의 `BT XML filter: removed ... empty attributes`에서 확인할 수 있습니다.
별도의 수동 정리 스크립트 실행은 필요하지 않습니다.

## 취소와 연속 요청

호출 측 ActionClient의 goal handle에서 `cancel_goal_async()`를 사용합니다.
실행 중 목표 교체(preemption)는 지원하지 않습니다. 취소 결과를 받은 뒤 다음 요청을 보냅니다.
취소는 BT 작업과 실행 액션에 전달되고 미부착 임시 장면을 정리합니다.
닫기 시작 이후·부착 이후에는 자동 개방하지 않습니다.

기본 `demo_multiview.xml`의 집기 준비 단계는 `release_previous=true`입니다.
이 트리로 새 pick을 요청하면 기존 부착 대상을 **현재 위치에서 개방·해제**할 수 있습니다.
지정한 배치 위치에서 놓을 때는 먼저 `mode: 2`를 실행합니다.
고정 폭 `demo.xml`은 자동 해제하지 않으며 부착 대상이 남으면 새 pick을 거부합니다.

## 외부 연동과 Groot2

호출 측은 [rwh_beyond_interfaces](https://github.com/ggh-png/rwh_beyond_interfaces)에서
프로젝트의 `moma-mj-lab.repos`에 고정된 리비전을 가져와 `rwh_beyond_msgs`를 빌드하고 source합니다.
ROS Domain·RMW·DDS 네트워크를 맞추고 `/run_demo` 서버가 하나인지 확인합니다.

Groot 노드 정의는 `rwh_beyond_bt/node_models/rwh_beyond_tree_nodes.xml`입니다.
기본 실행 트리는 `demo_multiview.xml`이며 노드 정의 파일 자체는 실행 트리가 아닙니다.
공통 YAML의 `run_demo.enable_groot_monitoring`, `run_demo.groot_server_port`로 모니터링을 설정합니다.
현재 포트는 5555입니다.

[실행 가이드](grasp-pipeline-guide.md) · [집기·배치 흐름](bt-flows.md) · [오류 진단](grasp-troubleshooting.md)
