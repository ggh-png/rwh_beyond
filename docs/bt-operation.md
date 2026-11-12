# Manipulation Action 인터페이스

외부 Navigation 시스템은 `/run_demo`를 호출해 PiPER의 공통 파지 시나리오를 실행할 수 있습니다.
Navigation 자체는 이 Action이 수행하지 않습니다.

- 이름: `/run_demo`
- 타입: `rwh_beyond_msgs/action/RunDemo`
- 서버: BT Navigator의 `rwh_beyond_navigator::DemoNavigator`

## Goal

| 필드 | 타입 | 의미 |
|---|---|---|
| `target_text` | string | 이번 실행에서 찾을 물체 설명. 빈 값은 YAML 기본값 |
| `behavior_tree` | string | 실행할 XML 경로. 빈 값은 공통 `demo.xml` |

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{target_text: "blue cup"}' --feedback
```

`target_text`는 해당 실행에만 적용됩니다. 다음 요청에서 생략하면 이전 대상이 아닌 YAML 기본값을 사용합니다.
Navigator → BT Blackboard → SegmentObject → coordinator → SAM 3로 전달됩니다.
기본 인스턴스 선택 정책은 `unique`이므로 같은 설명에 여러 물체가 해당하면 모호성으로 실패할 수 있습니다.

## Feedback와 Result

| 필드 | Feedback | Result | 의미 |
|---|---|---|---|
| `current_stage`, `current_node` | 사용 | — | 실행 중인 단계·BT 노드 |
| `failed_stage`, `failed_node` | — | 사용 | 실패한 단계·BT 노드 |
| `moveit_state` | 사용 | 사용 | 전달받은 MoveIt 상태 |
| `error_code` | 사용 | 사용 | 0은 성공, 오류는 원본 signed code 또는 BT 코드 |
| `error_name`, `error_source`, `error_msg` | 사용 | 사용 | 오류 이름·발생 위치·설명 |

ROS Action의 최종 status와 result를 함께 확인합니다. `SUCCEEDED`와 `error_code=0`은
시퀀스 완료를 뜻하며 물체 잡힘을 확인한 결과는 아닙니다.
MoveIt·controller 오류 코드는 원본 부호를 유지합니다. BT 내부 오류는 -1000 이하를 사용합니다.

## 취소와 연속 요청

호출 측 ActionClient의 goal handle에서 `cancel_goal_async()`를 사용합니다.
실행 중 요청을 새 목표로 교체하는 preemption은 지원하지 않습니다. 취소 결과를 받은 뒤 새 목표를 보냅니다.
동작 취소 시 궤적 취소와 임시 장면 정리를 요청하며 이미 부착된 대상은 유지합니다.
부착 대상이 남아 있으면 새 집기 요청의 준비 검사에서 거부합니다.

## 외부 연구실 연동

호출 측도 동일한 `rwh_beyond_msgs`를 빌드하고 ROS 환경을 source합니다.
ROS Domain, RMW 및 DDS 네트워크를 맞추고 `/run_demo` 서버가 하나인지 확인합니다.
물체 설명은 짧은 명사구로 전달합니다. 호출 측에서 결과와 취소 처리를 구현합니다.

## Groot2

BT 노드 모델은 `rwh_beyond_bt/node_models/rwh_beyond_tree_nodes.xml`, 기본 트리는 `demo.xml`입니다.
공통 YAML의 `run_demo.enable_groot_monitoring`, `run_demo.groot_server_port`로 모니터링을 설정합니다.
현재 포트 기본값은 5555입니다. Groot2 연결은 시퀀스의 성공이나 물체 잡힘을 보증하지 않습니다.

[실행 가이드](grasp-pipeline-guide.md) · [단계별 Action](bt-grasp-stages.md)
