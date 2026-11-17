# 점군 추정 폭으로 닫기

기본 집기 트리 `demo_width.xml`은 분할된 물체 점군에서 닫기 폭을 추정합니다.
GraspGenX 응답은 자세와 점수이며 폭은 BT가 계산합니다.

## 폭 계산과 출력

후보의 모델 로컬 X축(손가락 닫힘 방향)으로 점군을 변환하고 2·98백분위 범위를 구합니다.
파지 중심축 양쪽의 범위를 대칭으로 감싸는 전체 폭을 `grasp_width`로 사용합니다.
최소 100개의 유한한 점, 8mm 이상의 유효 폭과 중심축 양쪽의 관측이 필요합니다.

```text
close_width = min(grasp_width - grasp_width_closing_inset, gripper_open_width)
```

`gripper_open_width`의 상한은 0.10m입니다. 추정 폭이 상한을 넘더라도 `grasp_width`에는
원래 추정값을 저장하고 명령 폭만 제한합니다. 폭을 제한했다는 사실이 실제 파지 가능성을 뜻하지는 않습니다.
폭 추정에 실패한 후보는 탈락하며 고정 폭으로 대체하지 않습니다.

| 출력 | 위치 | 의미 |
|---|---|---|
| `grasp_width` | SelectFeasibleGrasp 출력·Blackboard | 관측 전체 폭(m) |
| `close_width` | SelectFeasibleGrasp 출력·GraspPlan | 보정·상한 적용 후 명령 간격(m) |

`AttachObject`는 선택된 GraspPlan의 폭으로 닫습니다. `/run_demo` Result에는 폭 필드가 없습니다.
선택 로그에는 `close_width`가 출력됩니다. 부분 깊이 관측은 가려진 형상을 놓칠 수 있으며,
이 추정과 시간 기반 닫기는 힘 제어나 잡힘 검출이 아닙니다.

## 닫기 보정값

`rwh_beyond_bringup/config/bt_demo.yaml`에서 설정합니다.

```yaml
bt_navigator:
  ros__parameters:
    bt_motion:
      grasp_width_closing_inset: 0.005
      gripper_open_width: 0.10
      gripper_close_duration: 2.0
```

inset은 전체 간격에서 빼는 양이며 허용 범위는 0~0.005m입니다.
현재 실행 YAML은 0.005m, 코드의 생략 시 기본값은 0.002m입니다.
추정 폭이 40mm라면 현재 설정의 명령 간격은 35mm입니다.
범위를 벗어난 설정은 후보 검사 전에 실패합니다. 수정 후 BT를 재활성화합니다.

## 실행과 반복 집기

```bash
ros2 action send_goal /run_demo rwh_beyond_msgs/action/RunDemo \
  '{mode: 1, target_text: "yellow block"}' --feedback
```

기본 XML의 `CheckGraspReady release_previous="true"`는 남은 대상을 **현재 위치에서**
개방하고, 개방 피드백 확인 후 부착을 해제합니다. 알 수 없는 부착 물체는 거부하며,
개방 실패 시 새 집기를 중단합니다. 배치 위치로 이동하는 기능은 `mode: 2`를 사용합니다.

고정 폭이 필요하면 현재 지원되는 대체 트리 `demo.xml`을 `behavior_tree`에 지정합니다.
이 트리는 `gripper_close_width=0.04m`를 사용하고, 기존 부착이 있으면 새 집기를 거부합니다.
[트리 선택과 차이](bt-flows.md#fixed-width-pick) · [오류 진단](grasp-troubleshooting.md)
