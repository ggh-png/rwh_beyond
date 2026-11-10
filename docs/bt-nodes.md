# BT 노드 사용법

그리퍼는 **정수 mm**, 팔 위치는 **m**, 팔 자세는 **quaternion**으로 입력합니다.
[전체 구조](bt-architecture.md) · [실행과 Groot2](bt-operation.md)

## 노드 목록

| 노드 | 바꿀 수 있는 값 | 동작 |
| --- | --- | --- |
| `SetGripper` | 개방 폭, 단위, 시간 | 지정한 폭으로 그리퍼 이동 |
| `MoveTcp` | 위치, 자세, 기준 프레임 | MoveIt pose goal 실행 |
| `CloseGripper` | 고정 | 0 mm로 닫기, 3초 |
| `OpenGripper` | 고정 | 100 mm로 열기, 3초 |
| `MoveHome` | 고정 | joint1~joint6 = 0 rad |
| `MoveTcpUp` | 고정 | base_link +Z 0.30 m, 현재 자세 유지 |

모든 노드는 Nav2의 `server_name`, `server_timeout` 포트를 상속합니다.

## SetGripper — 개방 폭 지정

**50 mm 열기:**

```xml
<SetGripper opening="50" duration="2.5"/>
```

| 입력 | 필수 / 기본값 | 단위와 범위 |
| --- | --- | --- |
| `opening` | 필수 | 총 개방 폭, 정수 0~100 mm |
| `unit` | `mm` | `mm` 또는 `percent` |
| `duration` | `3.0` | 초, 0 초과 3600 이하 |

`opening="50"`은 **양쪽 손가락 사이 총 50 mm**입니다.
Controller에는 `gripper=0.05` m로 전달합니다.
`0`이면 닫힘, `100`이면 현재 모델의 최대 개방 폭입니다.

Blackboard 값도 연결할 수 있습니다.

```xml
<SetGripper opening="{gripper_opening_mm}"/>
```

`unit="percent"`를 명시하면 0~100% 개방률로 해석합니다.
현재 모델의 최대 폭이 100 mm여서 두 단위의 숫자가 일치합니다.
소수·범위 초과·잘못된 단위·유효하지 않은 시간은 goal 전송 전에 거부합니다.

## MoveTcp — 위치와 자세 지정

**base_link 기준 위치와 자세를 함께 입력:**

```xml
<MoveTcp x="0.25" y="0" z="0.35"
         qx="0" qy="0" qz="0" qw="1"/>
```

| 입력 | 필수 / 기본값 | 의미 |
| --- | --- | --- |
| `x`, `y`, `z` | 필수 | 목표 위치, 단위 m |
| `qx`, `qy`, `qz`, `qw` | 생략 가능 | 목표 quaternion, x/y/z/w 순서 |
| `frame_id` | `base_link` | 위치와 자세의 기준 좌표계 |
| `tcp_link` | `tcp_link` | 제어할 TCP link |
| `relative` | `false` | true이면 xyz를 현재 위치에 더함 |

### 자세를 유지하려면 quaternion을 생략합니다

```xml
<MoveTcp x="0.25" y="0" z="0.35"/>
```

Quaternion은 **네 값을 모두 입력하거나 모두 생략**합니다.
입력값은 유한해야 하며 norm이 `1e-12` 이상이어야 합니다.
유효한 quaternion은 정규화하고, 일부 성분만 입력하거나 0 quaternion이면 거부합니다.

### 상대 이동은 기준 프레임의 축을 따릅니다

```xml
<!-- 현재 위치에서 base_link Z축으로 0.30 m 상승 -->
<MoveTcp x="0" y="0" z="0.30" relative="true"/>
```

| 설정 | 위치 | 자세 |
| --- | --- | --- |
| `relative="false"` | 기준 프레임의 절대 위치 | 입력 quaternion 또는 현재 자세 |
| `relative="true"` | 현재 위치 + 기준 프레임 축 방향 offset | 입력 quaternion 또는 현재 자세 |

!!! note "relative는 위치에만 적용"
    입력 quaternion은 항상 `frame_id` 기준의 **절대 자세**입니다.
    현재 자세에 더하는 회전 증분으로 해석하지 않습니다.

상대 위치 또는 자세 유지에는 TF가 필요합니다.
절대 위치와 quaternion을 모두 지정하면 현재 TCP TF를 조회하지 않습니다.
목표 프레임과 도달 가능성은 MoveIt이 검증합니다.

### Blackboard에서 pose를 받을 수도 있습니다

```xml
<MoveTcp x="{target_x}" y="{target_y}" z="{target_z}"
         qx="{target_qx}" qy="{target_qy}"
         qz="{target_qz}" qw="{target_qw}"/>
```

현재 `RunDemo` goal에는 좌표 필드가 없습니다. XML에 상수를 쓰거나,
다른 BT 노드 등으로 이 blackboard 값을 먼저 채워야 합니다.

### MoveIt 계획 설정

| 항목 | 값 |
| --- | --- |
| Planning group | `arm` |
| Pipeline / planner | OMPL / `RRTConnectkConfigDefault` |
| 계획 시간 | 10초 |
| 속도 / 가속도 비율 | 각각 0.1 |
| 위치 / 자세 허용오차 | 5 mm / 0.01 rad |

이 노드는 pose goal을 실행합니다. 직선 Cartesian 궤적을 보장하지 않으며,
도달 불가·충돌 목표를 임의 관절 목표로 바꾸지 않습니다.

## 실패와 그리퍼 접촉 처리

### 새 노드의 성공 판정

`MoveTcp`와 `SetGripper`는 **ROS action status와 application error code**를 함께 확인합니다.
오류나 cancel이면 BT FAILURE를 반환합니다.
잘못된 literal 값은 XML 로드 단계에서 거부될 수 있으며,
잘못된 runtime 입력이나 필요한 TF의 조회 실패는 goal 전송 전에 실패합니다.

기존 고정 노드 4개는 Nav2 기본 결과 처리에 의존하므로 같은 검증이 모두 적용된 것은 아닙니다.

### MuJoCo의 닫기 성공 조건

Trajectory의 마지막 시점 이후, 다음 중 하나를 만족하면 bridge가 성공을 반환합니다.

1. 위치와 속도가 허용오차 안에 들어옴.
2. 닫기 목표가 약 0 m (`desired <= 1e-4`)이고, 양쪽 손가락 force가 threshold 이상임.

Force는 MuJoCo `actuator_force`의 절댓값으로 읽습니다.
두 값 중 작은 값이 threshold 이상이어야 합니다.

| 파라미터 | 기본값 | 설정 의미 |
| --- | --- | --- |
| `gripper_contact_force_threshold` | `1.0` | 양쪽 손가락 force threshold |
| 같은 값이 0 이하 | — | force 기반 대체 성공 조건 비활성화 |

이는 actuator force를 통한 접촉 추정입니다. 물체가 안정적으로 파지됐는지를
독립적으로 검증하는 조건은 아닙니다.
