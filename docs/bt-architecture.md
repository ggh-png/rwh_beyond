# BT와 실행 구조

`DemoNavigator`가 `/run_demo`를 제공하고 BT가 데이터와 작업 흐름을 소유합니다.
별도 coordinator 프로세스와 단계 중계 액션은 없습니다.

```text
BT: RGB-D → SAM → 물체 점군 → 파지 추론 → TCP 변환·후보 선택
    → 사전 접근 → 직선 접근 → 닫기·부착 → 리프트 → 복귀
                   ↓ 직접 호출
MoveIt: IK / FK / 경로·충돌 검사 / Planning Scene / ExecuteTrajectory
Controller: FollowJointTrajectory
```

| 구성 | 역할 |
|---|---|
| Navigator | 트리 로딩, 요청별 대상, 결과·오류·취소 |
| BT XML | 단계 순서, 관측·접근·리프트 설정 |
| BT 인식 노드 | 촬영·SAM 호출·점군 생성·파지 브릿지 호출 |
| BT 모션 노드 | TCP 변환·후보 선택·폭·목표 생성, MoveIt/그리퍼 직접 호출 |
| BT 공통 MotionContext | 클라이언트·TF·피드백·충돌 장면 자원 공유; 별도 ROS 서버 없음 |
| 모델 브릿지 | 모델 프로토콜·ROS 메시지 변환 |
| MoveIt/controller | 계획·충돌 검사·실행 |

Blackboard는 촬영·마스크·점군·후보와 typed GraspPlan을 전달합니다.
촬영 유효성, 단계 순서, 손목 회전 제한, 경로 목표 도달, 피드백 최신성 검사를 유지합니다.
동시에 두 BT 분기가 같은 로봇을 명령하는 것은 거부합니다.

실패·취소 시 실행 액션을 취소하고 미부착 물체·임시 충돌 허용·점군 제외 영역을 정리합니다.
닫기 중단으로 잡힘 여부가 불명확하거나 이미 부착된 물체는 자동으로 놓지 않습니다.
심·실물은 같은 XML과 BT 코드를 쓰며 실물은 전송 준비·URDF 시작 한계·실측 도달을 확인합니다.
물체의 실제 잡힘은 판정하지 않습니다.

[포트와 직접 호출](bt-grasp-stages.md) · [공통 설정](real-bt.md)
