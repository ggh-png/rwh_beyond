# BT와 실행 구조

`DemoNavigator`가 `/run_demo`를 제공하고, `demo.xml`이 작업 순서를 결정합니다.
각 BT Action 플러그인은 coordinator의 단계별 ROS Action을 호출합니다.
coordinator는 관측·계획·장면 상태를 보관하고 모델 추론과 MoveIt을 연결합니다.

## 역할 분리

| 계층 | 책임 |
|---|---|
| Navigator | 트리 로딩, 요청별 target_text, feedback/result, 취소 |
| BT XML | 단계 순서, 관측 상승량·각도 |
| BT 플러그인 | 포트 데이터와 ROS Action 변환, 오류 전달 |
| Coordinator | 관측·계획 ID·장면·backend·취소 상태 관리 |
| 모델 bridge | ROS 메시지 검증, ZMQ 요청·응답 변환 |
| MoveIt/controller | 경로 계획·충돌 검사·궤적 실행 |

home·열기·관측은 `GraspStep`으로 `/grasp_step`을 호출합니다.
분할부터 리프트까지는 목적별 플러그인과 `/grasp/*` endpoint를 사용합니다.
닫기는 `AttachObject` 내부에서 수행하므로 기본 트리에 별도 `CloseGripper`를 추가하지 않습니다.

## 데이터와 상태

BT Blackboard는 마스크 Image, PointCloud2, GraspCandidate 배열, 계획 ID와 pregrasp를 전달합니다.
coordinator는 같은 관측에서 생성한 데이터인지 ID와 내용을 검사합니다.

계획 단계 순서는 `selected → pregrasp → approached → attached → lifted`입니다.
계획 ID 불일치·단계 생략·중복 실행은 거부합니다.

## 공통 backend 경계

심·실물은 XML과 시나리오 이동 구현을 공유합니다. backend 준비 확인·피드백·controller는
각 환경을 사용합니다. 실물은 MuJoCo 검증 서비스를 호출하지 않습니다.
통신·충돌·팔 동작 완료 확인과 물체 잡힘 판정은 구분합니다. 현재 물체 잡힘은 판정하지 않습니다.

실패·취소 시 미부착 계획의 임시 대상 제외와 손가락 충돌 허용 규칙을 정리합니다.
부착된 대상은 이후 경로의 충돌 검사에 포함하고 자동으로 놓지 않습니다.

[공통 시나리오](real-bt.md) · [포트와 Action](bt-grasp-stages.md)
