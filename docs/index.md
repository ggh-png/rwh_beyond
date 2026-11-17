# BEYOND Grasp

D405/D435 RGB-D로 텍스트에 해당하는 물체를 찾고, SAM 3·GraspGenX·MoveIt 2를 연결해
PiPER 로봇의 접근·그리퍼 닫기·리프트·복귀를 실행하는 ROS 2 프로젝트입니다.
심과 실물은 공통 YAML의 기본 집기 트리 `demo_width.xml`을 사용합니다.

## 시작하기

1. [시스템 구성](project-overview.md): 로봇 PC와 GPU 서버의 역할
2. [환경과 빌드](project-containers.md): workspace와 의존 저장소
3. [실행 가이드](grasp-pipeline-guide.md): runner 시작부터 Action 호출까지
4. [공통 BT와 설정](real-bt.md): 관측 자세·그리퍼 설정 변경
5. [Action 인터페이스](bt-operation.md): 대상 지정·결과·취소

## 개발과 디버깅

외부 서버의 접속·컨테이너·모델 시작/중지는 [추론 서버 운영](inference-server-operation.md)을 참고하세요.

- [데이터·좌표계](project-data.md) · [코드 안내](project-code-map.md)
- [모델별 ROS 인터페이스](model-bridges.md) · [SAM 3](sam3-perception.md) · [GraspGenX](graspgenx-model.md)
- [Runner 장치 설정](runner-device-settings.md) · [D405와 TF](d405-sensors.md) · [실물 제어](piper-hardware-moveit.md)
- [RViz](grasp-visualization.md) · [BT 구조](bt-architecture.md) · [플러그인 기능·포트](bt-grasp-stages.md)

기본 집기는 Pick·리프트·복귀이며 mode 2로 부착 물체의 배치·해제를 요청할 수 있습니다. Navigation은 별도 연동 대상입니다.
Action의 성공은 시퀀스 완료를 뜻하며 실제 잡힘 여부는 판정하지 않습니다.

## BT 구현 안내

- [집기·배치 BT 흐름](bt-flows.md): 기본·고정 폭 집기, 배치·해제, 후보 선택 플로우
- [플러그인별 기능·포트](bt-grasp-stages.md): 등록된 16개 노드의 기능·입출력·직접 호출
- [BT와 실행 구조](bt-architecture.md): 프로세스 경계·Blackboard·계획 상태·취소 처리

## 파지 튜닝과 오류 진단

- [손목 회전·MoveIt 계획](grasp-motion-tuning.md)
- [추정 폭으로 닫기](width-aware-bt.md)
- [후보 탈락·오류 진단](grasp-troubleshooting.md)
- [D435 BT 연결·장착 설정](d435-bt-demo.md)
