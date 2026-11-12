# BEYOND Grasp

D405 RGB-D로 텍스트에 해당하는 물체를 찾고, SAM 3·GraspGenX·MoveIt 2를 연결해
PiPER 로봇의 접근·그리퍼 닫기·리프트·복귀를 실행하는 ROS 2 프로젝트입니다.
심과 실물은 동일한 `rwh_beyond_bt/behavior_trees/demo.xml`을 사용합니다.

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
- [RViz](grasp-visualization.md) · [BT 구조](bt-architecture.md) · [단계별 포트](bt-grasp-stages.md)

현재 시나리오는 Pick·리프트·복귀까지입니다. Place와 Navigation 연동은 구현 범위에 포함하지 않습니다.
Action의 성공은 시퀀스 완료를 뜻하며 실제 잡힘 여부는 판정하지 않습니다.
