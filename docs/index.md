# RWH 프로젝트 안내

이 저장소에는 **PiPER 로봇 팔의 모델 기반 집기**와 **Husky의 SLAM·Nav2 운용**이 함께 있습니다.
지금 SAM 3·GraspGenX·MoveIt 실험을 이해하려면 아래 PiPER 안내부터 읽으세요.

## PiPER 집기: 이 순서로 읽으세요

| 순서 | 문서 | 읽고 나면 알 수 있는 것 |
|---|---|---|
| 1 | [전체 흐름 한눈에](project-overview.md) | 세 컨테이너와 네 저장소가 맡은 일, 한 번의 집기 순서 |
| 2 | [컨테이너는 어떻게 구축했나](project-containers.md) | 이미지·모델·볼륨·GPU·네트워크·시작 스크립트의 관계 |
| 3 | [모델과 어떤 데이터를 주고받나](project-data.md) | RGB → mask → XYZ → 파지 행렬 → 로봇 목표의 변환 |
| 4 | [처음 터미널부터 실행](grasp-pipeline-guide.md) | Docker 접근, 모델 시작, runner, RViz, 관측·집기·종료 |
| 5 | [코드를 따라 읽기](project-code-map.md) | BT XML에서 SAM·GraspGenX·MoveIt 호출까지 파일과 함수 추적 |

## 지금 실물 PiPER를 시험하려면

- [새 ROS 컨테이너·CAN·모델 네트워크](piper-runtime.md): 현재 작업 환경과 이미지 보존·복구
- [실물 MoveIt 실행·티칭 전환·설정 비교](piper-hardware-moveit.md): runner부터 상태·Plan·Execute 구분
- [11월 11일 변경과 검증 기록](work-log-2026-11-11.md): 완료한 것과 아직 지원하지 않는 것

현재 실물에서 검증한 범위는 CAN 피드백·MoveIt 기동입니다. 설정 통일 후의 계획 검증은
CAN driver를 제외한 격리 환경에서 수행했으며, 실물 이동 성공으로 집계하지 않습니다.

## 필요한 부분 바로 찾기

- **모델별 ROS 패키지·서버는 어디에 있나?** [ROS bridge 분리 구조](model-bridges.md)

- **두 모델을 구체적으로 어떻게 썼나?** [SAM 3 구현](sam3-perception.md), [GraspGenX 구현](graspgenx-model.md)
- **다른 물체를 지정하고 싶다:** [대상 변경](grasp-pipeline-guide.md#target)
- **마스크·점군·파지 후보가 안 보인다:** [RViz 토픽과 설정](grasp-visualization.md)
- **BT 노드를 수정하고 싶다:** [노드별 포트와 endpoint](bt-grasp-stages.md), [헤더·CPP 구성](bt-code-style.md)
- **추론만 호출하고 싶다:** [독립 추론 API](project-data.md#standalone)
- **이전 실험에서 무엇을 검증했나:** [검증 결과와 제한](project-overview.md#validation), [과거 SAM 실험 기록](sam3-history.md)
- **Husky를 주행시키고 싶다:** [Husky 시작 안내](husky-guide.md)

이 안내는 2026-11-11의 workspace 소스를 기준으로 합니다. 기본 집기 경로는 실제 SAM·GraspGenX를 사용하는
**MuJoCo 시뮬레이션**입니다. 실물 PiPER driver/MoveIt 실행 경로는 별도로 존재합니다.
