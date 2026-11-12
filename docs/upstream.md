# 구성 요소와 출처

| 구성 요소 | 역할·출처 |
|---|---|
| ROS 2 Jazzy | ROS 노드·Topic·Service·Action |
| Nav2 Behavior Tree Navigator | `DemoNavigator`의 BT 실행 기반 |
| BehaviorTree.CPP | BT XML·플러그인·Blackboard |
| MoveIt 2 | IK·충돌 검사·경로 계획·궤적 실행 |
| MuJoCo / moma-mj-lab | PiPER 시뮬레이션과 공통 로봇·카메라 모델 |
| AgileX agx_arm_ros | PiPER driver·description·MoveIt 패키지 |
| RealSense realsense-ros | D405 driver와 장치 내부 TF |
| SAM 3 | 텍스트 조건 이미지 분할 |
| GraspGenX | 물체 점군 기반 파지 후보 생성 |

의존 ROS 저장소는 `moma-mj-lab.repos`, GPU 모델 빌드 소스와 버전은 `deploy/`의 Dockerfile과
설정에서 관리합니다. 각 구성 요소의 라이선스와 모델 가중치 사용 조건을 따릅니다.

프로젝트에서 작성한 코드는 ROS 모델 bridge, coordinator, BT 플러그인, navigator,
센서·실물 실행 통합입니다. pretrained 모델은 별도로 학습하지 않습니다.
D405 mesh 출처와 라이선스는 `moma_description/meshes/D405-SOURCE.md`, `LICENSE.realsense`에 있습니다.

- [SAM 3](https://github.com/facebookresearch/sam3)
- [GraspGenX](https://github.com/NVlabs/GraspGenX)
- [AgileX ROS](https://github.com/agilexrobotics/agx_arm_ros)
- [RealSense ROS](https://github.com/realsenseai/realsense-ros)
- [MoveIt 2](https://github.com/moveit/moveit2)

[코드 안내](project-code-map.md) · [환경과 빌드](project-containers.md)
