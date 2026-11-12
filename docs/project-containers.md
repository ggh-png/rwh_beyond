# 환경과 빌드

## ROS workspace

현재 작업 컨테이너는 `moma_mj_lab_1.1.2_hostcan`, ROS workspace는 `/root/ros2_ws`입니다.
ROS 2 Jazzy와 MoveIt 2를 사용합니다. `/root/ros2_ws`는 컨테이너 내부 파일이며
호스트 bind mount인 `/work`와 구분합니다.

| 저장소 | 역할 |
|---|---|
| `rwh_beyond` | BT·Action·모델 bridge·파지 coordinator·bringup |
| `moma-mj-lab` | MuJoCo, 로봇·카메라 모델, 공통 MoveIt 설정 |
| `agx_arm_ros` | PiPER 공식 ROS driver·description·MoveIt 패키지 |
| `ros_runner` | 심·실물 의존 실행과 종료 관리 |
| `realsense-ros` | D405 ROS driver |

의존 저장소는 `moma-mj-lab.repos`에서 관리합니다. 저장소별 인증·submodule과 모델 가중치는 별도로 준비합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
vcs import --recursive --skip-existing src < src/rwh_beyond/moma-mj-lab.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

기반 컨테이너, GPU runtime, RealSense SDK는 장비에 준비되어 있어야 합니다.
이 저장소의 ROS 빌드만으로 GPU 모델 가중치와 장치 권한까지 구성되지는 않습니다.

## 모델 실행 환경

SAM 3와 GraspGenX는 각각 별도 GPU 모델 프로세스로 실행합니다.
ROS 컨테이너에 두 모델을 import하지 않고 ROS bridge가 모델 서버에 요청합니다.
배포 소스는 저장소의 `deploy/sam3-docker/`, `deploy/graspgenx-docker/`입니다.

- SAM 3: 체크포인트 `sam3.pt`, 이미지 분할 서버, 기본 포트 5567.
- GraspGenX: pretrained checkpoint와 `piper_hand` assets, 기본 포트 5566.
- 외부 서버 사용 시 SSH 터널 또는 접근 가능한 endpoint를 먼저 준비합니다.
- bridge가 사용할 실제 주소는 [런타임 설정](piper-runtime.md)에서 지정합니다.

서버 소스가 이미지에 복사된 배포에서는 소스 수정 후 이미지 재빌드와 컨테이너 재생성이 필요합니다.
체크포인트·캐시·로그 볼륨은 소스와 별도로 보관합니다.

## 변경 반영

Python과 XML은 symlink install을 사용합니다. 실행 중인 ROS 노드는 코드 변경 후 재시작합니다.
Action 정의나 C++ 플러그인을 변경하면 관련 패키지와 호출 측을 다시 빌드합니다.
카메라 모델은 `moma-mj-lab`에서 수정·빌드하며 MoveIt과 BT를 재시작합니다.

[실행 가이드](grasp-pipeline-guide.md) · [런타임 설정](piper-runtime.md)
