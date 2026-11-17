# 환경과 빌드

## ROS workspace

현재 작업 컨테이너는 `moma_mj_lab_1.1.2_hostcan`, ROS workspace는 `/root/ros2_ws`입니다.
ROS 2 Jazzy와 MoveIt 2를 사용합니다. `/root/ros2_ws`는 컨테이너 내부 파일이며
호스트 bind mount인 `/work`와 구분합니다.

| 저장소 | 역할 |
|---|---|
| `rwh_beyond` | BT·Action·모델 bridge·장치 전송·bringup |
| `moma-mj-lab` | MuJoCo, 로봇·카메라 모델, 공통 MoveIt 설정 |
| `agx_arm_ros` | PiPER 공식 ROS driver·description·MoveIt 패키지 |
| `ros_runner` | 심·실물 의존 실행과 종료 관리 |
| `realsense-ros` | D405/D435 ROS driver |

의존 저장소는 `moma-mj-lab.repos`에서 관리합니다. 저장소별 인증·submodule과 모델 가중치는 별도로 준비합니다.
아직 checkout이 없다면 ROS 컨테이너에서 프로젝트를 먼저 받습니다. 이미 있다면 clone을 반복하지 않습니다.

```bash
mkdir -p /root/ros2_ws/src
git clone https://github.com/ggh-png/rwh_beyond.git /root/ros2_ws/src/rwh_beyond
```

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

외부 서버의 접속·컨테이너·모델 시작/중지는 [추론 서버 운영](inference-server-operation.md)을 참고하세요.

SAM 3와 GraspGenX는 각각 별도 GPU 모델 프로세스로 실행합니다.
ROS 컨테이너에 두 모델을 import하지 않고 ROS bridge가 모델 서버에 요청합니다.
배포 소스는 저장소의 `deploy/sam3-docker/`, `deploy/graspgenx-docker/`입니다.

- SAM 3: 체크포인트 `sam3.pt`, 이미지 분할 서버, 기본 포트 5567.
- GraspGenX: pretrained checkpoint와 `piper_hand` assets. ROS 공통 연결은 5566이며
  서버 Compose 템플릿은 5556이므로 [포트 매핑](inference-server-operation.md)을 맞춥니다.
- 외부 서버 사용 시 SSH 터널 또는 접근 가능한 endpoint를 먼저 준비합니다.
- bridge가 사용할 실제 주소는 [런타임 설정](piper-runtime.md)에서 지정합니다.

서버 소스가 이미지에 복사된 배포에서는 소스 수정 후 이미지 재빌드와 컨테이너 재생성이 필요합니다.
체크포인트·캐시·로그 볼륨은 소스와 별도로 보관합니다.

## 변경 반영

Python과 XML은 symlink install을 사용합니다. 실행 중인 ROS 노드는 코드 변경 후 재시작합니다.
Action 정의나 C++ 플러그인을 변경하면 관련 패키지와 호출 측을 다시 빌드합니다.
기본 카메라 모델은 `moma-mj-lab`에서 관리합니다. 실물 runner의 장착 YAML만 수정하면
재빌드 없이 장치와 BT runner 재활성화로 반영합니다. [장착 설정](runner-device-settings.md)

### Navigator 링크 오류 { #bt-common-link }

`cannot find -lrwh_beyond_bt_common`이 발생하면 `rwh_beyond_navigator`가 BT 공통
라이브러리를 이름만으로 링크하고 있지 않은지 확인합니다. BT 패키지는 설치된 CMake
타깃 `rwh_beyond_bt::rwh_beyond_bt_common`을 내보내므로 navigator에서는 다음과 같이
사용합니다.

```cmake
find_package(rwh_beyond_bt REQUIRED)
target_link_libraries(${PROJECT_NAME} rwh_beyond_bt::rwh_beyond_bt_common)
```

라이브러리의 런타임 검색 경로가 있어도 `-l`의 **빌드 시** 검색 경로가 자동으로
추가되는 것은 아닙니다. 이름공간 타깃은 설치된 라이브러리 위치와 의존성을 전달합니다.
소스 수정 후에는 의존 패키지부터 다시 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --packages-up-to rwh_beyond_navigator
source install/setup.bash
colcon test --packages-select rwh_beyond_navigator
colcon test-result --test-result-base build/rwh_beyond_navigator --verbose
```

이 명령은 navigator와 그 의존성만 빌드합니다. 나머지 패키지까지 필요하면 일반
`colcon build`를 실행합니다. 실행 중인 ROS 노드는 재빌드 후 재시작해야 합니다.

[실행 가이드](grasp-pipeline-guide.md) · [런타임 설정](piper-runtime.md)
