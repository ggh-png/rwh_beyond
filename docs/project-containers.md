# 환경과 빌드

## 배포 이미지 불러오기와 컨테이너 생성 { #deployment-containers }

아래 명령은 **컨테이너 밖의 Linux 호스트 터미널**에서 실행합니다.
확인한 배포 폴더는 `/home/ggh/RWH/beyond`이며, 다른 PC에서는 전달받은 폴더 경로로 바꿉니다.
이 배포 폴더는 Git 저장소의 `deploy/`와 별개입니다. `config/`, `scripts/`,
`compose.models.yaml`, `SHA256SUMS`를 이미지 파일과 함께 준비합니다.

20261117 배포본은 레지스트리에 게시한 이미지가 아니라 **이미지 아카이브를 전달하는 방식**입니다.
아래 `docker.io/rwh_beyond/...`는 아카이브를 불러왔을 때 등록되는 로컬 태그이며,
`docker pull`로 다운로드하는 주소로 사용하지 않습니다.
이 절은 배포 폴더를 전달받은 상태에서 시작합니다.

| 배포 파일 | 불러오는 이미지 | 생성할 컨테이너 |
|---|---|---|
| `rwh_beyond_robot_20261117.tar.gz` | `docker.io/rwh_beyond/robot:20261117` | `rwh_beyond_robot` |
| `rwh_beyond_models_20261117.tar.gz` | `docker.io/rwh_beyond/sam3:20261117`, `docker.io/rwh_beyond/graspgenx:20261117` | `rwh_beyond_sam3`, `rwh_beyond_graspgenx` |

### 1. 이미지 파일 불러오기

Docker를 사용하는 호스트:

```bash
cd /home/ggh/RWH/beyond
sha256sum --check SHA256SUMS
docker load --input rwh_beyond_robot_20261117.tar.gz
docker load --input rwh_beyond_models_20261117.tar.gz
docker image ls --filter reference='docker.io/rwh_beyond/*:20261117'
```

현재 개발 호스트는 `docker` 명령이 Podman으로 연결되는 환경입니다.
Podman을 명시해서 사용할 때는 다음 명령을 사용합니다.

```bash
cd /home/ggh/RWH/beyond
CONTAINER_ENGINE=podman ./scripts/load-images.sh
```

이 스크립트는 체크섬 확인 → 두 아카이브 `load` → 세 이미지 ID 확인을 수행합니다.
Docker에서도 `CONTAINER_ENGINE=docker ./scripts/load-images.sh`로 같은 작업을 할 수 있습니다.
로봇 PC와 GPU 서버를 분리하면 각 호스트에 필요한 아카이브만 `load`합니다.

### 2. 로봇 컨테이너 생성·접속

현재 호스트의 Podman에서는:

```bash
cd /home/ggh/RWH/beyond
CONTAINER_ENGINE=podman ./scripts/robot.sh start
CONTAINER_ENGINE=podman ./scripts/robot.sh status
CONTAINER_ENGINE=podman ./scripts/robot.sh shell
```

Docker를 사용하는 호스트에서는 대신:

```bash
cd /home/ggh/RWH/beyond
CONTAINER_ENGINE=docker ./scripts/robot.sh start
CONTAINER_ENGINE=docker ./scripts/robot.sh shell
```

**`start`가 최초 호출이면 `run -d`로 생성하고, 같은 이름의 컨테이너가 있으면 `start`로 재시작합니다.**
로봇·runner는 자동 실행하지 않으며 이미지의 `sleep infinity`로 대기합니다.
셸 접속은 이미지의 entrypoint를 거쳐 ROS 환경을 불러옵니다.

| 생성 설정 | `robot.sh`가 적용하는 옵션·경로 |
|---|---|
| 기본 실행 | `run -d --name rwh_beyond_robot --network host --ipc host --init --security-opt label=disable` |
| ROS domain | `ROS_DOMAIN_ID` 환경변수, 생략 시 0 |
| runner 설정 | `config/runners.yaml` → `/opt/rwh_beyond/config/runners.yaml` 읽기 전용 |
| BT 설정 | `config/bt_demo.yaml` → `/root/ros2_ws/src/rwh_beyond/rwh_beyond_bringup/config/bt_demo.yaml` 읽기 전용 |
| USB·장치 | 존재하는 `/dev/bus/usb`, `/run/udev`, `/dev/video*`, `/dev/media*`, `/dev/dri/renderD*` 연결 |
| 화면 | `DISPLAY`와 X11 socket이 있으면 연결; 유효한 `XAUTHORITY` 파일이 있으면 함께 연결 |
| Podman 그룹 | `--group-add keep-groups` |
| NVIDIA GPU | `RWH_GPU=1`일 때 Podman은 `--device nvidia.com/gpu=all`, Docker는 `--gpus all`과 `NVIDIA_DRIVER_CAPABILITIES=all` |

GPU를 사용하는 시뮬레이션·화면 구성이 필요하면 **최초 생성 시** 다음을 사용합니다.
NVIDIA driver와 Docker GPU runtime 또는 Podman CDI가 호스트에 준비되어 있어야 합니다.

```bash
# Podman
RWH_GPU=1 CONTAINER_ENGINE=podman ./scripts/robot.sh start
# Docker에서는 위 명령 대신:
# RWH_GPU=1 CONTAINER_ENGINE=docker ./scripts/robot.sh start
```

기존 컨테이너의 `start`는 생성 옵션을 바꾸지 않습니다.
GPU·장치·마운트 옵션을 바꾸려면 기존 생성 설정과 컨테이너 내부 변경분을 확인한 뒤 재생성합니다.
전체 장치 옵션은 배포 폴더의 `scripts/robot.sh`가 기준입니다.

runner 실행과 종료도 호스트에서 별도로 요청합니다.

```bash
# 별도 호스트 터미널에서 runner 실행
CONTAINER_ENGINE=podman ./scripts/robot.sh runner
# 컨테이너 종료
CONTAINER_ENGINE=podman ./scripts/robot.sh stop
```

Docker에서는 `CONTAINER_ENGINE=docker`로 바꿉니다.
이 배포 컨테이너 이름 `rwh_beyond_robot`은 기존 개발 컨테이너
`moma_mj_lab_1.1.2_hostcan`과 다릅니다. 새 컨테이너에도 `/root/ros2_ws`가 들어 있으며
기존 호스트의 `/work` bind mount는 필요하지 않습니다.
배포 runner는 `config/runners.yaml`을 사용하므로 모델 주소·카메라·CAN을 장비에 맞춥니다.
실행 절차는 [빠른 시작](grasp-pipeline-guide.md)을 참고하세요.

### 3. 모델 컨테이너 생성

두 모델 이미지와 가중치가 포함된 아카이브를 GPU 호스트에 불러온 뒤 실행합니다.
로봇 PC와 GPU 호스트가 같으면 같은 배포 폴더에서 이어서 실행합니다.

```bash
cd /home/ggh/RWH/beyond
CONTAINER_ENGINE=docker ./scripts/models.sh start
CONTAINER_ENGINE=docker ./scripts/models.sh status
# 가중치 로딩이 끝난 뒤
CONTAINER_ENGINE=docker ./scripts/models.sh health
```

Podman GPU 호스트에서는 위 명령에서 `CONTAINER_ENGINE=podman`을 사용합니다.
Docker Compose로 생성하려면 `models.sh start` 대신 다음을 사용합니다.

```bash
cd /home/ggh/RWH/beyond
docker compose -f compose.models.yaml config
docker compose -f compose.models.yaml up -d
docker compose -f compose.models.yaml ps -a
```

배포 Compose는 로컬 이미지를 사용하고 자동으로 pull하지 않습니다.
배포 이미지의 모델 공개 포트는 GraspGenX **5566**, SAM **5567**이며
호스트 `127.0.0.1`에 연결합니다. 원본 `deploy/graspgenx-docker/`의
소스 빌드 템플릿 포트 5556과 구분합니다.
컨테이너 생성·로그·health의 자세한 명령은
[배포 모델 컨테이너](inference-server-operation.md#deployment-model-containers)를 참고하세요.

## ROS workspace

기존 개발 컨테이너는 `moma_mj_lab_1.1.2_hostcan`, 위 배포본의 컨테이너는
`rwh_beyond_robot`이며 ROS workspace는 모두 `/root/ros2_ws`입니다.
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
