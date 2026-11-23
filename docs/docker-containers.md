# 도커 컨테이너 구성

**tar.gz 파일 받기 → 이미지 로드 → 컨테이너 생성 → 시작·접속 → 데이터 확인** 순서로 진행합니다.
아래 명령은 컨테이너 밖의 **호스트 터미널**에서 실행합니다.
로봇 PC와 GPU 서버를 분리하면 각 역할에 해당하는 파일과 명령만 사용합니다.
로봇용 `20261123-live` 스냅샷과 20261117 GPU 모델 아카이브를 다룹니다. 소스 수정·재빌드는
[환경과 빌드](project-containers.md)에서 설명합니다.


## amd64 로봇 스냅샷으로 Runner와 화면 연결 { #robot-runner-20261119 }

이 스냅샷은 `linux/amd64`입니다. Docker Engine을 사용하는 NVIDIA 노트북과
Podman을 사용하는 NVIDIA 개발 PC에서 실행할 수 있습니다. ARM64 Jetson은
[별도 로드·생성 절차](https://github.com/ggh-png/ros2-docker/blob/main/docs/docker-containers.md#jetson-robot)를 사용합니다.

아카이브와 `.sha256` 파일을 `~/RWH/beyond/docker`에 두고 로드합니다.
이미 로드했다면 [컨테이너 생성](#create-robot-snapshot)으로 이동합니다.

```bash
CONTAINER_ENGINE=podman  # Docker Engine 호스트에서는 docker
archive=rwh_beyond_robot_20261123-live-amd64.tar.gz
cd "$HOME/RWH/beyond/docker" &&
  sha256sum -c "$archive.sha256" &&
  "$CONTAINER_ENGINE" load -i "$archive"
```

### X11 인증 파일 준비 { #x11-auth }

호스트의 그래픽 데스크톱 터미널에서 실행합니다. 기존에 지정한 경로, 현재 셸의
`XAUTHORITY`, 기본 `~/.Xauthority` 순으로 확인하고, 필요하면 Mutter의 Xwayland 인증 파일을 찾습니다.

```bash
RWH_XAUTH_FILE="${RWH_XAUTH_FILE:-${XAUTHORITY:-$HOME/.Xauthority}}"
x11_runtime_dir="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [[ ( ! -f "$RWH_XAUTH_FILE" || ! -r "$RWH_XAUTH_FILE" ) && -d "$x11_runtime_dir" ]]; then
  RWH_XAUTH_FILE="$(find "$x11_runtime_dir" \
    -maxdepth 1 -type f -name '.mutter-Xwaylandauth.*' -print -quit)"
fi
test -n "${DISPLAY:-}" && test -f "$RWH_XAUTH_FILE" && test -r "$RWH_XAUTH_FILE"
```

### NVIDIA GPU 컨테이너 생성 { #create-robot-snapshot }

Podman에서는 `nvidia-ctk cdi list`에 `nvidia.com/gpu=all`이 표시되는지 확인합니다.
Docker Engine은 NVIDIA Container Toolkit을 사용합니다. `docker`가 Podman 래퍼라면
`CONTAINER_ENGINE=podman`을 선택합니다. [NVIDIA CDI 설정](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/cdi-support.html)

호스트 데스크톱의 Bash에서 아카이브가 있는 폴더로 이동한 뒤 아래 블록을 붙여넣습니다.
현재 폴더의 `config/`에 있는 YAML만 읽기 전용으로 연결합니다. 빠진 파일은 이미지에 들어
있는 기본 설정을 읽으므로 `config/` 없이 실행할 수 있습니다. `20261119-bb6e689-full`을
이미 로드했다면 아래 `image` 값에 해당 태그를 지정해 같은 명령을 사용할 수 있습니다.

```bash
(
  set -euo pipefail
  engine="${CONTAINER_ENGINE:-podman}"
  image=localhost/rwh_beyond_robot:20261123-live
  config_dir="$PWD/config"
  runner_config_dir=/root/ros2_ws/src/ros_runner/config
  bt_config=/root/ros2_ws/src/rwh_beyond/rwh_beyond_bringup/config/bt_demo.yaml
  case "$engine" in
    podman) gpu=(--device nvidia.com/gpu=all --security-opt label=disable) ;;
    docker) gpu=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all) ;;
    *) echo 'CONTAINER_ENGINE must be podman or docker' >&2; exit 2 ;;
  esac

  xauth_file="${RWH_XAUTH_FILE:-${XAUTHORITY:-$HOME/.Xauthority}}"
  x11_runtime_dir="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  if [[ ( ! -f "$xauth_file" || ! -r "$xauth_file" ) && -d "$x11_runtime_dir" ]]; then
    xauth_file="$(find "$x11_runtime_dir" -maxdepth 1 -type f \
      -name '.mutter-Xwaylandauth.*' -print -quit)"
  fi
  if [[ -z "${DISPLAY:-}" || ! -f "$xauth_file" || ! -r "$xauth_file" || ! -d /tmp/.X11-unix ]]; then
    echo 'Run from the host desktop with DISPLAY and readable Xauthority.' >&2
    exit 1
  fi

  mounts=()
  for file in runners robot_common robot_real robot_sim; do
    if [[ -f "$config_dir/$file.yaml" ]]; then
      mounts+=(-v "$config_dir/$file.yaml:$runner_config_dir/$file.yaml:ro")
    fi
  done
  if [[ -f "$config_dir/bt_demo.yaml" ]]; then
    mounts+=(-v "$config_dir/bt_demo.yaml:$bt_config:ro")
  fi
  [[ ! -d /dev/bus/usb ]] || mounts+=(-v /dev/bus/usb:/dev/bus/usb)
  [[ ! -d /run/udev ]] || mounts+=(-v /run/udev:/run/udev:ro)
  mkdir -p "$HOME/wsyoon_data"

  "$engine" run -d --pull=never --name rwh_beyond_robot \
    "${gpu[@]}" --hostname "$(hostname)" \
    --network host --ipc host --init --privileged \
    --restart unless-stopped \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY=/tmp/rwh_beyond.Xauthority \
    -e QT_QPA_PLATFORM=xcb \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v "$xauth_file:/tmp/rwh_beyond.Xauthority:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$HOME/wsyoon_data:/root/wsyoon_data" \
    "${mounts[@]}" "$image"
)
```

컨테이너는 `run -d`로 시작하고 Runner 노드는 `auto_start:=false`로 `unconfigured` 상태에서
대기합니다. 아래 명령으로 처음 한 번 `configure`할 때
`/root/ros2_ws/src/ros_runner/config/runners.yaml`을 읽습니다. `activate`하면 기본 YAML에서
`on_launch: true`인 `runtime_setup`이 장치와 추론 연결을 점검합니다. `piper_sim`과
`bt_demo_sim`은 각 프로필을 별도로 활성화할 때 실행됩니다.

```bash
"${CONTAINER_ENGINE:-podman}" exec rwh_beyond_robot /opt/rwh_beyond/entrypoint.sh \
  bash -ec 'ros2 lifecycle set /ros_runner configure && ros2 lifecycle set /ros_runner activate'
```

Runner 로그는 `"${CONTAINER_ENGINE:-podman}" logs -f rwh_beyond_robot`로 확인합니다.
`20261123-live`에서 비밀번호로 추론 서버에 연결하려면 호스트의 `config/runners.yaml`을
준비하고 `inference_connection.password`를 로컬에서 설정합니다.

GPU 옵션은 생성 시 적용됩니다. 컨테이너를 교체하기 전에 내부 소스·관절 자세·설정을
호스트에 복사한 뒤, 같은 엔진에서 기존 컨테이너를 정리하고 새로 생성합니다.

### MuJoCo 창이 열리지 않을 때 { #mujoco-glx }

`piper_sim`의 로그에 `No GLXFBConfigs returned` 또는 `Failed to find a suitable GLXFBConfig`가
나오면 X11 인증과 NVIDIA GLX 라이브러리 전달을 확인합니다. Podman에서는
`--device nvidia.com/gpu=all`로 드라이버 라이브러리와 GPU를 연결합니다. GPU 장치 파일이나
`DISPLAY`가 보여도 GLX 드라이버가 연결됐는지는 별도로 확인해야 합니다.

MuJoCo 창으로 PiPER 시뮬레이션을 실행할 때는 `robot_sim.yaml`의
`robot_settings.ros__parameters.simulation.gui`를 `true`로 두고 `piper_sim`을 활성화합니다.
화면 없이 관절·카메라·MoveIt을 실행할 때는 `gui: false`를 사용합니다.
전체 집기·배치 데모인 `bt_demo_sim`에는 SAM 3·GraspGenX 연결도 필요합니다.

```bash
"${CONTAINER_ENGINE:-podman}" exec -it rwh_beyond_robot /opt/rwh_beyond/entrypoint.sh \
  ros2 service call /ros_runner/activate_piper_sim std_srvs/srv/Empty '{}'
```

아래의 20261117 로봇 명령은 이전 배포를 다시 구성할 때 사용합니다.
GPU 모델 서버에는 20261117 모델 이미지와 아래 서버 명령을 사용합니다.

## 1. tar 파일 받기 { #deployment-containers }

배포 담당자가 제공하는 이미지 아카이브와 설정·스크립트를 받아 `~/RWH/beyond`에 둡니다.
현재 배포 원본 위치는 `/home/ggh/RWH/beyond`이며 다른 PC에서는 해당 폴더를 전달받습니다.
별도 다운로드 URL이나 레지스트리 게시를 전제로 하지 않습니다.
호스트에서 설정을 덮어쓸 때는 `config/`를, helper나 Compose를 사용할 때는 `scripts/`나
`compose.models.yaml`을 함께 준비합니다.
이 배포 폴더는 Git 저장소의 `deploy/`와 별개입니다.

| 받을 파일 | 사용할 호스트 | 기존 아카이브에 저장된 태그 |
|---|---|---|
| `rwh_beyond_robot_20261117.tar.gz` | 로봇 PC | `docker.io/rwh_beyond/robot:20261117` |
| `rwh_beyond_models_20261117.tar.gz` | GPU 서버 | `docker.io/rwh_beyond/graspgenx:20261117`, `docker.io/rwh_beyond/sam3:20261117` |

같은 PC에서 로봇과 모델을 모두 실행하면 두 아카이브를 모두 받습니다.
이미지 아카이브는 `docker load`가 읽으므로 미리 `tar`로 압축을 풀지 않습니다.
기존 아카이브에는 위 태그가 저장돼 있습니다. 다음 단계에서 프로젝트의 고정 이미지 이름을
등록합니다. 이 배포는 로컬 이미지로 실행하며 `docker pull`은 사용하지 않습니다.

## 2. 이미지 로드 { #load-images }

프로젝트에서 사용하는 이미지 이름은 아래처럼 **고정**합니다.
이미지 이름과 컨테이너 이름은 구분하며, `:20261117`은 이미지 버전 태그입니다.

| 역할 | 고정 이미지 이름 | 컨테이너 이름 |
|---|---|---|
| 로봇 | `rwh_beyond_robot:20261117` | `rwh_beyond_robot` |
| GraspGenX | `rwh_beyond_graspgenx:20261117` | `rwh_beyond_graspgenx` |
| SAM 3 | `rwh_beyond_sam3:20261117` | `rwh_beyond_sam3` |

직접 빌드할 때도 `-t rwh_beyond_robot:20261117`처럼 해당 고정 이름을 지정합니다.
이미 만들어진 tar 파일은 `docker load`로 불러오고, 아래 `docker tag`로 이름을 등록합니다.
`tag`는 같은 이미지에 이름을 추가하는 작업으로 이미지를 다시 빌드하지 않습니다.
Podman에서는 이미지 목록에 `localhost/` 접두사가 붙어 보일 수 있습니다.

해당 호스트에서 사용할 이미지를 로드하고 고정 이름을 등록합니다.

### 로봇 PC

```bash
# 배포 파일과 설정이 있는 디렉터리로 이동
cd ~/RWH/beyond
# 로봇 이미지 아카이브를 로컬에 불러오기
docker load --input rwh_beyond_robot_20261117.tar.gz
# 로봇 이미지에 프로젝트 고정 이름 등록
docker tag docker.io/rwh_beyond/robot:20261117 rwh_beyond_robot:20261117
```

### GPU 서버

```bash
# 배포 파일과 설정이 있는 디렉터리로 이동
cd ~/RWH/beyond
# GraspGenX·SAM 3 이미지 아카이브를 로컬에 불러오기
docker load --input rwh_beyond_models_20261117.tar.gz
# GraspGenX 이미지에 프로젝트 고정 이름 등록
docker tag docker.io/rwh_beyond/graspgenx:20261117 rwh_beyond_graspgenx:20261117
# SAM 3 이미지에 프로젝트 고정 이름 등록
docker tag docker.io/rwh_beyond/sam3:20261117 rwh_beyond_sam3:20261117
```

현재 개발 호스트처럼 `docker`가 Podman wrapper이면 위 `docker load`, `docker tag`를
각각 `podman load`, `podman tag`로 바꿉니다.
두 아카이브를 모두 받은 환경에서는 `CONTAINER_ENGINE=podman ./scripts/load-images.sh`로
이미지 로드와 고정 태그 등록을 한 번에 수행할 수 있습니다.
Docker Engine에서는 `CONTAINER_ENGINE=docker`로 바꿉니다.
예전 배포 스크립트에는 고정 태그 등록이 없을 수 있으므로 위 `tag` 명령을 먼저 수행하고,
생성 helper와 `compose.models.yaml`도 고정 이미지 이름을 사용하는지 확인합니다.

## 3. 20261117 컨테이너 생성 { #container-one-liners }

이미지를 불러온 뒤 아래 명령으로 컨테이너를 생성합니다.
로봇용은 **로봇 PC 호스트**, 서버용은 **GPU 서버 호스트**에서 실행합니다.
표는 NVIDIA Container Toolkit이 준비된 **Docker Engine 기준**입니다.
현재 개발 PC처럼 Docker 명령이 Podman wrapper인 환경에서는
[배포 helper 사용 방법](#robot-helper)을 따릅니다.

로봇 PC에서는 [X11 인증 파일 준비](#x11-auth)를 실행하고, 배포 폴더와 호스트 데이터 디렉터리를 준비합니다.

```bash
# 배포 파일과 설정이 있는 디렉터리로 이동
cd ~/RWH/beyond
# 지도·데이터를 보관할 호스트 디렉터리 준비
mkdir -p "$HOME/wsyoon_data"
```

로봇 컨테이너는 아래 명령으로 생성합니다. 실제 장치가 있는 경우 USB·udev를 연결하며,
`config/`에 있는 YAML만 읽기 전용으로 연결합니다. 빠진 파일은 이미지 안의 기본값을 사용합니다.
이 버전의 CAN·카메라·시뮬레이션 설정은 `runners.yaml`에 들어 있습니다.

```bash
(
  set -e
  if [[ -z "${DISPLAY:-}" || ! -f "${RWH_XAUTH_FILE:-}" || ! -r "$RWH_XAUTH_FILE" || ! -d /tmp/.X11-unix ]]; then
    echo 'Run from the host desktop with DISPLAY and readable Xauthority.' >&2
    exit 1
  fi
  mounts=()
  if [[ -f "$PWD/config/runners.yaml" ]]; then
    mounts+=(-v "$PWD/config/runners.yaml:/opt/rwh_beyond/config/runners.yaml:ro")
  fi
  if [[ -f "$PWD/config/bt_demo.yaml" ]]; then
    mounts+=(-v "$PWD/config/bt_demo.yaml:/root/ros2_ws/src/rwh_beyond/rwh_beyond_bringup/config/bt_demo.yaml:ro")
  fi
  [[ ! -d /dev/bus/usb ]] || mounts+=(-v /dev/bus/usb:/dev/bus/usb)
  [[ ! -d /run/udev ]] || mounts+=(-v /run/udev:/run/udev:ro)

  docker run -d --name rwh_beyond_robot \
    --gpus all --hostname "$(hostname)" \
    --network host --ipc host --init --privileged --restart unless-stopped \
    -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/rwh_beyond.Xauthority -e QT_QPA_PLATFORM=xcb \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v "$RWH_XAUTH_FILE:/tmp/rwh_beyond.Xauthority:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$HOME/wsyoon_data:/root/wsyoon_data" "${mounts[@]}" \
    rwh_beyond_robot:20261117
)
```

<div class="container-command-table" markdown="1">

| 이미지 이름 | 명령어 |
|---|---|
| **GraspGenX 서버** | `# GPU·5566 포트를 연결해 GraspGenX 서버 생성·실행`<br/>`docker run -d --name rwh_beyond_graspgenx --restart unless-stopped --shm-size 256m --gpus all -p 127.0.0.1:5566:5566 -v rwh_beyond_graspgenx_cache:/data/cache -v rwh_beyond_graspgenx_logs:/data/logs rwh_beyond_graspgenx:20261117` |
| **SAM 3 서버** | `# GPU·5567 포트를 연결해 SAM 3 서버 생성·실행`<br/>`docker run -d --name rwh_beyond_sam3 --restart unless-stopped --shm-size 256m --gpus all -p 127.0.0.1:5567:5567 -v rwh_beyond_sam3_cache:/data/cache -v rwh_beyond_sam3_logs:/data/logs rwh_beyond_sam3:20261117` |

</div>

로봇 예시는 `--privileged`로 실물 장치 접근을 허용합니다.
장치를 개별 전달하는 구성은 아래 helper를 사용합니다.
로봇은 host network이므로 `-p`를 추가하지 않으며, 모델 서버는 loopback 포트 매핑을 사용합니다.
20261117 로봇 이미지의 기본 명령은 `sleep infinity`여서 생성만으로 runner·로봇 동작을 실행하지 않습니다.
모델 이미지는 생성 시 모델 서버를 시작합니다.

## 4. 컨테이너 시작·접속 { #container-shell }

생성한 로봇 컨테이너에 ROS 환경을 불러온 셸로 접속합니다.

```bash
# 로봇 컨테이너에 ROS 환경을 불러온 셸로 접속
docker exec -it rwh_beyond_robot /opt/rwh_beyond/entrypoint.sh bash --noprofile --norc
```

컨테이너가 정지 상태면 먼저 시작합니다. `start`는 기존 생성 옵션을 그대로 사용합니다.

```bash
# 생성돼 있는 정지 컨테이너 시작
docker start rwh_beyond_robot
```

서버 내부 셸이 필요하면 다음을 사용합니다. 모델 health 명령은
[GPU 추론 서버 운영](inference-server-operation.md#model-docker-run)을 참고하세요.

```bash
# GraspGenX 서버 컨테이너 내부 셸에 접속
docker exec -it rwh_beyond_graspgenx bash
# SAM 3 서버 컨테이너 내부 셸에 접속
docker exec -it rwh_beyond_sam3 bash
```

## 5. 데이터 마운트·지도 파일 확인 { #persistent-data }

로봇 생성 명령의 `-v "$HOME/wsyoon_data:/root/wsyoon_data"`는
**호스트의 데이터 디렉터리를 컨테이너에 읽기·쓰기 연결**합니다.
`$HOME`은 명령을 실행하는 호스트 사용자의 홈입니다. 예를 들어 호스트 사용자가 `ggh`이면:

| 호스트 경로 | 컨테이너 경로 | 용도 |
|---|---|---|
| `/home/ggh/wsyoon_data/map/map.yaml` | `/root/wsyoon_data/map/map.yaml` | navigation의 기본 지도 YAML |
| `/home/ggh/wsyoon_data/map/` 안의 지도 이미지 | `/root/wsyoon_data/map/` 안의 지도 이미지 | YAML의 `image:`가 참조하는 파일 |
| `/home/ggh/wsyoon_data/maps/` | `/root/wsyoon_data/maps/` | map_save의 날짜별 저장 디렉터리 |

`mkdir -p`는 빈 디렉터리만 준비하며 지도 파일을 생성하지 않습니다.
기존 지도 YAML과 해당 이미지 파일을 함께 준비하고, YAML의 `image:`가
컨테이너에서 접근 가능한 경로인지 확인합니다. 다른 지도를 사용할 때는 runner의
`RWH_MAP_FILE`도 실제 컨테이너 경로에 맞춰야 합니다.

컨테이너 생성 후 파일이 보이는지 확인합니다. 아래 검사는 로봇을 움직이지 않습니다.

```bash
# 컨테이너에 연결된 호스트·컨테이너 마운트 경로 확인
docker inspect rwh_beyond_robot --format '{{json .Mounts}}'
# 컨테이너에서 기본 지도 YAML 파일이 보이는지 확인
docker exec rwh_beyond_robot ls -l /root/wsyoon_data/map/map.yaml
```

기존 컨테이너 안에만 지도가 있다면 호스트의 **기존 파일과 겹치지 않는 백업 경로**로
먼저 복사합니다. 새 bind mount는 이미지 안의 같은 경로를 가리므로 기존 내용을 자동 병합하지 않습니다.
마운트 누락은 `docker start`나 `restart`로 추가되지 않습니다.
내부 변경분·지도·생성 설정을 보존한 뒤 마운트를 포함해 재생성해야 합니다.
이 문서의 명령은 현재 실행 중인 개발 컨테이너를 자동 변경하지 않습니다.

## 6. 컨테이너를 다시 이미지로 저장 { #container-to-image }

컨테이너 안에서 설치·수정한 내용을 이미지로 남길 때는 **호스트에서 `docker commit`**을 실행합니다.
로봇 제어·지도 저장·모델 요청 등 진행 중인 작업을 마친 뒤 대기 상태에서 저장합니다.
기본 commit은 실행 중인 컨테이너를 저장하는 동안 잠시 일시정지합니다.

이미지 이름은 그대로 유지하고, 원본 `:20261117`과 구분되는 `:snapshot` 태그로 저장합니다.
아래 명령은 필요한 컨테이너에 대해서만 실행합니다. 같은 `:snapshot` 태그로 다시 저장하면
그 태그가 새 이미지로 갱신됩니다.

<div class="container-command-table" markdown="1">

| 대상 컨테이너 | 이미지로 저장하는 명령어 |
|---|---|
| **BEYOND Robot** | `# 로봇 컨테이너의 파일 변경분을 snapshot 이미지로 저장`<br/>`docker commit rwh_beyond_robot rwh_beyond_robot:snapshot` |
| **GraspGenX 서버** | `# GraspGenX 컨테이너의 파일 변경분을 snapshot 이미지로 저장`<br/>`docker commit rwh_beyond_graspgenx rwh_beyond_graspgenx:snapshot` |
| **SAM 3 서버** | `# SAM 3 컨테이너의 파일 변경분을 snapshot 이미지로 저장`<br/>`docker commit rwh_beyond_sam3 rwh_beyond_sam3:snapshot` |

</div>

### 다른 PC로 전달할 tar.gz 파일 만들기

`docker commit`은 로컬에 이미지를 만듭니다. 파일로 전달하려면 이어서 `docker save`로 저장합니다.
아래 파일명은 원본 배포 파일과 구분되며, 같은 이름의 출력 파일이 있으면 덮어씁니다.

```bash
# 로봇 snapshot 이미지를 압축된 배포 파일로 저장
docker save rwh_beyond_robot:snapshot | gzip > rwh_beyond_robot_snapshot.tar.gz
```

```bash
# 두 모델의 snapshot 이미지를 하나의 압축 파일에 함께 저장
docker save rwh_beyond_graspgenx:snapshot rwh_beyond_sam3:snapshot | gzip > rwh_beyond_models_snapshot.tar.gz
```

파일을 받은 PC에서는 해당 역할의 이미지를 로드합니다.

```bash
# 로봇 PC에서 snapshot 이미지를 불러오기
docker load --input rwh_beyond_robot_snapshot.tar.gz
```

```bash
# GPU 서버에서 두 모델의 snapshot 이미지를 불러오기
docker load --input rwh_beyond_models_snapshot.tar.gz
```

이 파일에는 이미 `rwh_beyond_robot:snapshot` 등의 이름이 저장돼 있으므로
예전 배포 태그를 바꾸는 `docker tag` 명령을 다시 실행하지 않습니다.
[컨테이너 생성 명령](#container-one-liners)의 이미지 태그를 `:20261117` 대신
`:snapshot`으로 바꿔 사용합니다. 기존 컨테이너는 이미지를 로드해도 자동 교체되지 않습니다.
현재 PC에서 별도 검증 컨테이너를 만들 때는 `--name`도 기존 이름과 겹치지 않게 지정합니다.
Podman 환경에서는 `docker commit`, `docker load`를 각각 `podman commit`, `podman load`로 바꿉니다.
로봇 이미지 하나는 `podman save`로 저장하고, 모델 두 개를 함께 저장할 때는 다음을 사용합니다.

```bash
# Podman에서 두 모델 이미지를 하나의 Docker 호환 압축 파일로 저장
podman save --multi-image-archive rwh_beyond_graspgenx:snapshot rwh_beyond_sam3:snapshot | gzip > rwh_beyond_models_snapshot.tar.gz
```

### 이미지에 포함되는 것과 별도로 전달할 것

`docker commit`은 컨테이너 내부 파일 변경을 저장하지만 **bind mount·volume의 데이터는 포함하지 않습니다.**
따라서 이번 구성에서는 다음 파일을 이미지 아카이브와 별도로 보관·전달합니다.

| 별도 전달 항목 | 이유 |
|---|---|
| 호스트 `~/wsyoon_data/` | `/root/wsyoon_data`는 bind mount이므로 지도·저장 데이터를 commit에 포함하지 않음 |
| 배포 폴더의 `config/` | 호스트에서 덮어쓴 Runner·장치·BT 설정을 보존할 때 전달 |
| 배포 폴더의 `scripts/`, `compose.models.yaml` | 컨테이너 생성·연결 옵션을 다시 적용하는 데 사용 |
| 서버의 cache·logs volume | 이미지가 아닌 별도 volume에 저장됨; 필요한 데이터만 별도 보관 |

GPU·USB·X11·host network·포트·데이터 마운트 같은 **컨테이너 생성 옵션도 다시 지정**해야 합니다.
서버 가중치를 외부 volume으로 바꾼 구성이라면 그 파일 역시 별도로 전달합니다.
자세한 동작은 Docker의 [commit](https://docs.docker.com/reference/cli/docker/container/commit/)과
[save](https://docs.docker.com/reference/cli/docker/image/save/) 문서를 참고하세요.

## 배포 helper를 사용하는 경우 { #engine-helpers }

위 직접 생성 명령 대신 배포 폴더의 스크립트로 생성할 수도 있습니다.
이미지 로드까지 완료한 뒤 아래 방법을 선택하며, 같은 이름의 컨테이너를 중복 생성하지 않습니다.

### 로봇 컨테이너 { #robot-helper }

GUI를 연결하려면 먼저 [X11 인증 파일 준비](#x11-auth)를 실행하고
`export XAUTHORITY="$RWH_XAUTH_FILE"`로 helper에 전달합니다.
Podman에서는 CDI 장치도 `nvidia-ctk cdi list`로 확인합니다.

```bash
# 배포 파일과 설정이 있는 디렉터리로 이동
cd /home/ggh/RWH/beyond
# GPU·GUI·장치·데이터를 연결해 로봇 컨테이너 생성 또는 기존 컨테이너 시작
RWH_GPU=1 CONTAINER_ENGINE=podman ./scripts/robot.sh start
# 로봇 컨테이너 실행 상태 확인
CONTAINER_ENGINE=podman ./scripts/robot.sh status
# ROS 환경을 불러온 로봇 컨테이너 셸에 접속
CONTAINER_ENGINE=podman ./scripts/robot.sh shell
```

Docker를 사용하는 호스트에서는 대신:

```bash
# 배포 파일과 설정이 있는 디렉터리로 이동
cd /home/ggh/RWH/beyond
# GPU·GUI·장치·데이터를 연결해 로봇 컨테이너 생성 또는 기존 컨테이너 시작
RWH_GPU=1 CONTAINER_ENGINE=docker ./scripts/robot.sh start
# ROS 환경을 불러온 로봇 컨테이너 셸에 접속
CONTAINER_ENGINE=docker ./scripts/robot.sh shell
```

**`start`가 최초 호출이면 `run -d`로 생성하고, 같은 이름의 컨테이너가 있으면 `start`로 재시작합니다.**
로봇·runner는 자동 실행하지 않으며 이미지의 `sleep infinity`로 대기합니다.
셸 접속은 이미지의 entrypoint를 거쳐 ROS 환경을 불러옵니다.

| 생성 설정 | `robot.sh`가 적용하는 옵션·경로 |
|---|---|
| 기본 실행 | `run -d --name rwh_beyond_robot --network host --ipc host --init --cap-add NET_ADMIN --security-opt label=disable` |
| 영구 데이터 | `${RWH_DATA_DIR:-$HOME/wsyoon_data}` → `/root/wsyoon_data` 읽기·쓰기 |
| ROS domain | `ROS_DOMAIN_ID` 환경변수, 생략 시 0 |
| runner 설정 | `config/runners.yaml` → `/opt/rwh_beyond/config/runners.yaml` 읽기 전용 |
| BT 설정 | `config/bt_demo.yaml` → `/root/ros2_ws/src/rwh_beyond/rwh_beyond_bringup/config/bt_demo.yaml` 읽기 전용 |
| USB·장치 | 존재하는 `/dev/bus/usb`, `/run/udev`, `/dev/video*`, `/dev/media*`, `/dev/dri/renderD*` 연결 |
| 화면 | `DISPLAY`와 X11 socket이 있으면 연결; 유효한 `XAUTHORITY` 파일이 있으면 함께 연결 |
| Podman 그룹 | `--group-add keep-groups` |
| NVIDIA GPU | `RWH_GPU=1`일 때 Podman은 `--device nvidia.com/gpu=all`, Docker는 `--gpus all`과 `NVIDIA_DRIVER_CAPABILITIES=all` |

위 로봇 생성 명령은 `RWH_GPU=1`로 NVIDIA GPU 연결을 활성화합니다.
NVIDIA driver와 Docker GPU runtime 또는 Podman CDI가 호스트에 준비되어 있어야 합니다.
GPU를 사용하지 않는 로봇 PC에서는 `RWH_GPU=1`을 생략합니다.

기존 컨테이너의 `start`는 생성 옵션을 바꾸지 않습니다.
GPU·장치·마운트 옵션을 바꾸려면 기존 생성 설정과 컨테이너 내부 변경분을 확인한 뒤 재생성합니다.
전체 장치 옵션은 배포 폴더의 `scripts/robot.sh`가 기준입니다.
예전 배포 helper에는 `wsyoon_data` 마운트가 없을 수 있습니다. 최초 생성 전에
`-v "$rwh_data_dir:/root/wsyoon_data"`가 포함돼 있는지 확인하고, 없으면 위 직접 생성 명령을 사용하거나
helper에 데이터 경로를 추가합니다. 호스트 경로를 바꾸려면 수정된 helper의 `RWH_DATA_DIR`을 사용합니다.
helper로 생성한 뒤에도 [마운트·지도 파일 확인](#persistent-data)을 수행합니다.

runner 실행과 종료도 호스트에서 별도로 요청합니다.

```bash
# 별도 터미널에서 ros_runner 실행
CONTAINER_ENGINE=podman ./scripts/robot.sh runner
# 로봇 컨테이너 종료
CONTAINER_ENGINE=podman ./scripts/robot.sh stop
```

Docker에서는 `CONTAINER_ENGINE=docker`로 바꿉니다.
이 배포 컨테이너 이름 `rwh_beyond_robot`은 기존 개발 컨테이너
`moma_mj_lab_1.1.2_hostcan`과 다릅니다. 새 컨테이너에도 `/root/ros2_ws`가 들어 있으며
기존 호스트의 `/work` bind mount는 필요하지 않습니다.
20261117 배포에서는 `config/runners.yaml`에서 모델 주소와 카메라·CAN 설정을 장비에 맞춥니다.
실행 절차는 [빠른 시작](grasp-pipeline-guide.md)을 참고하세요.

### 모델 서버 컨테이너

두 모델 이미지와 가중치가 포함된 아카이브를 GPU 호스트에 불러온 뒤 실행합니다.
로봇 PC와 GPU 호스트가 같으면 같은 배포 폴더에서 이어서 실행합니다.

```bash
# 배포 파일과 설정이 있는 디렉터리로 이동
cd /home/ggh/RWH/beyond
# GraspGenX·SAM 3 컨테이너를 함께 생성·시작
CONTAINER_ENGINE=docker ./scripts/models.sh start
# 두 모델 컨테이너 실행 상태 확인
CONTAINER_ENGINE=docker ./scripts/models.sh status
# 가중치 로딩 후 두 모델 서버 health 확인
CONTAINER_ENGINE=docker ./scripts/models.sh health
```

Podman GPU 호스트에서는 위 명령에서 `CONTAINER_ENGINE=podman`을 사용합니다.
Docker Compose로 생성하려면 `models.sh start` 대신 다음을 사용합니다.

```bash
# 배포 파일과 설정이 있는 디렉터리로 이동
cd /home/ggh/RWH/beyond
# Compose 설정과 환경변수가 올바르게 적용되는지 확인
docker compose -f compose.models.yaml config
# 지정한 모델 컨테이너를 생성하고 백그라운드 실행
docker compose -f compose.models.yaml up -d
# 모델 서비스의 실행·정지 상태 확인
docker compose -f compose.models.yaml ps -a
```

배포 Compose는 로컬 이미지를 사용하고 자동으로 pull하지 않습니다.
배포 이미지의 모델 공개 포트는 GraspGenX **5566**, SAM **5567**이며
호스트 `127.0.0.1`에 연결합니다. 원본 `deploy/graspgenx-docker/`의
소스 빌드 템플릿 포트 5556과 구분합니다.
컨테이너 생성·로그·health의 자세한 명령은
[배포 모델 컨테이너](inference-server-operation.md#deployment-model-containers)를 참고하세요.

[ROS 환경과 빌드](project-containers.md) · [GPU 서버 운영·health](inference-server-operation.md) · [실행 가이드](grasp-pipeline-guide.md)
